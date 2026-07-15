#!/usr/bin/env python3
import sys
import subprocess
from threading import Thread
from queue import Queue, Empty
from pprint import pprint
import sys
import os
import time
import yaml
import re
import pdb
import string
import hashlib
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

def randomNumber(stringLength=4):
    """Generate a random string of fixed length """
    number = random.randint(1000, 9999)
    return str(number)


def enqueue_output(out, queue):
    """ Saves the output of the process in a queue to be parsed
    afterwards """
    for line in iter(out.readline, b''):
        queue.put(line)
    out.close()


def ping_ip(ip_address):
    try:
        # Run the ping command with a single ping packet (-c 1) and a timeout of 1 second (-W 1)
        result = subprocess.run(["ping", "-c", "1", "-W", "1", ip_address],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        return result.returncode == 0
    except subprocess.CalledProcessError:
        # An error occurred (ping failed)
        return False


def line_parser(line_bytes):
    """ Returns parsed str version of bytes input line
    This is quite magic: rajant output is not yaml but it is very
    yamlish. If we replace the { with :, we remove }, and we do some
    minor modifications everything works out of the box!"""
    line_str = line_bytes.decode('unicode-escape')
    line_str = line_str.replace("{", ":")
    line_str = line_str.replace("}", "")
    # random numbers are added to avoid overwriting the key on the yaml
    line_str = re.sub("wireless",
                      "wireless-" + randomNumber(), line_str)
    line_str = re.sub("peer",
                      "peer-" + randomNumber(), line_str)
    # MACs are a little bit more tricky
    if line_str.replace(" ", "")[:4] == "mac:":
        separator = line_str.find(":") + 2
        mac_str = line_str[separator:]
        mac_bytes = bytes(mac_str, 'raw_unicode_escape')
        mac_decoded = ":".join(["%02x" % c for c in mac_bytes[1:-2]])
        line_str = line_str[:separator] + mac_decoded + "\n"
    return line_str


ON_POSIX = 'posix' in sys.builtin_module_names


class RajantQueryNode(Node):
    def __init__(self):
        super().__init__('rajant_query')

        # Declare parameters
        self.declare_parameter('robot_name', 'charon')
        self.declare_parameter('robot_configs', '')
        self.declare_parameter('radio_configs', '')
        self.declare_parameter('bcapi_jar_file', '')
        self.declare_parameter('wifi_backup_period', 10.0)

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        robot_configs_file = self.get_parameter('robot_configs').get_parameter_value().string_value
        radio_configs_file = self.get_parameter('radio_configs').get_parameter_value().string_value
        self.bcapi_jar_file = self.get_parameter('bcapi_jar_file').get_parameter_value().string_value
        self.wifi_backup_period = self.get_parameter('wifi_backup_period').get_parameter_value().double_value

        # Load robot configs
        with open(robot_configs_file, "r") as f:
            robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        if self.robot_name not in robot_configs.keys():
            self.get_logger().error("Robot not in config file")
            return

        # Load radio configs
        with open(radio_configs_file, "r") as f:
            radio_configs = yaml.load(f, Loader=yaml.FullLoader)
        radio = robot_configs[self.robot_name]["using-radio"]
        if radio not in radio_configs.keys():
            self.get_logger().error("Radio not in config file")
            return

        # Get target IP
        rajant_name = robot_configs[self.robot_name]['using-radio']
        if rajant_name in radio_configs.keys():
            self.target_ip = radio_configs[rajant_name]['computed-IP-address']
        else:
            self.get_logger().error(f"Radio {rajant_name} for robot {self.robot_name} not found in configs")
            return

        # Create ROS publisher
        self.pub = self.create_publisher(String, 'mocha/rajant/log', 10)

        # Get package path
        try:
            ros_path = get_package_share_directory('interface_rajant')
        except:
            self.get_logger().error("Could not find interface_rajant package")
            return

        # Java binary path
        if hasattr(self, 'bcapi_jar_file') and self.bcapi_jar_file:
            self.java_bin = self.bcapi_jar_file
        else:
            self.java_bin = os.path.join(ros_path,
                                    'thirdParty/watchstate/bcapi-watchstate-11.19.0-SNAPSHOT-jar-with-dependencies.jar')

        # Initialize subprocess variables
        self.p = None
        self.q = None
        self.t = None

        # Start the Java process
        self.start_java_process()

        # Go
        self.get_logger().info(f"{self.robot_name} - Rajant API Query - Starting on {rajant_name}")

        # Ping the assigned radio
        if ping_ip(self.target_ip):
            self.get_logger().info(f"{self.robot_name} - Rajant API Query - ping success")
        else:
            self.get_logger().error(f"{self.robot_name} - Rajant API Query - Rajant ping failed (will keep trying anyway)")

        self.last_data_time = time.time()
        self.accumulated_answ = ""
        # Create timer for main processing loop
        self.timer = self.create_timer(1.0, self.process_rajant_data)

    def start_java_process(self):
        """Start or restart the Java process"""
        self.p = subprocess.Popen(['java',
                   '-jar',
                   self.java_bin,
                   self.target_ip], stdout=subprocess.PIPE, close_fds=ON_POSIX)
        self.q = Queue()
        self.t = Thread(target=enqueue_output, args=(self.p.stdout, self.q))
        self.t.daemon = True  # thread dies with the program
        self.t.start()

    def process_rajant_data(self):
        """Main processing loop - called by timer"""
        now = time.time()
        
        # If the java process is dead, only try to restart it every wifi_backup_period seconds to avoid log spam and continuous pinging
        if self.t is not None and not self.t.is_alive():
            if not hasattr(self, 'last_restart_attempt') or now - self.last_restart_attempt > self.wifi_backup_period:
                self.last_restart_attempt = now
                self.get_logger().error(f'{self.robot_name}: Rajant Java process is dead. Checking connection...')
                if ping_ip(self.target_ip):
                    self.get_logger().info(f'{self.robot_name}: Rajant reachable, restarting Java process.')
                    self.start_java_process()
                    self.accumulated_answ = ""
                    self.last_data_time = now
                else:
                    self.get_logger().error(f'{self.robot_name}: Rajant unreachable, waiting to restart...')
            return

        try:
            line = self.q.get_nowait()
        except Empty:
            # We got no data this tick. 
            # If we've had complete radio silence for > wifi_backup_period seconds, ping to verify if the socket is hung.
            if now - self.last_data_time > self.wifi_backup_period:
                # To avoid pinging every second during a long legitimate silence, only ping every 5 seconds.
                if not hasattr(self, 'last_silence_ping_time') or now - self.last_silence_ping_time > 5.0:
                    self.last_silence_ping_time = now
                    if not ping_ip(self.target_ip):
                        self.get_logger().error(f'{self.robot_name}: Rajant radio physically unreachable during silence! Killing hung process...')
                        if self.p:
                            self.p.kill()
                            self.p.wait()
                        # The next timer tick will see t is not alive and handle restarting
            return
        else:  # got line
            self.last_data_time = now
            answ_array = self.accumulated_answ + line_parser(line)
            while True:
                try:
                    newline = self.q.get_nowait()
                except Empty:
                    break
                else:
                    answ_array += line_parser(newline)
            try:
                yaml_res = yaml.load(answ_array, Loader=yaml.Loader)
                if type(yaml_res) == type({}):
                    msg = String()
                    msg.data = str(yaml_res)
                    self.pub.publish(msg)
                    self.accumulated_answ = ""
                else:
                    self.get_logger().error(f"{self.robot_name}: YAML from Rajant did not look like an object!")
                    self.accumulated_answ = ""
            except yaml.scanner.ScannerError:
                self.accumulated_answ = answ_array
                if len(self.accumulated_answ) > 500000:
                    self.get_logger().error(f"{self.robot_name}: Buffer too large, discarding YAML")
                    self.accumulated_answ = ""


def main(args=None):
    rclpy.init(args=args)

    rajant_query_node = RajantQueryNode()

    try:
        rclpy.spin(rajant_query_node)
    except KeyboardInterrupt:
        pass
    finally:
        rajant_query_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
