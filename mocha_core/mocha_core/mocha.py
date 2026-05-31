#!/usr/bin/env python3
import os
import random
import signal
import sys
import time
import pdb
import traceback
from functools import partial

import rclpy
from rclpy.logging import LoggingSeverity
import rclpy.logging
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import yaml
import std_msgs.msg
import subprocess

import mocha_core.database_server as ds
import mocha_core.database_utils as du
import mocha_core.synchronize_channel as sync
from mocha_core.zmq_comm_node import Transport


def ping(host):
    command = ["ping", "-c", "1", host]
    try:
        result = subprocess.run(command, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        return result.returncode == 0
    except Exception as e:
        print(f"Error pinging {host}: {e}")
        return False


class Mocha(Node):
    def __init__(self):
        super().__init__("mocha")
        self.logger = self.get_logger()
        # self.logger.set_level(LoggingSeverity.DEBUG)

        # Handle shutdown signal
        self.shutdownTriggered = threading.Event()
        self.shutdownTriggered.clear()

        def signal_handler(sig, frame):
            if not self.shutdownTriggered.is_set():
                self.logger.warning(f"{self.this_robot} - MOCHA Server - Got SIGINT. Triggering shutdown.")
                self.shutdown("Killed by user")
        signal.signal(signal.SIGINT, signal_handler)


        # Declare parameters
        self.declare_parameter("robot_name", "")
        self.declare_parameter("rssi_threshold", 20)
        self.declare_parameter("client_timeout", 6.0)
        self.declare_parameter("wifi_fallback_enabled", True)
        self.declare_parameter("wifi_backup_always_on", False)
        self.declare_parameter("wifi_backup_period", 10.0)
        self.declare_parameter("rajant_signal_timeout", 10.0)
        self.declare_parameter("robot_configs", "")
        self.declare_parameter("radio_configs", "")
        self.declare_parameter("topic_configs", "")

        self.this_robot = self.get_parameter("robot_name").get_parameter_value().string_value
        self.rssi_threshold = self.get_parameter("rssi_threshold").get_parameter_value().integer_value
        self.wifi_fallback_enabled = self.get_parameter(
            "wifi_fallback_enabled").get_parameter_value().bool_value
        self.wifi_backup_always_on = self.get_parameter(
            "wifi_backup_always_on").get_parameter_value().bool_value
        self.wifi_backup_period = self.get_parameter(
            "wifi_backup_period").get_parameter_value().double_value
        self.rajant_signal_timeout = self.get_parameter(
            "rajant_signal_timeout").get_parameter_value().double_value

        if len(self.this_robot) == 0:
            self.logger.error(f"{self.this_robot} - MOCHA Server - Empty robot name")
            raise ValueError("Empty robot name")

        self.logger.info(f"{self.this_robot} - MOCHA Server - " +
                        f"RSSI threshold: {self.rssi_threshold}")
        self.client_timeout = self.get_parameter("client_timeout").get_parameter_value().double_value
        self.logger.info(f"{self.this_robot} - MOCHA Server - " +
                        f"Client timeout: {self.client_timeout}")
        self.logger.info(f"{self.this_robot} - MOCHA Server - " +
                        f"WiFi fallback: {self.wifi_fallback_enabled}, " +
                        f"always-on WiFi backup: {self.wifi_backup_always_on}")

        # Load and check robot configs
        self.robot_configs_file = self.get_parameter("robot_configs").get_parameter_value().string_value
        try:
            with open(self.robot_configs_file, "r") as f:
                self.robot_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - MOCHA Server - robot_configs file")
            raise e
        if self.this_robot not in self.robot_configs.keys():
            self.logger.error(f"{self.this_robot} - MOCHA Server - robot_configs file")
            raise ValueError("Robot not in config file")

        # Load and check radio configs
        self.radio_configs_file = self.get_parameter("radio_configs").get_parameter_value().string_value
        try:
            with open(self.radio_configs_file, "r") as f:
                self.radio_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - MOCHA Server - radio_configs file")
            raise e
        self.radio = self.robot_configs[self.this_robot]["using-radio"]
        if self.radio not in self.radio_configs.keys():
            self.logger.error(f"{self.this_robot} - MOCHA Server - radio_configs file")
            raise ValueError("Radio {self.radio} not in config file")

        # Load and check topic configs
        self.topic_configs_file = self.get_parameter("topic_configs").get_parameter_value().string_value
        try:
            with open(self.topic_configs_file, "r") as f:
                self.topic_configs = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            self.logger.error(f"{self.this_robot} - MOCHA Server - topics_configs file")
            raise e
        self_type = self.robot_configs[self.this_robot]["node-type"]
        if self_type not in self.topic_configs.keys():
            self.logger.error(f"{self.this_robot} - MOCHA Server - topics_configs file")
            raise ValueError("Node type not in config file")

        # Check that we can ping the radios
        ip = self.robot_configs[self.this_robot]["IP-address"]
        if not ping(ip):
            wifi_ip = self.robot_configs[self.this_robot].get("wifi-IP-address")
            if self.wifi_fallback_enabled and wifi_ip:
                self.logger.warning(f"{self.this_robot} - MOCHA Server - " +
                                    f"Cannot ping Rajant self {ip}; " +
                                    f"continuing with WiFi fallback {wifi_ip}")
            else:
                self.logger.error(f"{self.this_robot} - MOCHA Server - " +
                                 f"Cannot ping self {ip}. Is the radio on?")
                raise ValueError("Cannot ping self")

        # Create database server
        self.DBServer = ds.DatabaseServer(self.robot_configs,
                                          self.topic_configs, self.this_robot, self)


        self.num_robot_in_comm = 0

        self.logger.info(f"{self.this_robot} - MOCHA Server - " +
                        "Created all communication channels!")

        # Start comm channels with other robots
        self.all_channels = []
        self.channels_by_robot = {}
        self.link_state = {}
        self.missing_wifi_warned = set()
        self.other_robots = [i for i in list(self.robot_configs.keys()) if i !=
                             self.this_robot]
        for other_robot in self.other_robots:
            if other_robot not in self.robot_configs[self.this_robot]["clients"]:
                self.logger.warning(
                    f"{self.this_robot} - MOCHA Server - "+
                    f"Skipping channel {self.this_robot}->{other_robot} " +
                    "as it is not in the graph of this robot"
                )
                continue
            # Start communication channel
            channel = sync.Channel(self.DBServer.dbl,
                                   self.this_robot,
                                   other_robot, self.robot_configs,
                                   self.client_timeout, self)
            channel.run()
            self.all_channels.append(channel)
            self.channels_by_robot[other_robot] = channel
            self.link_state[other_robot] = {
                "last_rssi": None,
                "last_rssi_time": None,
                "last_wifi_sync": 0.0,
            }

            # Attach a radio trigger to each channel. This will be triggered
            # when the RSSI is high enough. You can use another approach here
            # such as using a timer to periodically trigger the sync
            def make_callback(ch):
                return lambda msg: self.rssi_cb(msg, ch)

            self.create_subscription(
                std_msgs.msg.Int32,
                'mocha/rajant/rssi/' + other_robot,
                make_callback(channel),
                10
            )
        if self.wifi_fallback_enabled or self.wifi_backup_always_on:
            self.create_timer(1.0, self.wifi_backup_timer_cb)

    def shutdown(self, reason):
        # Only trigger shutdown once
        if self.shutdownTriggered.is_set():
            return
        self.shutdownTriggered.set()

        assert isinstance(reason, str)
        self.logger.error(f"{self.this_robot} - MOCHA Server - " + reason)
        # Shutting down communication channels
        if hasattr(self, 'all_channels') and len(self.all_channels) != 0:
            for channel in self.all_channels:
                channel.stop()
            self.logger.warning(f"{self.this_robot} - MOCHA Server - " + "Killed Channels")
            # Wait for all the channels to be gone. This needs to be slightly
            # larger than RECV_TIMEOUT
            time.sleep(3.5)
        self.logger.warning(f"{self.this_robot} - MOCHA Server - " + "Shutdown complete")

    def rssi_cb(self, data, comm_node):
        rssi = data.data
        state = self.link_state.get(comm_node.target_robot)
        if state is not None:
            state["last_rssi"] = rssi
            state["last_rssi_time"] = time.monotonic()
        if rssi > self.rssi_threshold:
            self.num_robot_in_comm += 1
            try:
                self.trigger_channel(comm_node, Transport.RAJANT,
                                     f"Rajant RSSI {rssi}")
            except:
                traceback.print_exception(*sys.exc_info())
        elif self.wifi_fallback_enabled:
            self.trigger_wifi_if_ready(comm_node, f"Rajant RSSI {rssi} below threshold")

    def trigger_channel(self, channel, transport, reason):
        self.logger.info(
            f"{self.this_robot} <- {channel.target_robot}: " +
            f"Triggering comms via {transport.value} ({reason})"
        )
        channel.trigger_sync(transport)

    def trigger_wifi_if_ready(self, channel, reason):
        target_config = self.robot_configs[channel.target_robot]
        if not target_config.get("wifi-IP-address"):
            if channel.target_robot not in self.missing_wifi_warned:
                self.missing_wifi_warned.add(channel.target_robot)
                self.logger.warning(
                    f"{self.this_robot} <- {channel.target_robot}: " +
                    "WiFi fallback requested, but wifi-IP-address is not configured"
                )
            return
        state = self.link_state[channel.target_robot]
        now = time.monotonic()
        if now - state["last_wifi_sync"] < self.wifi_backup_period:
            return
        state["last_wifi_sync"] = now
        self.trigger_channel(channel, Transport.WIFI, reason)

    def wifi_backup_timer_cb(self):
        now = time.monotonic()
        for target_robot, channel in self.channels_by_robot.items():
            state = self.link_state[target_robot]
            signal_stale = (
                state["last_rssi_time"] is None or
                now - state["last_rssi_time"] > self.rajant_signal_timeout
            )
            signal_low = (
                state["last_rssi"] is not None and
                state["last_rssi"] <= self.rssi_threshold
            )
            if self.wifi_backup_always_on:
                self.trigger_wifi_if_ready(channel, "always-on WiFi backup")
            elif self.wifi_fallback_enabled and (signal_stale or signal_low):
                self.trigger_wifi_if_ready(channel, "Rajant signal stale or weak")


def main(args=None):
    # Initialize ROS2 with command line arguments
    rclpy.init(args=args)

    # Start the node
    try:
        mocha = Mocha()
    except Exception as e:
        print(f"Node initialization failed: {e}")
        rclpy.shutdown()
        return

    # Load mtexecutor
    mtexecutor = MultiThreadedExecutor(num_threads=4)
    mtexecutor.add_node(mocha)

    # Use context manager for clean shutdown
    try:
        # Spin with periodic checking for shutdown
        while rclpy.ok() and not mocha.shutdownTriggered.is_set():
            mtexecutor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        mocha.shutdown("KeyboardInterrupt")
    except Exception as e:
        print(f"Exception: {e}")
        mocha.shutdown(f"Exception: {e}")

if __name__ == "__main__":
    main()
