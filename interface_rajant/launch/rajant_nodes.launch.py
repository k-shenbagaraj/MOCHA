#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    from ament_index_python.packages import get_package_share_directory
    robot_name = LaunchConfiguration('robot_name')
    robot_configs = LaunchConfiguration('robot_configs')
    radio_configs = LaunchConfiguration('radio_configs')
    bcapi_jar_file = LaunchConfiguration('bcapi_jar_file')
    wifi_backup_period = LaunchConfiguration('wifi_backup_period')
    
    # Evaluate the path of the jar file
    jar_file_path = bcapi_jar_file.perform(context)
    
    if not os.path.exists(jar_file_path):
        # Search the thirdParty directory
        third_party_dir = os.path.join(
            get_package_share_directory('interface_rajant'),
            'thirdParty'
        )
        
        found_jar = None
        for root, dirs, files in os.walk(third_party_dir):
            for file in files:
                if file.endswith('.jar'):
                    if 'PeerRSSI-bcapi' in file:
                        found_jar = os.path.join(root, file)
                        break
                    elif 'bcapi-watchstate' in file and not found_jar:
                        found_jar = os.path.join(root, file)
            if found_jar and 'PeerRSSI-bcapi' in found_jar:
                break
                
        if found_jar:
            jar_file_path = found_jar
            # We must override the launch configuration so nodes receive the actual found path!
            bcapi_jar_file = found_jar

    jar_file_name = os.path.basename(jar_file_path)
    
    nodes_to_start = []
    
    if 'PeerRSSI-bcapi' in jar_file_name:
        rajant_peer_rssi = Node(
            package='interface_rajant',
            executable='rajant_peer_rssi.py',
            name='rajant_peer_rssi',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'robot_configs': robot_configs,
                'radio_configs': radio_configs,
                'bcapi_jar_file': bcapi_jar_file
            }]
        )
        nodes_to_start.append(rajant_peer_rssi)
    elif 'bcapi-watchstate' in jar_file_name:
        rajant_query_node = Node(
            package='interface_rajant',
            executable='rajant_query.py',
            name='rajant_query',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'robot_configs': robot_configs,
                'radio_configs': radio_configs,
                'bcapi_jar_file': bcapi_jar_file,
                'wifi_backup_period': wifi_backup_period
            }]
        )

        rajant_parser_node = Node(
            package='interface_rajant',
            executable='rajant_parser.py',
            name='rajant_parser',
            output='screen',
            parameters=[{
                'robot_name': robot_name,
                'robot_configs': robot_configs,
                'radio_configs': radio_configs
            }]
        )
        nodes_to_start.append(rajant_query_node)
        nodes_to_start.append(rajant_parser_node)
        
    return nodes_to_start


def generate_launch_description():
    """Launch Rajant interface nodes dynamically."""
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='charon',
        description='Name of the robot'
    )

    robot_configs_arg = DeclareLaunchArgument(
        'robot_configs',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocha_core'),
            'config', 'robot_configs.yaml'
        ]),
        description='Path to robot configuration file'
    )

    radio_configs_arg = DeclareLaunchArgument(
        'radio_configs',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocha_core'),
            'config', 'radio_configs.yaml'
        ]),
        description='Path to radio configuration file'
    )

    bcapi_jar_file_arg = DeclareLaunchArgument(
        'bcapi_jar_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('interface_rajant'),
            'thirdParty', 'PeerRSSI-bcapi-11.26.1.jar'
        ]),
        description='Path to jar file used to get rssi'
    )
    
    wifi_backup_period_arg = DeclareLaunchArgument(
        'wifi_backup_period',
        default_value='10.0',
        description='Minimum seconds between WiFi backup sync attempts per peer'
    )

    return LaunchDescription([
        robot_name_arg,
        robot_configs_arg,
        radio_configs_arg,
        bcapi_jar_file_arg,
        wifi_backup_period_arg,
        OpaqueFunction(function=launch_setup)
    ])
