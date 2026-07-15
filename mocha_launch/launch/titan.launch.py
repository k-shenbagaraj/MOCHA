#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch titan robot with database, translators, publishers, and Rajant interface."""
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='titan',
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

    topic_configs_arg = DeclareLaunchArgument(
        'topic_configs',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocha_core'),
            'config', 'topic_configs.yaml'
        ]),
        description='Path to topic configuration file'
    )

    radio_configs_arg = DeclareLaunchArgument(
        'radio_configs',
        default_value=PathJoinSubstitution([
            FindPackageShare('mocha_core'),
            'config', 'radio_configs.yaml'
        ]),
        description='Path to radio configuration file'
    )
    wifi_backup_always_on_arg = DeclareLaunchArgument(
        'wifi_backup_always_on',
        default_value='false',
        description='Periodically sync over WiFi even when Rajant is healthy'
    )
    wifi_fallback_enabled_arg = DeclareLaunchArgument(
        'wifi_fallback_enabled',
        default_value='true',
        description='Use WiFi when Rajant RSSI is weak or missing'
    )
    wifi_backup_period_arg = DeclareLaunchArgument(
        'wifi_backup_period',
        default_value='10.0',
        description='Minimum seconds between WiFi backup sync attempts per peer'
    )
    rajant_signal_timeout_arg = DeclareLaunchArgument(
        'rajant_signal_timeout',
        default_value='10.0',
        description='Seconds without RSSI before Rajant is considered stale'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    robot_configs = LaunchConfiguration('robot_configs')
    topic_configs = LaunchConfiguration('topic_configs')
    radio_configs = LaunchConfiguration('radio_configs')
    wifi_fallback_enabled = LaunchConfiguration('wifi_fallback_enabled')
    wifi_backup_always_on = LaunchConfiguration('wifi_backup_always_on')
    wifi_backup_period = LaunchConfiguration('wifi_backup_period')
    rajant_signal_timeout = LaunchConfiguration('rajant_signal_timeout')

    # Include database, translators and publishers launch file
    database_translators_publishers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mocha_core'),
                'launch',
                'database_translators_publishers.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'topic_configs': topic_configs,
            'radio_configs': radio_configs,
            'wifi_fallback_enabled': wifi_fallback_enabled,
            'wifi_backup_period': wifi_backup_period,
            'rajant_signal_timeout': rajant_signal_timeout,
            'wifi_backup_always_on': wifi_backup_always_on
        }.items()
    )

    # Include Rajant interface launch file
    rajant_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interface_rajant'),
                'launch',
                'rajant_nodes.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'robot_configs': robot_configs,
            'topic_configs': topic_configs,
            'radio_configs': radio_configs,
            'wifi_backup_period': wifi_backup_period
        }.items()
    )

    return LaunchDescription([
        robot_name_arg,
        robot_configs_arg,
        topic_configs_arg,
        radio_configs_arg,
        wifi_fallback_enabled_arg,
        wifi_backup_always_on_arg,
        wifi_backup_period_arg,
        rajant_signal_timeout_arg,
        database_translators_publishers_launch,
        rajant_interface_launch
    ])
