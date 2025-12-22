#!/usr/bin/env python3
"""
Full Localization Launch File with LiDAR

Launches everything needed for localization:
- RPLidar driver
- AMCL localization
- Map server
- RViz visualization

Usage:
    ros2 launch localization_with_lidar.launch.py

    # With specific map
    ros2 launch localization_with_lidar.launch.py \
        map:=/home/devel/ros2_ws/maps/first_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get config directory
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.join(os.path.dirname(pkg_dir), 'config')
    amcl_config = os.path.join(config_dir, 'amcl_params.yaml')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/devel/ros2_ws/maps/first_map.yaml',
        description='Path to map yaml file'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='Serial port for RPLidar'
    )

    # RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',
        }],
        output='screen'
    )

    # Static transforms
    # odom -> base_link (static for testing, replace with real odometry)
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # base_link -> laser
    static_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map')
        }]
    )

    # AMCL localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config]
    )

    # Lifecycle manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        serial_port_arg,
        rplidar_node,
        static_odom_to_base,
        static_base_to_laser,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
    ])
