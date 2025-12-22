#!/usr/bin/env python3
"""
Localization Launch File

Launches:
- RPLidar driver
- Map server
- AMCL localization
- Static TF publishers
- Lifecycle manager
- RViz visualization

Usage:
    ros2 launch localization.launch.py map:=/path/to/map.yaml
    ros2 launch localization.launch.py map:=/path/to/map.yaml use_rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get config directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')
    amcl_params = os.path.join(config_dir, 'amcl_params.yaml')

    # Declare arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map YAML file'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LiDAR'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
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
            'scan_mode': 'DenseBoost',
        }],
        output='screen',
    )

    # Static TF: odom -> base_link
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )

    # Static TF: base_link -> laser
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
    )

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False,
        }],
        output='screen',
    )

    # AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[amcl_params],
        output='screen',
    )

    # Lifecycle manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
        output='screen',
    )

    # RViz
    rviz_config = os.path.join(config_dir, 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        map_arg,
        serial_port_arg,
        use_rviz_arg,
        rplidar_node,
        static_tf_odom,
        static_tf_laser,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
    ])
