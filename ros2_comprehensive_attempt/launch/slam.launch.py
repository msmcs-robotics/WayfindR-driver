#!/usr/bin/env python3
"""
SLAM Mapping Launch File

Launches:
- RPLidar driver
- SLAM Toolbox (async mapping)
- Static TF publishers
- RViz visualization

Usage:
    ros2 launch slam.launch.py
    ros2 launch slam.launch.py serial_port:=/dev/rplidar
    ros2 launch slam.launch.py use_rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get config directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')

    # Declare arguments
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

    # Static TF: odom -> base_link (identity for stationary)
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

    # SLAM Toolbox
    slam_params_file = os.path.join(config_dir, 'slam_params.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file],
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
        serial_port_arg,
        use_rviz_arg,
        rplidar_node,
        static_tf_odom,
        static_tf_laser,
        slam_node,
        rviz_node,
    ])
