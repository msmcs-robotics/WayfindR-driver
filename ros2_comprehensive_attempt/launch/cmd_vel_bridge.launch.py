#!/usr/bin/env python3
"""
ROS2 Launch File for cmd_vel Bridge

Launches the cmd_vel bridge node with configurable parameters.

Usage:
    # Default parameters
    ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py

    # Custom API URL
    ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py \
        api_url:=http://192.168.1.100:8000

    # Custom robot parameters
    ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py \
        wheelbase:=0.35 max_linear_velocity:=0.6

    # Use parameter file
    ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py \
        use_params_file:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for cmd_vel bridge."""

    # Declare launch arguments
    api_url_arg = DeclareLaunchArgument(
        'api_url',
        default_value='http://localhost:8000',
        description='PI_API endpoint URL'
    )

    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.30',
        description='Distance between left and right wheels (meters)'
    )

    max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )

    use_params_file_arg = DeclareLaunchArgument(
        'use_params_file',
        default_value='false',
        description='Whether to use parameter file instead of launch arguments'
    )

    # Get launch configurations
    api_url = LaunchConfiguration('api_url')
    wheelbase = LaunchConfiguration('wheelbase')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    use_params_file = LaunchConfiguration('use_params_file')

    # Parameters from launch arguments
    node_parameters = {
        'api_url': api_url,
        'wheelbase': wheelbase,
        'max_linear_velocity': max_linear_velocity,
        'max_angular_velocity': max_angular_velocity,
    }

    # cmd_vel bridge node
    cmd_vel_bridge_node = Node(
        package='cmd_vel_bridge',  # Replace with actual package name
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[node_parameters],
        emulate_tty=True,
    )

    return LaunchDescription([
        api_url_arg,
        wheelbase_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        use_params_file_arg,
        cmd_vel_bridge_node,
    ])
