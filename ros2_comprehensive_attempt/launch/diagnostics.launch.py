#!/usr/bin/env python3
"""
Diagnostics Launch File

Launches the comprehensive diagnostics system for WayfindR.

Usage:
    ros2 launch <package> diagnostics.launch.py
    ros2 launch <package> diagnostics.launch.py enable_dashboard:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    enable_dashboard_arg = DeclareLaunchArgument(
        'enable_dashboard',
        default_value='false',
        description='Enable the monitoring dashboard'
    )

    # System diagnostics node
    diagnostics_node = Node(
        package='your_package_name',  # Replace with actual package name
        executable='system_diagnostics.py',
        name='system_diagnostics',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # Monitoring dashboard (optional)
    dashboard_node = Node(
        package='your_package_name',  # Replace with actual package name
        executable='monitoring_dashboard.py',
        name='monitoring_dashboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_dashboard')),
        parameters=[{
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        enable_dashboard_arg,
        diagnostics_node,
        dashboard_node,
    ])
