#!/usr/bin/env python3
"""
Robot State Publisher Launch File for WayfindR Robot

This launch file publishes the robot's URDF description and transforms (TF tree).
It is used by all navigation and visualization components to understand the robot's
physical structure and coordinate frames.

Usage:
    ros2 launch robot_state_publisher.launch.py

    Optional with RViz:
    ros2 launch robot_state_publisher.launch.py use_rviz:=true

Author: WayfindR Development Team
Date: 2026-01-11
ROS2 Version: Humble
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for robot state publisher.

    This function sets up:
    1. Robot state publisher node (publishes URDF and static transforms)
    2. Joint state publisher node (publishes joint states for visualization)
    3. Optional RViz visualization

    Returns:
        LaunchDescription: Complete launch description
    """

    # Get the path to this launch file's directory
    # Since this is a standalone folder, we use absolute paths
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_file = os.path.join(pkg_dir, 'urdf', 'wayfinder_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'robot_description.rviz')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for robot visualization'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Process the URDF/xacro file
    # Command substitution processes xacro macros into URDF
    robot_description = Command(['xacro ', urdf_file])

    # Robot State Publisher Node
    # Publishes robot_description to /robot_description topic
    # Publishes static transforms from URDF to /tf_static
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 30.0,  # Hz - how often to publish transforms
        }],
        remappings=[
            ('/joint_states', '/joint_states'),
        ]
    )

    # Joint State Publisher Node
    # Publishes fake joint states for visualization purposes
    # In real operation, this would be replaced by actual joint state readings
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 30,  # Hz - publishing rate
        }]
    )

    # RViz Node (optional)
    # Launches RViz with robot model visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_rviz_arg)

    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)

    return ld


if __name__ == '__main__':
    generate_launch_description()
