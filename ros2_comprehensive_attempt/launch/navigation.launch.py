#!/usr/bin/env python3
"""
Nav2 Navigation Launch File
For WayfindR Differential Drive Robot with RP LIDAR C1M1

This launch file starts:
- Map Server (loads a pre-built map)
- AMCL (Adaptive Monte Carlo Localization)
- Nav2 Navigation Stack (planner, controller, behavior server, etc.)
- RViz2 with Nav2 visualization

Usage:
    ros2 launch ros2_comprehensive_attempt navigation.launch.py map:=<path_to_map.yaml>

Parameters:
    map: Path to the map YAML file (required)
    use_sim_time: Use simulation time (default: false)
    params_file: Path to Nav2 parameters file (default: nav2_params.yaml in config folder)
    rviz_config: Path to RViz config file (default: rviz_nav2.rviz in config folder)
    use_rviz: Launch RViz (default: true)
    autostart: Automatically startup the Nav2 stack (default: true)

Author: Claude Code
Date: 2026-01-11
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory of this package
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Default paths
    default_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'config', 'rviz_nav2.rviz')

    # Launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    # Declare launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map YAML file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file to use'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Set environment variable for logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Map Server Node
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    # AMCL (Localization) Node
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Controller Server (path following)
    controller_server_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav')
        ]
    )

    # Planner Server (global path planning)
    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Behavior Server (recovery behaviors)
    behavior_server_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # BT Navigator (behavior tree navigation)
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Waypoint Follower
    waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Velocity Smoother
    velocity_smoother_cmd = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ]
    )

    # Lifecycle Manager for Localization (Map Server + AMCL)
    lifecycle_manager_localization_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )

    # RViz2
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add the nodes to the launch description
    ld.add_action(map_server_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(controller_server_cmd)
    ld.add_action(planner_server_cmd)
    ld.add_action(behavior_server_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(waypoint_follower_cmd)
    ld.add_action(velocity_smoother_cmd)
    ld.add_action(lifecycle_manager_localization_cmd)
    ld.add_action(lifecycle_manager_navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld
