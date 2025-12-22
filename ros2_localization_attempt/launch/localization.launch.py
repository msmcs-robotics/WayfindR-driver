#!/usr/bin/env python3
"""
Localization Launch File

Launches AMCL (Adaptive Monte Carlo Localization) with a pre-built map
to determine the robot's position using LiDAR data.

Usage:
    ros2 launch localization.launch.py map:=/path/to/map.yaml

    # With custom initial pose
    ros2 launch localization.launch.py map:=/path/to/map.yaml \
        initial_pose_x:=1.0 initial_pose_y:=2.0 initial_pose_yaw:=1.57
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the directory of this launch file
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.join(os.path.dirname(pkg_dir), 'config')

    # Declare launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/devel/ros2_ws/maps/first_map.yaml',
        description='Full path to the map yaml file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial x position'
    )

    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial y position'
    )

    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial yaw orientation (radians)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes'
    )

    # Configuration file path
    amcl_config = os.path.join(config_dir, 'amcl_params.yaml')

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': LaunchConfiguration('map')}
        ]
    )

    # AMCL node for localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'initial_pose.x': LaunchConfiguration('initial_pose_x'),
                'initial_pose.y': LaunchConfiguration('initial_pose_y'),
                'initial_pose.yaw': LaunchConfiguration('initial_pose_yaw'),
            }
        ]
    )

    # Lifecycle manager to bring up nodes
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Static transform: odom -> base_link (for testing without odometry)
    # In a real robot, this would come from wheel encoders
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Static transform: base_link -> laser
    # Adjust these values based on your actual LiDAR mounting
    static_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )

    # RViz for visualization
    rviz_config = os.path.join(config_dir, 'localization_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    return LaunchDescription([
        # Launch arguments
        map_arg,
        use_sim_time_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        autostart_arg,

        # Static transforms
        static_odom_to_base,
        static_base_to_laser,

        # Localization nodes
        map_server_node,
        amcl_node,
        lifecycle_manager_node,

        # Visualization
        rviz_node,
    ])
