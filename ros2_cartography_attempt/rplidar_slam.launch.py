#!/usr/bin/env python3
"""
Launch file for RPLidar + SLAM Toolbox mapping with RViz visualization.
This is configured for the Slamtec C1 LiDAR.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('lidar_mapping')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='Serial port for the RPLidar'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # RPLidar node - configured for C1 (which uses rplidar_a2m12 or similar protocol)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 460800,  # C1 uses 460800 baud
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',  # Best mode for C1
        }],
        output='screen'
    )
    
    # Static transform from base_link to laser
    # Adjust these values based on your actual LiDAR mounting position
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )
    
    # Static transform from odom to base_link (for stationary testing)
    # In a real robot, this would come from wheel odometry
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # RViz2 for visualization
    rviz_config_file = os.path.join(pkg_dir, 'config', 'slam_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        rplidar_node,
        static_tf_base_to_laser,
        static_tf_odom_to_base,
        slam_toolbox_node,
        rviz_node,
    ])
