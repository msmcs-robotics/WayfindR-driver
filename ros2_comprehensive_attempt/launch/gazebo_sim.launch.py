#!/usr/bin/env python3
"""
WayfindR Gazebo Simulation Launch File
========================================

This launch file starts a Gazebo Fortress simulation environment for the WayfindR robot.
It provides a complete simulation stack including:
- Gazebo Fortress simulator
- Robot spawning from URDF
- ROS-Gazebo bridge for sensor data and control
- Robot state publisher
- Optional RViz visualization

The URDF already includes Gazebo plugins for:
- Differential drive control (subscribes to /cmd_vel, publishes /odom)
- LIDAR sensor (publishes to /scan)
- Joint state publisher

USAGE:
------
# Launch with default empty world
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py

# Launch with custom world
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \\
    world:=/path/to/world.sdf

# Launch without RViz
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \\
    use_rviz:=false

# Launch with custom spawn position
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \\
    x_pose:=2.0 \\
    y_pose:=3.0 \\
    z_pose:=0.1

# Launch with test world
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \\
    world:=test_room

PARAMETERS:
-----------
world       : Path to world SDF file or world name (default: empty world)
              Use 'test_room' for the simple test environment
use_rviz    : Launch RViz visualization (default: true)
use_gui     : Launch Gazebo GUI (default: true)
x_pose      : Robot spawn X position (default: 0.0)
y_pose      : Robot spawn Y position (default: 0.0)
z_pose      : Robot spawn Z position (default: 0.1)
yaw_pose    : Robot spawn yaw orientation (default: 0.0)

NOTES:
------
- This uses Gazebo Fortress (modern Gazebo, formerly Ignition)
- The URDF contains all necessary Gazebo plugins
- Sensor data appears on standard ROS2 topics (/scan, /odom, etc.)
- Control via /cmd_vel topic (geometry_msgs/Twist)
- use_sim_time is automatically set to true for all nodes

Author: WayfindR Development Team
Date: 2026-01-11
ROS2 Version: Humble
Gazebo Version: Fortress
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
import xacro


def launch_setup(context, *args, **kwargs):
    """Setup function to resolve launch configurations."""

    # Get package directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_dir = os.path.join(pkg_dir, 'urdf')
    world_dir = os.path.join(pkg_dir, 'worlds')
    config_dir = os.path.join(pkg_dir, 'config')

    # Get launch configurations
    world_arg = LaunchConfiguration('world').perform(context)
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    # Determine world file path
    if world_arg == '' or world_arg == 'empty':
        # Use Gazebo's default empty world
        world_path = ''
    elif world_arg == 'test_room':
        # Use our test room world
        world_path = os.path.join(world_dir, 'test_room.sdf')
    elif world_arg == 'obstacle_course':
        # Use our obstacle course world
        world_path = os.path.join(world_dir, 'obstacle_course.sdf')
    elif os.path.isabs(world_arg):
        # Absolute path provided
        world_path = world_arg
    else:
        # Try to find in worlds directory
        world_path = os.path.join(world_dir, world_arg)
        if not world_path.endswith('.sdf'):
            world_path += '.sdf'

    # URDF file path
    urdf_file = os.path.join(urdf_dir, 'wayfinder_robot.urdf.xacro')

    # Process URDF with xacro
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toprettyxml(indent='  ')

    # RViz config
    rviz_config = os.path.join(config_dir, 'gazebo_sim.rviz')
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(config_dir, 'rviz_config.rviz')

    # Create nodes list
    nodes = []

    # ============================================================================
    # GAZEBO SIMULATION
    # ============================================================================

    # Start Gazebo server
    gazebo_server_args = ['-r', '-v', '4']
    if world_path:
        gazebo_server_args.extend([world_path])

    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim'] + gazebo_server_args,
        output='screen',
        shell=False
    )
    nodes.append(gazebo_server)

    # Start Gazebo client (GUI) - conditional
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v', '4'],
        output='screen',
        shell=False,
        condition=IfCondition(use_gui)
    )
    nodes.append(gazebo_client)

    # ============================================================================
    # ROBOT STATE PUBLISHER
    # ============================================================================

    # Robot state publisher - publishes TF tree from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )
    nodes.append(robot_state_publisher)

    # ============================================================================
    # SPAWN ROBOT IN GAZEBO
    # ============================================================================

    # Spawn robot entity
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wayfinder',
            '-topic', '/robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
        ],
        output='screen'
    )
    nodes.append(spawn_robot)

    # ============================================================================
    # ROS-GAZEBO BRIDGES
    # ============================================================================

    # Bridge for /scan topic (LIDAR)
    # Gazebo publishes on /scan, ROS2 subscribes on /scan
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    nodes.append(bridge_scan)

    # Bridge for /cmd_vel topic (velocity commands)
    # ROS2 publishes on /cmd_vel, Gazebo subscribes on /cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    nodes.append(bridge_cmd_vel)

    # Bridge for /odom topic (odometry)
    # Gazebo publishes on /odom, ROS2 subscribes on /odom
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    nodes.append(bridge_odom)

    # Bridge for /tf topic (transforms)
    # Bidirectional bridge for TF data
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    nodes.append(bridge_tf)

    # Bridge for /clock (simulation time)
    # Gazebo publishes clock, ROS2 nodes use it with use_sim_time=true
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    nodes.append(bridge_clock)

    # ============================================================================
    # VISUALIZATION
    # ============================================================================

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )
    nodes.append(rviz)

    return nodes


def generate_launch_description():
    """Generate launch description for Gazebo simulation."""

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to world SDF file or world name (empty, test_room, obstacle_course)'
    )

    declare_use_gui_cmd = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch Gazebo GUI (client)'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Robot spawn X position'
    )

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Robot spawn Y position'
    )

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Robot spawn Z position'
    )

    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose',
        default_value='0.0',
        description='Robot spawn yaw orientation'
    )

    # Set use_sim_time parameter globally
    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=True
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_gui_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_yaw_pose_cmd)

    # Set global parameters
    ld.add_action(set_use_sim_time)

    # Add opaque function for node creation
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


if __name__ == '__main__':
    generate_launch_description()
