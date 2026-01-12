#!/usr/bin/env python3
"""
WayfindR Unified Bringup Launch File
=====================================

This is the main launch file for the WayfindR navigation system. It provides a unified
interface to launch the complete navigation stack with a single command.

MODES:
------
1. SLAM Mapping (mode=slam)
   - Creates new maps using SLAM Toolbox
   - Use for initial mapping or updating existing maps
   - Does NOT require a map file

2. Localization Only (mode=localization)
   - Uses AMCL for localization on an existing map
   - Does NOT include Nav2 planning/control
   - Requires a map file

3. Full Navigation (mode=navigation)
   - Complete autonomous navigation stack
   - Includes localization (AMCL), planning, control, and waypoint following
   - Requires a map file
   - Default mode

4. Simulation (use_sim_time=true)
   - For testing with rosbag playback or Gazebo
   - Can be combined with any mode

COMPONENTS:
-----------
- Robot State Publisher (URDF/TF tree)
- RPLidar Driver
- cmd_vel Bridge (optional, for PI_API integration)
- SLAM Toolbox OR AMCL Localization
- Nav2 Stack (optional, only in navigation mode)
- RViz Visualization
- Lifecycle Managers

USAGE:
------
# Full Navigation (default)
ros2 launch ros2_comprehensive_attempt bringup.launch.py \\
    mode:=navigation \\
    map:=/path/to/map.yaml

# SLAM Mapping
ros2 launch ros2_comprehensive_attempt bringup.launch.py \\
    mode:=slam

# Localization Only
ros2 launch ros2_comprehensive_attempt bringup.launch.py \\
    mode:=localization \\
    map:=/path/to/map.yaml

# Simulation Mode
ros2 launch ros2_comprehensive_attempt bringup.launch.py \\
    mode:=navigation \\
    map:=/path/to/map.yaml \\
    use_sim_time:=true

# Without RViz
ros2 launch ros2_comprehensive_attempt bringup.launch.py \\
    mode:=navigation \\
    map:=/path/to/map.yaml \\
    use_rviz:=false

# With cmd_vel bridge
ros2 launch ros2_comprehensive_attempt bringup.launch.py \\
    mode:=navigation \\
    map:=/path/to/map.yaml \\
    use_cmd_vel_bridge:=true \\
    pi_api_url:=http://192.168.1.100:8000

PARAMETERS:
-----------
mode              : slam|localization|navigation (default: navigation)
map               : Path to map YAML file (required for localization/navigation)
use_sim_time      : true|false (default: false)
use_rviz          : true|false (default: true)
use_cmd_vel_bridge: true|false (default: false)
pi_api_url        : PI_API endpoint URL (default: http://localhost:8000)
serial_port       : LiDAR serial port (default: /dev/ttyUSB0)
nav2_params       : Nav2 parameters file (default: nav2_params.yaml)

Author: WayfindR Development Team
Date: 2026-01-11
ROS2 Version: Humble
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with conditional components based on mode."""

    # Get package directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, 'config')
    launch_dir = os.path.join(pkg_dir, 'launch')
    urdf_dir = os.path.join(pkg_dir, 'urdf')

    # Default file paths
    default_urdf = os.path.join(urdf_dir, 'wayfinder_robot.urdf.xacro')
    default_nav2_params = os.path.join(config_dir, 'nav2_params.yaml')
    default_slam_params = os.path.join(config_dir, 'slam_params.yaml')
    default_amcl_params = os.path.join(config_dir, 'amcl_params.yaml')
    default_rviz_config = os.path.join(config_dir, 'rviz_config.rviz')

    # Declare launch arguments
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        description='Launch mode: slam, localization, or navigation',
        choices=['slam', 'localization', 'navigation']
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map YAML file (required for localization/navigation modes)'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    declare_use_cmd_vel_bridge_cmd = DeclareLaunchArgument(
        'use_cmd_vel_bridge',
        default_value='false',
        description='Launch cmd_vel bridge for PI_API integration'
    )

    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )

    declare_pi_api_url_cmd = DeclareLaunchArgument(
        'pi_api_url',
        default_value='http://localhost:8000',
        description='PI_API endpoint URL for cmd_vel bridge'
    )

    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params',
        default_value=default_nav2_params,
        description='Full path to Nav2 parameters file'
    )

    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf,
        description='Full path to robot URDF file'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to RViz config file'
    )

    # Get launch configurations
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_cmd_vel_bridge = LaunchConfiguration('use_cmd_vel_bridge')
    serial_port = LaunchConfiguration('serial_port')
    pi_api_url = LaunchConfiguration('pi_api_url')
    nav2_params = LaunchConfiguration('nav2_params')
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config = LaunchConfiguration('rviz_config')

    # Set environment variable for logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Set use_sim_time parameter globally
    set_use_sim_time = SetParameter(
        name='use_sim_time',
        value=use_sim_time
    )

    # =============================================================================
    # CORE COMPONENTS (Always launched)
    # =============================================================================

    # Robot State Publisher - Publishes URDF and TF tree
    robot_description = Command(['xacro ', urdf_file])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 30,
        }]
    )

    # RPLidar Driver (disabled in sim mode)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'scan_mode': 'DenseBoost',
        }],
        output='screen',
        condition=UnlessCondition(use_sim_time)
    )

    # Static TF: base_link -> laser
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # cmd_vel Bridge (optional)
    cmd_vel_bridge_node = Node(
        package='cmd_vel_bridge',
        executable='cmd_vel_bridge.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'api_url': pi_api_url,
            'wheelbase': 0.30,
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
        }],
        emulate_tty=True,
        condition=IfCondition(use_cmd_vel_bridge)
    )

    # =============================================================================
    # SLAM MODE COMPONENTS (mode=slam)
    # =============================================================================

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            default_slam_params,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'slam'"])
        )
    )

    # =============================================================================
    # LOCALIZATION COMPONENTS (mode=localization or navigation)
    # =============================================================================

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization' or '", mode, "' == 'navigation'"])
        )
    )

    # AMCL Localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            default_amcl_params,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization' or '", mode, "' == 'navigation'"])
        )
    )

    # Lifecycle Manager for Localization
    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization' or '", mode, "' == 'navigation'"])
        )
    )

    # =============================================================================
    # NAVIGATION COMPONENTS (mode=navigation only)
    # =============================================================================

    # Controller Server (path following)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav')
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        )
    )

    # Planner Server (global path planning)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        )
    )

    # Behavior Server (recovery behaviors)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        )
    )

    # BT Navigator (behavior tree navigation)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        )
    )

    # Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        )
    )

    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        )
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ],
            'bond_timeout': 4.0,
        }],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        )
    )

    # =============================================================================
    # VISUALIZATION
    # =============================================================================

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # =============================================================================
    # BUILD LAUNCH DESCRIPTION
    # =============================================================================

    ld = LaunchDescription()

    # Environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare arguments
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_cmd_vel_bridge_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_pi_api_url_cmd)
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_urdf_file_cmd)
    ld.add_action(declare_rviz_config_cmd)

    # Set global parameters
    ld.add_action(set_use_sim_time)

    # Core components (always launched)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rplidar_node)
    ld.add_action(static_tf_laser)
    ld.add_action(cmd_vel_bridge_node)

    # SLAM mode components
    ld.add_action(slam_node)

    # Localization components
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_localization_node)

    # Navigation components
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(waypoint_follower_node)
    ld.add_action(velocity_smoother_node)
    ld.add_action(lifecycle_manager_navigation_node)

    # Visualization
    ld.add_action(rviz_node)

    return ld


if __name__ == '__main__':
    generate_launch_description()
