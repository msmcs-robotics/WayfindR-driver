# WayfindR Unified Launch System Documentation

**Date:** 2026-01-11  
**Author:** WayfindR Development Team  
**Version:** 1.0  

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Launch Modes](#launch-modes)
4. [Quick Start Guide](#quick-start-guide)
5. [Helper Scripts](#helper-scripts)
6. [Launch Arguments Reference](#launch-arguments-reference)
7. [System Components](#system-components)
8. [Troubleshooting](#troubleshooting)
9. [Advanced Usage](#advanced-usage)

---

## Overview

The WayfindR Unified Launch System provides a single entry point for launching the complete ROS2 navigation stack. Instead of managing multiple launch files and complex configurations, you can now launch the entire system with a single command.

### Key Features

- **Single Launch File**: `bringup.launch.py` manages all components
- **Multiple Modes**: SLAM mapping, localization only, full navigation, simulation
- **Helper Scripts**: Easy-to-use bash scripts for common operations
- **Flexible Configuration**: Comprehensive launch arguments for customization
- **Lifecycle Management**: Automatic node lifecycle management
- **Safety Checks**: Built-in validation and error checking

### What's Included

- Robot state publisher (URDF/TF tree)
- RPLidar driver
- SLAM Toolbox OR AMCL localization
- Nav2 navigation stack (optional)
- cmd_vel bridge (optional)
- RViz visualization
- Lifecycle managers

---

## Architecture

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                     WayfindR Navigation System                      │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                        bringup.launch.py                            │
│                    (Unified Launch System)                          │
└─────────────────────────────────────────────────────────────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        │                           │                           │
        ▼                           ▼                           ▼
┌───────────────┐          ┌────────────────┐         ┌─────────────────┐
│  SLAM Mode    │          │ Localization   │         │   Navigation    │
│               │          │     Mode       │         │      Mode       │
└───────────────┘          └────────────────┘         └─────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                          Core Components                            │
│                        (Always Launched)                            │
├─────────────────────────────────────────────────────────────────────┤
│  • Robot State Publisher (URDF/TF)                                  │
│  • Joint State Publisher                                            │
│  • RPLidar Driver (disabled in sim mode)                            │
│  • Static TF Publishers (base_link → laser)                         │
│  • cmd_vel Bridge (optional, for PI_API)                            │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                      Mode-Specific Components                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  SLAM MODE:                                                         │
│    • SLAM Toolbox (async mapping)                                   │
│    • Creates map while robot moves                                  │
│                                                                     │
│  LOCALIZATION MODE:                                                 │
│    • Map Server (loads existing map)                                │
│    • AMCL (particle filter localization)                            │
│    • Lifecycle Manager (map_server, amcl)                           │
│                                                                     │
│  NAVIGATION MODE:                                                   │
│    • Map Server + AMCL (same as localization)                       │
│    • Controller Server (path following)                             │
│    • Planner Server (global path planning)                          │
│    • Behavior Server (recovery behaviors)                           │
│    • BT Navigator (behavior tree)                                   │
│    • Waypoint Follower                                              │
│    • Velocity Smoother                                              │
│    • Lifecycle Managers (localization + navigation)                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                        Visualization Layer                          │
├─────────────────────────────────────────────────────────────────────┤
│  • RViz2 (with appropriate configuration)                           │
│  • Real-time sensor data visualization                              │
│  • Interactive tools (2D Pose Estimate, Nav Goal)                   │
└─────────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
Hardware Layer:
  RPLidar C1M1 → /scan topic
  
Localization Layer:
  /scan + map → AMCL → /tf (map→odom→base_link→laser)
  
Planning Layer:
  Goal + map + /tf → Planner → /plan
  
Control Layer:
  /plan + /scan + /tf → Controller → /cmd_vel_nav
  
Velocity Processing:
  /cmd_vel_nav → Velocity Smoother → /cmd_vel
  
Hardware Interface (Optional):
  /cmd_vel → cmd_vel_bridge → PI_API → Motors
```

---

## Launch Modes

### Mode 1: SLAM Mapping

**Purpose:** Create new maps or update existing maps

**When to Use:**
- First time setting up in a new environment
- Environment has changed significantly
- Need to update or refine existing maps

**Components Launched:**
- Robot state publisher
- RPLidar driver
- SLAM Toolbox (async mapping)
- RViz (optional)

**Example:**
```bash
# Using helper script (recommended)
./scripts/start_mapping.sh

# Or direct launch
ros2 launch ros2_comprehensive_attempt bringup.launch.py mode:=slam
```

**After Mapping:**
```bash
# Save your map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_office_map
```

---

### Mode 2: Localization Only

**Purpose:** Track robot position without autonomous navigation

**When to Use:**
- Testing localization accuracy
- Developing custom navigation logic
- Lower resource usage than full navigation
- Position tracking for data collection

**Components Launched:**
- Robot state publisher
- RPLidar driver
- Map server (loads existing map)
- AMCL localization
- Lifecycle manager
- RViz (optional)

**Example:**
```bash
# Using helper script (recommended)
./scripts/start_localization.sh --map ~/maps/office.yaml

# Or direct launch
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=localization \
    map:=/home/user/maps/office.yaml
```

---

### Mode 3: Full Navigation

**Purpose:** Complete autonomous navigation system

**When to Use:**
- Production deployment
- Autonomous waypoint following
- Complete navigation missions
- Full Nav2 capabilities needed

**Components Launched:**
- All localization components (Mode 2)
- Nav2 controller server
- Nav2 planner server
- Behavior server (recovery behaviors)
- BT Navigator
- Waypoint follower
- Velocity smoother
- Additional lifecycle manager
- RViz (optional)

**Example:**
```bash
# Using helper script (recommended)
./scripts/start_navigation.sh --map ~/maps/office.yaml

# With cmd_vel bridge for PI_API
./scripts/start_navigation.sh \
    --map ~/maps/office.yaml \
    --with-bridge \
    --pi-api-url http://192.168.1.100:8000

# Or direct launch
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/office.yaml \
    use_cmd_vel_bridge:=true
```

---

### Mode 4: Simulation

**Purpose:** Test with rosbag playback or Gazebo

**When to Use:**
- Algorithm development without hardware
- Testing with recorded data
- Debugging navigation behavior
- Training and education

**Components Launched:**
- Same as other modes but with `use_sim_time:=true`
- RPLidar driver is disabled (expects /scan from sim)

**Example:**
```bash
# Using helper script (recommended)
./scripts/start_simulation.sh --mode navigation --map ~/maps/office.yaml

# In another terminal, play rosbag
ros2 bag play my_scan_data.bag --clock

# Or direct launch
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/office.yaml \
    use_sim_time:=true
```

---

## Quick Start Guide

### Prerequisites

1. ROS2 Humble installed
2. Nav2 packages installed
3. RPLidar ROS2 package installed
4. WayfindR URDF configured

### Step 1: Create a Map

```bash
cd /path/to/ros2_comprehensive_attempt
./scripts/start_mapping.sh
```

Drive the robot around to build the map, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_first_map
```

### Step 2: Test Localization

```bash
./scripts/start_localization.sh --map ~/maps/my_first_map.yaml
```

In RViz:
1. Click "2D Pose Estimate"
2. Click on the map where your robot is
3. Drag to set orientation
4. Watch particles converge

### Step 3: Run Full Navigation

```bash
./scripts/start_navigation.sh --map ~/maps/my_first_map.yaml
```

In RViz:
1. Set initial pose (as above)
2. Click "2D Nav Goal"
3. Click destination and drag for orientation
4. Watch robot navigate autonomously

---

## Helper Scripts

All helper scripts are located in `scripts/` directory.

### start_mapping.sh

Launch SLAM mapping mode.

**Usage:**
```bash
./scripts/start_mapping.sh [OPTIONS]
```

**Options:**
- `--no-rviz`: Don't launch RViz
- `--serial-port PORT`: Specify LiDAR port (default: /dev/ttyUSB0)
- `--help`: Show help message

**Examples:**
```bash
# Default - with RViz
./scripts/start_mapping.sh

# Without RViz (headless)
./scripts/start_mapping.sh --no-rviz

# Custom serial port
./scripts/start_mapping.sh --serial-port /dev/rplidar
```

---

### start_localization.sh

Launch localization-only mode.

**Usage:**
```bash
./scripts/start_localization.sh --map /path/to/map.yaml [OPTIONS]
```

**Required:**
- `--map PATH`: Path to map YAML file

**Options:**
- `--no-rviz`: Don't launch RViz
- `--serial-port PORT`: Specify LiDAR port (default: /dev/ttyUSB0)
- `--help`: Show help message

**Examples:**
```bash
# Basic usage
./scripts/start_localization.sh --map ~/maps/office.yaml

# Without RViz
./scripts/start_localization.sh --map ~/maps/office.yaml --no-rviz
```

---

### start_navigation.sh

Launch full navigation mode.

**Usage:**
```bash
./scripts/start_navigation.sh --map /path/to/map.yaml [OPTIONS]
```

**Required:**
- `--map PATH`: Path to map YAML file

**Options:**
- `--no-rviz`: Don't launch RViz
- `--serial-port PORT`: Specify LiDAR port (default: /dev/ttyUSB0)
- `--with-bridge`: Enable cmd_vel bridge for PI_API
- `--pi-api-url URL`: PI_API endpoint (default: http://localhost:8000)
- `--help`: Show help message

**Examples:**
```bash
# Basic navigation
./scripts/start_navigation.sh --map ~/maps/office.yaml

# With hardware integration
./scripts/start_navigation.sh \
    --map ~/maps/office.yaml \
    --with-bridge \
    --pi-api-url http://192.168.1.100:8000

# Headless with custom LiDAR port
./scripts/start_navigation.sh \
    --map ~/maps/office.yaml \
    --no-rviz \
    --serial-port /dev/rplidar
```

---

### start_simulation.sh

Launch simulation mode for testing.

**Usage:**
```bash
./scripts/start_simulation.sh --mode MODE [OPTIONS]
```

**Required:**
- `--mode MODE`: slam, localization, or navigation

**Options:**
- `--map PATH`: Path to map (required for localization/navigation)
- `--no-rviz`: Don't launch RViz
- `--help`: Show help message

**Examples:**
```bash
# SLAM in simulation
./scripts/start_simulation.sh --mode slam
# Then in another terminal:
ros2 bag play my_data.bag --clock

# Navigation in simulation
./scripts/start_simulation.sh --mode navigation --map ~/maps/office.yaml
ros2 bag play my_data.bag --clock
```

---

## Launch Arguments Reference

### bringup.launch.py

Complete reference for the main launch file.

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `mode` | string | navigation | Launch mode: slam, localization, or navigation |
| `map` | string | "" | Full path to map YAML file (required for localization/navigation) |
| `use_sim_time` | bool | false | Use simulation clock |
| `use_rviz` | bool | true | Launch RViz visualization |
| `use_cmd_vel_bridge` | bool | false | Launch cmd_vel bridge for PI_API |
| `serial_port` | string | /dev/ttyUSB0 | RPLidar serial port |
| `pi_api_url` | string | http://localhost:8000 | PI_API endpoint URL |
| `nav2_params` | string | (auto) | Path to Nav2 parameters file |
| `urdf_file` | string | (auto) | Path to robot URDF file |
| `rviz_config` | string | (auto) | Path to RViz config file |

### Usage Examples

```bash
# Minimal - full navigation
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/office.yaml

# Advanced - all options
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/office.yaml \
    use_sim_time:=false \
    use_rviz:=true \
    use_cmd_vel_bridge:=true \
    serial_port:=/dev/rplidar \
    pi_api_url:=http://192.168.1.100:8000 \
    nav2_params:=/custom/path/nav2_params.yaml

# SLAM mapping
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=slam \
    use_rviz:=true

# Simulation
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/office.yaml \
    use_sim_time:=true
```

---

## System Components

### Core Components

#### Robot State Publisher
- Publishes robot URDF description
- Manages TF tree (static transforms)
- Provides robot geometry to all nodes
- Always active

#### Joint State Publisher
- Publishes joint states for visualization
- Required for URDF visualization in RViz
- Always active

#### RPLidar Driver
- Interfaces with RP LIDAR C1M1
- Publishes laser scan data to /scan
- Configurable serial port
- Disabled in simulation mode

#### Static TF Publishers
- Publishes base_link → laser transform
- Defines sensor mounting position
- Always active

### SLAM Components

#### SLAM Toolbox
- Performs simultaneous localization and mapping
- Creates 2D occupancy grid maps
- Publishes map to /map topic
- Provides map → odom transform
- Configuration: `config/slam_params.yaml`

### Localization Components

#### Map Server
- Loads pre-built maps from YAML files
- Publishes map to /map topic
- Lifecycle-managed node
- Required for localization and navigation modes

#### AMCL (Adaptive Monte Carlo Localization)
- Particle filter-based localization
- Provides map → odom transform
- Uses laser scan and odometry
- Configuration: `config/amcl_params.yaml`

#### Lifecycle Manager (Localization)
- Manages map_server and amcl lifecycle
- Automatic startup
- Bond monitoring for fault detection
- Auto-restart on failure

### Navigation Components

#### Controller Server
- Local trajectory following
- DWB controller for differential drive
- Dynamic obstacle avoidance
- Publishes velocity commands to /cmd_vel_nav

#### Planner Server
- Global path planning
- Smac Planner 2D (cost-aware A*)
- Plans from current position to goal
- Publishes paths for visualization

#### Behavior Server
- Recovery behaviors (spin, backup, wait)
- Handles stuck situations
- Configurable behavior parameters

#### BT Navigator
- Behavior tree-based navigation logic
- Coordinates planning and control
- Handles navigation requests
- Manages recovery sequences

#### Waypoint Follower
- Sequential waypoint navigation
- Task execution at waypoints
- Mission-level navigation

#### Velocity Smoother
- Smooths velocity commands
- Enforces acceleration limits
- Connects /cmd_vel_nav → /cmd_vel
- Improves motion quality

#### Lifecycle Manager (Navigation)
- Manages all Nav2 components
- Automatic startup
- Bond monitoring
- Coordinated lifecycle management

### Optional Components

#### cmd_vel Bridge
- Converts ROS velocity commands to PI_API calls
- Enables hardware motor control
- Optional, only when use_cmd_vel_bridge:=true
- Configuration: `config/cmd_vel_bridge_params.yaml`

#### RViz
- 3D visualization tool
- Interactive navigation tools
- Real-time sensor data display
- Optional, controlled by use_rviz argument

---

## Troubleshooting

### Common Issues and Solutions

#### 1. LiDAR Not Found

**Symptom:**
```
Error: LiDAR device not found at /dev/ttyUSB0
```

**Solution:**
```bash
# List USB devices
ls -l /dev/ttyUSB*

# Check LiDAR permissions
sudo chmod 666 /dev/ttyUSB0

# Or add user to dialout group (permanent)
sudo usermod -a -G dialout $USER
# Then log out and back in
```

#### 2. Map File Not Found

**Symptom:**
```
Error: Map file not found: /path/to/map.yaml
```

**Solution:**
```bash
# Use absolute path
./scripts/start_navigation.sh --map /home/user/maps/office.yaml

# Or check available maps
ls -l ~/ros2_comprehensive_attempt/maps/

# Create maps directory if needed
mkdir -p ~/maps
```

#### 3. Nodes Not Starting

**Symptom:**
```
Lifecycle manager timeout waiting for nodes
```

**Solution:**
```bash
# Check if Nav2 is installed
ros2 pkg list | grep nav2

# Install missing packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Source workspace
source /opt/ros/humble/setup.bash
```

#### 4. TF Transform Errors

**Symptom:**
```
Could not transform from map to base_link
```

**Solution:**
- Set initial pose in RViz using "2D Pose Estimate"
- Check that AMCL or SLAM is running
- Verify map is loaded correctly
- Check /tf tree: `ros2 run tf2_tools view_frames`

#### 5. cmd_vel Bridge Connection Failed

**Symptom:**
```
Failed to connect to PI_API at http://localhost:8000
```

**Solution:**
```bash
# Check if PI_API is running
curl http://localhost:8000/health

# Use correct IP address
./scripts/start_navigation.sh \
    --map ~/maps/office.yaml \
    --with-bridge \
    --pi-api-url http://192.168.1.100:8000

# Check network connectivity
ping 192.168.1.100
```

#### 6. High CPU Usage

**Symptom:**
System is slow, high CPU usage

**Solution:**
- Reduce particle count in AMCL (edit config/amcl_params.yaml)
- Lower update frequencies in costmaps
- Disable RViz: `--no-rviz`
- Use localization mode instead of full navigation

#### 7. Robot Not Moving

**Symptom:**
Navigation goal sent, but robot doesn't move

**Solution:**
1. Check velocity commands: `ros2 topic echo /cmd_vel`
2. Verify cmd_vel_bridge is running (if using hardware)
3. Check controller parameters in nav2_params.yaml
4. Look for obstacles blocking path in RViz
5. Check behavior server for recovery mode

---

## Advanced Usage

### Custom Configuration Files

You can override default configuration files:

```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=~/maps/office.yaml \
    nav2_params:=~/my_custom_nav2_params.yaml
```

### Multiple Robot Support

For multiple robots, use namespaces:

```bash
# Robot 1
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=~/maps/office.yaml \
    --ros-args --ros-namespace robot1

# Robot 2  
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=~/maps/office.yaml \
    --ros-args --ros-namespace robot2
```

### Remote Monitoring

Run without RViz on robot, with RViz on remote computer:

**On Robot:**
```bash
./scripts/start_navigation.sh --map ~/maps/office.yaml --no-rviz
```

**On Remote Computer:**
```bash
# Set ROS_DOMAIN_ID to match robot
export ROS_DOMAIN_ID=0

# Launch only RViz
rviz2 -d /path/to/rviz_config.rviz
```

### Logging and Debugging

Enable debug logging for specific nodes:

```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=~/maps/office.yaml \
    --log-level amcl:=debug \
    --log-level controller_server:=debug
```

Save logs for analysis:

```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=~/maps/office.yaml \
    2>&1 | tee navigation_log.txt
```

### Performance Tuning

Edit `config/nav2_params.yaml` to tune performance:

**For Faster Navigation:**
- Increase max_vel_x (but be careful!)
- Increase controller_frequency
- Reduce costmap update frequencies

**For Better Obstacle Avoidance:**
- Increase vtheta_samples (more trajectory samples)
- Adjust inflation_radius in costmaps
- Tune critic weights in controller

**For Lower CPU Usage:**
- Reduce particle count in AMCL
- Lower costmap update frequencies
- Reduce vx_samples and vtheta_samples

---

## File Organization

```
ros2_comprehensive_attempt/
├── launch/
│   ├── bringup.launch.py          # Main unified launch file
│   ├── slam.launch.py              # Legacy SLAM launch
│   ├── localization.launch.py     # Legacy localization launch
│   ├── navigation.launch.py       # Legacy navigation launch
│   ├── robot_state_publisher.launch.py
│   └── cmd_vel_bridge.launch.py
├── config/
│   ├── nav2_params.yaml           # Nav2 configuration
│   ├── slam_params.yaml           # SLAM Toolbox configuration
│   ├── amcl_params.yaml           # AMCL configuration
│   ├── lidar_params.yaml          # LiDAR configuration
│   └── rviz_config.rviz           # RViz configuration
├── scripts/
│   ├── start_mapping.sh           # SLAM mapping helper
│   ├── start_localization.sh      # Localization helper
│   ├── start_navigation.sh        # Full navigation helper
│   ├── start_simulation.sh        # Simulation helper
│   ├── cmd_vel_bridge.py          # cmd_vel bridge node
│   ├── waypoint_manager.py        # Waypoint management
│   └── save_map.sh                # Map saving helper
├── urdf/
│   └── wayfinder_robot.urdf.xacro # Robot description
├── maps/
│   └── (your saved maps here)
└── findings/
    └── unified_launch_system.md   # This document
```

---

## Summary

The WayfindR Unified Launch System simplifies robot deployment by:

1. **Single Entry Point**: One launch file for all modes
2. **Helper Scripts**: Easy-to-use bash scripts
3. **Flexible Modes**: SLAM, localization, navigation, simulation
4. **Comprehensive Configuration**: All parameters accessible
5. **Safety Features**: Validation and error checking
6. **Documentation**: Complete usage guide

### Next Steps

1. Create your first map with `start_mapping.sh`
2. Test localization with `start_localization.sh`
3. Run autonomous navigation with `start_navigation.sh`
4. Integrate with PI_API using `--with-bridge`
5. Tune parameters in `config/nav2_params.yaml`

For questions or issues, refer to the troubleshooting section or consult the WayfindR development team.

---

**End of Documentation**
