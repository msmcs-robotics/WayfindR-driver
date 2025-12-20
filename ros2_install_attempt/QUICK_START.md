# Quick Start Guide

**Get ROS2 Humble with SLAM and Navigation running in under 30 minutes**

## TL;DR - Three Commands to Install

```bash
cd /home/devel/WayfindR-driver/ros2_install_attempt

# 1. Install ROS2 Humble (10-15 minutes)
sudo bash 01_install_ros2_humble.sh

# 2. Install SLAM & Navigation (10-15 minutes)
sudo bash 02_install_slam_navigation.sh

# 3. Verify everything works
bash 03_verify_installation.sh
```

**After installation, open a new terminal or run:**
```bash
source ~/.bashrc
```

## What Gets Installed

### Core ROS2
- ROS2 Humble Desktop (with RViz2, RQt)
- Development tools (colcon, rosdep)
- ~/ros2_ws workspace

### SLAM & Mapping
- **slam_toolbox** - Create maps with LiDAR
- **nav2_map_server** - Save and load maps

### Navigation
- **navigation2** - Full Nav2 stack for autonomous navigation
- **nav2_simple_commander** - Python API for waypoint navigation
- **robot_localization** - Fuse IMU + odometry data

### Sensors & Hardware
- **imu_tools** - IMU data processing
- **ros2_control** - Hardware interface framework

## Tested On

- Ubuntu 22.04 LTS (Jammy)
- Python 3.10
- x86_64, ARM64 (Raspberry Pi 4)
- WSL2 (headless mode)

## What You Need

### Required
- Ubuntu 22.04
- LiDAR sensor (RPLIDAR, Slamtec, YouYeeToo, etc.)
- Robot base that can respond to `/cmd_vel` commands

### Recommended
- IMU (MPU6050, BNO055, etc.) for better localization
- Wheel encoders for odometry

### Optional
- Another Ubuntu machine with GUI for RViz2 (not needed for WSL)

## First Test After Installation

### Test 1: Verify ROS2 Works

```bash
# In terminal 1
ros2 run demo_nodes_cpp talker

# In terminal 2
ros2 run demo_nodes_py listener
```

You should see messages being sent and received.

### Test 2: Check SLAM Toolbox

```bash
ros2 pkg list | grep slam_toolbox
# Should output: slam_toolbox

ros2 launch slam_toolbox online_async_launch.py --help
# Should show launch file options
```

### Test 3: Check Navigation2

```bash
ros2 pkg list | grep nav2
# Should show multiple nav2 packages

ros2 launch nav2_bringup navigation_launch.py --help
# Should show launch file options
```

## Your First Map (5-Minute Tutorial)

Once you have your LiDAR connected:

### Step 1: Start SLAM

```bash
# Terminal 1: SLAM
ros2 launch slam_toolbox online_async_launch.py
```

### Step 2: Start Your LiDAR

```bash
# Terminal 2: LiDAR driver (example for RPLIDAR)
ros2 run rplidar_ros rplidar_composition
```

### Step 3: Drive Around

```bash
# Terminal 3: Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive your robot around the room for a few minutes.

### Step 4: Save the Map

```bash
# Terminal 4: Save
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_first_map
```

**Done!** You now have:
- `~/maps/my_first_map.pgm` (map image)
- `~/maps/my_first_map.yaml` (map metadata)

## Your First Autonomous Navigation

### Step 1: Load Map and Start Navigation

```bash
# Terminal 1: Localization with your map
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/maps/my_first_map.yaml

# Terminal 2: Navigation stack
ros2 launch nav2_bringup navigation_launch.py
```

### Step 2: Set Initial Position

```bash
# Tell robot where it is (at origin)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'},
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                  orientation: {w: 1.0}}}}"
```

### Step 3: Send a Goal

```bash
# Navigate to x=2.0, y=1.0
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'},
           pose: {position: {x: 2.0, y: 1.0, z: 0.0},
                  orientation: {w: 1.0}}}}"
```

Watch your robot drive autonomously to the goal!

## Common Issues

### "ros2: command not found"
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# Or just: source ~/.bashrc
```

### "Permission denied" on serial port
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### "No node named /slam_toolbox"
```bash
# Check if SLAM is running
ros2 node list | grep slam

# Check topics
ros2 topic list | grep scan
```

### Can't see RViz2 in WSL
- RViz2 doesn't work in headless WSL
- Use RViz2 on another Ubuntu machine to visualize
- Or set up X11 forwarding / remote desktop

## Next Steps

Once basics are working:

1. **Read the full guides**:
   - [README.md](README.md) - Complete overview
   - [07_complete_workflow.md](07_complete_workflow.md) - Detailed workflow
   - [04_waypoint_workflow.md](04_waypoint_workflow.md) - Waypoint management
   - [05_map_saving_loading.md](05_map_saving_loading.md) - Map operations

2. **Configure for your robot**:
   - Edit [config/slam_params.yaml](config/slam_params.yaml)
   - Edit [config/nav2_params.yaml](config/nav2_params.yaml)
   - Adjust robot dimensions, speeds, sensor ranges

3. **Create waypoint patrol routes**:
   - Use Python examples in [06_navigation_python_examples.py](06_navigation_python_examples.py)
   - Set up autonomous patrol routes

4. **Deploy to Raspberry Pi**:
   - Follow deployment section in [07_complete_workflow.md](07_complete_workflow.md)
   - Set up auto-start services
   - Integrate with fleet management

## Getting Help

If you get stuck:

1. Check the verification script output:
   ```bash
   bash 03_verify_installation.sh
   ```

2. Review the troubleshooting section in [07_complete_workflow.md](07_complete_workflow.md#troubleshooting)

3. Check ROS2 documentation: https://docs.ros.org/en/humble/

4. Review your existing project docs: [../docs/](../docs/)

## Why SLAM Toolbox?

According to 2025 research:
- **Cartographer is poorly maintained for ROS2**
- SLAM Toolbox is the official ROS2 SLAM library
- Better accuracy (0.13m vs 0.21m error)
- Actively developed and optimized for ROS2 Humble
- Better for indoor environments

Your existing installation scripts already use SLAM Toolbox - you made the right choice!

## File Overview

```
ros2_install_attempt/
├── QUICK_START.md                     ← You are here
├── README.md                          ← Full documentation
├── 01_install_ros2_humble.sh          ← Base ROS2
├── 02_install_slam_navigation.sh      ← SLAM/Nav packages
├── 03_verify_installation.sh          ← Check everything works
├── 04_waypoint_workflow.md            ← How to use waypoints
├── 05_map_saving_loading.md           ← Map operations
├── 06_navigation_python_examples.py   ← Python code examples
├── 07_complete_workflow.md            ← End-to-end guide
└── config/
    ├── slam_params.yaml               ← SLAM configuration
    ├── nav2_params.yaml               ← Navigation config
    └── robot_localization_params.yaml ← Sensor fusion config
```

**Start with this file, then read [README.md](README.md) for the big picture.**
