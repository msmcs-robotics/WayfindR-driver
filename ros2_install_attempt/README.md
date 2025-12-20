# ROS2 Humble SLAM & Navigation Setup Guide

This directory contains everything you need to set up ROS2 Humble for indoor mapping, localization, and navigation on Ubuntu 22.04.

## Overview

This setup enables you to:
1. **Map indoor environments** using SLAM Toolbox with LiDAR
2. **Save and load maps** using nav2_map_server
3. **Add waypoints** to saved maps using RViz2 (on GUI systems)
4. **Navigate autonomously** to waypoints using Nav2 stack
5. **Localize** using LiDAR and IMU data

## Why SLAM Toolbox over Cartographer?

Based on 2025 research:
- **SLAM Toolbox is the officially supported ROS2 SLAM library**
- Cartographer ROS2 port is poorly maintained
- SLAM Toolbox achieves better accuracy (0.13m vs 0.21m ATE)
- Better performance in indoor environments
- Actively maintained and optimized for ROS2 Humble

## System Requirements

- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **Python**: 3.10 (comes with Ubuntu 22.04)
- **ROS2**: Humble Hawksbill
- **Hardware**:
  - LiDAR (RPLIDAR, Slamtec C1M1, YouYeeToo, etc.)
  - IMU (MPU6050 or similar) - optional but recommended
  - Raspberry Pi 4 or any x86_64/ARM64 system

## Installation Steps

### Step 1: Install ROS2 Humble Base

```bash
cd /home/devel/WayfindR-driver/ros2_install_attempt
sudo bash 01_install_ros2_humble.sh
```

This installs:
- ROS2 Humble Desktop
- Development tools (colcon, rosdep)
- Creates ~/ros2_ws workspace
- Configures ~/.bashrc to source ROS2

### Step 2: Install SLAM & Navigation Packages

```bash
sudo bash 02_install_slam_navigation.sh
```

This installs:
- **SLAM Toolbox** - mapping and localization
- **Nav2 (Navigation2)** - path planning and navigation
- **nav2_map_server** - save/load maps
- **robot_localization** - sensor fusion (IMU + odometry)
- **RViz2** - 3D visualization (headless compatible)
- **Gazebo** - simulation environment (optional)

### Step 3: Verify Installation

```bash
source ~/.bashrc
bash 03_verify_installation.sh
```

This checks:
- ROS2 environment is sourced
- All required packages are installed
- Workspace is built correctly

## WSL Specific Notes

Since you're running in WSL without GUI:
- **RViz2 is installed** but won't launch in WSL
- You can use RViz2 on another Ubuntu machine to:
  - View maps
  - Add waypoints graphically
  - Monitor robot navigation
- Alternative: Use X11 forwarding or remote desktop to access RViz2

## Workflow

### Phase 1: Create a Map

On your robot (Raspberry Pi):

```bash
# Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Drive robot around to map environment
# (Use teleop or your motor control scripts)
```

### Phase 2: Save the Map

```bash
# Save map to files
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_indoor_map

# This creates:
# - my_indoor_map.pgm (image file)
# - my_indoor_map.yaml (metadata)
```

### Phase 3: Add Waypoints

On a computer with GUI (not WSL):

1. Load the map in RViz2
2. Use "2D Pose Estimate" tool to mark waypoints
3. Record coordinates
4. Save to waypoints.yaml file

See [04_waypoint_workflow.md](04_waypoint_workflow.md) for details.

### Phase 4: Navigate to Waypoints

Load map and navigate:

```bash
# Launch localization with saved map
ros2 launch nav2_bringup localization_launch.py map:=$HOME/maps/my_indoor_map.yaml

# Launch navigation stack
ros2 launch nav2_bringup navigation_launch.py

# Send navigation goals (Python script or CLI)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}"
```

## Directory Structure

```
ros2_install_attempt/
├── README.md                          # This file
├── 01_install_ros2_humble.sh          # Base ROS2 installation
├── 02_install_slam_navigation.sh      # SLAM/Nav packages
├── 03_verify_installation.sh          # Installation checker
├── 04_waypoint_workflow.md            # Waypoint management guide
├── 05_map_saving_loading.md           # Map file format and usage
├── 06_navigation_python_examples.py   # Python scripts for navigation
└── config/                            # Configuration files
    ├── slam_params.yaml               # SLAM Toolbox parameters
    ├── nav2_params.yaml               # Navigation parameters
    └── localization_params.yaml       # Localization parameters
```

## Key Packages Installed

| Package | Purpose |
|---------|---------|
| `slam_toolbox` | SLAM mapping and localization |
| `navigation2` | Full navigation stack |
| `nav2_map_server` | Map loading/saving |
| `nav2_simple_commander` | Python API for navigation |
| `robot_localization` | Sensor fusion (EKF/UKF) |
| `rviz2` | 3D visualization |
| `imu_tools` | IMU processing |
| `ros2_control` | Hardware interface |

## Map File Locations

Default location for saved maps:
```
~/maps/
├── warehouse.pgm
├── warehouse.yaml
├── office.pgm
└── office.yaml
```

Waypoint files:
```
~/waypoints/
├── warehouse_waypoints.yaml
└── office_waypoints.yaml
```

## Topics You'll Use

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/scan` | sensor_msgs/LaserScan | LiDAR data |
| `/imu` | sensor_msgs/Imu | IMU data |
| `/odom` | nav_msgs/Odometry | Wheel odometry |
| `/map` | nav_msgs/OccupancyGrid | SLAM-generated map |
| `/cmd_vel` | geometry_msgs/Twist | Motor commands |
| `/goal_pose` | geometry_msgs/PoseStamped | Navigation goals |

## Next Steps After Installation

1. **Configure your LiDAR driver** to publish `/scan` topic
2. **Configure IMU** to publish `/imu` topic
3. **Set up differential drive controller** for `/cmd_vel`
4. **Create robot URDF** describing your robot's geometry
5. **Test SLAM** by driving robot around
6. **Save your first map**
7. **Test localization** by loading the map

## Troubleshooting

### ROS2 commands not found
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Permission denied on serial devices
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Can't launch RViz2 in WSL
- Use RViz2 on another Ubuntu machine
- Or set up X11 forwarding: https://wiki.ros.org/ROS/Tutorials/WorkingWithRVizAndWSL

## Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Robot & Chisel SLAM Tutorial](https://www.robotandchisel.com/2020/08/19/slam-in-ros2/)
- [SLAM Toolbox vs Cartographer Research Paper](https://www.mdpi.com/2079-9292/14/24/4822)

## Support

If you encounter issues:
1. Check [03_verify_installation.sh](03_verify_installation.sh) output
2. Verify LiDAR is publishing to `/scan`: `ros2 topic echo /scan`
3. Check TF tree: `ros2 run tf2_tools view_frames`
4. Review SLAM Toolbox logs: `ros2 launch slam_toolbox online_async_launch.py --debug`
