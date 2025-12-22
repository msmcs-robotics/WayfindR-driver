# ROS2 Humble Installation Guide

## Overview

This document provides a complete guide to installing ROS2 Humble with SLAM and navigation capabilities on Ubuntu 22.04.

**Target OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
**ROS2 Version:** Humble Hawksbill
**Python Version:** 3.10

---

## Prerequisites

### System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| RAM | 4 GB | 8+ GB |
| Storage | 10 GB free | 20+ GB free |
| CPU | Dual-core | Quad-core |
| GPU | None required | For RViz performance |

### Ubuntu Version Check

```bash
lsb_release -a
# Should show: Ubuntu 22.04.x LTS
```

---

## Installation Steps

### Step 1: Set Locale

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Setup Sources

```bash
# Install required tools
sudo apt install software-properties-common curl -y

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS2 Humble

```bash
sudo apt update
sudo apt upgrade -y

# Desktop install (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop -y

# Or base install only (headless, for robots)
# sudo apt install ros-humble-ros-base -y
```

### Step 4: Install Development Tools

```bash
sudo apt install ros-dev-tools python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Step 5: Setup Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## SLAM & Navigation Packages

### Install SLAM Toolbox

```bash
sudo apt install ros-humble-slam-toolbox -y
```

### Install Navigation2 (Nav2)

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
```

### Install Additional Useful Packages

```bash
sudo apt install \
    ros-humble-tf2-tools \
    ros-humble-robot-localization \
    ros-humble-rviz2 \
    ros-humble-rqt-* \
    -y
```

### Install LiDAR Drivers

```bash
# RPLidar (Slamtec)
sudo apt install ros-humble-rplidar-ros -y

# Velodyne
# sudo apt install ros-humble-velodyne -y

# SICK
# sudo apt install ros-humble-sick-scan2 -y
```

---

## Verify Installation

### Check ROS2 Version

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Check installation
ros2 doctor --report
```

### Test Publisher/Subscriber

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_py listener
```

### Check Installed Packages

```bash
# List all packages
ros2 pkg list | wc -l

# Check specific packages
ros2 pkg list | grep -E "slam|nav2|rplidar"
```

---

## Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash

# Add to ~/.bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## USB/Serial Permissions

### For LiDAR Access

```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Create udev rule for persistent naming
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes
```

---

## Common Issues & Solutions

### Issue: "ros2: command not found"

```bash
source /opt/ros/humble/setup.bash
```

### Issue: "Package not found"

```bash
sudo apt update
sudo apt install ros-humble-<package-name>
```

### Issue: "Permission denied" on serial port

```bash
sudo chmod 666 /dev/ttyUSB0
# Or add to dialout group
```

### Issue: "Cannot locate rosdep definition"

```bash
rosdep update
```

### Issue: colcon build fails with missing dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

---

## Complete Package List

### Core ROS2 (ros-humble-desktop)

- rclcpp, rclpy (C++/Python client libraries)
- rviz2 (visualization)
- rqt-* (GUI tools)
- demo_nodes_* (examples)
- tf2_* (transforms)

### SLAM & Navigation

```bash
sudo apt install \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    -y
```

### LiDAR Drivers

```bash
sudo apt install \
    ros-humble-rplidar-ros \
    ros-humble-laser-filters \
    -y
```

---

## Raspberry Pi Notes

For deploying on Raspberry Pi 4:

1. Use Ubuntu 22.04 Server (not Raspbian)
2. Install `ros-humble-ros-base` (no desktop)
3. Run RViz on a separate machine
4. Consider CPU/memory constraints

```bash
# Headless install for Pi
sudo apt install ros-humble-ros-base ros-humble-slam-toolbox ros-humble-navigation2 -y
```

---

## Quick Reference

### Environment Setup

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Useful Commands

```bash
ros2 topic list          # List all topics
ros2 topic echo /scan    # Echo topic data
ros2 topic hz /scan      # Topic publish rate
ros2 node list           # List active nodes
ros2 param list          # List parameters
ros2 service list        # List services
ros2 run tf2_tools view_frames  # View TF tree
```

---

**Last Updated:** 2025-12-22
