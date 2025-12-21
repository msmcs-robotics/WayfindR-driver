# Actual ROS2 Installation Log - Remote Server

**Date:** 2025-12-20
**Target System:** devel@192.168.0.7
**OS:** Ubuntu 22.04.5 LTS (Desktop, not WSL)
**Kernel:** 6.8.0-90-generic
**Hardware:** x86_64 with monitor attached

---

## Executive Summary

Successfully installed ROS2 Humble with SLAM Toolbox on a remote Ubuntu 22.04 desktop system via SSH. The Slamtec C1M1RP LiDAR was detected and confirmed working.

**Overall Status: SUCCESS**

---

## What Went Well

### 1. ROS2 Repository Already Configured
- The ROS2 apt repository was already set up on the target machine
- GPG keys were already in place
- This saved significant setup time

### 2. ROS2 Humble Desktop Already Installed
- `ros-humble-desktop` package was already installed
- All core ROS2 tools (rviz2, rqt, etc.) were available
- rosdep was already initialized

### 3. Most SLAM/Nav2 Packages Already Present
The following were already installed:
- `ros-humble-slam-toolbox`
- `ros-humble-navigation2`
- `ros-humble-nav2-bringup`
- `ros-humble-robot-localization`
- `ros-humble-tf2-tools`
- `ros-humble-rviz2`

### 4. LiDAR Detection
- Slamtec C1 LiDAR detected immediately as `Silicon Labs CP210x UART Bridge`
- USB device at Bus 003 Device 003
- Device ID: 10c4:ea60

### 5. RPLidar ROS2 Driver
- `ros-humble-rplidar-ros` installed successfully (168 KB)
- Driver started without issues
- Scan data immediately available at 10Hz

---

## What Required Attention

### 1. Sudo Password Required (Resolved)
**Issue:** Remote SSH session couldn't handle interactive sudo password prompt.

**Solution:** User configured passwordless sudo by adding to `/etc/sudoers.d/devel`:
```
devel ALL=(ALL) NOPASSWD: ALL
```

### 2. USB Permissions
**Issue:** LiDAR device `/dev/ttyUSB0` had restricted permissions (660, root:dialout).

**Solution:** Created udev rule at `/etc/udev/rules.d/99-rplidar.rules`:
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
```

Then ran:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Result:** Device now accessible at both `/dev/ttyUSB0` and `/dev/rplidar` with 666 permissions.

### 3. User Not in dialout Group
**Issue:** User needed dialout group membership for serial port access.

**Solution:** `sudo usermod -aG dialout devel`

---

## LiDAR Verification Results

### Device Information
```
RPLIDAR S/N: C117E0F6C0E292CDB5E099F044C7400A
Firmware Ver: 1.02
Hardware Rev: 18
Health Status: OK
```

### Scan Configuration
```
Scan Mode: DenseBoost
Sample Rate: 5 KHz
Max Distance: 40.0 m
Scan Frequency: 10.0 Hz
```

### Sample Scan Data
```yaml
angle_min: -3.13 rad
angle_max: 3.12 rad
angle_increment: 0.0127 rad
range_min: 0.15 m
range_max: 40.0 m
ranges: [1.13, 0.73, 0.72, 5.0, 4.95, 6.39, 8.01, 9.05, ...] # meters
```

---

## Packages Installed This Session

| Package | Size | Status |
|---------|------|--------|
| ros-humble-rplidar-ros | 168 KB | NEW |
| ros-humble-rqt-* (various) | ~22 MB | NEW |
| ros-humble-moveit-* (various) | Included | NEW |

---

## Workspace Created

### Location: `~/ros2_ws/`

### Structure:
```
~/ros2_ws/
├── src/
│   ├── lidar_mapping/           # Custom package
│   │   ├── launch/
│   │   │   └── rplidar_slam.launch.py
│   │   ├── config/
│   │   │   ├── slam_toolbox_params.yaml
│   │   │   └── slam_view.rviz
│   │   ├── setup.py
│   │   └── package.xml
│   └── AutoFrontierSearch_ros2-humble/  # Pre-existing
├── install/
├── build/
├── log/
└── maps/                        # For saved maps
```

### Helper Scripts: `~/scripts/`
```
~/scripts/
├── start_mapping.sh     # Launch SLAM + RViz
├── save_map.sh          # Save current map
├── test_lidar.sh        # Test LiDAR only
└── view_map.sh          # View saved map
```

---

## Configuration Files

### SLAM Toolbox Parameters
Key settings in `slam_toolbox_params.yaml`:
- **Resolution:** 0.05m (5cm per pixel)
- **Max Laser Range:** 12.0m (C1 supports 40m, but 12m is more reliable indoors)
- **Loop Closure:** Enabled
- **Mode:** Mapping (async)

### Transform Tree
```
map → odom → base_link → laser
```

Static transforms configured for stationary LiDAR testing.

---

## Environment Setup

Added to `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
```

Workspace sourcing:
```bash
source ~/ros2_ws/install/setup.bash
```

---

## Recommendations

### For Production Use

1. **Remove static odom→base_link transform** when using actual robot with wheel encoders
2. **Adjust `base_link→laser` transform** based on actual LiDAR mounting position
3. **Tune SLAM parameters** after initial mapping tests
4. **Consider Standard scan mode** if DenseBoost causes CPU issues on lower-end hardware

### For Raspberry Pi Deployment

1. ROS2 Humble requires Ubuntu 22.04 (not Raspbian)
2. Use `ros-humble-ros-base` instead of `ros-humble-desktop` (no GUI)
3. Stream visualization to a separate machine running RViz
4. Consider `ros-humble-rplidar-ros` baud rate adjustments for Pi UART

---

## Test Commands Reference

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start mapping with RViz
ros2 launch lidar_mapping rplidar_slam.launch.py

# Or use helper script
~/scripts/start_mapping.sh

# Check topics while running
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /scan

# Save map
~/scripts/save_map.sh my_first_map

# View saved map
~/scripts/view_map.sh my_first_map
```

---

## Files Modified on Remote System

| File | Action |
|------|--------|
| `/etc/sudoers.d/devel` | Created (by user) |
| `/etc/udev/rules.d/99-rplidar.rules` | Created |
| `~/.bashrc` | Already had ROS2 source |
| `~/ros2_ws/` | Created workspace |
| `~/scripts/` | Created helper scripts |

---

## Conclusion

The installation was largely smooth due to pre-existing ROS2 setup on the target machine. The main work involved:
1. Installing the RPLidar driver package
2. Configuring USB permissions
3. Creating a custom ROS2 package with launch files
4. Setting up SLAM Toolbox configuration

The system is now ready for SLAM mapping with the Slamtec C1 LiDAR.

---

**Log Created By:** Claude Code (Opus 4.5)
**Session Date:** 2025-12-20
**Remote Target:** devel@192.168.0.7
