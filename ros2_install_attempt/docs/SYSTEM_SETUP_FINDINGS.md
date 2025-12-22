# System Setup Findings

## Overview

This document captures lessons learned from setting up ROS2 Humble on the remote system (192.168.0.7).

---

## Remote System Specifications

| Component | Value |
|-----------|-------|
| OS | Ubuntu 22.04.5 LTS (Desktop) |
| Kernel | 6.8.0-90-generic |
| Architecture | x86_64 |
| Display | Physical monitor attached |
| Network | 192.168.0.7 |
| User | devel |

---

## What Was Already Installed

The remote system had much of ROS2 already configured:

| Component | Status |
|-----------|--------|
| ROS2 apt repository | Already configured |
| GPG keys | Already in place |
| ros-humble-desktop | Already installed |
| rosdep | Already initialized |
| slam-toolbox | Already installed |
| navigation2 | Already installed |

This suggests prior ROS2 work on this machine.

---

## What We Installed

| Package | Size | Purpose |
|---------|------|---------|
| ros-humble-rplidar-ros | 168 KB | LiDAR driver |
| Various rqt-* packages | ~22 MB | GUI tools |

---

## Key Configuration Steps

### 1. Passwordless Sudo (User Action)

For SSH automation, the user configured passwordless sudo:

```bash
# /etc/sudoers.d/devel
devel ALL=(ALL) NOPASSWD: ALL
```

**Note:** This is convenient for development but NOT recommended for production.

### 2. USB Permissions for LiDAR

Created udev rule:

```bash
# /etc/udev/rules.d/99-rplidar.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
```

**Result:**
- Device accessible at `/dev/ttyUSB0` AND `/dev/rplidar`
- Permissions 666 (world readable/writable)
- Symlink for consistent naming

### 3. User Group Membership

```bash
sudo usermod -aG dialout devel
```

Required for serial port access without root.

### 4. ROS2 Environment

Already in `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
```

---

## Workspace Structure

Created workspace at `~/ros2_ws/`:

```
~/ros2_ws/
├── src/
│   └── lidar_mapping/           # Custom SLAM package
│       ├── launch/
│       │   └── rplidar_slam.launch.py
│       ├── config/
│       │   ├── slam_toolbox_params.yaml
│       │   └── slam_view.rviz
│       ├── setup.py
│       └── package.xml
├── install/                     # Built packages
├── build/                       # Build artifacts
├── log/                         # Build logs
└── maps/                        # Saved maps
    ├── first_map.pgm
    ├── first_map.yaml
    ├── first_map_offices.yaml   # Waypoints
    └── first_map_posegraph.*
```

---

## Helper Scripts

Created at `~/scripts/`:

| Script | Purpose |
|--------|---------|
| `start_mapping.sh` | Launch LiDAR + SLAM + RViz |
| `save_map.sh <name>` | Save current map |
| `view_map.sh <name>` | View saved map |
| `test_lidar.sh` | Test LiDAR only |

Also created:
- `~/start_localization.sh` - Launch AMCL localization

---

## LiDAR Verification

### Device Detection

```bash
lsusb | grep -i silicon
# Bus 003 Device 003: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
```

### Device Info

```
RPLIDAR S/N: C117E0F6C0E292CDB5E099F044C7400A
Firmware Ver: 1.02
Hardware Rev: 18
Health Status: OK
Scan Mode: DenseBoost
Sample Rate: 5 KHz
Max Distance: 40.0 m
Scan Frequency: 10.0 Hz
```

---

## Transform Tree Configuration

For stationary testing:

```bash
# odom → base_link (identity, no odometry)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

# base_link → laser (LiDAR 10cm above base)
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser
```

For real robot, replace odom→base_link with actual wheel odometry.

---

## SSH Considerations

### Running ROS2 Over SSH

```bash
# Source ROS2 in SSH session
source /opt/ros/humble/setup.bash
```

### RViz Over SSH

RViz requires X11 display. Options:

1. **Use physical monitor** - Best performance
2. **X11 forwarding**: `ssh -X devel@192.168.0.7`
3. **VNC/Remote desktop** - Works but laggy

### Passwordless SSH

```bash
# On local machine
ssh-keygen -t ed25519
ssh-copy-id devel@192.168.0.7
```

---

## File Sync Strategy

Using rsync from local to remote:

```bash
rsync -avz /home/devel/WayfindR-driver/ros2_localization_attempt/ \
    devel@192.168.0.7:~/Desktop/WayfindR-driver/ros2_localization_attempt/
```

Key locations on remote:
- Code: `~/Desktop/WayfindR-driver/`
- Workspace: `~/ros2_ws/`
- Maps: `~/ros2_ws/maps/`
- Scripts: `~/scripts/`

---

## Common Problems Encountered

### 1. SSH Sudo Password Prompt

**Problem:** Interactive sudo prompts don't work over SSH.

**Solution:** Configure passwordless sudo (or use `-t` flag for pseudo-tty).

### 2. USB Device Permissions

**Problem:** `/dev/ttyUSB0` owned by root with 660 permissions.

**Solution:** udev rules + dialout group.

### 3. RViz Display Issues

**Problem:** "could not connect to display" when running over SSH.

**Solution:** Use physical display or X11 forwarding.

### 4. BrokenPipeError in ros2 doctor

**Problem:** Output truncated, pipe errors.

**Solution:** Ignore - output still captured, just display issue.

---

## Recommendations

### For Development

1. Keep passwordless sudo for convenience
2. Use rsync for code sync
3. SSH with -X for occasional RViz use
4. Primary testing on physical display

### For Production

1. Remove passwordless sudo
2. Proper user/group permissions
3. Systemd services for auto-start
4. Log rotation and monitoring

### For Raspberry Pi Deployment

1. Use `ros-humble-ros-base` (headless)
2. Stream data to separate RViz machine
3. Configure ROS_DOMAIN_ID for multi-robot
4. Monitor CPU/memory carefully

---

**Last Updated:** 2025-12-22
