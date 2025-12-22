# LiDAR Setup Guide

## Overview

This document covers setting up the Slamtec C1M1RP (RPLidar C1) LiDAR for ROS2 Humble.

---

## Hardware Specifications

### Slamtec C1M1RP

| Parameter | Value |
|-----------|-------|
| Model | RPLidar C1 |
| Range | 0.1m - 12m (indoor), up to 40m (Sensitivity mode) |
| Accuracy | ±30mm @ 5m |
| Scan Rate | 10 Hz |
| Angular Resolution | 0.225° |
| Sample Rate | 2-5 KHz (mode dependent) |
| Power | 5V DC, 450mA typical |
| Interface | USB (CP210x UART Bridge) |

### Scan Modes

| Mode | Description |
|------|-------------|
| Standard | Balanced range/speed |
| DenseBoost | High sample rate for indoor |
| Sensitivity | Maximum range (40m) |
| Express | Fast scanning |

---

## USB Setup

### 1. Connect LiDAR

Plug USB cable into computer. The LiDAR motor should spin.

### 2. Verify Detection

```bash
# Check USB device
lsusb | grep -i silicon
# Expected: Silicon Labs CP210x UART Bridge

# Check serial port
ls -la /dev/ttyUSB*
# Expected: /dev/ttyUSB0 (or similar)
```

### 3. Create udev Rules (Persistent Naming)

Create `/etc/udev/rules.d/99-rplidar.rules`:

```bash
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
# Slamtec RPLidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
EOF

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify symlink
ls -la /dev/rplidar
# Should link to /dev/ttyUSB0
```

### 4. Set Permissions

```bash
# Option A: Add user to dialout group (requires logout)
sudo usermod -a -G dialout $USER

# Option B: Set wide-open permissions (for testing)
sudo chmod 666 /dev/ttyUSB0
```

---

## ROS2 Driver Installation

### Install RPLidar ROS2 Package

```bash
sudo apt update
sudo apt install ros-humble-rplidar-ros
```

### Test LiDAR

```bash
source /opt/ros/humble/setup.bash

# Start LiDAR node
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser \
    -p scan_mode:=DenseBoost
```

### Verify Data

```bash
# Check topic exists
ros2 topic list | grep scan

# Check data rate
ros2 topic hz /scan
# Expected: ~10 Hz

# View single scan
ros2 topic echo /scan --once
```

---

## ROS2 Node Parameters

### rplidar_node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | /dev/ttyUSB0 | Serial port path |
| `serial_baudrate` | 460800 | Baud rate |
| `frame_id` | laser | TF frame name |
| `inverted` | false | Flip scan direction |
| `angle_compensate` | true | Compensate for motor speed |
| `scan_mode` | Standard | Scan mode name |
| `scan_frequency` | 10.0 | Target scan rate (Hz) |

### Launch File Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'scan_mode': 'DenseBoost',
            }],
            output='screen',
        ),
    ])
```

---

## LaserScan Message Format

```
sensor_msgs/LaserScan:
  header:
    stamp: <timestamp>
    frame_id: "laser"
  angle_min: -3.14159    # Start angle (radians)
  angle_max: 3.14159     # End angle (radians)
  angle_increment: 0.004 # Angular resolution
  time_increment: 0.0    # Time between readings
  scan_time: 0.1         # Time for full scan
  range_min: 0.1         # Minimum valid range (m)
  range_max: 12.0        # Maximum valid range (m)
  ranges: [...]          # Array of distances (m)
  intensities: [...]     # Signal strength (optional)
```

---

## Transform Setup

The LiDAR needs a TF frame connecting it to the robot:

```
base_link
 └── laser (LiDAR frame)
```

### Static Transform Publisher

```bash
# laser is 10cm above base_link
ros2 run tf2_ros static_transform_publisher \
    0 0 0.1 0 0 0 base_link laser
```

### In Launch File

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
)
```

---

## Troubleshooting

### LiDAR Not Spinning

1. Check power (USB provides 5V)
2. Check USB cable connection
3. Try different USB port

### "Permission denied" Error

```bash
sudo chmod 666 /dev/ttyUSB0
# Or add to dialout group and relog
```

### "Device not found"

```bash
# Check device exists
ls /dev/ttyUSB*

# Check udev rules
cat /etc/udev/rules.d/99-rplidar.rules

# Reload rules
sudo udevadm control --reload-rules
```

### Scan Data is Noisy

1. Clean LiDAR lens
2. Check for reflective surfaces
3. Ensure stable mounting
4. Try different scan mode

### Motor Runs But No Data

```bash
# Check baud rate matches
# C1 uses 460800, not 115200
```

---

## RViz Visualization

### Add LaserScan Display

1. Click "Add" in Displays panel
2. Select "By topic" → `/scan` → LaserScan
3. Or "By display type" → LaserScan, set topic to `/scan`

### Display Settings

| Setting | Recommended |
|---------|-------------|
| Topic | /scan |
| Style | Points or Flat Squares |
| Size | 0.03m |
| Color Transformer | FlatColor or Intensity |
| Color | Red or rainbow |

---

## Performance Tips

1. **Use DenseBoost mode** for indoor mapping
2. **Mount rigidly** to avoid vibration noise
3. **Keep lens clean** for accurate readings
4. **Avoid direct sunlight** which interferes with infrared
5. **Position at 0.1-0.3m height** for good floor coverage

---

**Last Updated:** 2025-12-22
