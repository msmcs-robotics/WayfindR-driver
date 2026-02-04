# LD19 LiDAR Protocol Research

> Date: 2026-02-03
> Device: YOUYEETOO LD19 (LDRobot LD19)
> Location: /dev/ttyUSB0 via CP2102 USB-UART bridge

## Key Findings

The LD19 does **NOT** use RPLidar protocol. It uses a proprietary LDRobot protocol.

### Protocol Summary

| Parameter | Value |
|-----------|-------|
| Baud Rate | **230400** (not 460800) |
| Packet Size | 47 bytes |
| Points per Packet | 12 |
| Header Byte | 0x54 |
| VerLen Byte | 0x2C |
| Communication | **One-way only** (no commands) |

### Key Differences from RPLidar

1. **One-way communication** - Device streams data continuously, cannot receive commands
2. **No handshaking** - Starts streaming 2-3 seconds after power-on
3. **No start/stop commands** - Control via power or PWM pin
4. **Fixed 47-byte packets** with exactly 12 measurement points each

## Packet Structure (47 bytes)

```
Offset  Size  Field         Description
------  ----  -----------   -----------
0       1     Header        Fixed 0x54
1       1     VerLen        Fixed 0x2C
2-3     2     Speed         Rotation speed (degrees/second), little-endian
4-5     2     Start Angle   Start angle in 0.01° units, little-endian
6-41    36    Point Data    12 points × 3 bytes each
42-43   2     End Angle     End angle in 0.01° units, little-endian
44-45   2     Timestamp     Milliseconds (0-30000, wraps)
46      1     CRC8          Checksum of bytes 0-45
```

### Point Data (3 bytes per point)

```
Offset  Size  Field       Description
------  ----  ----------  -----------
0-1     2     Distance    Distance in mm, little-endian
2       1     Intensity   Signal strength (0-255)
```

### Angle Calculation

Points are linearly interpolated between start and end angles:

```python
angle_step = (end_angle - start_angle) / 11  # 11 intervals for 12 points
angle[i] = start_angle + (i * angle_step)    # for i = 0 to 11
```

Handle wraparound when end_angle < start_angle:
```python
if end_angle < start_angle:
    angle_diff = (end_angle + 360) - start_angle
else:
    angle_diff = end_angle - start_angle
```

## CRC8 Algorithm

- **Polynomial:** 0x4D
- **Initial Value:** 0x00

Complete lookup table in pathfinder/lidar_ld19.py

## Motor Control

The LD19 motor speed is controlled via PWM on a dedicated pin (not via serial):

| Method | Description |
|--------|-------------|
| Power On/Off | Motor starts automatically |
| PWM Pin | 5V PWM controls RPM |
| Default | ~10 Hz rotation when PWM not connected |

## Hardware Notes

- Detected as Silicon Labs CP210x UART Bridge
- USB VID:PID = 10c4:ea60
- Outputs ~55KB/sec of data at 230400 baud
- Range: 0.1m to 12m
- Scan rate: ~10 Hz (adjustable via PWM)

## Resources

### Official

- **[YOUYEETOO Wiki - FHL-LD19](https://wiki.youyeetoo.com/en/Lidar/D300)** - Official product page
- [LD19 Development Manual v2.3](https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf)
- [LDRobot SDK (C++)](https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk)
- [LDRobot ROS2 Package](https://github.com/ldrobotSensorTeam/ldlidar_ros2)

### Community

- [Python Visualizer](https://github.com/halac123b/Visualize-data-from-Lidar-LD19_Matplotlib-Python)
- [Python Parser](https://github.com/Nannigalaxy/ldrobot-lidar)

## Existing Code in Project

Found existing parser at:
`/home/devel/WayfindR-driver/ros_tank_xiaor/lidar_youyeetoo_visual_plotter.py`

This file correctly parses LD19 data with 230400 baud.

## Action Items

1. Create `pathfinder/lidar_ld19.py` - dedicated LD19 driver
2. Update `pathfinder/config.py` - add LD19 configuration
3. Make pathfinder auto-detect LiDAR type (LD19 vs C1M1)
4. Update test scripts to use 230400 baud for LD19
