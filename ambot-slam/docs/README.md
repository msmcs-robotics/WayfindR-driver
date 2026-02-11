# AMBOT-SLAM

ROS2 Humble SLAM navigation for the AMBOT robot platform.

## Overview

AMBOT-SLAM brings autonomous indoor mapping and navigation to the AMBOT robot using ROS2 Humble, SLAM Toolbox, and Nav2. It reuses the hardware and drivers from the `ambot/` project (LD19 LiDAR, MPU6050 IMU, L298N motors, camera) and integrates them into a proper ROS2 navigation stack. Supports deployment to Raspberry Pi (Ubuntu 22.04) and Jetson Nano.

## Quick Start

```bash
# Prerequisites: Ubuntu 22.04 + ROS2 Humble installed
# See docs/scope.md "Existing Research References" for installation guides

# Source ROS2
source /opt/ros/humble/setup.bash

# Launch robot bringup (LiDAR + IMU + motors)
ros2 launch ambot_slam bringup.launch.py

# In another terminal: teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# For SLAM mapping mode:
ros2 launch ambot_slam slam.launch.py

# For autonomous navigation (with saved map):
ros2 launch ambot_slam navigation.launch.py map:=/path/to/map.yaml
```

## Hardware

| Component | Model | Interface | Status |
|-----------|-------|-----------|--------|
| LiDAR | LD19 / Slamtec C1M1 | USB (230400 / 460800 baud) | Both supported |
| IMU | MPU6050 | I2C bus 1, addr 0x68 | Driver ready, needs wiring |
| Motors | L298N dual H-bridge | GPIO (BOARD pins) | Right working, left needs fix |
| Camera | EMEET SmartCam S600 | USB, /dev/video0 | Working (optional for SLAM) |

For wiring details, see `ambot/docs/findings/hardware-wiring-quickref.md`.

## Project Structure

```
ambot-slam/
├── docs/
│   ├── README.md        # This file
│   ├── scope.md         # Project boundaries and technical decisions
│   ├── roadmap.md       # Feature milestones
│   ├── todo.md          # Current tasks
│   ├── features/        # Feature specifications
│   ├── findings/        # Research and investigation docs
│   └── archive/         # Session summaries
├── config/              # ROS2 YAML parameter files
├── launch/              # ROS2 launch files
├── src/                 # ROS2 nodes (Python)
├── tests/               # Test scripts
│   ├── results/         # Test output (gitignored)
│   └── outputs/         # Generated artifacts (gitignored)
└── [future: package.xml, setup.py for ROS2 package]
```

## Documentation

- [scope.md](scope.md) - What this project is and isn't, technical decisions, hardware references
- [roadmap.md](roadmap.md) - 5 milestones from foundation to polish
- [todo.md](todo.md) - Current session tasks

### Key References (in parent repository)

| Topic | Path |
|-------|------|
| ROS2 Installation | `ros2_install_attempt/docs/ROS2_HUMBLE_INSTALLATION.md` |
| SLAM Findings | `ros2_cartography_attempt/docs/SLAM_MAPPING_FINDINGS.md` |
| Nav2 Research | `ros2_comprehensive_attempt/findings/nav2_research_findings_2026-01-11.md` |
| Hardware Wiring | `ambot/docs/findings/hardware-wiring-quickref.md` |
| Power System | `ambot/docs/findings/power-system-research.md` |

## Requirements

- Ubuntu 22.04 (RPi or Jetson Nano)
- ROS2 Humble
- SLAM Toolbox (`ros-humble-slam-toolbox`)
- Navigation2 (`ros-humble-navigation2`)
- LD19 LiDAR ROS2 driver (TBD)
- Python 3.10

---

*For detailed project boundaries and technical decisions, see `docs/scope.md`*
