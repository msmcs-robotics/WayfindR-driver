# AMBOT-SLAM - Scope

> Last updated: 2026-02-11
> Status: Draft

---

## Overview

AMBOT-SLAM is a ROS2 Humble SLAM implementation for the AMBOT robot platform. It takes the proven hardware and knowledge from the `ambot/` project (LiDAR, IMU, motors, camera) and integrates them into a proper ROS2 navigation stack with SLAM Toolbox, Nav2, and AMCL localization. This is the "production" implementation that replaces the research experiments in the various `ros2_*_attempt/` folders.

## Objectives

- Deploy a ROS2 Humble SLAM-capable robot using existing AMBOT hardware
- Create autonomous indoor mapping using SLAM Toolbox
- Enable autonomous navigation using Nav2 with saved maps
- Support deployment to both Raspberry Pi (Ubuntu 22.04) and Jetson Nano
- Produce a clean, replicable project that can be re-deployed from scratch

## Requirements

### Functional Requirements
*What the system must do*

- [ ] Publish LiDAR scans on `/scan` topic (configurable: LD19 or Slamtec C1M1, one at a time)
- [ ] Publish IMU data on `/imu/data` topic (MPU6050, I2C bus 1, addr 0x68)
- [ ] Subscribe to `/cmd_vel` and drive L298N motors (differential drive)
- [ ] Run SLAM Toolbox to build 2D occupancy grid maps
- [ ] Save and load maps (`.pgm` + `.yaml` format)
- [ ] Localize on a saved map using AMCL
- [ ] Navigate autonomously to waypoints using Nav2
- [ ] Publish proper TF tree: `map` -> `odom` -> `base_link` -> `laser` / `imu_link`
- [ ] Web-based remote control dashboard (FastAPI) for manual driving over WiFi from any device
- [ ] RC receiver support for physical manual control
- [ ] Camera node for optional visual features (not required for core SLAM)

### Technical Requirements
*Technical constraints, compatibility, performance needs*

- [ ] ROS2 Humble on Ubuntu 22.04
- [ ] Python 3.10 (ROS2 Humble default)
- [ ] SLAM Toolbox for mapping (chosen over Cartographer — see technical decisions)
- [ ] Nav2 for autonomous navigation
- [ ] URDF robot description with accurate TF frames
- [ ] Launch files for modular startup (individual nodes, mapping mode, nav mode, full bringup)
- [ ] YAML configuration files for all tunable parameters

### Resource Requirements
*Hardware, software, dependencies, services*

- [ ] LiDAR sensor — support both LD19 (USB, 230400 baud) and Slamtec C1M1 (USB, 460800 baud)
- [ ] MPU6050 IMU (I2C bus 1, addr 0x68) — driver ready in ambot, needs hardware wiring verification
- [ ] L298N motor driver — RIGHT motor working, LEFT motor has wiring issue (see ambot hardware status)
- [ ] EMEET SmartCam S600 (/dev/video0, 640x480) — optional, for camera features
- [ ] Raspberry Pi (re-imaged to Ubuntu 22.04) OR Jetson Nano (Ubuntu 22.04)
- [ ] ROS2 Humble base installation + SLAM Toolbox + Nav2 packages
- [ ] Wheel encoders (future — for improved odometry)

## Constraints

| Constraint | Reason | Flexible? |
|------------|--------|-----------|
| ROS2 Humble only | LTS release, best package support for SLAM/Nav2 | No |
| Ubuntu 22.04 required | ROS2 Humble binary packages target this OS | No |
| Configurable hardware (one sensor per type) | Support swappable LiDARs (LD19 or C1M1), cameras, IMUs — one active at a time, selected via config | No |
| No cloud dependencies | Robot must operate fully offline | No |
| Python-first nodes | Rapid development, consistent with ambot codebase | Yes |
| Memory-conscious design | RPi has only 906MB RAM | No |

## Assumptions

- [ASSUMED] LD19 LiDAR has a ROS2 driver available (ldlidar_stl_ros2 or similar)
- [ASSUMED] RPi can run ROS2 Humble + SLAM Toolbox within 906MB RAM (may need swap)
- [ASSUMED] SLAM Toolbox works acceptably with LiDAR-only (no wheel encoders initially)
- [ASSUMED] L298N left motor wiring issue will be fixed before SLAM testing
- [VERIFIED] SLAM Toolbox chosen over Cartographer (better maintained for ROS2, better accuracy)
- [VERIFIED] Nav2 stack works with differential drive robots
- [VERIFIED] MPU6050 IMU driver exists and works (ambot/pathfinder/imu.py)

## Boundaries

### In Scope
- ROS2 Humble installation and configuration on RPi and/or Jetson Nano
- LD19 LiDAR ROS2 driver integration
- MPU6050 IMU ROS2 node
- L298N motor driver ROS2 node (cmd_vel subscriber)
- URDF robot description
- SLAM Toolbox integration for mapping
- AMCL localization with saved maps
- Nav2 navigation stack (path planning, obstacle avoidance, recovery behaviors)
- Waypoint navigation
- Web-based control dashboard (FastAPI on robot, browser UI for driving over network)
- RC receiver for physical manual control
- Camera ROS2 node (optional, not blocking)
- Wheel encoder integration (planned, not blocking)
- Deploy scripts for RPi and Jetson
- Launch files and YAML configs

### Out of Scope (Exclusions)
- LLM/conversational AI (stays in ambot/bootylicious)
- Face detection/tracking logic (stays in ambot — camera node here is raw feed only)
- 3D SLAM or 3D LiDAR
- Multi-robot fleet management
- ROS2 Galactic, Iron, or Jazzy (Humble only)
- Custom SLAM algorithm development (we use SLAM Toolbox)
- Gazebo simulation (reference the existing `ros2_comprehensive_attempt/launch/gazebo_sim.launch.py` if needed)
- C/C++ node implementations (Python first)

## Technical Decisions

| Decision | Choice | Rationale | Date |
|----------|--------|-----------|------|
| ROS2 Distribution | Humble | LTS, best package ecosystem, Ubuntu 22.04 | 2026-02-11 |
| SLAM Package | SLAM Toolbox | Actively maintained, better accuracy than Cartographer (0.13m vs 0.21m ATE), official ROS2 support | 2026-02-11 |
| Navigation Stack | Nav2 | Default ROS2 navigation, DWB controller for diff drive | 2026-02-11 |
| Localization | AMCL | Robust particle filter, handles ambiguity, recovers from kidnapping | 2026-02-11 |
| LiDAR Drivers | ldlidar_stl_ros2 (LD19) + rplidar_ros (C1M1) | Support both LiDARs with config-based selection | 2026-02-11 |
| IMU Integration | Custom ROS2 node wrapping ambot driver | Reuse existing MPU6050 code from `ambot/pathfinder/imu.py` | 2026-02-11 |
| Motor Control | Custom ROS2 node wrapping ambot driver | Reuse existing L298N code from `ambot/pathfinder/drivers.py` | 2026-02-11 |
| Target Platforms | RPi (Ubuntu 22.04) + Jetson Nano | Both supported, same ROS2 stack | 2026-02-11 |

## Integration Points

- **ambot/ project**: Hardware drivers (LiDAR, IMU, motors, camera) — reference implementations
- **ros2_comprehensive_attempt/**: Launch files, configs, and Nav2 research findings to adapt
- **ros2_cartography_attempt/**: SLAM Toolbox parameters and mapping workflow
- **ros2_localization_attempt/**: AMCL configuration and waypoint navigation patterns
- **ros2_install_attempt/**: Installation scripts and verification procedures

## Existing Research References

> These files in the parent repository contain valuable research that informed this project's design.
> Reference by path rather than duplicating content.

| Topic | File Path |
|-------|-----------|
| ROS2 Humble Installation | `ros2_install_attempt/docs/ROS2_HUMBLE_INSTALLATION.md` |
| ROS2 Setup Guide | `docs/setup/ros2_setup_ubu22.04_humble.md` |
| SLAM Mapping Findings | `ros2_cartography_attempt/docs/SLAM_MAPPING_FINDINGS.md` |
| Nav2 Research | `ros2_comprehensive_attempt/findings/nav2_research_findings_2026-01-11.md` |
| Localization Findings | `ros2_localization_attempt/docs/LOCALIZATION_FINDINGS.md` |
| Waypoint Navigation | `ros2_localization_attempt/docs/WAYPOINT_NAVIGATION.md` |
| Pre-SLAM Localization | `ambot/docs/findings/localization-pre-slam.md` |
| System Verification | `system_scripts_humble_ubu22.04/findings/2026-01-11-ros2-humble-system-test.md` |
| Hardware Wiring | `ambot/docs/findings/hardware-wiring-quickref.md` |
| Power System Research | `ambot/docs/findings/power-system-research.md` |
| Indoor Mapping Overview | `docs/Indoor Mapping and Navigation System Overview.md` |
| SLAM Toolbox Config | `ros2_cartography_attempt/config/slam_toolbox_params.yaml` |
| Nav2 Config | `ros2_comprehensive_attempt/config/nav2_params.yaml` |
| AMCL Config | `ros2_comprehensive_attempt/config/amcl_params.yaml` |
| LiDAR Config | `ros2_comprehensive_attempt/config/lidar_params.yaml` |
| Existing Launch Files | `ros2_comprehensive_attempt/launch/*.launch.py` (8 files) |

## Open Questions

- [ ] What LD19-compatible ROS2 driver is available? (ldlidar_stl_ros2? ldlidar_ros2?)
- [ ] Can RPi (906MB RAM) run ROS2 + SLAM Toolbox simultaneously? May need swap.
- [ ] Will we re-image the existing RPi or use a second SD card for Ubuntu 22.04?
- [ ] What wheel encoders will be used? (type, resolution, mounting)
- [ ] Is the Jetson Nano already running Ubuntu 22.04 or does it need setup?
- [ ] Should the motor cmd_vel node use PWM directly (RPi.GPIO) or go through a microcontroller?

## Critical Notes

- **RPi OS change required (deferred)**: Current RPi runs Debian 13 / Python 3.13 for ambot. ROS2 Humble needs Ubuntu 22.04 / Python 3.10. Will re-image when ready to transition from ambot to ambot-slam. Not immediate.
- **Swappable hardware**: System supports different models of each sensor type (e.g., LD19 or C1M1 for LiDAR), but only one of each runs at a time. Selection is config-based (launch parameters). Same principle applies to cameras, IMUs, etc.
- **Left motor broken**: L298N left motor doesn't spin — wiring/power issue. Must fix before SLAM testing.
- **Pin numbering**: ambot uses BOARD pin numbering for GPIO. ROS2 nodes should be consistent.
- **Memory constraints**: RPi has 906MB RAM. ROS2 + SLAM Toolbox + Nav2 may be tight. Monitor memory usage.

---

## Revision History

| Date | Changes | By |
|------|---------|-----|
| 2026-02-11 | Initial draft from comprehensive codebase review | LLM |

---

*This document evolves as the project develops. Requirements, constraints, and boundaries can be added, modified, or removed as understanding improves. Major scope changes should be discussed with the user.*
