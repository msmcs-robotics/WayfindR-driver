# AMBOT-SLAM - Roadmap

> Last updated: 2026-02-11

## Overview

This roadmap tracks project-level features and milestones. For immediate tasks, see `todo.md`.

**Note**: No time estimates. Focus on WHAT needs to be done, not WHEN.

### Goal Horizons

| Horizon | Goal | Question |
|---------|------|----------|
| **Short-term** | Platform setup + basic ROS2 nodes working | "Can I drive the robot via teleop with ROS2?" |
| **Midterm (First Stable Release)** | SLAM mapping and autonomous navigation | "Can the robot map a room and navigate it autonomously?" |
| **Long-term** | Full sensor suite + deploy to both platforms | "Is this a replicable, polished SLAM robot?" |

---

## Milestone 1: Platform & ROS2 Foundation

### Comprehensive Code Review (Next Session)
- [ ] Thorough code review of all `ambot/` source files relevant to SLAM
  - LiDAR drivers, IMU driver, motor drivers, behavior system
- [ ] Review all `ros2_*_attempt/` folders for reusable configs, launch files, and findings
- [ ] Document reusable components and adaptation notes in `docs/findings/`
- [ ] Update scope and roadmap with refined requirements from code review

### OS & ROS2 Installation
- [ ] Install Ubuntu 22.04 on RPi (or prepare second SD card) — **deferred until ambot work complete**
  - Dependencies: Physical access to RPi, Ubuntu 22.04 image
  - Related: `ros2_install_attempt/docs/ROS2_HUMBLE_INSTALLATION.md`
- [ ] Install ROS2 Humble base packages
  - Dependencies: Ubuntu 22.04 running
  - Related: `docs/setup/ros2_setup_ubu22.04_humble.md`
- [ ] Install SLAM Toolbox + Nav2 packages
  - Dependencies: ROS2 Humble base
  - Related: `ros2_install_attempt/02_install_slam_navigation.sh`
- [ ] Verify installation (all packages, executables, Python bindings)
  - Related: `system_scripts_humble_ubu22.04/findings/2026-01-11-ros2-humble-system-test.md`
- [ ] Jetson Nano Ubuntu 22.04 + ROS2 setup (parallel track)
  - Dependencies: Jetson Nano hardware access

### LiDAR ROS2 Drivers (LD19 + C1M1)
- [ ] Research and select LD19 ROS2 driver (ldlidar_stl_ros2 or similar)
  - LD19: 230400 baud, one-way protocol, auto-streams
- [ ] Research/verify C1M1 ROS2 driver (rplidar_ros2)
  - C1M1: 460800 baud, Slamtec protocol
- [ ] Install/build driver(s) for whichever LiDAR is on the active robot
- [ ] Config-based LiDAR selection (launch parameter to choose driver)
- [ ] Verify `/scan` topic publishes `sensor_msgs/LaserScan`
- [ ] Calibrate LiDAR frame orientation (front = 0 degrees)
  - Related: `ambot/tests/gui_lidar_nav.py` for reference calibration

### MPU6050 IMU ROS2 Node
- [ ] Create ROS2 node wrapping `ambot/pathfinder/imu.py` driver
  - Publishes `sensor_msgs/Imu` on `/imu/data`
- [ ] Wire MPU6050 to I2C bus (VCC->Pin1, GND->Pin9, SCL->Pin5, SDA->Pin3)
  - Related: `ambot/docs/findings/hardware-wiring-quickref.md`
- [ ] Verify IMU data publishing at stable rate
- [ ] Run IMU calibration
  - Related: `ambot/tests/test_imu_calibrate.py`

### L298N Motor ROS2 Node
- [ ] Create ROS2 node subscribing to `/cmd_vel` (geometry_msgs/Twist)
  - Converts linear/angular velocity to differential drive PWM
  - Reuse logic from `ambot/pathfinder/drivers.py`
- [ ] Fix left motor wiring issue
  - Related: `ambot/docs/findings/hardware-wiring-quickref.md`
- [ ] Verify both motors respond to teleop commands
- [ ] Implement smooth acceleration/deceleration

### Robot Description
- [ ] Create URDF for robot (base_link, laser frame, imu_link, wheel frames)
  - Related: `ros2_comprehensive_attempt/findings/` (URDF research)
- [ ] Robot state publisher launch
- [ ] Verify TF tree: `odom` -> `base_link` -> `laser_frame` / `imu_link`

### Teleop & Basic Driving
- [ ] Install `teleop_twist_keyboard`
- [ ] Create bringup launch file (LiDAR + IMU + motors + robot_state_publisher)
- [ ] Verify: can drive robot manually while viewing `/scan` in RViz on desktop

---

## Milestone 2: SLAM Mapping

### SLAM Toolbox Integration
- [ ] Configure SLAM Toolbox parameters for LD19 LiDAR
  - Reference: `ros2_cartography_attempt/config/slam_toolbox_params.yaml`
  - Adapt scan range, resolution, update rates for LD19 specs
- [ ] Create SLAM launch file
  - Reference: `ros2_comprehensive_attempt/launch/slam.launch.py`
- [ ] Verify real-time map building while driving via teleop
- [ ] Test loop closure detection

### Map Management
- [ ] Map saving workflow (`ros2 run nav2_map_server map_saver_cli`)
- [ ] Map loading and visualization
- [ ] Document mapping procedure (drive patterns, tips)

---

## Milestone 3: Autonomous Navigation

### AMCL Localization
- [ ] Configure AMCL parameters
  - Reference: `ros2_comprehensive_attempt/config/amcl_params.yaml`
- [ ] Create localization launch file
  - Reference: `ros2_comprehensive_attempt/launch/localization.launch.py`
- [ ] Verify robot localizes on saved map

### Nav2 Navigation Stack
- [ ] Configure Nav2 parameters (costmaps, planners, controllers)
  - Reference: `ros2_comprehensive_attempt/config/nav2_params.yaml`
  - DWB Controller for differential drive
- [ ] Create navigation launch file
  - Reference: `ros2_comprehensive_attempt/launch/navigation.launch.py`
- [ ] Verify: send goal pose, robot plans and follows path
- [ ] Test obstacle avoidance during navigation
- [ ] Test recovery behaviors (spin, backup, wait)

### Waypoint Navigation
- [ ] Implement waypoint saving and loading
  - Reference: `ros2_localization_attempt/docs/WAYPOINT_NAVIGATION.md`
- [ ] Navigate to named waypoints
- [ ] Multi-waypoint patrol route

---

## Milestone 4: Enhanced Sensors & Robustness

### Wheel Encoders
- [ ] Select and acquire wheel encoders
- [ ] Create encoder ROS2 node publishing wheel odometry
- [ ] Fuse encoder odometry with IMU data
- [ ] Verify improved SLAM accuracy with odometry

### Camera Integration
- [ ] Create camera ROS2 node (EMEET SmartCam S600, /dev/video0)
  - Publishes `sensor_msgs/Image` on `/camera/image_raw`
- [ ] Verify camera feed visible in RViz
- [ ] Optional: visual odometry or feature detection

### Sensor Fusion
- [ ] robot_localization package for IMU + encoder fusion
- [ ] Evaluate SLAM accuracy improvement with fused odometry

---

## Milestone 5: Deployment & Polish

### Deploy Scripts
- [ ] Installation script for fresh Ubuntu 22.04 (apt packages + ROS2 + project deps)
- [ ] Deploy script (rsync from dev machine to robot)
  - Model after `ambot/deploy.sh`
- [ ] Systemd service files for auto-start on boot

### Testing & Verification
- [ ] Automated test suite for node health
- [ ] Map quality benchmarking
- [ ] Navigation reliability testing
- [ ] Memory usage profiling on RPi

### Documentation
- [ ] Complete deployment guide (RPi and Jetson)
- [ ] Mapping operation guide
- [ ] Navigation operation guide
- [ ] Troubleshooting guide

---

## Nice to Have (Lower Priority)

- [ ] Web dashboard for remote monitoring (map + robot pose)
- [ ] Behavior tree customization for navigation
- [ ] Dynamic obstacle detection during navigation
- [ ] Multi-floor mapping support
- [ ] ROS2 bag recording for debugging

---

## Completed

> Features moved here when done, for historical reference.

### Project Setup
- [x] Project structure and documentation initialized — 2026-02-11

---

## Notes

- Milestone 1 is the minimum viable product — robot drives via teleop with ROS2
- Milestone 2 is the first "wow" moment — robot builds a map
- Milestone 3 is the first stable release — robot navigates autonomously
- Milestones 4-5 are enhancement and polish
- Each milestone should be independently testable and demonstrable

---

*Update as features complete. Check boxes when done. Add new features as they're identified.*
