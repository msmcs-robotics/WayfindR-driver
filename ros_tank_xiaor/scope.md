# ros_tank_xiaor - Scope and Documentation

## Overview

This folder contains experimental code and documentation for integrating the **XiaoR GEEK ROS Tank** robot platform into the WayfindR indoor mapping and navigation system. This is a testing platform used to evaluate hardware compatibility and driver development before full integration into the main WayfindR project.

## Hardware Platform

### XiaoR ROS SLAM Robot Car (XR-ROS)

The XiaoR GEEK robot is a commercial ROS-compatible robot kit designed for SLAM, mapping, and autonomous navigation. Key specifications include:

#### Core Components
- **Compute Platform**: Raspberry Pi 4B or 3B+ compatible
- **Chassis**: Large aluminum alloy chassis with tank/tracked design
- **Motors**: Encoding motors (precise movement control with odometry feedback)
- **Drive System**: Differential drive / tank steering configuration
- **Control Board**: XiaoR professional robot expansion board (PWR.ROS.A)
  - Voltage regulation: 7-12V DC input, 5V output
  - Motor driver: Integrated H-bridge for forward/reverse/speed control
  - Servo outputs: 8-way servo interface (max 100mA per servo)
  - Sensor interfaces: GPIO pins for IMU, encoders, and additional sensors
  - Power requirements: 12-15V DC, minimum 3A
  - Connector: 5.5mm x 2.1mm DC jack

#### Sensors
- **LiDAR**: XR-Lidar S1 (likely RPLIDAR S1-based or YouYeeToo variant)
  - 360-degree 2D laser scanning
  - Range: Up to 40m (white objects), 10m (black objects)
  - Sample rate: 9,200 samples/second
  - Scanning frequency: 5-15 Hz (adjustable)
  - Interface: UART TTL (230400 baud)
  - Resolution: 3cm distance, 0.391° angular
- **IMU**: Integrated gyroscope/accelerometer (model not specified)
- **Camera**: HD camera for visual monitoring (optional in WayfindR)
- **Wheel Encoders**: Built-in encoding motors provide odometry data

#### User Interface
- **Display**: 3.5-inch touch display for direct control
- **Connectivity**: WiFi (via Raspberry Pi)
- **Mobile App**: Dedicated Android app for remote control

#### Software Support
- Pre-configured ROS packages (ROS1 Noetic / ROS2 Humble compatible)
- SLAM capabilities (Cartographer, SLAM Toolbox)
- Navigation stack integration
- Annotated source code and documentation
- Assembly time: ~1 hour with video instructions

### Manufacturer Resources

Official XiaoR GEEK documentation and tutorials:
- Product overview: https://www.xiaorgeek.net/blogs/guide/meet-your-next-robotics-project-xr-ros-slam-robot-car-with-lidar-for-raspberry-pi-4b
- Basic operation manual: https://www.xiaorgeek.net/blogs/news/raspberry-pi-ros-robot-basic-operation-manual-v1-0
- Technical documentation portal: https://xiaorgeek.yuque.com/mrht1w/rostank

## Relationship to WayfindR Project

### Project Context

WayfindR is an indoor mapping and navigation system designed for autonomous robot navigation in buildings (offices, warehouses, etc.). The project has three main phases:

1. **Mapping Phase**: Autonomous exploration and map generation using SLAM
2. **Waypoint Phase**: Manual driving to mark named waypoints (offices, rooms, landmarks)
3. **Navigation Phase**: Autonomous navigation to named waypoints via API commands

### Why XiaoR Platform?

The XiaoR robot serves as a **hardware evaluation platform** for WayfindR development:

**Advantages:**
- Pre-integrated hardware (motors, encoders, LiDAR, IMU)
- Known-working ROS configuration (reduces initial setup time)
- Commercial documentation and community support
- Allows focus on higher-level navigation algorithms rather than low-level hardware debugging
- Similar sensor suite to custom builds (LiDAR + IMU + encoders)

**Comparison to Other Platforms:**
- **Adeept PiCar Pro** (in `/adeept_car_stuff/`): Simpler platform with L298N motor drivers, no encoders, no LiDAR integration out-of-box
- **Custom L298N builds** (original WayfindR design): More flexible but requires extensive low-level driver development

The XiaoR platform provides a **known-good baseline** for testing WayfindR's SLAM, localization, and navigation algorithms before porting to custom hardware platforms.

## Key Files and Their Purposes

### Python Scripts

#### `lidar_youyeetoo_visual_plotter.py` (6,127 bytes)
**Status**: Fully implemented and functional

**Purpose**: Real-time LiDAR data visualization tool for the YouYeeToo/RPLIDAR S1 sensor

**Key Features:**
- Connects to LiDAR via serial port (default: `/dev/ttyUSB0` @ 230400 baud)
- Parses proprietary LiDAR packet format (47-byte packets, header 0x54)
- Extracts 12 distance measurements per packet with intensity data
- Real-time polar plot visualization using matplotlib
- Color-coded intensity mapping (hot colormap)
- Range filtering (0-12 meters)
- Visual feedback: 2000-point rolling window
- Command-line configurable port and baudrate

**Dependencies:**
- `serial` (pyserial)
- `struct`, `math`, `numpy`
- `matplotlib` (with animation support)

**Use Cases:**
- Testing LiDAR connectivity and functionality
- Debugging sensor placement and orientation
- Verifying scan quality before SLAM integration
- Visual confirmation of obstacle detection

**Example Usage:**
```bash
python3 lidar_youyeetoo_visual_plotter.py /dev/ttyUSB0 230400
```

#### `motors_xiaor_test.py` (0 bytes)
**Status**: Empty placeholder file

**Purpose**: Intended for motor control testing on XiaoR platform

**Expected Functionality** (not yet implemented):
- Interface with XiaoR expansion board motor drivers
- Test differential drive kinematics
- Verify encoder readings
- Validate ROS `/cmd_vel` to motor command translation
- Movement primitives (forward, backward, turn, stop)

**Similar Reference**: See `/adeept_car_stuff/motors_adeept_picar_pro_test.py` for motor control patterns (GPIO-based PWM control)

### Documentation

#### `docs/init.md` (20 lines)
**Status**: Link collection only

**Purpose**: Quick reference to manufacturer documentation and tutorials

**Contents:**
- Links to XiaoR GEEK product pages
- Links to operation manuals
- Links to Chinese documentation portal (Yuque)

**Note**: External links; requires internet access and may change over time

## Current State of Implementation

### Completed Components

1. **LiDAR Driver**: Fully functional visualization tool
   - Successfully parses YouYeeToo/RPLIDAR S1 data protocol
   - Tested and working polar plot visualization
   - Ready for ROS integration

### Incomplete Components

1. **Motor Control**: No implementation yet
   - Empty placeholder file
   - Requires XiaoR expansion board API documentation
   - May need reverse-engineering of existing ROS packages

2. **ROS Integration**: Not started
   - No ROS launch files
   - No URDF/robot description files
   - No TF (transform) configuration
   - No navigation parameter files

3. **Sensor Fusion**: Not started
   - No IMU driver integration
   - No wheel encoder odometry publisher
   - No robot_localization configuration

4. **Navigation Stack**: Not configured
   - No Nav2/move_base configuration
   - No costmap parameters
   - No local/global planner configuration

5. **WayfindR API Integration**: Not implemented
   - No Flask server for waypoint navigation
   - No HTTP endpoint definitions
   - No waypoint storage/retrieval system

### Testing Status

| Component | Status | Notes |
|-----------|--------|-------|
| LiDAR Communication | Tested | Working with visualization tool |
| LiDAR Data Parsing | Tested | Packet structure understood |
| Motor Control | Not Tested | No code available |
| Encoders | Not Tested | Interface unknown |
| IMU | Not Tested | Interface unknown |
| ROS2 Integration | Not Tested | No launch files |
| SLAM | Not Tested | No test environment |
| Navigation | Not Tested | Requires SLAM maps |

## Dependencies and Requirements

### Hardware Requirements

- XiaoR ROS SLAM Robot Car kit (assembled)
- Raspberry Pi 4B (2GB+ RAM recommended)
- MicroSD card (32GB+, Class 10 or better)
- 12-15V DC power supply (3A minimum)
- USB-to-Serial adapter (if LiDAR uses separate USB)
- WiFi network for remote development/visualization

### Software Requirements

#### Operating System
- Ubuntu 22.04 LTS (Jammy Jellyfish) - recommended
- Ubuntu 20.04 LTS (Focal Fossa) - alternative
- Raspberry Pi OS (64-bit) - possible but less tested

#### ROS Version
- **ROS2 Humble** (recommended) - actively maintained, better SLAM options
- ROS1 Noetic (alternative) - if using legacy XiaoR packages

**Rationale for ROS2**:
- Better SLAM Toolbox support (0.13m vs 0.21m accuracy over Cartographer)
- Active Nav2 development
- Long-term support through 2027
- Better Python 3.10+ compatibility

#### Python Packages

For LiDAR visualization (`lidar_youyeetoo_visual_plotter.py`):
```bash
pip install pyserial numpy matplotlib
```

For ROS2 integration:
```bash
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-robot-localization \
                 ros-humble-imu-tools \
                 ros-humble-rviz2
```

#### Serial Port Permissions
```bash
sudo usermod -a -G dialout $USER
# Logout and login required
```

### Network Requirements

- **mDNS/Avahi**: For hostname-based discovery (avoids static IP issues)
  ```bash
  sudo apt install avahi-daemon
  sudo systemctl enable avahi-daemon
  ```
- **ROS Environment Variables**:
  ```bash
  export ROS_DOMAIN_ID=42  # ROS2 only
  export ROS_HOSTNAME=$(hostname).local
  ```

### Development Tools

- `colcon` - ROS2 build tool
- `rosdep` - Dependency management
- Git for version control
- SSH for remote access
- Visual Studio Code / ROS extension (optional)

## Next Steps for Implementation

### Immediate Priorities

1. **Document Motor Control Interface**
   - Reverse-engineer XiaoR expansion board API
   - Identify GPIO pins / I2C addresses / serial protocol
   - Document encoder data format and resolution
   - Create basic motor test script (forward, backward, turn)

2. **Create ROS2 Workspace**
   - Initialize colcon workspace (`~/ros2_ws`)
   - Create `xiaor_description` package (URDF)
   - Create `xiaor_bringup` package (launch files)
   - Define TF tree: `map -> odom -> base_link -> laser_frame`

3. **LiDAR ROS Integration**
   - Port visualization code to ROS2 node
   - Publish to `/scan` topic (sensor_msgs/LaserScan)
   - Configure frame_id and transform
   - Test with `ros2 topic echo /scan`

### Medium-Term Goals

4. **Odometry Publisher**
   - Read encoder data from XiaoR board
   - Calculate wheel velocities
   - Publish to `/odom` topic (nav_msgs/Odometry)
   - Broadcast `odom -> base_link` transform

5. **IMU Integration**
   - Identify IMU model and interface
   - Publish to `/imu` topic (sensor_msgs/Imu)
   - Calibrate gyroscope/accelerometer
   - Configure robot_localization for sensor fusion

6. **Motor Controller Node**
   - Subscribe to `/cmd_vel` (geometry_msgs/Twist)
   - Convert to differential drive motor commands
   - Implement velocity PID control
   - Add safety limits (max speed, acceleration)

7. **SLAM Configuration**
   - Configure SLAM Toolbox parameters
   - Tune scan matching settings
   - Test mapping in controlled environment
   - Validate map quality

### Long-Term Integration

8. **Navigation Stack Setup**
   - Configure Nav2 costmaps (global and local)
   - Set inflation parameters
   - Configure DWA local planner
   - Test autonomous navigation

9. **WayfindR API Development**
   - Design waypoint storage format (YAML)
   - Implement Flask server with ROS2 integration
   - Create HTTP endpoints (`/goto`, `/status`, `/stop`)
   - Add authentication/security

10. **Testing and Validation**
    - Create test maps in controlled environments
    - Validate localization accuracy
    - Measure navigation success rate
    - Document failure modes and limitations

## Technical Challenges and Considerations

### Known Issues

1. **Serial Port Conflicts**: LiDAR and other sensors may compete for USB ports
   - Solution: Use USB hub; configure udev rules for persistent device names

2. **WiFi IP Changes**: Robot IP may change when roaming between access points
   - Solution: Use mDNS hostnames (`robot.local`) instead of IPs
   - ROS_HOSTNAME configuration critical

3. **Computational Load**: Raspberry Pi 4 may struggle with SLAM + Nav2 + visualization
   - Solution: Run RViz2 on desktop workstation, only SLAM/Nav on robot
   - Consider SLAM Toolbox's "localization mode" after mapping

4. **Encoder Drift**: Odometry accumulates error over time
   - Solution: Fuse with IMU using robot_localization (EKF)
   - SLAM provides corrections via scan matching

5. **LiDAR Mounting**: Sensor height and orientation affect map quality
   - XiaoR platform provides fixed mounting; document exact position
   - Measure laser_frame offset from base_link

### Performance Considerations

- **SLAM CPU Usage**: ~30-50% on RPi4 (SLAM Toolbox async mode)
- **Navigation CPU Usage**: ~20-30% on RPi4 (Nav2 stack)
- **Network Bandwidth**: ~1-5 Mbps for ROS topic streaming to desktop
- **Mapping Speed**: Recommended 0.2-0.5 m/s for quality maps

### Safety Considerations

- Emergency stop mechanism (hardware kill switch recommended)
- Velocity limits in navigation parameters
- Obstacle inflation radius in costmaps
- Battery voltage monitoring (low battery = stop)
- Watchdog timer for motor commands (timeout = stop)

## Integration with Main WayfindR System

### Data Flow Architecture

```
XiaoR Hardware -> ROS2 Nodes -> SLAM/Nav2 -> Flask API -> HTTP Clients
     |                |              |            |            |
   LiDAR          /scan          /map       /goto         REST API
   Encoders       /odom        /goal_pose  /status       waypoints
   IMU            /imu         /cmd_vel    /stop
   Motors       cmd_vel
```

### Shared Components with WayfindR

1. **SLAM Configuration**: Reusable across platforms
2. **Navigation Parameters**: May need tuning per robot dimensions
3. **Waypoint Format**: Standardized YAML structure
4. **Flask API**: Common interface for all robot platforms
5. **Map Storage**: PGM/YAML format (ROS standard)

### Platform-Specific Code

1. **Motor Control**: XiaoR expansion board vs. L298N vs. others
2. **URDF**: Robot dimensions and sensor positions
3. **Encoder Resolution**: Ticks per revolution varies by platform
4. **IMU Calibration**: Device-specific

## References and Resources

### Manufacturer Documentation
- [XiaoR ROS SLAM Robot Car Overview](https://www.xiaorgeek.net/blogs/guide/meet-your-next-robotics-project-xr-ros-slam-robot-car-with-lidar-for-raspberry-pi-4b)
- [XiaoR Basic Operation Manual](https://www.xiaorgeek.net/blogs/news/raspberry-pi-ros-robot-basic-operation-manual-v1-0)
- [XiaoR Chinese Documentation Portal](https://xiaorgeek.yuque.com/mrht1w/rostank)

### LiDAR Resources
- [YouYeeToo LiDAR Products](https://www.youyeetoo.com/collections/lidar)
- [RPLIDAR S1M1 Specifications](https://www.youyeetoo.com/products/rplidar-s1m1)
- [YouYeeToo LiDAR FAQ](https://wiki.youyeetoo.com/en/Lidar/FAQ)

### WayfindR Documentation
- Main README: `/home/devel/Desktop/WayfindR-driver/README.md`
- Project Overview: `/home/devel/Desktop/WayfindR-driver/docs/overview.md`
- System Design: `/home/devel/Desktop/WayfindR-driver/docs/Indoor Mapping and Navigation System Overview.md`
- ROS2 Setup Guide: `/home/devel/Desktop/WayfindR-driver/ros2_install_attempt/README.md`

### ROS Resources
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox vs Cartographer Analysis](https://www.mdpi.com/2079-9292/14/24/4822)

### Alternative Platforms
- Adeept PiCar Pro: `/home/devel/Desktop/WayfindR-driver/adeept_car_stuff/`
- Custom L298N builds: `/home/devel/Desktop/WayfindR-driver/old_stuff/motors_telemetry/`

## Conclusion

The `ros_tank_xiaor` folder represents an early-stage integration effort for a commercial ROS robot platform. While the LiDAR visualization is functional, significant work remains to create a complete WayfindR-compatible system. The XiaoR platform's pre-integrated hardware makes it an excellent testing ground for navigation algorithms, but custom motor control and ROS integration code must be developed.

**Current Status**: Experimental / Proof of Concept
**Readiness Level**: 15% (LiDAR working, motors and navigation not implemented)
**Recommended Use**: Hardware evaluation and algorithm testing only

**Next Milestone**: Complete motor control implementation and basic teleoperation before advancing to SLAM integration.

---

*Document created: 2026-01-11*
*Last updated: 2026-01-11*
*Author: Development Team*
*Project: WayfindR Indoor Navigation System*
