# Old Stuff Archive - Scope Documentation

## Purpose
This folder archives deprecated and experimental code from the early development phases of WayfindR-driver. It contains three main categories of historical implementations that have been superseded by more robust solutions.

## Folder Structure

### 1. `motors_telemetry/` - Early Motor Control & Sensor Experiments

#### Files:
- **skid_steer_l298n.py** - Basic skid-steer robot control using L298N motor drivers
- **skid_track_LM393.py** - Extended version with LM393 encoder tracking for odometry
- **mpu6050_rpi.py** - MPU6050 IMU sensor integration for gyroscope/accelerometer data
- **sensor_telem.ino** - Arduino-based motor control with MPU6050 yaw tracking

#### What They Were For:
Early experiments with motor control interfaces using pigpio library directly. These were proof-of-concept implementations testing different hardware configurations (L298N drivers, LM393 encoders, MPU6050 IMU).

#### Why Deprecated:
- **Replaced by**: `/PI_API/services/motor_driver.py` - A more robust, production-ready motor driver service
- **Issues with old code**:
  - No proper error handling or state management
  - Hardcoded GPIO pin configurations
  - Synchronous blocking code (not async-friendly)
  - No abstraction layer for different motor driver types
  - Mixed hardware interfaces (some on Pi GPIO, some on Arduino)
  - Basic PWM control without advanced features

#### Current Implementation:
The new `motor_driver.py` provides:
- Async/await support for FastAPI integration
- Simulation mode for testing on non-Pi systems
- Clean abstraction with both GPIO and Serial variants
- PWM-based speed control with emergency stop capability
- Better resource management and cleanup

#### Should Keep or Delete?
**KEEP** - These files demonstrate the evolution of the motor control system and contain useful hardware configuration notes (pin mappings, wiring diagrams in comments). They may be valuable reference when troubleshooting hardware issues or supporting different motor driver variants.

#### Salvageable Components:
- Pin mapping configurations and wiring diagrams in comments
- Encoder calculation formulas (pulses per revolution, wheel circumference constants)
- MPU6050 yaw integration logic (could be useful if IMU is added back)
- Arduino serial protocol design pattern

---

### 2. `rplidar_setup/` - LiDAR Testing & Visualization Scripts

#### Files:
- **0.1-list_ports.py** - Serial port detection utility
- **0.2-RPLidar_info.py** - Device information and health status reader
- **0.3-RPLIDAR_meas.py** - Basic scan data acquisition
- **0.4-RPLIDAR_plot.py** - Real-time polar plot visualization with matplotlib

#### What They Were For:
Testing and validation scripts for the RPLidar sensor. Used during initial hardware setup to verify LiDAR functionality, visualize scan data, and troubleshoot connection issues. The numbering (0.1-0.4) suggests a step-by-step setup workflow.

#### Why Deprecated:
- **Replaced by**: ROS2 integration in `/ros2_comprehensive_attempt/` with proper launch files
- **Issues with old code**:
  - Standalone scripts not integrated with navigation system
  - Direct adafruit-rplidar library usage (now using ROS2 rplidar driver)
  - Manual device path configuration for different OS platforms
  - Visualization blocking the main thread
  - No integration with SLAM or mapping systems

#### Current Implementation:
LiDAR is now integrated through:
- ROS2 rplidar_ros package (`/ros2_comprehensive_attempt/config/lidar_params.yaml`)
- Proper launch file integration (`/ros2_comprehensive_attempt/launch/slam.launch.py`)
- Cartographer SLAM for real-time mapping
- No need for standalone visualization (RViz2 handles this)

#### Should Keep or Delete?
**KEEP** - These are valuable diagnostic tools. When LiDAR hardware fails or needs debugging, these standalone scripts are much faster to run than spinning up entire ROS2 stack. The visualization script is particularly useful for verifying sensor functionality.

#### Salvageable Components:
- Port detection logic (works across Windows/Linux/macOS)
- Device health check procedure
- Real-time visualization with point lifetime/accumulation (good for demos)
- Polar plot setup optimizations for performance

---

### 3. `setup_scripts/` - Installation & Environment Setup Scripts

#### Files:
- **setup_ros.sh** - ROS Melodic + Cartographer setup for Raspberry Pi
- **setup_desktop.sh** - Minimal cartographer-rviz installation
- **setup_raspbian.sh** - Combined setup with systemd service creation
- **setup_rpi_cartographer.sh** - Comprehensive RPi setup with gbot_core
- **setup_desktop_cartographer.sh** - Desktop visualization setup with network configuration

#### What They Were For:
Automated installation scripts for setting up ROS Melodic-based SLAM mapping system. These scripts configured:
- ROS Melodic environment (Ubuntu 18.04)
- Cartographer SLAM packages
- RPLidar drivers
- Network configuration for multi-machine ROS setups
- Systemd service for auto-start
- Catkin workspace with gbot_core repository

#### Why Deprecated:
- **Replaced by**: ROS2 Humble migration
- **Current setup**: `/ros2_comprehensive_attempt/scripts/install_dependencies.sh`
- **Major version change**:
  - ROS Melodic (2018) → ROS2 Humble (2022)
  - Ubuntu 18.04 → Ubuntu 22.04
  - Old Cartographer → Nav2 + SLAM Toolbox
  - Catkin build system → Colcon

#### Issues with Old Scripts:
- Hardcoded for obsolete ROS1/Melodic
- Relied on external GitHub repo (gbot_core) that may no longer be maintained
- Used deprecated catkin_make build system
- Network setup for ROS_MASTER_URI not needed in ROS2 (uses DDS)
- Aggressive cron job that reboots on internet loss (problematic for development)
- Downloads requirements.txt from external source (security concern)
- Overly permissive chmod 777 on system directories

#### Current Implementation:
The project has migrated to ROS2 Humble with:
- Modern ROS2 infrastructure (`/ros2_comprehensive_attempt/`)
- Nav2 for navigation
- SLAM Toolbox or Cartographer for mapping
- Proper launch files instead of roslaunch
- `/ros2_install_attempt/docs/ROS2_HUMBLE_INSTALLATION.md` for setup docs

#### Should Keep or Delete?
**DELETE CANDIDATES** - These scripts are for a completely different ROS version and could confuse future developers. However, some legacy systems might still be running ROS Melodic.

**Recommendation**: Keep one reference script with clear documentation that it's for ROS1/Melodic only, delete the rest. The network configuration patterns (WiFi IP detection, multi-machine setup) could be useful reference.

#### Salvageable Components:
- WiFi IP detection logic (works across interfaces)
- udev rule creation for RPLidar device symlinks
- Systemd service template structure
- Multi-machine ROS networking patterns (even though ROS2 DDS is different, the concept is similar)

---

## Overall Assessment

### Technology Stack Evolution

**Old Stack (archived in old_stuff/):**
- ROS Melodic (ROS1)
- Python 2/3 transitional code
- Direct pigpio GPIO control
- Adafruit RPLidar library
- Standalone Python scripts
- Catkin build system
- Ubuntu 18.04

**Current Stack:**
- ROS2 Humble
- Python 3.10+
- FastAPI-based PI_API service
- ROS2 rplidar_ros package
- Integrated navigation system
- Colcon build system
- Ubuntu 22.04

### Deprecation Timeline Inference

Based on file structure and README references, the deprecation likely happened during:
1. **Phase 1**: Move from direct GPIO scripts to API-based control (motors_telemetry)
2. **Phase 2**: Integration of LiDAR into ROS ecosystem (rplidar_setup)
3. **Phase 3**: ROS1 to ROS2 migration (setup_scripts)

### Why Keep This Folder?

1. **Historical Context**: Shows the project's evolution from simple scripts to complex ROS2 system
2. **Hardware Reference**: Contains pin mappings, wiring diagrams, and hardware constants
3. **Debugging Tools**: Standalone LiDAR scripts useful for hardware troubleshooting
4. **Educational Value**: Good examples of different approaches to same problems
5. **Legacy System Support**: Some robots might still run older software versions

### Why Delete This Folder?

1. **Confusion Risk**: New developers might use deprecated code
2. **Maintenance Burden**: Outdated dependencies and Python versions
3. **Security Concerns**: Old scripts have poor security practices (chmod 777, external downloads)
4. **Disk Space**: Minimal, but adds clutter
5. **Outdated Dependencies**: ROS Melodic, Python 2 compatibility code

---

## Recommendations

### Keep With Documentation
Create a clear `README.md` in this folder warning that all code is deprecated and listing what replaced each component. This prevents accidental use while preserving history.

### Archive Further
Move to a separate git branch called `archive/ros1-melodic` or `archive/early-development` to remove from main branch while preserving in git history.

### Extract Useful Parts
Consider extracting these specific components into the current codebase:

1. **Motor Encoder Constants**: From `skid_track_LM393.py`
   - Wheel circumference
   - Pulses per revolution
   - Could add odometry to current motor_driver.py

2. **LiDAR Diagnostic Tools**: From `rplidar_setup/`
   - Create `/tools/diagnostics/lidar_check.py` for hardware testing
   - Port the visualization to work with ROS2 topics

3. **Hardware Setup Docs**: From setup scripts
   - Extract pin mappings and wiring diagrams
   - Create `/docs/hardware_configuration.md`

4. **Multi-Machine Network Setup**: From cartographer scripts
   - Document ROS2 DDS multi-machine configuration
   - Add to `/docs/network_setup.md`

### Delete Safely
If deleting:
1. Ensure all useful information is documented elsewhere
2. Tag the last commit with folder as `pre-cleanup-archive`
3. Add notes to main README about where archived code exists in git history

---

## Final Recommendation: **KEEP WITH CLEAR WARNINGS**

**Action Items:**
1. Add `README.md` to this folder with deprecation warnings
2. Add comments to each file explaining what replaced it
3. Consider moving to git branch if folder becomes too cluttered
4. Extract hardware constants and diagnostic tools into current codebase
5. Document hardware configurations from comments into proper docs

The historical and diagnostic value outweighs the minor confusion risk, especially with proper documentation.
