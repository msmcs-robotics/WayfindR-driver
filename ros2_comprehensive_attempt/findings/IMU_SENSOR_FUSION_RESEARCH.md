# IMU (MPU6050) Sensor Fusion Integration Research

**Created:** 2026-01-11
**Version:** 1.0.0
**Status:** Research Complete
**Target Platform:** ROS2 Humble on Ubuntu 22.04 / Raspberry Pi

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [MPU6050 Hardware Specifications](#mpu6050-hardware-specifications)
3. [Hardware Integration with Raspberry Pi](#hardware-integration-with-raspberry-pi)
4. [ROS2 Driver Options](#ros2-driver-options)
5. [Sensor Fusion Architecture](#sensor-fusion-architecture)
6. [robot_localization Package Configuration](#robot_localization-package-configuration)
7. [EKF Configuration and Tuning](#ekf-configuration-and-tuning)
8. [IMU Calibration Procedures](#imu-calibration-procedures)
9. [Integration with Existing System](#integration-with-existing-system)
10. [Testing and Validation](#testing-and-validation)
11. [Implementation Roadmap](#implementation-roadmap)
12. [References](#references)

---

## Executive Summary

This document provides comprehensive research and implementation guidance for integrating an MPU6050 IMU sensor with the WayfindR robot navigation system. The integration will enhance localization accuracy by fusing:

- **Wheel Encoders** (via cmd_vel_bridge odometry) - Linear velocity and position
- **IMU MPU6050** - Angular velocity and orientation
- **LiDAR** (via AMCL) - Global pose correction

The recommended architecture uses the `robot_localization` package with Extended Kalman Filter (EKF) for sensor fusion, producing a filtered odometry estimate that combines the strengths of each sensor while minimizing their individual weaknesses.

### Key Benefits

1. **Improved Rotation Estimation**: IMU provides accurate angular velocity, eliminating reliance on dead-reckoning for rotation
2. **Reduced Drift**: Sensor fusion corrects for wheel slippage and encoder errors
3. **Smoother Navigation**: Continuous, high-rate odometry (50-100 Hz) for better controller performance
4. **AMCL Enhancement**: More accurate local odometry improves AMCL convergence and stability

### System Requirements

- **Hardware**: MPU6050 IMU module (~$5), I2C connection to Raspberry Pi
- **Software**: ROS2 Humble, robot_localization package, IMU driver
- **Calibration**: One-time IMU calibration procedure (~10 minutes)
- **Integration**: Configuration of EKF parameters and coordinate frames

---

## MPU6050 Hardware Specifications

### Overview

The **MPU-6050** is a 6-axis Motion Processing Unit that combines a 3-axis gyroscope and 3-axis accelerometer on a single silicon die, manufactured by InvenSense (now TDK).

**Key Features:**
- **Digital Motion Processor (DMP)**: Onboard processing for sensor fusion algorithms
- **Small Form Factor**: 4x4x0.9mm QFN package
- **Low Power**: Suitable for battery-operated robots
- **I2C Interface**: Easy integration with Raspberry Pi and microcontrollers
- **Cost-Effective**: ~$3-5 USD per module

### Gyroscope Specifications

| Parameter | Specification |
|-----------|---------------|
| **Full-Scale Range** | ±250, ±500, ±1000, ±2000 °/sec (user-programmable) |
| **Sensitivity** | 131 LSB/(°/s) at ±250°/s<br>65.5 LSB/(°/s) at ±500°/s<br>32.8 LSB/(°/s) at ±1000°/s<br>16.4 LSB/(°/s) at ±2000°/s |
| **Output Data Rate** | Up to 8 kHz |
| **ADC Resolution** | 16-bit |
| **Nonlinearity** | ±0.2% of full scale |
| **Cross-Axis Sensitivity** | ±2% |

**Recommended Setting for Robotics:**
- **Range**: ±250°/s (sufficient for most mobile robots, max sensitivity)
- **Rate**: 100-200 Hz for navigation applications

### Accelerometer Specifications

| Parameter | Specification |
|-----------|---------------|
| **Full-Scale Range** | ±2g, ±4g, ±8g, ±16g (user-programmable) |
| **Sensitivity** | 16,384 LSB/g at ±2g<br>8,192 LSB/g at ±4g<br>4,096 LSB/g at ±8g<br>2,048 LSB/g at ±16g |
| **Output Data Rate** | Up to 1 kHz |
| **ADC Resolution** | 16-bit |
| **Nonlinearity** | ±0.5% of full scale |

**Recommended Setting for Robotics:**
- **Range**: ±2g (sufficient for smooth mobile robot motion, max sensitivity)
- **Rate**: 100-200 Hz

### Temperature Sensor

- **Range**: -40°C to +85°C
- **Sensitivity**: 340 LSB/°C
- **Accuracy**: ±1°C typical
- **Use Case**: Compensate for temperature drift in gyro/accel readings

### Digital Motion Processor (DMP)

The DMP is an onboard coprocessor that can:
- Perform sensor fusion (6-axis orientation)
- Calculate quaternion output
- Execute motion processing algorithms
- Offload computation from host processor

**Note**: Many ROS2 implementations use external filters (Madgwick, Complementary) instead of DMP for better configurability.

### Electrical Specifications

| Parameter | Value |
|-----------|-------|
| **Operating Voltage (VDD)** | 2.375V - 3.46V |
| **Logic Voltage (VDDIO)** | 1.71V - VDD |
| **Supply Current** | 3.9 mA (gyro + accel mode) |
| **I2C Clock Speed** | Up to 400 kHz (Fast Mode) |
| **I2C Address** | 0x68 (AD0 low), 0x69 (AD0 high) |

---

## Hardware Integration with Raspberry Pi

### Wiring Connections

The MPU6050 communicates with the Raspberry Pi via the I2C bus. Most modules operate at 3.3V and include voltage regulation for 5V compatibility.

#### Pin Connections

| MPU6050 Pin | Raspberry Pi Pin | GPIO | Description |
|-------------|------------------|------|-------------|
| **VCC** | Pin 1 or 17 | 3.3V | Power supply (3.3V recommended) |
| **GND** | Pin 6, 9, 14, 20, 25, 30, 34, 39 | GND | Ground reference |
| **SCL** | Pin 5 | GPIO 3 (SCL1) | I2C Clock |
| **SDA** | Pin 3 | GPIO 2 (SDA1) | I2C Data |
| **XDA** | Not connected | - | Auxiliary I2C (optional) |
| **XCL** | Not connected | - | Auxiliary I2C (optional) |
| **AD0** | GND or 3.3V | - | I2C address select |
| **INT** | Optional | GPIO pin | Interrupt output (optional) |

#### Wiring Diagram (Text)

```
Raspberry Pi                    MPU6050 Module
┌─────────────┐                ┌──────────────┐
│             │                │              │
│  Pin 1 (3.3V)───────────────▶│ VCC          │
│             │                │              │
│  Pin 3 (SDA)◀───────────────▶│ SDA          │
│             │                │              │
│  Pin 5 (SCL)◀───────────────▶│ SCL          │
│             │                │              │
│  Pin 6 (GND)───────────────▶│ GND          │
│             │                │              │
└─────────────┘                └──────────────┘
```

### Important Hardware Notes

#### Voltage Levels

- **3.3V Operation**: The Raspberry Pi GPIO operates at 3.3V. Always use 3.3V for VCC.
- **5V Tolerance**: Some MPU6050 modules have onboard voltage regulators and can accept 5V on VCC, but 3.3V is safer for GPIO protection.
- **DO NOT** connect 5V directly to SDA/SCL - this can damage the Raspberry Pi GPIO.

#### I2C Pull-up Resistors

- The Raspberry Pi has built-in **1.8 kΩ pull-up resistors** on SDA and SCL to 3.3V.
- Most MPU6050 modules also have pull-ups (typically 4.7 kΩ or 10 kΩ).
- Multiple pull-ups in parallel may result in ~1.2 kΩ equivalent resistance - this is acceptable for short I2C cables (<30cm).
- For long cables or multiple I2C devices, you may need to adjust pull-up values.

#### I2C Address Configuration

- **Default Address**: 0x68 (when AD0 pin is grounded or left floating)
- **Alternate Address**: 0x69 (when AD0 pin is connected to VCC/3.3V)
- Use alternate address to avoid conflicts if using multiple MPU6050 sensors

### Raspberry Pi I2C Setup

#### Enable I2C Interface

```bash
# Method 1: Using raspi-config
sudo raspi-config
# Navigate to: 3 Interface Options → I5 I2C → Enable

# Method 2: Edit config.txt
sudo nano /boot/config.txt
# Add or uncomment:
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000  # Set to 400 kHz (Fast Mode)

# Reboot
sudo reboot
```

#### Install I2C Tools

```bash
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus
```

#### Verify I2C Connection

```bash
# List I2C buses
ls /dev/i2c*
# Should show: /dev/i2c-1

# Scan for I2C devices
sudo i2cdetect -y 1

# Expected output (MPU6050 at 0x68):
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
# 70: -- -- -- -- -- -- -- --
```

If you see `68` (or `69`), the MPU6050 is successfully connected.

### Mounting Considerations

#### Physical Mounting

- **Orientation**: Mount IMU with axes aligned to robot coordinate frame
  - X-axis: Forward (robot front)
  - Y-axis: Left (robot left side)
  - Z-axis: Up (perpendicular to ground)
- **Vibration Damping**: Use foam or rubber mounts to reduce motor/wheel vibrations
- **Proximity to Motors**: Keep IMU away from motors and power cables to reduce electromagnetic interference
- **Secure Mounting**: Ensure rigid mounting - any flex or rotation invalidates measurements

#### Coordinate Frame Alignment

```
Robot Body Frame (REP 103)          MPU6050 Orientation
       ↑ Z (up)                           ↑ Z
       |                                  |
       |___→ X (forward)                  |___→ X
      ╱                                  ╱
     ╱ Y (left)                         ╱ Y
```

If IMU is mounted in a different orientation, you'll need to apply a static transform in software (covered later).

---

## ROS2 Driver Options

Several ROS2 packages are available for interfacing with the MPU6050. This section compares the options and provides installation instructions.

### Option 1: ros2_mpu6050_driver (Recommended)

**Repository**: https://github.com/hiwad-aziz/ros2_mpu6050_driver

#### Features

- ✓ Native ROS2 implementation (Python)
- ✓ Publishes `sensor_msgs/Imu` to `/imu` topic
- ✓ Configurable I2C address and bus
- ✓ Bias calibration support
- ✓ Configurable gyro/accel ranges
- ✓ Tested with ROS2 Humble

#### Installation

```bash
# Navigate to ROS2 workspace
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git

# Install dependencies
sudo apt-get install -y python3-smbus2

# Build package
cd ~/ros2_ws
colcon build --packages-select ros2_mpu6050_driver

# Source workspace
source install/setup.bash
```

#### Configuration

Create a configuration file at `config/mpu6050_params.yaml`:

```yaml
mpu6050_node:
  ros__parameters:
    i2c_bus: 1                  # I2C bus number (1 for Raspberry Pi)
    i2c_address: 0x68           # Default I2C address
    frame_id: "imu_link"        # IMU coordinate frame
    publish_rate: 100.0         # Hz (50-200 recommended)

    # Gyroscope configuration
    gyro_range: 0               # 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s

    # Accelerometer configuration
    accel_range: 0              # 0=±2g, 1=±4g, 2=±8g, 3=±16g

    # Calibration offsets (run calibration first)
    gyro_x_offset: 0.0
    gyro_y_offset: 0.0
    gyro_z_offset: 0.0
    accel_x_offset: 0.0
    accel_y_offset: 0.0
    accel_z_offset: 0.0
```

#### Launch

```bash
ros2 run ros2_mpu6050_driver mpu6050_node --ros-args \
    --params-file config/mpu6050_params.yaml
```

### Option 2: ros2-mpu6050 (JCorbin406)

**Repository**: https://github.com/JCorbin406/ros2-mpu6050

#### Features

- ✓ Tested on Raspberry Pi with ROS2 Jazzy
- ✓ Basic calibration support
- ✓ Configurable publishing rates
- ✓ Clean Python implementation

#### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/JCorbin406/ros2-mpu6050.git
cd ~/ros2_ws
colcon build --packages-select mpu6050_driver
source install/setup.bash
```

### Option 3: ros2_imu_mpu6050 (C++ Implementation)

**Repository**: https://github.com/kimsniper/ros2_mpu6050

#### Features

- ✓ C++ implementation (better performance)
- ✓ Lower CPU usage
- ✓ Direct I2C access via i2cdev

#### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/kimsniper/ros2_mpu6050.git
cd ~/ros2_ws
colcon build --packages-select ros2_mpu6050
source install/setup.bash
```

### Option 4: imu_tools with Generic I2C Driver

**Repository**: https://github.com/CCNYRoboticsLab/imu_tools

This approach uses a simple I2C reader to publish raw IMU data, then uses `imu_filter_madgwick` or `imu_complementary_filter` for orientation estimation.

#### Features

- ✓ Mature, well-tested filters
- ✓ Support for magnetometer (if added)
- ✓ Available for all ROS2 distributions
- ✓ Good documentation

#### Installation

```bash
# Install imu_tools
sudo apt-get install ros-humble-imu-tools

# Or build from source
cd ~/ros2_ws/src
git clone -b humble https://github.com/CCNYRoboticsLab/imu_tools.git
cd ~/ros2_ws
colcon build --packages-select imu_filter_madgwick imu_complementary_filter
```

### Comparison Matrix

| Feature | ros2_mpu6050_driver | ros2-mpu6050 | ros2_imu_mpu6050 (C++) | imu_tools |
|---------|-------------------|--------------|----------------------|-----------|
| **Language** | Python | Python | C++ | C++/Python |
| **CPU Usage** | Medium | Medium | Low | Low |
| **Easy Setup** | ★★★★★ | ★★★★☆ | ★★★☆☆ | ★★★★☆ |
| **Calibration** | Built-in | Built-in | Manual | External |
| **Orientation** | Raw only | Raw only | Raw only | Computed |
| **ROS2 Humble** | ✓ | ✓ | ✓ | ✓ |
| **Documentation** | Good | Basic | Limited | Excellent |

### Recommendation

**For WayfindR Project**: Use **ros2_mpu6050_driver** + **imu_filter_madgwick**

This combination provides:
1. Easy setup and configuration
2. Good documentation
3. Orientation filtering (if needed for future use)
4. Integration with `robot_localization`

---

## Sensor Fusion Architecture

### Overview

The sensor fusion architecture combines three data sources to produce accurate robot localization:

1. **Local Odometry**: Wheel encoders + IMU → EKF → `/odometry/filtered`
2. **Global Localization**: AMCL using LiDAR scans → `map → odom` transform
3. **Navigation**: Nav2 uses fused odometry for planning and control

### Coordinate Frame Hierarchy

```
map (global, fixed)
 └─ odom (local, continuous, drifts over time)
     └─ base_link (robot center)
         ├─ imu_link (IMU sensor frame)
         └─ laser (LiDAR sensor frame)
```

### Transform Responsibilities

| Transform | Published By | Rate | Characteristics |
|-----------|-------------|------|-----------------|
| `map → odom` | AMCL | 1-5 Hz | Discontinuous, globally accurate, corrects drift |
| `odom → base_link` | robot_localization (EKF) | 50 Hz | Continuous, smooth, locally accurate |
| `base_link → imu_link` | static_transform_publisher | Once | Static, describes IMU mounting position |
| `base_link → laser` | static_transform_publisher | Once | Static, describes LiDAR mounting position |

### Data Flow Diagram

```
┌─────────────────┐        ┌──────────────────┐        ┌─────────────────┐
│  Wheel Encoders │        │    MPU6050 IMU   │        │  RPLidar Laser  │
│  (cmd_vel_bridge)│        │  (mpu6050_driver)│        │   (rplidar_ros) │
└────────┬────────┘        └─────────┬────────┘        └────────┬────────┘
         │                           │                          │
         │ /odom (nav_msgs/Odometry) │ /imu (sensor_msgs/Imu)   │ /scan
         │                           │                          │
         └──────────┬────────────────┘                          │
                    ↓                                           │
         ┌─────────────────────────┐                            │
         │  robot_localization     │                            │
         │     (ekf_node)          │                            │
         │                         │                            │
         │  Fuses: /odom + /imu    │                            │
         └────────────┬────────────┘                            │
                      │                                         │
                      │ /odometry/filtered                      │
                      │ TF: odom → base_link                    │
                      │                                         │
                      ↓                                         │
         ┌────────────────────────┐                             │
         │        AMCL            │◄────────────────────────────┘
         │  (particle filter)     │
         │                        │
         │  Uses: /odometry/filtered (motion model)
         │        /scan (sensor model)
         └───────────┬────────────┘
                     │
                     │ TF: map → odom
                     │ /amcl_pose
                     │
                     ↓
         ┌────────────────────────┐
         │       Nav2 Stack       │
         │  - Planner             │
         │  - Controller          │
         │  - Behavior Server     │
         └───────────┬────────────┘
                     │
                     │ /cmd_vel
                     ↓
         ┌────────────────────────┐
         │   cmd_vel_bridge       │
         │   → Motor Control      │
         └────────────────────────┘
```

### Sensor Characteristics and Fusion Strategy

#### Wheel Odometry (from cmd_vel_bridge)

**Strengths:**
- Good short-term linear velocity estimation
- Direct measurement of robot motion commands
- High update rate (50 Hz)

**Weaknesses:**
- Accumulates drift over time (dead reckoning)
- Sensitive to wheel slippage
- Poor rotation estimation (depends on wheelbase accuracy)
- No absolute position reference

**EKF Configuration:**
- Fuse: `vx` (linear velocity X), `yaw` (differential mode)
- High confidence for linear motion
- Lower confidence for rotation

#### IMU (MPU6050)

**Strengths:**
- Excellent angular velocity measurement
- Not affected by wheel slippage
- High update rate (100-200 Hz)
- Provides gravity-referenced orientation (when filtered)

**Weaknesses:**
- Gyroscope drift over time
- Accelerometer noise during motion
- No absolute position reference
- Sensitive to vibrations

**EKF Configuration:**
- Fuse: `yaw`, `vyaw` (angular velocity Z)
- High confidence for rotation
- Reject linear acceleration (too noisy on mobile robots)

#### LiDAR + AMCL

**Strengths:**
- Global position correction
- Eliminates long-term drift
- Accurate in structured environments

**Weaknesses:**
- Lower update rate (1-5 Hz)
- Can jump/reset position (kidnapped robot problem)
- Requires known map

**Integration:**
- AMCL consumes `/odometry/filtered` for motion model
- AMCL publishes `map → odom` transform
- Corrects accumulated drift from local sensors

### Single EKF vs Dual EKF Architecture

#### Single EKF (Recommended for WayfindR)

**Configuration:**
- One `ekf_node` fuses wheel odometry + IMU
- Publishes `/odometry/filtered`
- Publishes `odom → base_link` transform
- AMCL publishes `map → odom` transform separately

**Advantages:**
- Simpler configuration
- Lower CPU usage
- Suitable for indoor navigation with reliable maps

**Use When:**
- Operating primarily with AMCL localization
- Indoor environment with good map
- Single sensor suite (no GPS or external positioning)

#### Dual EKF (For Advanced Applications)

**Configuration:**
- **EKF Local**: Fuses wheel odometry + IMU → `/odometry/local` → `odom → base_link`
- **EKF Global**: Fuses local odometry + GPS/external positioning → `/odometry/global` → `map → odom`

**Advantages:**
- Better handling of GPS integration
- Separation of local continuous and global discontinuous estimates
- Smoother transitions when global corrections occur

**Use When:**
- Adding GPS for outdoor navigation
- Multi-floor buildings with map switching
- Need both continuous local and corrected global odometry

**Recommendation**: Start with **Single EKF** for the WayfindR project. The architecture can be upgraded to Dual EKF later if GPS or multi-floor navigation is added.

---

## robot_localization Package Configuration

### Installation

```bash
# Install from apt (recommended)
sudo apt-get install ros-humble-robot-localization

# Or build from source
cd ~/ros2_ws/src
git clone -b humble https://github.com/cra-ros-pkg/robot_localization.git
cd ~/ros2_ws
colcon build --packages-select robot_localization
source install/setup.bash
```

### EKF Node Overview

The `ekf_node` is the core of `robot_localization`. It implements an Extended Kalman Filter that:

- Fuses data from multiple odometry sources (wheel encoders, IMU, GPS, etc.)
- Publishes filtered odometry at a fixed rate
- Optionally publishes TF transforms
- Handles sensors with different update rates
- Supports differential integration to prevent oscillations

### State Vector

The EKF estimates a 15-element state vector:

| Index | Variable | Description | Units |
|-------|----------|-------------|-------|
| 0 | X | Position X | m |
| 1 | Y | Position Y | m |
| 2 | Z | Position Z | m |
| 3 | roll | Rotation around X | rad |
| 4 | pitch | Rotation around Y | rad |
| 5 | yaw | Rotation around Z (heading) | rad |
| 6 | vx | Linear velocity X | m/s |
| 7 | vy | Linear velocity Y | m/s |
| 8 | vz | Linear velocity Z | m/s |
| 9 | vroll | Angular velocity X | rad/s |
| 10 | vpitch | Angular velocity Y | rad/s |
| 11 | vyaw | Angular velocity Z | rad/s |
| 12 | ax | Linear acceleration X | m/s² |
| 13 | ay | Linear acceleration Y | m/s² |
| 14 | az | Linear acceleration Z | m/s² |

### Configuration File Structure

Create `config/robot_localization_params.yaml`:

```yaml
### ekf_node configuration for WayfindR robot
### Fuses wheel odometry (/odom) + IMU (/imu) → /odometry/filtered

ekf_filter_node:
  ros__parameters:
    # ========================================
    # Fundamental Settings
    # ========================================
    frequency: 50.0                  # Filter update rate (Hz)
    sensor_timeout: 0.1              # Discard measurements older than this (s)
    two_d_mode: true                 # Constrain to 2D (ignore Z, roll, pitch)

    # ========================================
    # Coordinate Frames
    # ========================================
    map_frame: map                   # Global fixed frame (published by AMCL)
    odom_frame: odom                 # Local continuous frame
    base_link_frame: base_link       # Robot body frame
    world_frame: odom                # Frame for filtered odometry

    # ========================================
    # Publication Settings
    # ========================================
    publish_tf: true                 # Publish odom → base_link transform
    publish_acceleration: false      # Don't publish acceleration (too noisy)

    # ========================================
    # Input 0: Wheel Odometry (from cmd_vel_bridge)
    # ========================================
    odom0: /odom
    odom0_config: [false, false, false,  # X, Y, Z position (don't fuse position - drifts)
                   false, false, false,  # roll, pitch, yaw orientation (don't fuse - use IMU)
                   true,  false, false,  # vx, vy, vz (FUSE vx - wheel velocity)
                   false, false, false,  # vroll, vpitch, vyaw (don't fuse - use IMU)
                   false, false, false]  # ax, ay, az (don't fuse - too noisy)

    odom0_differential: false        # Use absolute measurements
    odom0_relative: false            # Don't treat as relative
    odom0_queue_size: 10             # Message queue size

    # Reject outliers beyond N standard deviations
    odom0_rejection_threshold: 5.0   # Mahalanobis distance threshold

    # ========================================
    # Input 1: IMU (MPU6050)
    # ========================================
    imu0: /imu
    imu0_config: [false, false, false,  # X, Y, Z position (IMU doesn't measure position)
                  false, false, true,   # roll, pitch, yaw (FUSE yaw - heading from gyro integration)
                  false, false, false,  # vx, vy, vz (don't fuse - use wheel odometry)
                  false, false, true,   # vroll, vpitch, vyaw (FUSE vyaw - angular velocity)
                  false, false, false]  # ax, ay, az (don't fuse - too noisy on mobile robots)

    imu0_differential: false         # Use absolute orientation
    imu0_relative: false             # Don't treat as relative
    imu0_queue_size: 10              # Message queue size
    imu0_remove_gravitational_acceleration: true  # Remove gravity from accelerometer

    # Reject outliers
    imu0_rejection_threshold: 5.0

    # ========================================
    # Process Noise Covariance (Q matrix)
    # ========================================
    # Diagonal values for state vector (15 elements)
    # Higher values = trust process model less, trust measurements more
    # Lower values = trust process model more, trust measurements less

    process_noise_covariance: [0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # X
                               0.0,   0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # Y
                               0.0,   0.0,   0.06,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # Z
                               0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # roll
                               0.0,   0.0,   0.0,   0.0,   0.03,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # pitch
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.06,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # yaw
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # vx
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.025, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # vy
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.04,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,     # vz
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,   0.0,     # vroll
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,   0.0,   0.0,     # vpitch
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.02,  0.0,   0.0,   0.0,     # vyaw
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,   0.0,     # ax
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.01,  0.0,     # ay
                               0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.015]   # az

    # ========================================
    # Initial State Covariance (P matrix)
    # ========================================
    # Uncertainty of initial state estimate
    # Start with higher values to allow filter to converge quickly

    initial_estimate_covariance: [1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # X
                                  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # Y
                                  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # Z
                                  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # roll
                                  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # pitch
                                  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # yaw
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # vx
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # vy
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,      # vz
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,      # vroll
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,      # vpitch
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,      # vyaw
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,      # ax
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,      # ay
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9]     # az
```

### Launch File

Create `launch/robot_localization.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch file for robot_localization EKF node.
Fuses wheel odometry + IMU for improved pose estimation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ros2_comprehensive_attempt')

    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'robot_localization_params.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node
    ])
```

---

## EKF Configuration and Tuning

### Understanding Covariance Matrices

The EKF uses three covariance matrices:

1. **Process Noise Covariance (Q)**: Uncertainty in the motion model
2. **Measurement Covariance (R)**: Uncertainty in sensor measurements
3. **State Covariance (P)**: Uncertainty in state estimate

### Sensor Covariance Configuration

#### Wheel Odometry Covariance

The `cmd_vel_bridge` already publishes covariance in the odometry message:

```python
# In cmd_vel_bridge.py (already implemented)
odom_msg.pose.covariance = [
    self.pose_cov_x, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
    0.0, self.pose_cov_y, 0.0, 0.0, 0.0, 0.0,  # y
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,              # z
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,              # roll
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,              # pitch
    0.0, 0.0, 0.0, 0.0, 0.0, self.pose_cov_yaw # yaw
]

# Default values from cmd_vel_bridge_params.yaml
pose_covariance_x: 0.1      # 10 cm std dev
pose_covariance_y: 0.1      # 10 cm std dev
pose_covariance_yaw: 0.5    # ~28° std dev (high - dead reckoning is poor for rotation)
```

**Tuning Recommendation**:
- Keep `pose_covariance_x/y` at **0.1-0.2** (decent for short distances)
- Increase `pose_covariance_yaw` to **0.5-1.0** (rotation is unreliable without encoders)

#### IMU Covariance

Most ROS2 IMU drivers don't compute accurate covariance - you'll need to measure it.

**Method 1: Static Measurement**

```bash
# Record 1000 samples with IMU stationary
ros2 topic echo --once /imu | grep -A 3 "angular_velocity:"

# Calculate standard deviation of measurements
# Repeat 1000 times, calculate σ for each axis
```

**Typical MPU6050 Values** (±250°/s range, after calibration):

```yaml
# Add to IMU driver config or override in robot_localization
imu0_angular_velocity_covariance: [0.0001, 0.0,    0.0,
                                   0.0,    0.0001, 0.0,
                                   0.0,    0.0,    0.0001]  # ~0.01 rad/s std dev

imu0_linear_acceleration_covariance: [0.01, 0.0,  0.0,
                                      0.0,  0.01, 0.0,
                                      0.0,  0.0,  0.01]     # ~0.1 m/s² std dev

imu0_orientation_covariance: [0.001, 0.0,   0.0,
                              0.0,   0.001, 0.0,
                              0.0,   0.0,   0.001]          # ~0.03 rad (~1.7°) std dev
```

**Method 2: Empirical Tuning**

Start conservative (high variance = low confidence), then reduce:

```yaml
# Start with these values
imu0_angular_velocity_covariance: [0.01, 0.0, 0.0,
                                   0.0, 0.01, 0.0,
                                   0.0, 0.0, 0.01]

# If EKF ignores IMU (too noisy), reduce covariance:
# Divide by 10 → 0.001

# If EKF over-trusts IMU (jumpy estimates), increase covariance:
# Multiply by 10 → 0.1
```

### Process Noise Covariance Tuning

Process noise represents uncertainty in the motion model (how much the robot state changes between updates).

**Tuning Strategy:**

1. **Start High** (trust measurements more than model):
   ```yaml
   process_noise_covariance: [0.1, 0.0, ... ]  # Diagonal values
   ```

2. **Observe Behavior**:
   - If odometry is **too jumpy** → Reduce process noise (trust model more)
   - If odometry **lags behind** actual motion → Increase process noise (trust measurements more)

3. **Final Values** (after tuning):
   ```yaml
   # Position noise: 0.05 m (moderate trust in motion model)
   # Velocity noise: 0.025 m/s (high trust - measurements are good)
   # Angular noise: 0.02 rad/s (high trust - IMU is accurate)
   ```

### Differential vs Absolute Integration

**Absolute Integration** (default):
- Fuses absolute measurements (e.g., IMU reports yaw = 1.57 rad)
- Suitable when sensor provides accurate absolute values

**Differential Integration**:
- Fuses change in measurement (e.g., Δyaw = 0.01 rad since last update)
- Prevents oscillations when two sensors report conflicting absolute values
- Set `odom0_differential: true` or `imu0_differential: true`

**When to Use Differential**:
- Both wheel odometry and IMU report yaw orientation
- Conflicting measurements cause filter oscillation
- Example: Set `odom0_differential: true` for yaw measurement

**WayfindR Configuration**:
- **Odometry yaw**: Don't fuse (unreliable without encoders)
- **IMU yaw**: Fuse as absolute (no conflict)
- Result: No need for differential mode

### Two-D Mode

For planar (2D) mobile robots:

```yaml
two_d_mode: true
```

This automatically:
- Sets Z, roll, pitch to 0
- Ignores measurements for 3D components
- Reduces computational complexity
- Prevents covariance explosion

**Always enable for WayfindR** (differential drive robot on flat ground).

### Common Tuning Issues and Solutions

| Issue | Symptom | Solution |
|-------|---------|----------|
| **Oscillating yaw** | Robot heading jumps back and forth | Enable `odom0_differential: true` for yaw |
| **Lagging odometry** | Filtered odom lags behind actual motion | Increase process noise covariance |
| **Jumpy odometry** | Filtered odom jumps/jitters | Decrease process noise covariance |
| **IMU ignored** | Filter doesn't use IMU data | Reduce IMU covariance (increase trust) |
| **Wheels ignored** | Filter doesn't use wheel data | Reduce odom covariance (increase trust) |
| **Covariance explosion** | State uncertainty grows unbounded | Enable `two_d_mode`, check for NaN in messages |
| **Slow convergence** | Takes long time to stabilize | Reduce `initial_estimate_covariance` |

### Debugging Tools

#### View Diagnostics

```bash
# Monitor EKF diagnostics
ros2 topic echo /diagnostics

# Expected status: "OK", no warnings
```

#### Plot Odometry Comparison

```bash
# Install plotjuggler
sudo apt-get install ros-humble-plotjuggler-ros

# Launch
ros2 run plotjuggler plotjuggler

# Stream topics:
# - /odom (raw wheel odometry)
# - /odometry/filtered (fused odometry)
# - /imu (IMU data)

# Compare: pose.pose.position.x, pose.pose.orientation.z
```

#### Check Transform Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Should show:
# map → odom → base_link → imu_link
#                       → laser
```

#### Monitor Update Rates

```bash
# Check sensor rates
ros2 topic hz /odom           # Should be ~50 Hz
ros2 topic hz /imu            # Should be 100-200 Hz
ros2 topic hz /odometry/filtered  # Should match EKF frequency (50 Hz)
```

---

## IMU Calibration Procedures

### Why Calibration is Necessary

The MPU6050, like all MEMS sensors, has manufacturing variations that introduce:

1. **Gyroscope Zero-Rate Offset**: Gyro reports rotation even when stationary
2. **Accelerometer Bias**: Accelerometer doesn't read exactly 1g when aligned with gravity
3. **Temperature Drift**: Offset changes with temperature (mitigated by temperature sensor)

Calibration measures these offsets and compensates for them in software.

### Calibration Types

#### 1. Zero-Rate Gyroscope Calibration (Essential)

**Purpose**: Measure gyroscope output when stationary, subtract this offset from all readings.

**Procedure**:

```bash
# 1. Place robot on level, stable surface
# 2. Ensure robot is completely stationary (no vibration)
# 3. Run calibration script

ros2 run ros2_mpu6050_driver calibrate_imu.py

# Sample output:
# Collecting 1000 samples...
# Gyro X offset: -0.034 rad/s
# Gyro Y offset: 0.021 rad/s
# Gyro Z offset: -0.015 rad/s
#
# Add to config:
# gyro_x_offset: -0.034
# gyro_y_offset: 0.021
# gyro_z_offset: -0.015
```

**Manual Calculation** (if no calibration script):

```python
#!/usr/bin/env python3
"""
Manual IMU gyro calibration.
Place IMU stationary, run this script.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class ImuCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        self.subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        self.samples = []
        self.num_samples = 1000

    def imu_callback(self, msg):
        if len(self.samples) < self.num_samples:
            gyro = [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]
            self.samples.append(gyro)

            if len(self.samples) % 100 == 0:
                self.get_logger().info(f'Collected {len(self.samples)} samples...')

        elif len(self.samples) == self.num_samples:
            # Calculate offsets
            samples_array = np.array(self.samples)
            offsets = np.mean(samples_array, axis=0)
            std_devs = np.std(samples_array, axis=0)

            self.get_logger().info('Calibration Complete!')
            self.get_logger().info(f'Gyro X offset: {offsets[0]:.6f} rad/s (σ={std_devs[0]:.6f})')
            self.get_logger().info(f'Gyro Y offset: {offsets[1]:.6f} rad/s (σ={std_devs[1]:.6f})')
            self.get_logger().info(f'Gyro Z offset: {offsets[2]:.6f} rad/s (σ={std_devs[2]:.6f})')
            self.get_logger().info('')
            self.get_logger().info('Add to mpu6050_params.yaml:')
            self.get_logger().info(f'  gyro_x_offset: {offsets[0]:.6f}')
            self.get_logger().info(f'  gyro_y_offset: {offsets[1]:.6f}')
            self.get_logger().info(f'  gyro_z_offset: {offsets[2]:.6f}')

            # Increment to stop further processing
            self.samples.append([0, 0, 0])


def main():
    rclpy.init()
    node = ImuCalibrator()

    print('IMU Gyroscope Calibration')
    print('=' * 40)
    print('Place IMU on level, stable surface.')
    print('Keep robot completely stationary.')
    print('Collecting 1000 samples...')
    print('')

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Save as `scripts/calibrate_imu_gyro.py`, run:

```bash
chmod +x scripts/calibrate_imu_gyro.py
ros2 run ros2_comprehensive_attempt calibrate_imu_gyro.py
```

#### 2. Accelerometer Calibration (Recommended)

**Purpose**: Measure accelerometer bias and scale factor.

**Simplified Procedure** (6-position calibration):

```bash
# Position 1: Z-axis up (normal orientation)
# Record 100 samples: should read (0, 0, +1g)

# Position 2: Z-axis down (upside down)
# Record 100 samples: should read (0, 0, -1g)

# Position 3: X-axis up
# Record 100 samples: should read (+1g, 0, 0)

# Position 4: X-axis down
# Record 100 samples: should read (-1g, 0, 0)

# Position 5: Y-axis up
# Record 100 samples: should read (0, +1g, 0)

# Position 6: Y-axis down
# Record 100 samples: should read (0, -1g, 0)

# Calculate offsets and scale factors
```

**Note**: Accelerometer calibration is less critical for robots that don't fuse linear acceleration. For WayfindR (only using gyro + orientation), gyro calibration is sufficient.

#### 3. Magnetometer Calibration (Optional)

The MPU6050 does not have a built-in magnetometer. If you add an external magnetometer (e.g., HMC5883L) for absolute heading:

```bash
# Use imu_tools magnetometer calibration
ros2 run imu_tools calibrate_magnetometer.py
```

### Post-Calibration Verification

After applying offsets, verify calibration:

```bash
# 1. Restart IMU node with new offsets
ros2 launch ros2_comprehensive_attempt imu.launch.py

# 2. Monitor gyro output (robot stationary)
ros2 topic echo /imu --field angular_velocity

# Expected: All values near 0.0 (< ±0.01 rad/s)
# angular_velocity:
#   x: 0.003
#   y: -0.002
#   z: 0.001

# 3. Slowly rotate robot by hand, watch gyro values
# They should be smooth and return to ~0 when stopped
```

### Environmental Considerations

**Temperature Drift**:
- MPU6050 offset changes with temperature (~0.01°/s per °C)
- Perform calibration at operating temperature (after warmup)
- For outdoor robots, re-calibrate seasonally

**Vibration**:
- Calibrate on same surface/mounting as operational use
- If mounted on robot with vibration, account for this in covariance (increase noise)

**Magnetic Interference**:
- Keep IMU away from motors, power cables, speakers
- Gyroscope is not affected by magnetic fields (uses MEMS vibration)
- Only matters if using magnetometer

---

## Integration with Existing System

### Current System Overview

The WayfindR robot currently has:

1. **cmd_vel_bridge**: Publishes `/odom` (dead reckoning from commanded velocities)
2. **RPLidar**: Publishes `/scan` (laser scans)
3. **AMCL**: Publishes `map → odom` transform (global localization)
4. **Nav2**: Uses `/odom` for local planning and control

### Integration Steps

#### Step 1: Add IMU Hardware

1. Connect MPU6050 to Raspberry Pi I2C (see wiring section)
2. Enable I2C interface (`sudo raspi-config`)
3. Verify connection (`sudo i2cdetect -y 1`)

#### Step 2: Install IMU Driver

```bash
cd ~/ros2_ws/src
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
cd ~/ros2_ws
colcon build --packages-select ros2_mpu6050_driver
source install/setup.bash
```

#### Step 3: Install robot_localization

```bash
sudo apt-get install ros-humble-robot-localization
```

#### Step 4: Create Configuration Files

**A. IMU Configuration** (`config/mpu6050_params.yaml`):

```yaml
mpu6050_node:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 0x68
    frame_id: "imu_link"
    publish_rate: 100.0
    gyro_range: 0        # ±250°/s
    accel_range: 0       # ±2g

    # Offsets (from calibration)
    gyro_x_offset: 0.0   # Replace after calibration
    gyro_y_offset: 0.0
    gyro_z_offset: 0.0
    accel_x_offset: 0.0
    accel_y_offset: 0.0
    accel_z_offset: 0.0
```

**B. robot_localization Configuration**: (Already shown in previous section)

#### Step 5: Update Launch Files

**A. Create IMU Launch File** (`launch/imu.launch.py`):

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_comprehensive_attempt')
    config_file = os.path.join(pkg_dir, 'config', 'mpu6050_params.yaml')

    imu_node = Node(
        package='ros2_mpu6050_driver',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[config_file]
    )

    # Static transform: base_link → imu_link
    # Adjust translation based on actual IMU mounting position
    imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_publisher',
        arguments=[
            '0.0', '0.0', '0.05',  # x, y, z (IMU 5cm above base_link)
            '0.0', '0.0', '0.0',    # roll, pitch, yaw (aligned with base)
            'base_link', 'imu_link'
        ]
    )

    return LaunchDescription([
        imu_node,
        imu_transform
    ])
```

**B. Update Localization Launch** to include EKF:

```python
#!/usr/bin/env python3
"""
Updated localization.launch.py with robot_localization EKF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_comprehensive_attempt')

    # Configuration files
    amcl_config = os.path.join(pkg_dir, 'config', 'amcl_params.yaml')
    lidar_config = os.path.join(pkg_dir, 'config', 'lidar_params.yaml')
    ekf_config = os.path.join(pkg_dir, 'config', 'robot_localization_params.yaml')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map YAML file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # LiDAR driver
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[lidar_config],
        output='screen'
    )

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}],
        output='screen'
    )

    # AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[amcl_config],
        output='screen',
        remappings=[
            ('odom', 'odometry/filtered')  # Use filtered odometry from EKF
        ]
    )

    # Lifecycle manager
    lifecycle_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # robot_localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        map_arg,
        use_rviz_arg,
        lidar_node,
        map_server_node,
        amcl_node,
        lifecycle_mgr_node,
        ekf_node,
        rviz_node
    ])
```

**C. Create Combined Launch** (`launch/robot_with_imu.launch.py`):

```python
#!/usr/bin/env python3
"""
Combined launch: cmd_vel_bridge + IMU + robot_localization
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_comprehensive_attempt')

    # cmd_vel_bridge launch
    cmd_vel_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'cmd_vel_bridge.launch.py')
        )
    )

    # IMU launch
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'imu.launch.py')
        )
    )

    # robot_localization launch
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'robot_localization.launch.py')
        )
    )

    return LaunchDescription([
        cmd_vel_bridge_launch,
        imu_launch,
        ekf_launch
    ])
```

#### Step 6: Update AMCL Configuration

Modify `config/amcl_params.yaml` to use filtered odometry:

```yaml
amcl:
  ros__parameters:
    # ... (existing parameters)

    # Base frame (already set)
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"

    # Robot model (already set to diff-drive)
    robot_model_type: "nav2_amcl::DifferentialMotionModel"

    # Update these for better performance with filtered odometry:

    # Odometry is now more accurate (from EKF fusion)
    # Can reduce particle spread
    alpha1: 0.1   # Reduce from 0.2 (rotation noise from rotation)
    alpha2: 0.1   # Reduce from 0.2 (rotation noise from translation)
    alpha3: 0.1   # Reduce from 0.2 (translation noise from translation)
    alpha4: 0.1   # Reduce from 0.2 (translation noise from rotation)

    # Can use fewer particles (more accurate odometry = faster convergence)
    min_particles: 500   # Reduce from 500
    max_particles: 2000  # Keep same
```

### TF Tree After Integration

```
map (published by AMCL)
 └─ odom (published by AMCL)
     └─ base_link (published by robot_localization EKF)
         ├─ imu_link (static transform)
         └─ laser (static transform)
```

**Transform Publishers**:
- `map → odom`: AMCL (1-5 Hz, discontinuous corrections)
- `odom → base_link`: robot_localization EKF (50 Hz, smooth, fused odometry)
- `base_link → imu_link`: static_transform_publisher (IMU mounting position)
- `base_link → laser`: static_transform_publisher (LiDAR mounting position)

---

## Testing and Validation

### Test Sequence

#### Test 1: IMU Driver Verification

```bash
# 1. Launch IMU node
ros2 launch ros2_comprehensive_attempt imu.launch.py

# 2. Check topic
ros2 topic list | grep imu
# Should show: /imu

# 3. Inspect message
ros2 topic echo /imu

# 4. Verify frame_id
# header.frame_id should be "imu_link"

# 5. Check update rate
ros2 topic hz /imu
# Should show: ~100 Hz

# 6. Test gyroscope (slowly rotate robot by hand)
ros2 topic echo /imu --field angular_velocity
# Should see non-zero values when rotating

# 7. Test accelerometer (tilt robot)
ros2 topic echo /imu --field linear_acceleration
# Should see gravity vector change direction
```

#### Test 2: EKF Node Verification

```bash
# 1. Launch full stack
ros2 launch ros2_comprehensive_attempt robot_with_imu.launch.py

# 2. Check EKF is running
ros2 node list | grep ekf
# Should show: /ekf_filter_node

# 3. Check filtered odometry topic
ros2 topic list | grep odometry/filtered
# Should show: /odometry/filtered

# 4. Verify update rate
ros2 topic hz /odometry/filtered
# Should show: ~50 Hz (matches EKF frequency parameter)

# 5. Check transform
ros2 run tf2_ros tf2_echo odom base_link
# Should show continuous updates
```

#### Test 3: Sensor Fusion Comparison

```bash
# Launch plotjuggler for side-by-side comparison
ros2 run plotjuggler plotjuggler

# Add data streams:
# - /odom/pose/pose/orientation/z (raw wheel odometry yaw)
# - /imu/orientation/z (IMU yaw)
# - /odometry/filtered/pose/pose/orientation/z (fused yaw)

# Test: Rotate robot 90° clockwise
# Observe:
# - Raw odom yaw: May drift or be inaccurate
# - IMU yaw: Accurate rotation
# - Filtered yaw: Should match IMU closely (EKF trusts IMU for rotation)
```

#### Test 4: Wheel Slippage Handling

```bash
# Setup: Place robot on smooth surface (tile/wood)

# 1. Launch full stack
ros2 launch ros2_comprehensive_attempt robot_with_imu.launch.py

# 2. Record data
ros2 bag record /odom /imu /odometry/filtered /cmd_vel

# 3. Send forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"

# 4. While moving, manually push robot sideways (simulate slippage)

# 5. Stop recording, analyze
ros2 bag play <bag_file>

# 6. Compare in plotjuggler:
# - /odom: Will show incorrect path (assumes no slippage)
# - /odometry/filtered: Should be more accurate (IMU detects unexpected rotation)
```

#### Test 5: AMCL with Filtered Odometry

```bash
# 1. Launch localization with map
ros2 launch ros2_comprehensive_attempt localization.launch.py \
    map:=maps/my_map.yaml

# 2. Set initial pose in RViz (2D Pose Estimate tool)

# 3. Teleoperate robot around environment
# Use keyboard teleop or controller

# 4. Monitor AMCL convergence
ros2 topic echo /amcl_pose

# 5. Observe particle cloud in RViz
# Should converge faster and tighter with filtered odometry

# 6. Test recovery from kidnapping
# Physically move robot to different location
# AMCL should re-localize using laser scans
```

#### Test 6: Nav2 Autonomous Navigation

```bash
# 1. Launch full navigation stack
ros2 launch ros2_comprehensive_attempt navigation.launch.py \
    map:=maps/my_map.yaml

# 2. Set initial pose in RViz

# 3. Set navigation goal (2D Goal Pose tool)

# 4. Monitor:
# - /odometry/filtered (should be smooth)
# - /cmd_vel (Nav2 commands)
# - Robot trajectory in RViz

# 5. Test tight turns
# IMU should provide accurate rotation feedback

# 6. Test path following accuracy
# Filtered odometry should improve trajectory tracking
```

### Validation Metrics

| Metric | Target | Test Method |
|--------|--------|-------------|
| **IMU Update Rate** | 100 Hz | `ros2 topic hz /imu` |
| **EKF Update Rate** | 50 Hz | `ros2 topic hz /odometry/filtered` |
| **Gyro Noise (static)** | < 0.01 rad/s | Record 1000 samples, calculate σ |
| **Rotation Accuracy** | < 5° error per 360° turn | Rotate robot 10 times, measure cumulative error |
| **AMCL Convergence** | < 5 seconds | Time from initial pose to converged particle cloud |
| **Path Following Error** | < 10 cm RMS | Compare planned vs actual path |

### Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| **IMU not detected** | I2C not enabled or wrong address | Run `sudo i2cdetect -y 1`, enable I2C |
| **Jumpy filtered odometry** | Process noise too high | Reduce process_noise_covariance |
| **EKF ignores IMU** | IMU covariance too high | Reduce imu0 covariance values |
| **Yaw oscillates** | Conflicting absolute yaw sources | Enable odom0_differential for yaw |
| **TF timeout errors** | EKF not publishing transform | Check `publish_tf: true` in config |
| **AMCL diverges** | Wrong odom topic | Verify AMCL remapping to /odometry/filtered |

---

## Implementation Roadmap

### Phase 1: Hardware Setup and Testing

**Goals**: Install IMU hardware, verify I2C communication

**Tasks**:
1. ☐ Wire MPU6050 to Raspberry Pi I2C
   - VCC → 3.3V
   - GND → GND
   - SDA → GPIO 2 (Pin 3)
   - SCL → GPIO 3 (Pin 5)

2. ☐ Enable I2C interface
   ```bash
   sudo raspi-config
   # Interface Options → I2C → Enable
   sudo reboot
   ```

3. ☐ Install I2C tools
   ```bash
   sudo apt-get install i2c-tools python3-smbus
   ```

4. ☐ Verify IMU detection
   ```bash
   sudo i2cdetect -y 1
   # Should see 0x68
   ```

5. ☐ Test raw I2C communication
   - Read WHO_AM_I register (should return 0x68)
   - Read gyro/accel data
   - Verify sensor is functional

6. ☐ Mount IMU on robot
   - Align axes with robot frame
   - Use vibration damping
   - Secure mounting

7. ☐ Document IMU mounting position
   - Measure x, y, z offset from base_link
   - Record orientation (if not aligned)

**Validation**:
- [ ] IMU detected at I2C address 0x68
- [ ] Gyro values change when rotating IMU
- [ ] Accel values change when tilting IMU
- [ ] IMU securely mounted on robot

---

### Phase 2: ROS2 Driver Integration

**Goals**: Install and configure IMU driver, publish `/imu` topic

**Tasks**:
1. ☐ Install ROS2 dependencies
   ```bash
   sudo apt-get install python3-smbus2 python3-numpy
   ```

2. ☐ Clone and build IMU driver
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
   cd ~/ros2_ws
   colcon build --packages-select ros2_mpu6050_driver
   source install/setup.bash
   ```

3. ☐ Create IMU configuration file
   - Copy template from this document
   - Set I2C bus and address
   - Configure frame_id, publish_rate

4. ☐ Create IMU launch file
   - Launch IMU driver node
   - Launch static transform (base_link → imu_link)

5. ☐ Test IMU driver
   ```bash
   ros2 launch ros2_comprehensive_attempt imu.launch.py
   ros2 topic echo /imu
   ```

6. ☐ Verify message format
   - Check header.frame_id = "imu_link"
   - Verify angular_velocity units (rad/s)
   - Verify linear_acceleration units (m/s²)

7. ☐ Check transform tree
   ```bash
   ros2 run tf2_tools view_frames
   ```
   - Verify base_link → imu_link exists

8. ☐ Measure update rate
   ```bash
   ros2 topic hz /imu
   ```
   - Should be ~100 Hz

**Validation**:
- [ ] `/imu` topic publishes at 100 Hz
- [ ] IMU data looks reasonable (gyro ~0 when static, accel = 9.8 m/s² Z when upright)
- [ ] TF tree includes base_link → imu_link
- [ ] No error messages in logs

---

### Phase 3: IMU Calibration

**Goals**: Measure and correct gyro/accel biases

**Tasks**:
1. ☐ Create calibration script
   - Use template from this document
   - Collect 1000 samples when stationary

2. ☐ Warm up IMU
   - Run IMU for 5 minutes before calibration
   - Allows temperature stabilization

3. ☐ Run gyro calibration
   - Place robot on stable surface
   - Run calibration script
   - Record offsets

4. ☐ Update IMU config with offsets
   ```yaml
   gyro_x_offset: -0.034
   gyro_y_offset: 0.021
   gyro_z_offset: -0.015
   ```

5. ☐ Restart IMU node and verify
   ```bash
   ros2 topic echo /imu --field angular_velocity
   ```
   - All axes should read ~0.0 when stationary

6. ☐ Test rotation
   - Slowly rotate robot by hand
   - Verify gyro values are smooth
   - Verify values return to ~0 when stopped

**Validation**:
- [ ] Gyro reads < ±0.01 rad/s when stationary
- [ ] Gyro responds smoothly to rotation
- [ ] Offsets documented in config file

---

### Phase 4: robot_localization Installation and Configuration

**Goals**: Install EKF, create configuration for wheel odometry + IMU fusion

**Tasks**:
1. ☐ Install robot_localization
   ```bash
   sudo apt-get install ros-humble-robot-localization
   ```

2. ☐ Create EKF configuration file
   - Use template from this document
   - Configure odom0 (wheel odometry)
   - Configure imu0 (IMU)
   - Set frame IDs
   - Configure covariance matrices

3. ☐ Create robot_localization launch file
   - Launch ekf_node with config file

4. ☐ Test EKF standalone
   ```bash
   ros2 launch ros2_comprehensive_attempt robot_localization.launch.py
   ```
   - Check for error messages
   - Verify /odometry/filtered topic

5. ☐ Create combined launch file
   - Combine cmd_vel_bridge + IMU + robot_localization

6. ☐ Update AMCL configuration
   - Remap AMCL to use /odometry/filtered

7. ☐ Document configuration
   - Record all parameter values
   - Explain fusion strategy

**Validation**:
- [ ] EKF node starts without errors
- [ ] /odometry/filtered publishes at 50 Hz
- [ ] TF odom → base_link published by EKF
- [ ] No covariance explosion warnings

---

### Phase 5: Sensor Fusion Tuning

**Goals**: Optimize EKF parameters for accurate, smooth odometry

**Tasks**:
1. ☐ Measure sensor covariances
   - Record 1000 static IMU samples → calculate σ
   - Update imu0_angular_velocity_covariance
   - Verify wheel odometry covariance (already in cmd_vel_bridge)

2. ☐ Baseline test
   - Drive robot in square pattern (1m x 1m)
   - Record ground truth (measure final position)
   - Record /odom, /odometry/filtered
   - Calculate position error

3. ☐ Tune process noise covariance
   - Start with conservative values (0.1)
   - Reduce if odometry lags
   - Increase if odometry is jumpy
   - Iterate until smooth, responsive

4. ☐ Test rotation accuracy
   - Rotate robot 360° (10 times)
   - Compare IMU-based vs wheel-based yaw
   - Verify EKF uses IMU for rotation

5. ☐ Test wheel slippage handling
   - Drive on smooth surface
   - Manually push robot sideways while moving
   - Verify filtered odometry more accurate than raw odometry

6. ☐ Validate with plotjuggler
   - Plot raw vs filtered odometry
   - Verify filtering reduces noise
   - Check for oscillations or lag

**Validation**:
- [ ] Rotation error < 5° per 360° turn
- [ ] Position error < 10% over 10m travel
- [ ] Filtered odometry smoother than raw odometry
- [ ] No oscillations or jumps in filtered estimate

---

### Phase 6: AMCL and Nav2 Integration Testing

**Goals**: Test full navigation stack with fused odometry

**Tasks**:
1. ☐ Update localization launch
   - Include IMU, robot_localization, AMCL
   - Verify AMCL uses /odometry/filtered

2. ☐ Test AMCL localization
   - Load map
   - Set initial pose
   - Teleoperate robot
   - Verify AMCL converges correctly
   - Compare convergence speed vs non-fused odometry

3. ☐ Test Nav2 autonomous navigation
   - Set navigation goal
   - Monitor path following accuracy
   - Test tight turns (IMU should help)
   - Verify smooth velocity commands

4. ☐ Benchmark performance
   - Measure AMCL convergence time
   - Measure path following error (RMS)
   - Compare with baseline (no IMU fusion)

**Validation**:
- [ ] AMCL converges < 5 seconds
- [ ] Path following error < 10 cm RMS
- [ ] Robot completes navigation goals reliably
- [ ] No oscillations in rotation during tight turns

---

### Phase 7: Documentation and Deployment

**Goals**: Finalize documentation, create deployment guide

**Tasks**:
1. ☐ Document final configuration
   - All parameter values
   - Calibration offsets
   - Tuning decisions

2. ☐ Create user guide
   - How to launch system
   - How to re-calibrate IMU
   - Troubleshooting common issues

3. ☐ Create deployment checklist
   - Hardware setup
   - Software installation
   - Configuration
   - Testing

4. ☐ Update README
   - Add IMU integration section
   - Update system architecture diagram

5. ☐ Create backup of working configuration
   ```bash
   tar -czf robot_config_backup.tar.gz config/ launch/
   ```

6. ☐ Test deployment on clean system
   - Follow deployment checklist
   - Verify all steps work

**Validation**:
- [ ] Documentation complete and accurate
- [ ] Deployment checklist tested
- [ ] Configuration backed up
- [ ] System can be deployed on new robot

---

### Implementation Summary

**Phase Breakdown**:
- Phase 1: Hardware setup and testing
- Phase 2: ROS2 driver integration
- Phase 3: IMU calibration
- Phase 4: robot_localization configuration
- Phase 5: Sensor fusion tuning
- Phase 6: AMCL and Nav2 integration testing
- Phase 7: Documentation and deployment

**Recommended Sequence**:
- **First**: Phases 1-2 (Hardware + Driver)
- **Second**: Phases 3-4 (Calibration + EKF Setup)
- **Third**: Phase 5 (Tuning)
- **Fourth**: Phases 6-7 (Integration + Documentation)

---

## References

### Research Sources

1. **MPU6050 Hardware**
   - [MPU6050 Datasheet Explained](https://www.ultralibrarian.com/2025/12/16/mpu6050-datasheet-explained-ulc)
   - [MPU6050 Module Specifications](https://components101.com/sensors/mpu6050-module)
   - [InvenSense MPU-6050 Product Page](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
   - [MPU6050 Accelerometer and Gyroscope Guide](https://www.electronicwings.com/sensors-modules/mpu6050-gyroscope-accelerometer-temperature-sensor-module)

2. **Raspberry Pi I2C Integration**
   - [MPU6050 Gyro Sensor Interfacing with Raspberry Pi](https://circuitdigest.com/microcontroller-projects/mpu6050-gyro-sensor-interfacing-with-raspberry-pi)
   - [Using the MPU-6050 with the Raspberry Pi](https://38-3d.co.uk/blogs/blog/using-the-mpu-6050-with-the-raspberry-pi)
   - [Raspberry Pi I2C GPIO Pinout](https://pinout.xyz/pinout/i2c)

3. **ROS2 IMU Drivers**
   - [ros2_mpu6050_driver (hiwad-aziz)](https://github.com/hiwad-aziz/ros2_mpu6050_driver)
   - [ros2-mpu6050 (JCorbin406)](https://github.com/JCorbin406/ros2-mpu6050)
   - [ros2_mpu6050 (C++ - kimsniper)](https://github.com/kimsniper/ros2_mpu6050)
   - [ros2_imu_mpu6050 (GuangfuWang)](https://github.com/GuangfuWang/ros2_imu_mpu6050)
   - [ROS2 Humble MPU6050 Interface for Jetson Nano](https://medium.com/@kabilankb2003/ros2-humble-mpu6050-imu-sensor-interface-for-nvidia-jetson-nano-c4d616647ee5)

4. **imu_tools Package**
   - [CCNYRoboticsLab/imu_tools GitHub](https://github.com/CCNYRoboticsLab/imu_tools)
   - [imu_filter_madgwick ROS Index](https://index.ros.org/p/imu_filter_madgwick/)
   - [imu_tools Humble Branch](https://github.com/CCNYRoboticsLab/imu_tools/tree/humble)

5. **robot_localization Package**
   - [Sensor Fusion Using robot_localization (ROS2)](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)
   - [Sensor Fusion and Robot Localization (ROS2 Jazzy)](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/)
   - [Nav2 - Smoothing Odometry using Robot Localization](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
   - [robot_localization GitHub (ROS2 branch)](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml)
   - [Fusing IMU + Encoders with ROS Robot Localization](https://roverrobotics.com/blogs/guides/fusing-imu-encoders-with-ros-robot-localization)

6. **EKF Sensor Fusion Architecture**
   - [ROS2 Sensor Fusion: EKF & AMCL](https://robotisim.com/ros2-sensor-fusion-ekf-amcl/)
   - [Mastering ROS2 Sensor Fusion with EKF & AMCL](https://dev.to/robotisimseo_76f72fc9b6cb/sharpen-your-robots-eyes-mastering-ros2-sensor-fusion-with-ekf-amcl-3mf2)
   - [Robot Localization with AMCL and EKF](https://answers.ros.org/question/231826/robot-localization-with-amcl-and-ekf/)
   - [ROS Sensor Fusion Tutorial (methylDragon)](https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md)

7. **Dual EKF Configuration**
   - [Nav2 - Navigating Using GPS Localization](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)
   - [robot_localization Dual EKF Launch Example](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/launch/dual_ekf_navsat_example.launch.py)

8. **Covariance Tuning**
   - [Fusing Wheel Odometry and IMU Data](https://blog.abdurrosyid.com/2021/07/21/fusing-wheel-odometry-and-imu-data-using-robot_localization-in-ros/)
   - [AI-Driven Dynamic Covariance for ROS 2](https://www.mdpi.com/1424-8220/25/10/3026)
   - [How to use robot_localization package](https://medium.com/@zillur-rahman/how-to-use-the-ros-robot-localization-package-534fe04014d3)
   - [Working with robot_localization (Tom Moore - ROSCon 2015)](https://roscon.ros.org/2015/presentations/robot_localization.pdf)

9. **IMU Calibration**
   - [Calibrating & Optimising the MPU6050](https://wired.chillibasket.com/2015/01/calibrating-mpu6050/)
   - [MPU6050 Sensor Setup: Technical Guide (2026)](https://stevezafeiriou.com/mpu6050-sensor-setup/)
   - [MPU6050 Setup and Calibration Guide (Instructables)](https://www.instructables.com/MPU6050-Setup-and-Calibration-Guide/)
   - [Adafruit Gyroscope Calibration Guide](https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration?view=all)
   - [MPU6050 Calibration GitHub (Raspberry Pi)](https://github.com/Blokkendoos/mpu-calibration)

10. **Differential Drive Odometry**
    - [Differential Drive Odometry Noise Covariance](https://answers.ros.org/question/391483/process_noise_covariance-and-initial_estimate_covariance-in-ekf-global-and-ekf-local)
    - [MATLAB Odometry Motion Model](https://www.mathworks.com/help/robotics/ref/robotics.odometrymotionmodel-class.html)

---

## Appendix A: Quick Reference Commands

### Hardware Verification

```bash
# Enable I2C
sudo raspi-config  # Interface → I2C → Enable

# Install I2C tools
sudo apt-get install i2c-tools

# Scan for devices
sudo i2cdetect -y 1

# Test I2C speed
sudo i2cdetect -y -a 1
```

### IMU Testing

```bash
# Launch IMU driver
ros2 launch ros2_comprehensive_attempt imu.launch.py

# Monitor IMU topic
ros2 topic echo /imu

# Check update rate
ros2 topic hz /imu

# Monitor specific field
ros2 topic echo /imu --field angular_velocity
```

### EKF Testing

```bash
# Launch robot_localization
ros2 launch ros2_comprehensive_attempt robot_localization.launch.py

# Monitor filtered odometry
ros2 topic echo /odometry/filtered

# Check transform
ros2 run tf2_ros tf2_echo odom base_link

# View TF tree
ros2 run tf2_tools view_frames
```

### Plotting and Analysis

```bash
# Install plotjuggler
sudo apt-get install ros-humble-plotjuggler-ros

# Launch
ros2 run plotjuggler plotjuggler

# Record bag for offline analysis
ros2 bag record /odom /imu /odometry/filtered /cmd_vel /scan
```

### Diagnostics

```bash
# Monitor diagnostics
ros2 topic echo /diagnostics

# List all topics
ros2 topic list

# Check node status
ros2 node list
ros2 node info /ekf_filter_node
```

---

## Appendix B: Configuration Templates

### Minimal IMU Config

```yaml
mpu6050_node:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 0x68
    frame_id: "imu_link"
    publish_rate: 100.0
    gyro_range: 0
    accel_range: 0
```

### Minimal EKF Config

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    publish_tf: true

    odom0: /odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true, false, false,
                   false, false, false,
                   false, false, false]

    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
```

---

**End of Document**

**Version**: 1.0.0
**Date**: 2026-01-11
**Author**: WayfindR Development Team
**Status**: Ready for Implementation
