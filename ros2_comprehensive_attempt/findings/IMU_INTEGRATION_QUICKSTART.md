# IMU Integration Quick Start Guide

**Created:** 2026-01-11
**Version:** 1.0.0
**For:** WayfindR Robot - ROS2 Humble

---

## Overview

This is a condensed, action-oriented guide for integrating the MPU6050 IMU with the WayfindR robot. For detailed explanations, see `IMU_SENSOR_FUSION_RESEARCH.md`.

---

## Hardware Setup

### 1. Wiring

Connect MPU6050 to Raspberry Pi:

```
MPU6050        Raspberry Pi
--------       ------------
VCC     →      Pin 1 (3.3V)
GND     →      Pin 6 (GND)
SDA     →      Pin 3 (GPIO 2)
SCL     →      Pin 5 (GPIO 3)
```

### 2. Enable I2C

```bash
sudo raspi-config
# Navigate: 3 Interface Options → I5 I2C → Enable → Reboot

# After reboot, verify
sudo apt-get install i2c-tools
sudo i2cdetect -y 1
# Should show 0x68
```

### 3. Mount IMU

- **Position**: Center of robot, close to base_link
- **Orientation**: X-forward, Y-left, Z-up (aligned with robot)
- **Mounting**: Use foam/rubber for vibration damping
- **Location**: Away from motors and power cables

---

## Software Setup

### 1. Install Dependencies

```bash
# Install I2C library
sudo apt-get install python3-smbus2

# Install robot_localization
sudo apt-get install ros-humble-robot-localization

# Install IMU driver
cd ~/ros2_ws/src
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
cd ~/ros2_ws
colcon build --packages-select ros2_mpu6050_driver
source install/setup.bash
```

### 2. Create Configuration Files

**A. IMU Config** (`config/mpu6050_params.yaml`):

```yaml
mpu6050_node:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 0x68
    frame_id: "imu_link"
    publish_rate: 100.0
    gyro_range: 0        # ±250°/s
    accel_range: 0       # ±2g

    # Set after calibration
    gyro_x_offset: 0.0
    gyro_y_offset: 0.0
    gyro_z_offset: 0.0
```

**B. EKF Config** (`config/robot_localization_params.yaml`):

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    publish_tf: true

    # Wheel odometry input
    odom0: /odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, false,
                   false, false, false]
    odom0_differential: false

    # IMU input
    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true

    # Process noise (adjust during tuning)
    process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]

    initial_estimate_covariance: [1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9]
```

### 3. Create Launch Files

**A. IMU Launch** (`launch/imu.launch.py`):

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

    imu_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_publisher',
        arguments=['0.0', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    return LaunchDescription([imu_node, imu_transform])
```

**B. robot_localization Launch** (`launch/robot_localization.launch.py`):

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_comprehensive_attempt')
    config_file = os.path.join(pkg_dir, 'config', 'robot_localization_params.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([ekf_node])
```

---

## Calibration

### 1. Create Calibration Script

Save as `scripts/calibrate_imu.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.samples = []
        self.num_samples = 1000

    def imu_callback(self, msg):
        if len(self.samples) < self.num_samples:
            gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            self.samples.append(gyro)
            if len(self.samples) % 100 == 0:
                self.get_logger().info(f'Collected {len(self.samples)}/{self.num_samples} samples...')
        elif len(self.samples) == self.num_samples:
            samples_array = np.array(self.samples)
            offsets = np.mean(samples_array, axis=0)
            std_devs = np.std(samples_array, axis=0)

            self.get_logger().info('\n' + '='*50)
            self.get_logger().info('Calibration Complete!')
            self.get_logger().info('='*50)
            self.get_logger().info(f'Gyro X offset: {offsets[0]:.6f} rad/s (σ={std_devs[0]:.6f})')
            self.get_logger().info(f'Gyro Y offset: {offsets[1]:.6f} rad/s (σ={std_devs[1]:.6f})')
            self.get_logger().info(f'Gyro Z offset: {offsets[2]:.6f} rad/s (σ={std_devs[2]:.6f})')
            self.get_logger().info('\nAdd to config/mpu6050_params.yaml:')
            self.get_logger().info(f'  gyro_x_offset: {offsets[0]:.6f}')
            self.get_logger().info(f'  gyro_y_offset: {offsets[1]:.6f}')
            self.get_logger().info(f'  gyro_z_offset: {offsets[2]:.6f}')
            self.samples.append([0, 0, 0])  # Stop processing

def main():
    rclpy.init()
    print('\nIMU Gyroscope Calibration')
    print('='*50)
    print('Place robot on level, stable surface.')
    print('Keep completely stationary.')
    print('Collecting 1000 samples...\n')

    node = ImuCalibrator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Run Calibration

```bash
# Terminal 1: Launch IMU
ros2 launch ros2_comprehensive_attempt imu.launch.py

# Terminal 2: Run calibration (after warmup period)
chmod +x scripts/calibrate_imu.py
ros2 run ros2_comprehensive_attempt calibrate_imu.py

# Copy output offsets to config/mpu6050_params.yaml
```

---

## Testing

### Test 1: IMU Verification

```bash
# Launch IMU
ros2 launch ros2_comprehensive_attempt imu.launch.py

# Check topic
ros2 topic hz /imu        # Should be ~100 Hz
ros2 topic echo /imu --field angular_velocity

# Verify: Values near 0.0 when stationary
# Verify: Values change when rotating robot
```

### Test 2: EKF Verification

```bash
# Terminal 1: Launch cmd_vel_bridge
ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py

# Terminal 2: Launch IMU
ros2 launch ros2_comprehensive_attempt imu.launch.py

# Terminal 3: Launch EKF
ros2 launch ros2_comprehensive_attempt robot_localization.launch.py

# Terminal 4: Check outputs
ros2 topic hz /odometry/filtered   # Should be ~50 Hz
ros2 topic echo /odometry/filtered

# Check transform
ros2 run tf2_ros tf2_echo odom base_link
```

### Test 3: Rotation Accuracy

```bash
# Launch full stack (all terminals above)

# Install plotjuggler
sudo apt-get install ros-humble-plotjuggler-ros
ros2 run plotjuggler plotjuggler

# Plot:
# - /imu/orientation/z
# - /odometry/filtered/pose/pose/orientation/z

# Test: Slowly rotate robot 360° by hand
# Verify: Filtered odometry tracks IMU closely
```

---

## Integration with AMCL

### Update AMCL Configuration

Edit `config/amcl_params.yaml`:

```yaml
amcl:
  ros__parameters:
    # Motion model parameters (reduce - odometry more accurate now)
    alpha1: 0.1   # Was 0.2
    alpha2: 0.1   # Was 0.2
    alpha3: 0.1   # Was 0.2
    alpha4: 0.1   # Was 0.2
```

### Update Localization Launch

Edit `launch/localization.launch.py` - add remapping for AMCL:

```python
# In amcl_node definition
amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    parameters=[amcl_config],
    output='screen',
    remappings=[
        ('odom', 'odometry/filtered')  # ← Add this line
    ]
)

# Also add ekf_node to this launch file
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_config]
)

# Add to return statement
return LaunchDescription([
    # ... existing nodes ...
    ekf_node,  # ← Add this
    # ... rest of nodes ...
])
```

---

## Quick Commands Reference

### Launch Commands

```bash
# Individual components
ros2 launch ros2_comprehensive_attempt imu.launch.py
ros2 launch ros2_comprehensive_attempt robot_localization.launch.py
ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py

# Full odometry stack (create combined launch or run separately)
# Terminal 1:
ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py

# Terminal 2:
ros2 launch ros2_comprehensive_attempt imu.launch.py

# Terminal 3:
ros2 launch ros2_comprehensive_attempt robot_localization.launch.py

# Navigation with sensor fusion
ros2 launch ros2_comprehensive_attempt localization.launch.py map:=maps/my_map.yaml
ros2 launch ros2_comprehensive_attempt navigation.launch.py map:=maps/my_map.yaml
```

### Monitoring Commands

```bash
# Check topics
ros2 topic list | grep -E 'imu|odom'

# Monitor rates
ros2 topic hz /imu
ros2 topic hz /odom
ros2 topic hz /odometry/filtered

# View transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link

# Check nodes
ros2 node list
ros2 node info /ekf_filter_node

# Live plotting
ros2 run plotjuggler plotjuggler
```

### Debugging Commands

```bash
# IMU diagnostics
ros2 topic echo /imu --field angular_velocity
ros2 topic echo /imu --field linear_acceleration

# EKF diagnostics
ros2 topic echo /diagnostics

# Parameter inspection
ros2 param list /ekf_filter_node
ros2 param get /ekf_filter_node frequency

# Log level adjustment
ros2 run rqt_logger_level rqt_logger_level
```

---

## Troubleshooting

### IMU Not Detected

```bash
# Check I2C
sudo i2cdetect -y 1

# If nothing at 0x68:
# - Check wiring
# - Check 3.3V power supply
# - Try alternate address (0x69) if AD0 pin pulled high
```

### IMU Topic Not Publishing

```bash
# Check if node is running
ros2 node list | grep mpu6050

# Check parameters
ros2 param list /mpu6050_node

# Check logs
ros2 node info /mpu6050_node

# Restart with verbose logging
ros2 launch ros2_comprehensive_attempt imu.launch.py --log-level debug
```

### EKF Not Fusing IMU Data

```bash
# Check if IMU covariance is too high
ros2 topic echo /imu --field angular_velocity_covariance

# Reduce covariance in driver config or EKF config
# Edit robot_localization_params.yaml:
# Add explicit covariance override
```

### Filtered Odometry Jumpy

```bash
# Reduce process noise covariance
# Edit robot_localization_params.yaml
# Divide diagonal values by 2-10
```

### Filtered Odometry Lags

```bash
# Increase process noise covariance
# Edit robot_localization_params.yaml
# Multiply diagonal values by 2-10
```

### TF Timeout Errors

```bash
# Check that publish_tf is enabled
ros2 param get /ekf_filter_node publish_tf

# Check transform tree
ros2 run tf2_tools view_frames

# Verify base_link → imu_link static transform is published
ros2 run tf2_ros tf2_echo base_link imu_link
```

---

## Performance Targets

| Metric | Target | Test |
|--------|--------|------|
| **IMU Rate** | 100 Hz | `ros2 topic hz /imu` |
| **EKF Rate** | 50 Hz | `ros2 topic hz /odometry/filtered` |
| **Gyro Noise** | < 0.01 rad/s | `ros2 topic echo /imu` (stationary) |
| **Rotation Error** | < 5° per 360° | Rotate robot 10 times, measure drift |
| **AMCL Convergence** | < 5 seconds | Time from initial pose to stable |
| **Path Following** | < 10 cm RMS | Nav2 trajectory tracking |

---

## Next Steps

After successful integration:

1. **Tune covariance matrices** for your specific robot
2. **Benchmark performance** vs baseline (no IMU)
3. **Document calibration values** in robot-specific config
4. **Create automated tests** for regression testing
5. **Consider magnetometer** for absolute heading (future enhancement)

---

## Additional Resources

- Full Research Document: `findings/IMU_SENSOR_FUSION_RESEARCH.md`
- cmd_vel_bridge Design: `findings/CMD_VEL_BRIDGE_DESIGN.md`
- TF Tree Documentation: `findings/tf_tree_and_coordinate_frames.md`
- Nav2 Integration: `findings/INTEGRATION_GUIDE.md`

---

**Quick Start Version:** 1.0.0
**Last Updated:** 2026-01-11
**Companion Document:** IMU_SENSOR_FUSION_RESEARCH.md
