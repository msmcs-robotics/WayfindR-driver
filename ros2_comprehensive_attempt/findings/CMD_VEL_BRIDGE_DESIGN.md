# cmd_vel Bridge Design Document

**Project:** WayfindR ROS2 Navigation Integration
**Component:** cmd_vel Bridge (ROS2 Nav2 to PI_API)
**Author:** System Design
**Date:** 2026-01-11
**Version:** 1.0.0

---

## Executive Summary

This document describes the architecture and implementation of a bridge between ROS2's Nav2 navigation stack and the WayfindR robot's FastAPI-based motor control system (PI_API). The bridge translates high-level navigation commands (`geometry_msgs/Twist` on `/cmd_vel`) into low-level motor control HTTP requests, while publishing odometry feedback to enable closed-loop navigation.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture Options](#architecture-options)
3. [Message Flow](#message-flow)
4. [Coordinate Frame Transformations](#coordinate-frame-transformations)
5. [Differential Drive Kinematics](#differential-drive-kinematics)
6. [Safety Features](#safety-features)
7. [Odometry Publishing](#odometry-publishing)
8. [Implementation Details](#implementation-details)
9. [Configuration Parameters](#configuration-parameters)
10. [Testing Strategy](#testing-strategy)
11. [References](#references)

---

## System Overview

### Components

```
┌─────────────────────────────────────────────────────────────────┐
│                         ROS2 Nav2 Stack                         │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐                 │
│  │  AMCL    │───▶│   Nav2   │───▶│ cmd_vel  │                 │
│  │Localizer │    │ Planner  │    │Publisher │                 │
│  └──────────┘    └──────────┘    └─────┬────┘                 │
└────────────────────────────────────────┼──────────────────────┘
                                         │
                            geometry_msgs/Twist
                            (/cmd_vel topic)
                                         │
                                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                    cmd_vel_bridge.py (This Component)            │
│                                                                  │
│  ┌────────────────┐      ┌──────────────┐    ┌──────────────┐ │
│  │ Twist          │──┬──▶│ Differential │───▶│ HTTP Client  │ │
│  │ Subscriber     │  │   │ Drive        │    │ (requests)   │ │
│  │ (/cmd_vel)     │  │   │ Kinematics   │    └──────┬───────┘ │
│  └────────────────┘  │   └──────────────┘           │         │
│                      │                               │         │
│  ┌────────────────┐  │   ┌──────────────┐           │         │
│  │ Odometry       │◀─┴───│ Dead         │           │         │
│  │ Publisher      │      │ Reckoning    │           │         │
│  │ (/odom)        │      │ Estimator    │           │         │
│  └────────────────┘      └──────────────┘           │         │
│                                                      │         │
│  ┌────────────────┐                                 │         │
│  │ TF Broadcaster │                                 │         │
│  │ (odom->base)   │                                 │         │
│  └────────────────┘                                 │         │
│                                                      │         │
│  ┌────────────────┐                                 │         │
│  │ Safety Monitor │                                 │         │
│  │ (watchdog)     │                                 │         │
│  └────────────────┘                                 │         │
└──────────────────────────────────────────────────────┼─────────┘
                                                       │
                                                  HTTP POST
                                             /api/control/move
                                                       │
                                                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                        PI_API (FastAPI)                         │
│                                                                  │
│  ┌────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │  Control   │───▶│    Robot     │───▶│    Motor     │       │
│  │  Router    │    │  Controller  │    │   Driver     │       │
│  └────────────┘    └──────────────┘    └──────┬───────┘       │
└──────────────────────────────────────────────────┼─────────────┘
                                                   │
                                                   ▼
                                         ┌──────────────────┐
                                         │   L298N Motor    │
                                         │    Drivers       │
                                         │  (4x DC Motors)  │
                                         └──────────────────┘
```

### Purpose

The cmd_vel bridge enables:
- **Nav2 Integration**: Allows ROS2's Nav2 stack to control the robot
- **Autonomous Navigation**: Goal-based navigation using waypoints
- **Path Planning**: Obstacle avoidance and optimal path generation
- **Localization**: Position estimation using AMCL with LiDAR
- **Mapping**: SLAM-based map building for navigation

---

## Architecture Options

### Option 1: ROS2 Node (RECOMMENDED)

**Implementation:** Native ROS2 Python node using rclpy

**Advantages:**
- Native ROS2 integration with proper lifecycle management
- Built-in QoS (Quality of Service) configuration
- Easy integration with other ROS2 tools (RViz, rqt, etc.)
- Standard ROS2 parameter server support
- Can be launched via ROS2 launch files
- Proper node lifecycle (configure, activate, shutdown)

**Disadvantages:**
- Requires ROS2 runtime environment
- More complex initial setup

**Verdict:** ✅ **Selected** - Best for production use, proper ROS2 ecosystem integration

---

### Option 2: Standalone Bridge Script

**Implementation:** Python script that imports rclpy but runs independently

**Advantages:**
- Simpler to understand and debug
- Can run without launch files
- Easier to integrate with non-ROS systems
- Lower barrier to entry

**Disadvantages:**
- Less integration with ROS2 tools
- Manual lifecycle management
- No launch file support
- Parameters must be hardcoded or use separate config

**Verdict:** ❌ Not selected - Good for prototyping only

---

### Option 3: ROS2 Control Plugin

**Implementation:** Full ros2_control hardware interface

**Advantages:**
- Industry-standard approach
- Built-in diff_drive_controller
- Automatic odometry computation
- Safety features included

**Disadvantages:**
- Extremely complex to implement
- Requires URDF robot description
- Overkill for simple HTTP bridge
- Difficult to maintain

**Verdict:** ❌ Not selected - Too complex for this use case

---

## Message Flow

### 1. Navigation Command Flow (Nav2 → Robot)

```
Nav2 Planner
    │
    │ publishes at ~20 Hz
    ▼
/cmd_vel (geometry_msgs/Twist)
    │
    │ {linear: {x: 0.5, y: 0, z: 0},
    │  angular: {x: 0, y: 0, z: 0.2}}
    │
    ▼
cmd_vel_bridge.py
    │
    │ Differential Drive Kinematics
    │ v_left = v_x - ω_z × (wheelbase/2)
    │ v_right = v_x + ω_z × (wheelbase/2)
    │
    │ Normalize to [-1.0, 1.0] range
    │ throttle = (v_left + v_right) / 2
    │ steering = (v_right - v_left) / wheelbase_effect
    │
    ▼
HTTP POST http://localhost:8000/api/control/move
    │
    │ {throttle: 0.5, steering: 0.3, duration: null}
    │
    ▼
PI_API Robot Controller
    │
    │ _calculate_differential(throttle, steering)
    │
    ▼
Motor Driver (L298N)
    │
    ▼
DC Motors (4x)
```

### 2. Odometry Feedback Flow (Robot → Nav2)

```
Motor Commands (last sent)
    │
    ▼
Dead Reckoning Estimator
    │
    │ x_new = x + v × cos(θ) × dt
    │ y_new = y + v × sin(θ) × dt
    │ θ_new = θ + ω × dt
    │
    ▼
nav_msgs/Odometry message
    │
    │ Published at 50 Hz
    │
    ▼
/odom topic
    │
    ├──▶ Nav2 (for velocity info)
    │
    └──▶ robot_localization (optional sensor fusion)
```

### 3. Transform Tree (TF)

```
map
 └── odom (published by AMCL)
      └── base_link (published by cmd_vel_bridge)
           ├── base_footprint
           ├── laser (RPLidar)
           └── imu (optional)
```

---

## Coordinate Frame Transformations

### Frame Definitions

1. **map**: Global fixed frame
   - Origin: Arbitrary point in the environment
   - Never moves
   - Set by map server

2. **odom**: Continuous local frame
   - Origin: Robot's starting position
   - Drifts over time (dead reckoning)
   - Published by cmd_vel_bridge

3. **base_link**: Robot's center
   - Origin: Geometric center of robot
   - Moves with robot
   - Child of odom frame

4. **base_footprint**: Ground projection
   - Origin: base_link projected to ground (z=0)
   - Used for 2D navigation

5. **laser**: LiDAR sensor
   - Origin: Physical location of RPLidar
   - Fixed offset from base_link

### Transform Publishers

**cmd_vel_bridge responsibility:**
```python
# Publish odom → base_link transform at 50 Hz
transform = TransformStamped()
transform.header.stamp = self.get_clock().now().to_msg()
transform.header.frame_id = 'odom'
transform.child_frame_id = 'base_link'
transform.transform.translation.x = self.x
transform.transform.translation.y = self.y
transform.transform.translation.z = 0.0
# Convert yaw to quaternion
quat = quaternion_from_euler(0, 0, self.theta)
transform.transform.rotation = quat
self.tf_broadcaster.sendTransform(transform)
```

**AMCL responsibility:**
- Publishes `map → odom` transform
- Corrects odometry drift using LiDAR scans
- Updates when robot position is re-localized

### Coordinate Conventions (REP 103)

- **X-axis**: Forward (robot front)
- **Y-axis**: Left (robot left side)
- **Z-axis**: Up (vertical)
- **Yaw (θ)**: Counter-clockwise rotation around Z-axis
  - 0° = facing +X
  - 90° = facing +Y (left)
  - -90° = facing -Y (right)

---

## Differential Drive Kinematics

### Forward Kinematics (Wheel Velocities → Body Twist)

Given wheel velocities `v_left` and `v_right`:

```
v_x = (v_left + v_right) / 2        # Linear velocity (m/s)
ω_z = (v_right - v_left) / L        # Angular velocity (rad/s)

where:
  L = wheel separation (wheelbase) in meters
```

### Inverse Kinematics (Body Twist → Wheel Velocities)

Given cmd_vel twist with `v_x` (linear.x) and `ω_z` (angular.z):

```
v_left = v_x - (ω_z × L / 2)
v_right = v_x + (ω_z × L / 2)

where:
  L = wheel separation (wheelbase) in meters
```

**Example:**
```
Robot parameters:
  wheelbase (L) = 0.30 m
  max_linear_vel = 0.5 m/s
  max_angular_vel = 1.0 rad/s

Command: {linear.x: 0.3, angular.z: 0.5}

Calculation:
  v_left = 0.3 - (0.5 × 0.30 / 2) = 0.3 - 0.075 = 0.225 m/s
  v_right = 0.3 + (0.5 × 0.30 / 2) = 0.3 + 0.075 = 0.375 m/s
```

### Conversion to PI_API Format

PI_API expects `throttle` and `steering` in range [-1.0, 1.0]:

```python
def twist_to_motor_commands(v_x, omega_z, wheelbase, max_v):
    """
    Convert Twist to motor commands.

    Args:
        v_x: Linear velocity (m/s)
        omega_z: Angular velocity (rad/s)
        wheelbase: Distance between wheels (m)
        max_v: Maximum linear velocity (m/s)

    Returns:
        (throttle, steering) in range [-1.0, 1.0]
    """
    # Calculate wheel velocities
    v_left = v_x - (omega_z * wheelbase / 2.0)
    v_right = v_x + (omega_z * wheelbase / 2.0)

    # Normalize to [-1.0, 1.0]
    throttle = (v_left + v_right) / (2.0 * max_v)

    # Steering based on difference
    if abs(v_x) > 0.01:
        # Normal steering (arc)
        steering = (v_right - v_left) / (2.0 * max_v)
    else:
        # Pivot turn (in place rotation)
        steering = omega_z / max_angular_vel

    # Clamp values
    throttle = max(-1.0, min(1.0, throttle))
    steering = max(-1.0, min(1.0, steering))

    return throttle, steering
```

### Robot Parameters (WayfindR)

Based on typical skid-steer configurations:

```yaml
# Physical parameters
wheelbase: 0.30          # meters (30 cm between left/right wheels)
wheel_radius: 0.065      # meters (65mm diameter wheels)
wheel_circumference: 0.408  # meters (2πr)

# Velocity limits
max_linear_velocity: 0.5    # m/s
max_angular_velocity: 1.0   # rad/s

# Control parameters
cmd_vel_timeout: 0.5     # seconds (stop if no cmd received)
control_frequency: 20    # Hz (how often to process cmd_vel)
```

---

## Safety Features

### 1. Command Timeout (Watchdog)

**Problem:** If ROS2 crashes or network fails, robot continues at last commanded velocity.

**Solution:** Watchdog timer that stops robot if no cmd_vel received within timeout.

```python
class CmdVelBridge(Node):
    def __init__(self):
        self.cmd_vel_timeout = 0.5  # seconds
        self.last_cmd_time = time.time()

        # Create timer to check for stale commands
        self.create_timer(0.1, self.watchdog_callback)

    def watchdog_callback(self):
        if time.time() - self.last_cmd_time > self.cmd_vel_timeout:
            # No command received in timeout period
            self.send_stop_command()
            self.get_logger().warn('cmd_vel timeout - stopping robot')
```

**Timeout Value:** 0.5 seconds (Nav2 publishes at ~20 Hz, so 10 missed messages)

---

### 2. Velocity Limiting

**Problem:** Nav2 might command velocities exceeding robot capabilities.

**Solution:** Clamp velocities to safe maximum values.

```python
def limit_velocity(self, v_x, omega_z):
    """Limit velocities to robot capabilities."""
    v_x = max(-self.max_linear_vel, min(self.max_linear_vel, v_x))
    omega_z = max(-self.max_angular_vel, min(self.max_angular_vel, omega_z))
    return v_x, omega_z
```

---

### 3. Emergency Stop Integration

**Problem:** Need ability to immediately halt robot from ROS2.

**Solution:** Subscribe to emergency stop topic.

```python
# Subscribe to emergency stop
self.estop_sub = self.create_subscription(
    Bool, '/emergency_stop', self.estop_callback, 10
)

def estop_callback(self, msg):
    if msg.data:
        self.emergency_stop()
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
```

**HTTP Call:**
```python
def emergency_stop(self):
    try:
        response = requests.post(
            f'{self.api_url}/api/emergency_stop',
            timeout=0.5
        )
    except Exception as e:
        self.get_logger().error(f'E-stop failed: {e}')
```

---

### 4. Connection Monitoring

**Problem:** HTTP connection to PI_API might fail.

**Solution:** Monitor connection health and handle failures gracefully.

```python
def send_motor_command(self, throttle, steering):
    try:
        response = requests.post(
            f'{self.api_url}/api/control/move',
            json={'throttle': throttle, 'steering': steering},
            timeout=0.1  # Fast timeout for real-time control
        )

        if response.status_code == 200:
            self.connection_failures = 0
        else:
            self.handle_connection_error()

    except requests.exceptions.RequestException as e:
        self.connection_failures += 1

        if self.connection_failures >= 5:
            self.get_logger().error('Lost connection to PI_API')
            # Could trigger emergency stop or safe mode
```

---

### 5. Graceful Shutdown

**Problem:** Node shutdown should safely stop robot.

**Solution:** Override destroy_node to send stop command.

```python
def destroy_node(self):
    """Safely shutdown node."""
    self.get_logger().info('Shutting down - stopping robot')
    self.send_stop_command()
    super().destroy_node()
```

---

## Odometry Publishing

### Message Structure

```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

odom_msg = Odometry()
odom_msg.header.stamp = self.get_clock().now().to_msg()
odom_msg.header.frame_id = 'odom'
odom_msg.child_frame_id = 'base_link'

# Position (from dead reckoning)
odom_msg.pose.pose.position.x = self.x
odom_msg.pose.pose.position.y = self.y
odom_msg.pose.pose.position.z = 0.0

# Orientation (yaw to quaternion)
quat = quaternion_from_euler(0, 0, self.theta)
odom_msg.pose.pose.orientation = quat

# Velocity (last commanded)
odom_msg.twist.twist.linear.x = self.v_x
odom_msg.twist.twist.linear.y = 0.0
odom_msg.twist.twist.angular.z = self.omega_z

# Covariance (dead reckoning is inaccurate)
odom_msg.pose.covariance = [
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # y variance
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # z variance (not used)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # roll variance (not used)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # pitch variance (not used)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.5,  # yaw variance
]

odom_msg.twist.covariance = [
    0.05, 0.0, 0.0, 0.0, 0.0, 0.0,  # vx variance
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vy variance (not used)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vz variance (not used)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vroll variance (not used)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vpitch variance (not used)
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1,   # vyaw variance
]
```

### Dead Reckoning Algorithm

Simple integration-based odometry (without encoders):

```python
def update_odometry(self, dt):
    """
    Update robot pose using dead reckoning.

    Args:
        dt: Time delta since last update (seconds)
    """
    # Current velocities (last commanded)
    v_x = self.current_v_x      # m/s
    omega_z = self.current_omega_z  # rad/s

    # Update heading first
    self.theta += omega_z * dt

    # Wrap angle to [-π, π]
    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    # Update position (in robot frame, then transform to odom frame)
    delta_x = v_x * math.cos(self.theta) * dt
    delta_y = v_x * math.sin(self.theta) * dt

    self.x += delta_x
    self.y += delta_y
```

**Limitations:**
- No encoder feedback (open-loop)
- Assumes commanded velocity = actual velocity
- Drift accumulates over time
- Wheel slip not accounted for

**Future Improvements:**
- Add wheel encoders for closed-loop odometry
- Integrate IMU for better heading
- Use sensor fusion (robot_localization package)

### Publishing Rate

**Odometry:** 50 Hz (recommended for Nav2)
```python
self.create_timer(0.02, self.publish_odometry)  # 50 Hz
```

**TF Broadcast:** 50 Hz (same as odometry)
```python
# Published in same callback as odometry
self.tf_broadcaster.sendTransform(transform)
```

**Rationale:**
- Nav2 expects 20-50 Hz odometry
- Higher rates improve localization accuracy
- 50 Hz balances accuracy and CPU usage

---

## Implementation Details

### ROS2 Node Structure

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import requests
import math
import time

class CmdVelBridge(Node):
    """
    Bridge between ROS2 cmd_vel and PI_API motor control.
    """

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Declare parameters
        self.declare_parameters()

        # Initialize state
        self.init_state()

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timers
        self.create_timer(0.02, self.odometry_timer_callback)  # 50 Hz
        self.create_timer(0.1, self.watchdog_callback)  # 10 Hz

        self.get_logger().info('cmd_vel bridge initialized')

    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages."""
        self.last_cmd_time = time.time()

        # Extract velocities
        v_x = msg.linear.x
        omega_z = msg.angular.z

        # Limit velocities
        v_x, omega_z = self.limit_velocity(v_x, omega_z)

        # Convert to motor commands
        throttle, steering = self.twist_to_motor_commands(v_x, omega_z)

        # Send to robot
        self.send_motor_command(throttle, steering)

        # Update state for odometry
        self.current_v_x = v_x
        self.current_omega_z = omega_z
```

### HTTP Communication

```python
def send_motor_command(self, throttle, steering):
    """Send motor command to PI_API."""
    try:
        response = requests.post(
            f'{self.api_url}/api/control/move',
            json={
                'throttle': float(throttle),
                'steering': float(steering),
                'duration': None  # Continuous until next command
            },
            timeout=0.1
        )

        if response.status_code != 200:
            self.get_logger().warn(f'Motor command failed: {response.status_code}')

    except requests.exceptions.Timeout:
        self.get_logger().error('Motor command timeout')
    except Exception as e:
        self.get_logger().error(f'Motor command error: {e}')
```

---

## Configuration Parameters

### ROS2 Parameters (YAML)

```yaml
cmd_vel_bridge:
  ros__parameters:
    # Network
    api_url: "http://localhost:8000"
    api_timeout: 0.1  # seconds

    # Robot physical parameters
    wheelbase: 0.30  # meters
    wheel_radius: 0.065  # meters

    # Velocity limits
    max_linear_velocity: 0.5  # m/s
    max_angular_velocity: 1.0  # rad/s

    # Control parameters
    cmd_vel_timeout: 0.5  # seconds
    control_frequency: 20  # Hz

    # Odometry parameters
    odom_frequency: 50  # Hz
    publish_tf: true
    odom_frame: "odom"
    base_frame: "base_link"

    # Covariance (dead reckoning uncertainty)
    pose_covariance_x: 0.1
    pose_covariance_y: 0.1
    pose_covariance_yaw: 0.5
    twist_covariance_vx: 0.05
    twist_covariance_vyaw: 0.1
```

### Loading Parameters

```python
def declare_parameters(self):
    """Declare ROS2 parameters with defaults."""
    self.declare_parameter('api_url', 'http://localhost:8000')
    self.declare_parameter('wheelbase', 0.30)
    self.declare_parameter('max_linear_velocity', 0.5)
    self.declare_parameter('max_angular_velocity', 1.0)
    self.declare_parameter('cmd_vel_timeout', 0.5)
    self.declare_parameter('odom_frequency', 50)
    self.declare_parameter('publish_tf', True)

def get_parameters(self):
    """Load parameters."""
    self.api_url = self.get_parameter('api_url').value
    self.wheelbase = self.get_parameter('wheelbase').value
    self.max_linear_vel = self.get_parameter('max_linear_velocity').value
    self.max_angular_vel = self.get_parameter('max_angular_velocity').value
```

---

## Testing Strategy

### Unit Tests

1. **Kinematics Tests**
```python
def test_twist_to_motor_commands():
    # Test forward motion
    throttle, steering = twist_to_motor_commands(0.5, 0.0, 0.30, 0.5)
    assert abs(throttle - 1.0) < 0.01
    assert abs(steering) < 0.01

    # Test rotation in place
    throttle, steering = twist_to_motor_commands(0.0, 1.0, 0.30, 0.5)
    assert abs(throttle) < 0.01
    assert abs(steering - 1.0) < 0.1
```

2. **Odometry Tests**
```python
def test_dead_reckoning():
    # Test straight line motion
    bridge = CmdVelBridge()
    bridge.current_v_x = 0.5
    bridge.current_omega_z = 0.0
    bridge.update_odometry(1.0)  # 1 second
    assert abs(bridge.x - 0.5) < 0.01
    assert abs(bridge.y) < 0.01
```

### Integration Tests

1. **HTTP Communication Test**
```bash
# Start PI_API
cd PI_API && python3 main.py

# In another terminal, test bridge
ros2 run cmd_vel_bridge cmd_vel_bridge

# In another terminal, send test command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0.2}"
```

2. **Odometry Verification**
```bash
# Check odometry publishing
ros2 topic echo /odom

# Check TF tree
ros2 run tf2_tools view_frames
```

3. **Nav2 Integration Test**
```bash
# Full navigation stack
ros2 launch ros2_comprehensive_attempt navigation.launch.py

# Send navigation goal
ros2 run cmd_vel_bridge test_navigation.py
```

### Simulation Testing

Use Gazebo for safe testing before deploying to hardware:

```bash
# Launch Gazebo simulation
ros2 launch gazebo_ros gazebo.launch.py

# Launch cmd_vel_bridge in simulation mode
ros2 run cmd_vel_bridge cmd_vel_bridge --ros-args -p api_url:=http://simulation:8000
```

---

## References

### ROS2 Documentation

1. **diff_drive_controller** - Official ROS2 Control documentation
   [ROS2 Control: Humble Jan 2026](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)

2. **Wheeled Mobile Robot Kinematics** - Differential drive math
   [ROS2 Control: Kinematics](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html)

3. **Setting Up Odometry** - Nav2 odometry requirements
   [Nav2 Documentation](https://navigation.ros.org/setup_guides/odom/setup_odom.html)

4. **nav_msgs/Odometry** - Message specification
   [ROS2 API Documentation](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

5. **geometry_msgs/Twist** - Velocity command message
   [ROS Documentation](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)

### Best Practices

6. **REP 103** - Standard Units of Measure and Coordinate Conventions
   [ROS Enhancement Proposal 103](https://www.ros.org/reps/rep-0103.html)

7. **REP 105** - Coordinate Frames for Mobile Platforms
   [ROS Enhancement Proposal 105](https://www.ros.org/reps/rep-0105.html)

8. **Nav2 Tuning Guide** - Performance optimization
   [Nav2 Tuning](https://navigation.ros.org/tuning/index.html)

### Tutorials

9. **How to Publish Odometry Information** - Python implementation guide
   [Automatic Addison Tutorial](https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/)

10. **Driving a Robot** - ROS2 velocity control basics
    [Clearpath Robotics Docs](https://docs.clearpathrobotics.com/docs/ros/tutorials/driving/)

### Community Resources

11. **ROS Answers** - cmd_vel to wheel velocities conversion
    [ROS Answers Discussion](https://answers.ros.org/question/334022/how-to-split-cmd_vel-into-left-and-right-wheel-of-2wd-robot/)

12. **GitHub - merose/diff_drive** - Reference implementation
    [GitHub Repository](https://github.com/merose/diff_drive)

---

## Future Enhancements

### Phase 2: Encoder Integration

- Add wheel encoder feedback
- Closed-loop velocity control
- Accurate odometry

### Phase 3: IMU Integration

- Better heading estimation
- Accelerometer-based motion detection
- Sensor fusion with robot_localization

### Phase 4: Battery Monitoring

- Subscribe to battery voltage from PI_API
- Publish battery state to /battery_state
- Low battery warnings to Nav2

### Phase 5: Diagnostic Publishing

- Robot status to /diagnostics
- Motor health monitoring
- Connection quality metrics

---

## Appendix A: Mathematical Proofs

### Differential Drive Kinematics Derivation

For a differential drive robot with wheel separation `L`:

**Forward Kinematics:**
```
Given: v_left, v_right (wheel velocities)

The robot's linear velocity is the average of wheel velocities:
  v_x = (v_left + v_right) / 2

The robot's angular velocity comes from wheel speed difference:
  ω_z = (v_right - v_left) / L

Proof: The outer wheel travels farther in a turn.
  For radius R: v_outer = (R + L/2) × ω
               v_inner = (R - L/2) × ω
  Difference: v_outer - v_inner = L × ω
  Therefore: ω = (v_right - v_left) / L
```

**Inverse Kinematics:**
```
Given: v_x, ω_z (robot body velocities)

From the center of rotation:
  left wheel: v_left = v_center - ω × (L/2)
  right wheel: v_right = v_center + ω × (L/2)

Where v_center = v_x (robot forward velocity)

Therefore:
  v_left = v_x - ω_z × (L/2)
  v_right = v_x + ω_z × (L/2)
```

---

## Appendix B: Coordinate Frame Transforms

### Euler to Quaternion Conversion

```python
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.

    For 2D robots: roll=0, pitch=0, yaw=θ
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return (qx, qy, qz, qw)
```

### Quaternion to Euler Conversion

```python
def euler_from_quaternion(qx, qy, qz, qw):
    """
    Convert quaternion to Euler angles.

    Returns: (roll, pitch, yaw)
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)
```

---

**Document Version:** 1.0.0
**Last Updated:** 2026-01-11
**Status:** Final Design - Ready for Implementation
