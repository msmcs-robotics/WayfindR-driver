# cmd_vel Bridge Usage Guide

**Component:** ROS2 Nav2 to PI_API Bridge
**Version:** 1.0.0
**Date:** 2026-01-11

---

## Quick Start

### Prerequisites

1. **ROS2 Humble** installed and sourced
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **PI_API running** on the robot
   ```bash
   cd /home/devel/Desktop/WayfindR-driver/PI_API
   python3 main.py
   ```

3. **Python dependencies**
   ```bash
   pip3 install requests
   ```

### Basic Usage

```bash
# Terminal 1: Start PI_API
cd /home/devel/Desktop/WayfindR-driver/PI_API
python3 main.py

# Terminal 2: Start cmd_vel bridge
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt
python3 scripts/cmd_vel_bridge.py

# Terminal 3: Test with manual command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0.2}" \
  --rate 10
```

---

## Installation

### Option 1: Standalone Script (No Package)

This is the simplest approach for testing:

```bash
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts
python3 cmd_vel_bridge.py
```

### Option 2: ROS2 Package (Recommended for Production)

Create a proper ROS2 package:

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create cmd_vel_bridge \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs nav_msgs std_msgs tf2_ros

# Copy files
cp /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/cmd_vel_bridge.py \
   ~/ros2_ws/src/cmd_vel_bridge/cmd_vel_bridge/

# Edit setup.py to add entry point
# Add to entry_points:
#   'console_scripts': [
#       'cmd_vel_bridge = cmd_vel_bridge.cmd_vel_bridge:main',
#   ],

# Build
cd ~/ros2_ws
colcon build --packages-select cmd_vel_bridge

# Source
source install/setup.bash

# Run
ros2 run cmd_vel_bridge cmd_vel_bridge
```

---

## Configuration

### Method 1: Command Line Parameters

```bash
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p api_url:=http://192.168.1.100:8000 \
  -p wheelbase:=0.35 \
  -p max_linear_velocity:=0.6 \
  -p max_angular_velocity:=1.2
```

### Method 2: Parameter File

```bash
python3 scripts/cmd_vel_bridge.py --ros-args \
  --params-file config/cmd_vel_bridge_params.yaml
```

### Method 3: Launch File

```bash
ros2 launch ros2_comprehensive_attempt cmd_vel_bridge.launch.py \
  api_url:=http://192.168.1.100:8000
```

---

## Testing

### 1. Test API Connection

```bash
# Check if PI_API is running
curl http://localhost:8000/api/health

# Expected response:
# {"status":"healthy","timestamp":"...","robot_connected":true,"uptime":...}
```

### 2. Test cmd_vel Subscription

```bash
# Start bridge
python3 scripts/cmd_vel_bridge.py

# In another terminal, check if subscribed
ros2 topic info /cmd_vel

# Expected output:
# Subscription count: 1
```

### 3. Test Forward Movement

```bash
# Send forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0}" \
  --rate 10

# Robot should move forward at 0.3 m/s
# Press Ctrl+C to stop
```

### 4. Test Rotation

```bash
# Send rotation command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0.5}" \
  --rate 10

# Robot should rotate in place at 0.5 rad/s (counter-clockwise)
```

### 5. Test Combined Movement

```bash
# Send arc turn command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0.3}" \
  --rate 10

# Robot should drive in an arc (forward + turning)
```

### 6. Test Odometry Publishing

```bash
# Check odometry topic
ros2 topic echo /odom

# Expected output (continuously updating):
# header:
#   stamp: ...
#   frame_id: odom
# child_frame_id: base_link
# pose:
#   pose:
#     position: {x: ..., y: ..., z: 0.0}
#     orientation: {x: ..., y: ..., z: ..., w: ...}
# twist:
#   twist:
#     linear: {x: ..., y: 0.0, z: 0.0}
#     angular: {x: 0.0, y: 0.0, z: ...}
```

### 7. Test TF Broadcasting

```bash
# Check TF tree
ros2 run tf2_ros tf2_echo odom base_link

# Expected output:
# At time ...
# - Translation: [x, y, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, z, w]
```

### 8. Test Watchdog (Safety)

```bash
# Start bridge
python3 scripts/cmd_vel_bridge.py

# Send command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0}" \
  --once

# Robot should move, then stop after 0.5 seconds (timeout)
# Check logs for "cmd_vel timeout - stopping robot"
```

### 9. Test Emergency Stop

```bash
# Send emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once

# Robot should stop immediately
# Check logs for "EMERGENCY STOP ACTIVATED"
```

---

## Integration with Nav2

### Full Navigation Stack

```bash
# Terminal 1: Start PI_API
cd /home/devel/Desktop/WayfindR-driver/PI_API
python3 main.py

# Terminal 2: Start LiDAR
ros2 launch sllidar_ros2 sllidar_c1_launch.py

# Terminal 3: Start localization (AMCL)
ros2 launch ros2_comprehensive_attempt localization.launch.py \
  map:=/path/to/map.yaml

# Terminal 4: Start cmd_vel bridge
python3 /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/cmd_vel_bridge.py

# Terminal 5: Start Nav2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 6: Send navigation goal
ros2 run ros2_comprehensive_attempt navigator.py \
  --waypoints waypoints/office.yaml \
  --goto reception
```

### Verify Integration

```bash
# Check all topics
ros2 topic list

# Should see:
# /cmd_vel           (Nav2 -> cmd_vel_bridge)
# /odom              (cmd_vel_bridge -> Nav2)
# /scan              (LiDAR -> Nav2)
# /map               (map_server -> Nav2)
# /tf                (all nodes)
# /tf_static         (all nodes)

# Check TF tree
ros2 run tf2_tools view_frames

# Should see:
# map -> odom -> base_link -> laser
```

---

## Troubleshooting

### Problem: Bridge won't start

**Symptom:** ImportError or module not found

**Solution:**
```bash
# Install missing dependencies
pip3 install requests

# Source ROS2
source /opt/ros/humble/setup.bash

# Check Python version (need 3.10+)
python3 --version
```

---

### Problem: Cannot connect to PI_API

**Symptom:** "Cannot connect to API" in logs

**Solution:**
```bash
# Check if PI_API is running
curl http://localhost:8000/api/health

# If not running, start it
cd /home/devel/Desktop/WayfindR-driver/PI_API
python3 main.py

# Check if port is correct
netstat -an | grep 8000

# If on different machine, update api_url parameter
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p api_url:=http://ROBOT_IP:8000
```

---

### Problem: Robot not moving

**Symptom:** cmd_vel messages received but robot doesn't move

**Checklist:**
1. Check PI_API logs for incoming requests
   ```bash
   # Should see POST /api/control/move requests
   ```

2. Check motor driver connection
   ```bash
   # In PI_API terminal, should see motor commands
   ```

3. Check velocity limits
   ```bash
   # Verify cmd_vel values are within limits
   ros2 topic echo /cmd_vel
   ```

4. Test motor control directly
   ```bash
   # Bypass bridge and test PI_API directly
   curl -X POST http://localhost:8000/api/control/move \
     -H "Content-Type: application/json" \
     -d '{"throttle": 0.3, "steering": 0}'
   ```

---

### Problem: Robot moves but odometry is wrong

**Symptom:** /odom publishes incorrect position

**Possible Causes:**

1. **Incorrect wheelbase parameter**
   ```bash
   # Measure actual distance between wheels
   # Update parameter
   python3 scripts/cmd_vel_bridge.py --ros-args \
     -p wheelbase:=MEASURED_VALUE
   ```

2. **Dead reckoning drift** (expected)
   - Dead reckoning accumulates error over time
   - Solution: Use AMCL for localization correction
   - AMCL publishes map->odom transform to correct drift

3. **Wheel slip**
   - Commanded velocity ≠ actual velocity
   - Solution: Add wheel encoders for closed-loop control

---

### Problem: cmd_vel timeout errors

**Symptom:** "cmd_vel timeout - stopping robot" in logs

**Causes:**
1. Nav2 not publishing to /cmd_vel
2. Publishing rate too slow
3. Network latency

**Solutions:**
```bash
# Check if Nav2 is publishing
ros2 topic hz /cmd_vel

# Should be ~20 Hz

# Increase timeout if needed
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p cmd_vel_timeout:=1.0
```

---

### Problem: TF tree broken

**Symptom:** "Transform timeout" errors in Nav2

**Solution:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Should see continuous chain:
# map -> odom -> base_link -> laser

# If missing odom->base_link:
# - Check cmd_vel_bridge is running
# - Check publish_tf parameter is true

# If missing map->odom:
# - Start AMCL (localization)
# - Set initial pose in RViz
```

---

## Performance Tuning

### Reduce CPU Usage

```yaml
# In config/cmd_vel_bridge_params.yaml
odom_frequency: 30  # Lower from 50 Hz
```

### Improve Accuracy

```yaml
# Higher odometry frequency (if CPU allows)
odom_frequency: 100

# Reduce covariance (if odometry is accurate)
pose_covariance_x: 0.05
pose_covariance_y: 0.05
```

### Increase Responsiveness

```yaml
# Faster command processing
cmd_vel_timeout: 0.3  # Lower timeout

# Faster HTTP requests (may fail on slow networks)
api_timeout: 0.05
```

---

## Monitoring and Debugging

### View Bridge Status

```bash
# Check node info
ros2 node info /cmd_vel_bridge

# Check parameters
ros2 param list /cmd_vel_bridge

# Get specific parameter
ros2 param get /cmd_vel_bridge wheelbase
```

### Monitor Topics

```bash
# Monitor cmd_vel input
ros2 topic echo /cmd_vel

# Monitor odometry output
ros2 topic echo /odom

# Check publishing rates
ros2 topic hz /cmd_vel
ros2 topic hz /odom
```

### View Logs

```bash
# Real-time logs
ros2 run rqt_console rqt_console

# Or use command line
ros2 topic echo /rosout
```

### Visualize in RViz

```bash
# Start RViz
rviz2

# Add displays:
# - Odometry -> Topic: /odom
# - TF -> Show odom, base_link
# - LaserScan -> Topic: /scan
# - Map -> Topic: /map
```

---

## Advanced Usage

### Record Data for Analysis

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /cmd_vel /odom /scan /tf

# Play back recording
ros2 bag play <bag_file>
```

### Custom Velocity Limits

For different environments (indoor vs outdoor):

```bash
# Indoor (slower, safer)
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p max_linear_velocity:=0.3 \
  -p max_angular_velocity:=0.5

# Outdoor (faster)
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p max_linear_velocity:=0.8 \
  -p max_angular_velocity:=1.5
```

### Multiple Robots

Run separate bridges for multiple robots:

```bash
# Robot 1
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p api_url:=http://192.168.1.101:8000 \
  -r __ns:=/robot1

# Robot 2
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p api_url:=http://192.168.1.102:8000 \
  -r __ns:=/robot2
```

---

## Safety Considerations

### 1. Always Test in Safe Environment

- Clear area with no obstacles
- Emergency stop button accessible
- Person ready to catch robot

### 2. Start with Low Velocities

```bash
# Test with 20% speed first
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p max_linear_velocity:=0.1 \
  -p max_angular_velocity:=0.2
```

### 3. Monitor Watchdog

The bridge automatically stops the robot if:
- No cmd_vel received within timeout (default 0.5s)
- API connection lost
- Emergency stop triggered

### 4. Emergency Stop Always Available

```bash
# Keep this command ready in a terminal
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
```

Or use the PI_API web dashboard emergency stop button.

---

## Next Steps

After verifying the bridge works:

1. **Calibrate wheelbase** - Measure actual distance for accurate turns
2. **Add encoders** - Upgrade from dead reckoning to closed-loop odometry
3. **Tune Nav2** - Adjust navigation parameters for your robot
4. **Create maps** - Use SLAM to map your environment
5. **Add waypoints** - Define navigation goals
6. **Test autonomy** - Full waypoint navigation missions

See the design document for future enhancement ideas:
- `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/findings/CMD_VEL_BRIDGE_DESIGN.md`

---

**Document Version:** 1.0.0
**Last Updated:** 2026-01-11
