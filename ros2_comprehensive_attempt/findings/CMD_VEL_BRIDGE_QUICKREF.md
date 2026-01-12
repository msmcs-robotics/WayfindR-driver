# cmd_vel Bridge Quick Reference

**One-page reference for common commands and troubleshooting**

---

## Quick Start (3 Steps)

```bash
# 1. Start PI_API
cd /home/devel/Desktop/WayfindR-driver/PI_API && python3 main.py

# 2. Start bridge
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt
python3 scripts/cmd_vel_bridge.py

# 3. Test
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0}" --rate 10
```

---

## File Locations

```
ros2_comprehensive_attempt/
├── scripts/
│   └── cmd_vel_bridge.py          # Main bridge node
├── config/
│   └── cmd_vel_bridge_params.yaml # Configuration
├── launch/
│   └── cmd_vel_bridge.launch.py   # Launch file
└── findings/
    ├── CMD_VEL_BRIDGE_DESIGN.md   # Full design doc
    ├── CMD_VEL_BRIDGE_USAGE.md    # Detailed usage guide
    ├── CMD_VEL_BRIDGE_RESEARCH.md # Research findings
    └── CMD_VEL_BRIDGE_QUICKREF.md # This file
```

---

## Common Commands

### Start/Stop

```bash
# Start with defaults
python3 scripts/cmd_vel_bridge.py

# Start with custom API
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p api_url:=http://192.168.1.100:8000

# Start with parameters file
python3 scripts/cmd_vel_bridge.py --ros-args \
  --params-file config/cmd_vel_bridge_params.yaml

# Stop (Ctrl+C or)
ros2 lifecycle set /cmd_vel_bridge shutdown
```

### Testing

```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}" --rate 10

# Backward
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: -0.3, y: 0, z: 0}" --rate 10

# Rotate left (counter-clockwise)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "angular: {x: 0, y: 0, z: 0.5}" --rate 10

# Rotate right (clockwise)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "angular: {x: 0, y: 0, z: -0.5}" --rate 10

# Arc turn (forward + rotate)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.3, y: 0, z: 0}
   angular: {x: 0, y: 0, z: 0.3}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once

# Emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
```

### Monitoring

```bash
# Check topics
ros2 topic list

# Monitor cmd_vel
ros2 topic echo /cmd_vel

# Monitor odometry
ros2 topic echo /odom

# Check rates
ros2 topic hz /cmd_vel
ros2 topic hz /odom

# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link

# Node info
ros2 node info /cmd_vel_bridge

# View parameters
ros2 param list /cmd_vel_bridge
ros2 param get /cmd_vel_bridge wheelbase
```

---

## Key Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `api_url` | `http://localhost:8000` | - | PI_API endpoint |
| `wheelbase` | `0.30` | 0.1-1.0 m | Distance between wheels |
| `max_linear_velocity` | `0.5` | 0.1-2.0 m/s | Max forward speed |
| `max_angular_velocity` | `1.0` | 0.1-3.0 rad/s | Max rotation speed |
| `cmd_vel_timeout` | `0.5` | 0.1-2.0 s | Command timeout |
| `odom_frequency` | `50` | 10-100 Hz | Odometry rate |

### Change Parameter

```bash
# At startup
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p wheelbase:=0.35

# At runtime (future feature)
ros2 param set /cmd_vel_bridge wheelbase 0.35
```

---

## Troubleshooting

### Cannot connect to PI_API

```bash
# Check API is running
curl http://localhost:8000/api/health

# If not running
cd /home/devel/Desktop/WayfindR-driver/PI_API
python3 main.py

# Check correct URL
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p api_url:=http://CORRECT_IP:8000
```

### Robot not moving

```bash
# 1. Check cmd_vel is being published
ros2 topic hz /cmd_vel

# 2. Check cmd_vel values
ros2 topic echo /cmd_vel

# 3. Test motor control directly
curl -X POST http://localhost:8000/api/control/move \
  -H "Content-Type: application/json" \
  -d '{"throttle": 0.3, "steering": 0}'

# 4. Check motor driver in PI_API logs
```

### Odometry not publishing

```bash
# Check topic
ros2 topic hz /odom

# If zero, check bridge logs
# Should see "cmd_vel bridge initialized"

# Check if node is running
ros2 node list | grep cmd_vel_bridge
```

### TF errors

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Should see: odom -> base_link

# If missing, check publish_tf parameter
ros2 param get /cmd_vel_bridge publish_tf

# Check transform
ros2 run tf2_ros tf2_echo odom base_link
```

### Watchdog timeout

```bash
# Normal if cmd_vel stops publishing
# Increase timeout if network is slow
python3 scripts/cmd_vel_bridge.py --ros-args \
  -p cmd_vel_timeout:=1.0
```

---

## Kinematics Formulas

### Twist to Wheel Velocities

```
v_left = v_x - (ω_z × wheelbase / 2)
v_right = v_x + (ω_z × wheelbase / 2)
```

### Wheel Velocities to Twist

```
v_x = (v_left + v_right) / 2
ω_z = (v_right - v_left) / wheelbase
```

### Motor Commands (PI_API format)

```
throttle = (v_left + v_right) / (2 × max_vel)
steering = (v_right - v_left) / (2 × max_vel)  # if moving
steering = ω_z / max_omega                      # if stopped
```

---

## Common Values

### Velocities

| Robot Type | Max Linear | Max Angular |
|------------|------------|-------------|
| Indoor (safe) | 0.3 m/s | 0.5 rad/s |
| Indoor (normal) | 0.5 m/s | 1.0 rad/s |
| Outdoor | 0.8 m/s | 1.5 rad/s |

### Conversions

```
0.5 m/s = 1.8 km/h = 1.1 mph
1.0 rad/s = 57.3 deg/s = 16 RPM (at 30cm wheelbase)
```

---

## Integration with Nav2

### Full Stack Launch

```bash
# Terminal 1: PI_API
cd /home/devel/Desktop/WayfindR-driver/PI_API
python3 main.py

# Terminal 2: LiDAR
ros2 launch sllidar_ros2 sllidar_c1_launch.py

# Terminal 3: Localization
ros2 launch ros2_comprehensive_attempt localization.launch.py \
  map:=/path/to/map.yaml

# Terminal 4: cmd_vel bridge
python3 /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/cmd_vel_bridge.py

# Terminal 5: Nav2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 6: RViz
rviz2

# Terminal 7: Send goal
ros2 run ros2_comprehensive_attempt navigator.py \
  --waypoints waypoints/office.yaml --goto reception
```

### Minimal Nav2 Test

```bash
# Start bridge + Nav2 in simulation mode
python3 scripts/cmd_vel_bridge.py &
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

# Set initial pose in RViz
# Send goal in RViz (2D Goal Pose)
# Watch cmd_vel bridge logs for commands
```

---

## Safety Checklist

Before running on robot:

- [ ] Clear test area (no obstacles, people, pets)
- [ ] Emergency stop button accessible
- [ ] Start with low velocities (0.1 m/s)
- [ ] Test forward/backward/rotation separately
- [ ] Verify watchdog stops robot (disconnect cmd_vel)
- [ ] Verify emergency stop works
- [ ] Check API connection is stable
- [ ] Battery is charged (> 50%)

---

## HTTP API Endpoints (PI_API)

```bash
# Health check
curl http://localhost:8000/api/health

# Move (continuous)
curl -X POST http://localhost:8000/api/control/move \
  -H "Content-Type: application/json" \
  -d '{"throttle": 0.5, "steering": 0.2}'

# Stop
curl -X POST http://localhost:8000/api/control/stop

# Emergency stop
curl -X POST http://localhost:8000/api/emergency_stop

# Get state
curl http://localhost:8000/api/control/state
```

---

## Performance Metrics

| Metric | Target | Typical |
|--------|--------|---------|
| cmd_vel latency | < 50ms | 10-30ms |
| Odometry rate | 50 Hz | 50 Hz |
| CPU usage | < 10% | 3-5% |
| Memory | < 100 MB | ~50 MB |
| Network | < 10 KB/s | 1-2 KB/s |

---

## Log Messages

### Normal Operation

```
[INFO] cmd_vel Bridge Initialized
[INFO] API connection successful
[DEBUG] cmd_vel: v=0.30 ω=0.20 -> throttle=0.60 steering=0.40
```

### Warnings

```
[WARN] cmd_vel timeout (0.52s) - stopping robot
[WARN] Motor command failed: 500
```

### Errors

```
[ERROR] Cannot connect to API: Connection refused
[ERROR] Lost connection to PI_API after 5 failures
[ERROR] EMERGENCY STOP ACTIVATED
```

---

## Environment Variables

```bash
# ROS2 domain (if using multiple robots)
export ROS_DOMAIN_ID=0

# ROS2 localhost only (security)
export ROS_LOCALHOST_ONLY=1

# Log level
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
```

---

## Useful Aliases

Add to `~/.bashrc`:

```bash
alias bridge='cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt && python3 scripts/cmd_vel_bridge.py'
alias piapi='cd /home/devel/Desktop/WayfindR-driver/PI_API && python3 main.py'
alias cmdvel='ros2 topic pub /cmd_vel geometry_msgs/Twist'
alias estop='ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once'
```

---

## Version Info

- **Bridge Version:** 1.0.0
- **ROS2 Version:** Humble
- **Python Version:** 3.10+
- **PI_API Version:** Compatible with all versions
- **Last Updated:** 2026-01-11

---

## Getting Help

1. **Read detailed docs:**
   - Design: `findings/CMD_VEL_BRIDGE_DESIGN.md`
   - Usage: `findings/CMD_VEL_BRIDGE_USAGE.md`
   - Research: `findings/CMD_VEL_BRIDGE_RESEARCH.md`

2. **Check logs:**
   ```bash
   # Bridge logs
   ros2 run rqt_console rqt_console

   # PI_API logs (in terminal where it's running)
   ```

3. **Test components separately:**
   - PI_API: `curl http://localhost:8000/api/health`
   - ROS2: `ros2 topic list`
   - Bridge: Check initialization message

4. **Common issues:**
   - ROS2 not sourced: `source /opt/ros/humble/setup.bash`
   - API not running: Start PI_API first
   - Wrong parameters: Check config file or arguments

---

**Quick Ref Version:** 1.0.0
**Date:** 2026-01-11
