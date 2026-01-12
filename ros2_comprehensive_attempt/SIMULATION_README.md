# WayfindR Gazebo Simulation Setup

Quick reference guide for getting started with WayfindR Gazebo simulation.

## Quick Start

### 1. Install Gazebo Fortress

```bash
cd ~/your_workspace/src/ros2_comprehensive_attempt/scripts
./install_gazebo.sh
```

This will install:
- Gazebo Fortress simulator
- ROS-Gazebo bridge packages
- Required dependencies

### 2. Build Workspace

```bash
cd ~/your_workspace
colcon build --symlink-install
source install/setup.bash
```

### 3. Test Installation

```bash
cd ~/your_workspace/src/ros2_comprehensive_attempt/scripts
./test_simulation.sh
```

### 4. Launch Simulation

```bash
# Simple launch (empty world)
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py

# Test room environment
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py world:=test_room

# Obstacle course environment
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py world:=obstacle_course
```

## Common Usage Patterns

### SLAM Mapping in Simulation

Terminal 1 - Launch Gazebo:
```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py world:=obstacle_course
```

Terminal 2 - Launch SLAM:
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=slam \
  use_sim_time:=true
```

Terminal 3 - Control Robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 4 - Save Map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/sim_map
```

### Autonomous Navigation in Simulation

Terminal 1 - Launch Gazebo:
```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py world:=obstacle_course
```

Terminal 2 - Launch Navigation:
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=navigation \
  map:=~/maps/sim_map.yaml \
  use_sim_time:=true
```

Use RViz to:
1. Set initial pose (2D Pose Estimate)
2. Set navigation goal (2D Nav Goal)

## Launch File Parameters

### gazebo_sim.launch.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world` | `''` | World file: empty, test_room, obstacle_course, or path |
| `use_gui` | `true` | Launch Gazebo GUI |
| `use_rviz` | `true` | Launch RViz |
| `x_pose` | `0.0` | Robot spawn X position |
| `y_pose` | `0.0` | Robot spawn Y position |
| `z_pose` | `0.1` | Robot spawn Z position |
| `yaw_pose` | `0.0` | Robot spawn yaw orientation |

### Examples

```bash
# Headless (no GUI) for faster simulation
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_gui:=false \
  use_rviz:=false

# Custom spawn position
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  x_pose:=2.0 \
  y_pose:=3.0 \
  yaw_pose:=1.57

# Custom world file
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=/path/to/custom_world.sdf
```

## Available Worlds

### test_room.sdf
- Simple 10m x 8m rectangular room
- No obstacles
- Good for: Basic testing, movement validation, LIDAR verification

### obstacle_course.sdf
- Complex 15m x 12m environment
- Multiple obstacles (boxes, cylinders)
- Narrow passages
- Good for: Navigation testing, path planning, obstacle avoidance

## File Structure

```
ros2_comprehensive_attempt/
├── launch/
│   ├── gazebo_sim.launch.py       # Main simulation launch file
│   └── bringup.launch.py           # Navigation stack (supports use_sim_time)
├── worlds/
│   ├── test_room.sdf               # Simple test environment
│   └── obstacle_course.sdf         # Complex test environment
├── urdf/
│   └── wayfinder_robot.urdf.xacro  # Robot model with Gazebo plugins
├── config/
│   └── gazebo_sim.rviz             # RViz config for simulation
├── scripts/
│   ├── install_gazebo.sh           # Installation script
│   └── test_simulation.sh          # Test script
└── findings/
    └── gazebo-simulation-guide.md  # Comprehensive documentation
```

## Topics Published by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LIDAR data (360°, 10 Hz) |
| `/odom` | nav_msgs/Odometry | Robot odometry |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/clock` | rosgraph_msgs/Clock | Simulation time |
| `/joint_states` | sensor_msgs/JointState | Wheel joint states |

## Topics Subscribed by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

## Troubleshooting

### Gazebo won't start
```bash
# Check installation
gz sim --version

# Reinstall if needed
sudo apt install --reinstall ignition-fortress
```

### Robot doesn't spawn
```bash
# Check robot_description topic
ros2 topic echo /robot_description --once

# Verify URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro /path/to/wayfinder_robot.urdf.xacro)"
```

### No sensor data
```bash
# Check topics exist
ros2 topic list | grep -E '(scan|odom)'

# Check topic rates
ros2 topic hz /scan
ros2 topic hz /odom

# Verify use_sim_time
ros2 param get /robot_state_publisher use_sim_time
```

### Performance issues
```bash
# Launch without GUI
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_gui:=false

# Launch without RViz
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_rviz:=false
```

## Additional Resources

- **Full Documentation:** `findings/gazebo-simulation-guide.md`
- **Installation Script:** `scripts/install_gazebo.sh`
- **Test Script:** `scripts/test_simulation.sh`

## Key Differences: Simulation vs Hardware

| Aspect | Hardware | Simulation |
|--------|----------|------------|
| **Setup** | RPLidar driver, GPIO | Just launch file |
| **Time** | Real-time only | use_sim_time:=true |
| **Safety** | Physical risks | Completely safe |
| **Repeatability** | Difficult | Perfect |
| **Speed** | Real-time | Faster/slower possible |

## When to Use Simulation

✓ **Algorithm development**
✓ **Parameter tuning**
✓ **Pre-deployment testing**
✓ **Edge case scenarios**
✓ **Learning and demonstrations**

## When to Use Hardware

✓ **Final validation**
✓ **Real-world performance**
✓ **Sensor calibration**
✓ **Actual deployment**

## Next Steps

1. Read full guide: `findings/gazebo-simulation-guide.md`
2. Run test script: `scripts/test_simulation.sh`
3. Create your own world files
4. Test SLAM in simulation
5. Test navigation in simulation
6. Compare with rosbag testing

## Support

For detailed troubleshooting and advanced usage, see:
- `findings/gazebo-simulation-guide.md` - Complete documentation
- [ROS2 Gazebo Tutorials](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo Documentation](https://gazebosim.org/docs/fortress)

---

**Version:** 1.0
**Last Updated:** 2026-01-11
**Author:** WayfindR Development Team
