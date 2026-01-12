# WayfindR Quick Start Guide

**Last Updated:** 2026-01-11

## TL;DR - Get Running in 5 Minutes

### 1. Create a Map (First Time Only)

```bash
cd /path/to/ros2_comprehensive_attempt
./scripts/start_mapping.sh
```

Drive around, then save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 2. Run Navigation

```bash
./scripts/start_navigation.sh --map ~/maps/my_map.yaml
```

In RViz:
1. Click "2D Pose Estimate" → Click on map where robot is → Drag for orientation
2. Click "2D Nav Goal" → Click destination → Drag for final orientation
3. Watch robot navigate!

---

## All Available Commands

### SLAM Mapping
```bash
# Start mapping
./scripts/start_mapping.sh

# Without RViz (headless)
./scripts/start_mapping.sh --no-rviz

# Custom LiDAR port
./scripts/start_mapping.sh --serial-port /dev/rplidar

# Save map when done
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Localization Only (No Navigation)
```bash
# Basic localization
./scripts/start_localization.sh --map ~/maps/my_map.yaml

# Without RViz
./scripts/start_localization.sh --map ~/maps/my_map.yaml --no-rviz
```

### Full Navigation
```bash
# Basic navigation
./scripts/start_navigation.sh --map ~/maps/my_map.yaml

# With hardware integration (PI_API)
./scripts/start_navigation.sh \
    --map ~/maps/my_map.yaml \
    --with-bridge \
    --pi-api-url http://192.168.1.100:8000

# Without RViz (headless)
./scripts/start_navigation.sh --map ~/maps/my_map.yaml --no-rviz
```

### Simulation Mode
```bash
# SLAM in simulation
./scripts/start_simulation.sh --mode slam

# Navigation in simulation
./scripts/start_simulation.sh --mode navigation --map ~/maps/my_map.yaml

# In another terminal, play rosbag
ros2 bag play my_data.bag --clock
```

---

## Direct Launch Commands (Alternative to Scripts)

### SLAM
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py mode:=slam
```

### Localization
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=localization \
    map:=/home/user/maps/my_map.yaml
```

### Navigation
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/my_map.yaml
```

### Navigation with All Options
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/my_map.yaml \
    use_rviz:=true \
    use_cmd_vel_bridge:=true \
    pi_api_url:=http://192.168.1.100:8000 \
    serial_port:=/dev/ttyUSB0
```

### Simulation
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/my_map.yaml \
    use_sim_time:=true
```

---

## Troubleshooting Quick Fixes

### LiDAR Not Found
```bash
# Check USB devices
ls -l /dev/ttyUSB*

# Fix permissions
sudo chmod 666 /dev/ttyUSB0

# Or add user to dialout (permanent, requires logout)
sudo usermod -a -G dialout $USER
```

### Robot Not Moving
1. Check velocity commands: `ros2 topic echo /cmd_vel`
2. Set initial pose in RViz (2D Pose Estimate)
3. Check for obstacles in RViz costmap
4. Verify PI_API is running if using hardware

### Localization Not Working
1. Set initial pose in RViz using "2D Pose Estimate"
2. Make sure you're in the correct area of the map
3. Move robot slightly to help particles converge
4. Check laser scan in RViz - should see walls/obstacles

### High CPU Usage
```bash
# Run without RViz
./scripts/start_navigation.sh --map ~/maps/my_map.yaml --no-rviz

# Or use localization only
./scripts/start_localization.sh --map ~/maps/my_map.yaml
```

---

## Useful ROS2 Commands

### Check Running Nodes
```bash
ros2 node list
```

### Check Topics
```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /cmd_vel
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
```

### Monitor Navigation Status
```bash
ros2 topic echo /plan
ros2 topic echo /amcl_pose
```

### Send Navigation Goal Programmatically
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## File Locations

| Item | Path |
|------|------|
| Main Launch File | `launch/bringup.launch.py` |
| Helper Scripts | `scripts/start_*.sh` |
| Nav2 Config | `config/nav2_params.yaml` |
| SLAM Config | `config/slam_params.yaml` |
| AMCL Config | `config/amcl_params.yaml` |
| Robot URDF | `urdf/wayfinder_robot.urdf.xacro` |
| Maps Directory | `maps/` |
| Documentation | `findings/` |

---

## Help Commands

Every script has built-in help:

```bash
./scripts/start_mapping.sh --help
./scripts/start_localization.sh --help
./scripts/start_navigation.sh --help
./scripts/start_simulation.sh --help
```

---

## Common Workflows

### Workflow 1: First-Time Setup
```bash
# 1. Create map
./scripts/start_mapping.sh
# Drive around...
ros2 run nav2_map_server map_saver_cli -f ~/maps/office

# 2. Test localization
./scripts/start_localization.sh --map ~/maps/office.yaml
# Set pose in RViz, verify it works

# 3. Run full navigation
./scripts/start_navigation.sh --map ~/maps/office.yaml
# Set pose, send goal, verify navigation
```

### Workflow 2: Daily Operation
```bash
# Start navigation
./scripts/start_navigation.sh --map ~/maps/office.yaml --with-bridge

# In RViz:
# 1. Set initial pose
# 2. Send navigation goals as needed
# Or use waypoint manager for missions
```

### Workflow 3: Testing with Simulation
```bash
# Terminal 1: Start simulation
./scripts/start_simulation.sh --mode navigation --map ~/maps/office.yaml

# Terminal 2: Play recorded data
ros2 bag play my_test_data.bag --clock

# Watch navigation in RViz
```

---

## Next Steps

- Read full documentation: `findings/unified_launch_system.md`
- Tune parameters: `config/nav2_params.yaml`
- Create waypoint missions: `scripts/waypoint_manager.py`
- Integrate with PI_API: Use `--with-bridge` option

---

**For detailed documentation, see:** `findings/unified_launch_system.md`
