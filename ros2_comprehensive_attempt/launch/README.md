# WayfindR Launch Files

This directory contains ROS2 launch files for the WayfindR navigation system.

## Main Launch File

### bringup.launch.py

**The unified launch file that replaces the need for multiple separate launch files.**

This is the primary entry point for launching the WayfindR navigation system. It provides a flexible, mode-based architecture that can launch:

- SLAM mapping
- Localization only
- Full autonomous navigation
- Simulation mode

**Quick Usage:**

```bash
# SLAM Mapping
ros2 launch ros2_comprehensive_attempt bringup.launch.py mode:=slam

# Localization Only
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=localization \
    map:=/path/to/map.yaml

# Full Navigation
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/path/to/map.yaml

# Simulation
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/path/to/map.yaml \
    use_sim_time:=true
```

**Key Features:**
- Single launch file for all modes
- Automatic component management based on mode
- Lifecycle node management
- Flexible configuration via launch arguments
- Support for hardware and simulation

**All Launch Arguments:**

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| mode | string | navigation | Launch mode: slam, localization, or navigation |
| map | string | "" | Full path to map YAML file |
| use_sim_time | bool | false | Use simulation clock |
| use_rviz | bool | true | Launch RViz visualization |
| use_cmd_vel_bridge | bool | false | Launch cmd_vel bridge |
| serial_port | string | /dev/ttyUSB0 | RPLidar serial port |
| pi_api_url | string | http://localhost:8000 | PI_API endpoint |
| nav2_params | string | (auto) | Nav2 parameters file |
| urdf_file | string | (auto) | Robot URDF file |
| rviz_config | string | (auto) | RViz config file |

## Helper Scripts

Instead of directly calling the launch file, use the convenient helper scripts in `../scripts/`:

```bash
# SLAM Mapping
../scripts/start_mapping.sh

# Localization Only
../scripts/start_localization.sh --map /path/to/map.yaml

# Full Navigation
../scripts/start_navigation.sh --map /path/to/map.yaml

# Simulation
../scripts/start_simulation.sh --mode navigation --map /path/to/map.yaml
```

All scripts include:
- Color-coded output
- Built-in help (--help flag)
- Validation and error checking
- Sensible defaults

## Legacy Launch Files

The following launch files are still available but are superseded by `bringup.launch.py`:

### slam.launch.py
Launches SLAM mapping mode (legacy). Use `bringup.launch.py mode:=slam` instead.

### localization.launch.py
Launches localization only (legacy). Use `bringup.launch.py mode:=localization` instead.

### navigation.launch.py
Launches full navigation (legacy). Use `bringup.launch.py mode:=navigation` instead.

### robot_state_publisher.launch.py
Standalone robot description publisher. Now integrated into bringup.launch.py.

### cmd_vel_bridge.launch.py
Standalone cmd_vel bridge. Now integrated into bringup.launch.py with `use_cmd_vel_bridge:=true`.

## Launch System Architecture

```
bringup.launch.py
    │
    ├─── Core Components (Always)
    │    ├─ robot_state_publisher
    │    ├─ joint_state_publisher
    │    ├─ rplidar_node (if not sim)
    │    ├─ static_tf_publishers
    │    └─ cmd_vel_bridge (optional)
    │
    ├─── SLAM Mode (mode:=slam)
    │    └─ slam_toolbox
    │
    ├─── Localization Mode (mode:=localization)
    │    ├─ map_server
    │    ├─ amcl
    │    └─ lifecycle_manager_localization
    │
    ├─── Navigation Mode (mode:=navigation)
    │    ├─ map_server
    │    ├─ amcl
    │    ├─ lifecycle_manager_localization
    │    ├─ controller_server
    │    ├─ planner_server
    │    ├─ behavior_server
    │    ├─ bt_navigator
    │    ├─ waypoint_follower
    │    ├─ velocity_smoother
    │    └─ lifecycle_manager_navigation
    │
    └─── Visualization (Optional)
         └─ rviz2
```

## Mode Comparison

| Component | SLAM | Localization | Navigation |
|-----------|------|--------------|------------|
| Robot State Publisher | ✓ | ✓ | ✓ |
| RPLidar Driver | ✓ | ✓ | ✓ |
| SLAM Toolbox | ✓ | ✗ | ✗ |
| Map Server | ✗ | ✓ | ✓ |
| AMCL | ✗ | ✓ | ✓ |
| Nav2 Stack | ✗ | ✗ | ✓ |
| RViz | Optional | Optional | Optional |

## Configuration Files

Launch files use configuration from `../config/`:

- `nav2_params.yaml` - Nav2 navigation parameters
- `slam_params.yaml` - SLAM Toolbox parameters
- `amcl_params.yaml` - AMCL localization parameters
- `lidar_params.yaml` - LiDAR driver parameters
- `cmd_vel_bridge_params.yaml` - cmd_vel bridge parameters
- `rviz_config.rviz` - RViz visualization configuration

## Documentation

For complete documentation, see:

- `../findings/unified_launch_system.md` - Comprehensive system documentation
- `../findings/quick_start_guide.md` - Quick reference guide
- Helper script help: `../scripts/start_*.sh --help`

## Examples

### Example 1: Create a Map

```bash
# Launch SLAM mode
ros2 launch ros2_comprehensive_attempt bringup.launch.py mode:=slam

# Drive robot around to build map
# When done, save the map:
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_office
```

### Example 2: Test Localization

```bash
# Launch localization mode
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=localization \
    map:=/home/user/maps/my_office.yaml

# In RViz:
# 1. Set initial pose with "2D Pose Estimate"
# 2. Watch particles converge
```

### Example 3: Autonomous Navigation

```bash
# Launch full navigation
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/my_office.yaml

# In RViz:
# 1. Set initial pose
# 2. Send goals with "2D Nav Goal"
```

### Example 4: Hardware Integration

```bash
# Launch with PI_API bridge
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/my_office.yaml \
    use_cmd_vel_bridge:=true \
    pi_api_url:=http://192.168.1.100:8000
```

### Example 5: Simulation Testing

```bash
# Terminal 1: Launch in simulation mode
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/my_office.yaml \
    use_sim_time:=true

# Terminal 2: Play rosbag with clock
ros2 bag play my_test_data.bag --clock
```

### Example 6: Headless Operation

```bash
# Run without RViz (lower resource usage)
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/my_office.yaml \
    use_rviz:=false
```

## Troubleshooting

### Issue: Launch file not found

Make sure you're in the correct directory or use the full package name:

```bash
cd /path/to/ros2_comprehensive_attempt
ros2 launch ros2_comprehensive_attempt bringup.launch.py mode:=slam
```

### Issue: Map not found

Use absolute paths for map files:

```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/home/user/maps/office.yaml
```

### Issue: Nodes not starting

Check that all dependencies are installed:

```bash
sudo apt install \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rplidar-ros
```

## Migration from Old Launch Files

If you were using the old individual launch files:

**Old:**
```bash
ros2 launch slam.launch.py
```

**New:**
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py mode:=slam
# Or use helper script:
./scripts/start_mapping.sh
```

**Old:**
```bash
ros2 launch localization.launch.py map:=/path/to/map.yaml
```

**New:**
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=localization \
    map:=/path/to/map.yaml
# Or use helper script:
./scripts/start_localization.sh --map /path/to/map.yaml
```

**Old:**
```bash
ros2 launch navigation.launch.py map:=/path/to/map.yaml
```

**New:**
```bash
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
    mode:=navigation \
    map:=/path/to/map.yaml
# Or use helper script:
./scripts/start_navigation.sh --map /path/to/map.yaml
```

## Support

For issues or questions:

1. Check `../findings/unified_launch_system.md` for detailed documentation
2. Use `--help` flag on helper scripts
3. Review configuration files in `../config/`
4. Check ROS2 logs for error messages

---

**Recommendation:** Use the helper scripts in `../scripts/` for the best user experience!
