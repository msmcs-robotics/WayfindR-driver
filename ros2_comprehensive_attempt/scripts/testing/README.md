# ROS2 Nav2 Testing Scripts

This directory contains scripts for testing ROS2 navigation systems without physical hardware using rosbags and synthetic data.

## Overview

These tools enable comprehensive testing of SLAM, localization, and navigation algorithms using:
- **Recorded rosbags** from real or simulated robots
- **Synthetic data** generated programmatically
- **Automated test workflows** for reproducible results

## Quick Start

### 1. Generate Synthetic Test Data

Create a synthetic rosbag with simulated robot motion and obstacles:

```bash
./generate_test_bag.sh my_test 30 circular
```

This creates a 30-second bag with circular motion pattern.

### 2. Test SLAM

Test slam_toolbox with your rosbag:

```bash
./test_slam_with_bag.sh my_test
```

Outputs a map file and quality analysis.

### 3. Test Localization

Test AMCL localization with a map and rosbag:

```bash
./test_localization_with_bag.sh maps/my_map.yaml my_test
```

### 4. Analyze Map Quality

Evaluate SLAM-generated maps:

```bash
python3 analyze_slam_quality.py slam_test_*/final_map.yaml
```

## Script Reference

### Data Generation

#### `fake_laser_scan_publisher.py`
Publishes synthetic LaserScan messages simulating a simple rectangular room.

**Usage:**
```bash
python3 fake_laser_scan_publisher.py
```

**Parameters:**
- `scan_topic`: Topic to publish on (default: `/scan`)
- `frame_id`: Frame ID for scans (default: `laser_frame`)
- `publish_rate`: Publishing frequency in Hz (default: `10.0`)
- `room_width`: Room width in meters (default: `4.0`)
- `room_length`: Room length in meters (default: `6.0`)
- `add_noise`: Add Gaussian noise (default: `true`)

**Example with parameters:**
```bash
python3 fake_laser_scan_publisher.py --ros-args \
  -p publish_rate:=20.0 \
  -p room_width:=8.0 \
  -p room_length:=10.0
```

#### `synthetic_nav_data_publisher.py`
Publishes complete navigation data including LaserScan, Odometry, and TF transforms with obstacles.

**Usage:**
```bash
python3 synthetic_nav_data_publisher.py
```

**Parameters:**
- `scan_topic`: LaserScan topic (default: `/scan`)
- `odom_topic`: Odometry topic (default: `/odom`)
- `motion_pattern`: Robot motion pattern (default: `circular`)
  - Options: `straight`, `circular`, `square`
- `linear_velocity`: Forward speed in m/s (default: `0.2`)
- `angular_velocity`: Rotation speed in rad/s (default: `0.1`)

**Example:**
```bash
python3 synthetic_nav_data_publisher.py --ros-args \
  -p motion_pattern:=square \
  -p linear_velocity:=0.5
```

#### `generate_test_bag.sh`
Automated script to run synthetic publisher and record to rosbag.

**Usage:**
```bash
./generate_test_bag.sh [output_name] [duration] [motion_pattern]
```

**Arguments:**
- `output_name`: Name for the rosbag (default: auto-generated timestamp)
- `duration`: Recording duration in seconds (default: `30`)
- `motion_pattern`: Motion pattern - `straight`, `circular`, or `square` (default: `circular`)

**Examples:**
```bash
# 30-second circular motion
./generate_test_bag.sh corridor_test 30 circular

# 60-second straight line
./generate_test_bag.sh hallway_test 60 straight

# 45-second square pattern
./generate_test_bag.sh room_test 45 square
```

### Data Recording

#### `record_nav_data.sh`
Records navigation data from live robot or simulation.

**Usage:**
```bash
./record_nav_data.sh [output_name] [duration]
```

**Arguments:**
- `output_name`: Name for the rosbag (default: auto-generated timestamp)
- `duration`: Recording duration in seconds (default: `0` = until Ctrl+C)

**Examples:**
```bash
# Record until Ctrl+C
./record_nav_data.sh office_map

# Record for 120 seconds
./record_nav_data.sh warehouse_scan 120
```

**Topics Recorded:**
- Essential: `/scan`, `/odom`, `/tf`, `/tf_static`
- Optional (if available): `/cmd_vel`, `/map`, `/amcl_pose`, `/robot_description`, `/joint_states`

### Testing Scripts

#### `test_slam_with_bag.sh`
Tests SLAM (slam_toolbox) with a rosbag and generates a map.

**Usage:**
```bash
./test_slam_with_bag.sh <bag_file> [config_file]
```

**Arguments:**
- `bag_file`: Path to rosbag file (required)
- `config_file`: Custom SLAM configuration YAML (optional, default config created if not provided)

**Examples:**
```bash
# Basic SLAM test with default config
./test_slam_with_bag.sh my_data.db3

# SLAM test with custom configuration
./test_slam_with_bag.sh my_data.db3 custom_slam_params.yaml
```

**Outputs:**
- `slam_test_YYYYMMDD_HHMMSS/`
  - `final_map.pgm` - Generated map image
  - `final_map.yaml` - Map metadata
  - `slam_toolbox.log` - SLAM logs
  - `slam_output/` - Recorded SLAM outputs
  - `test_report.txt` - Test summary

#### `test_localization_with_bag.sh`
Tests AMCL localization with a map and rosbag.

**Usage:**
```bash
./test_localization_with_bag.sh <map_file> <bag_file> [initial_x] [initial_y] [initial_yaw]
```

**Arguments:**
- `map_file`: Path to map YAML file (required)
- `bag_file`: Path to rosbag file (required)
- `initial_x`: Initial X position in meters (default: `0.0`)
- `initial_y`: Initial Y position in meters (default: `0.0`)
- `initial_yaw`: Initial yaw angle in radians (default: `0.0`)

**Examples:**
```bash
# Basic localization test at origin
./test_localization_with_bag.sh maps/office.yaml test_data.db3

# Localization test with custom initial pose
./test_localization_with_bag.sh maps/warehouse.yaml test_data.db3 2.5 1.0 1.57
```

**Outputs:**
- `localization_test_YYYYMMDD_HHMMSS/`
  - `amcl.log` - AMCL logs
  - `map_server.log` - Map server logs
  - `amcl_output/` - Recorded pose estimates
  - `amcl_config.yaml` - AMCL configuration used
  - `test_report.txt` - Test summary

### Analysis Tools

#### `analyze_slam_quality.py`
Analyzes SLAM map quality with detailed metrics.

**Usage:**
```bash
# Analyze saved map file
python3 analyze_slam_quality.py <map.yaml>

# Analyze live map from /map topic
python3 analyze_slam_quality.py --live

# Save analysis report
python3 analyze_slam_quality.py <map.yaml> --output report.yaml
```

**Examples:**
```bash
# Analyze generated map
python3 analyze_slam_quality.py slam_test_20260111/final_map.yaml

# Monitor live SLAM quality
python3 analyze_slam_quality.py --live

# Generate quality report
python3 analyze_slam_quality.py final_map.yaml -o quality_report.yaml
```

**Metrics Analyzed:**
- Map coverage (free, occupied, unknown percentages)
- Explored area
- Obstacle density
- Resolution quality
- Overall quality score (0-100)

## Common Workflows

### Workflow 1: Test SLAM with Synthetic Data

```bash
# 1. Generate synthetic test data
./generate_test_bag.sh slam_test 60 circular

# 2. Run SLAM test
./test_slam_with_bag.sh slam_test

# 3. Analyze map quality
python3 analyze_slam_quality.py slam_test_*/final_map.yaml
```

### Workflow 2: Test Localization

```bash
# 1. Generate or record test data
./generate_test_bag.sh localization_test 45 square

# 2. Create map with SLAM (if not already done)
./test_slam_with_bag.sh localization_test

# 3. Test localization with the map
./test_localization_with_bag.sh slam_test_*/final_map.yaml localization_test
```

### Workflow 3: Record from Real Robot

```bash
# 1. Start your robot hardware/drivers

# 2. Record navigation data
./record_nav_data.sh robot_test_$(date +%Y%m%d) 120

# 3. Test SLAM with recorded data
./test_slam_with_bag.sh robot_test_*

# 4. Analyze results
python3 analyze_slam_quality.py slam_test_*/final_map.yaml
```

### Workflow 4: Record from Gazebo Simulation

```bash
# Terminal 1: Start Gazebo simulation
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Start recording
./record_nav_data.sh gazebo_sim 60

# Terminal 3: Drive robot around
ros2 run turtlebot3_teleop teleop_keyboard

# After recording, test SLAM
./test_slam_with_bag.sh gazebo_sim
```

## Configuration Files

### SLAM Configuration Example

Default SLAM configuration is auto-generated, but you can customize:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true

    # Adjust these for your environment
    resolution: 0.05  # Map resolution (meters/cell)
    max_laser_range: 12.0  # Maximum laser range
    minimum_travel_distance: 0.2  # Min distance before update

    # Loop closure (set false for faster mapping)
    do_loop_closing: true

    # Increase for more accurate mapping
    correlation_search_space_dimension: 0.5
```

### AMCL Configuration Example

Customize localization parameters:

```yaml
amcl:
  ros__parameters:
    use_sim_time: true

    # Particle filter
    min_particles: 500
    max_particles: 2000

    # Update thresholds
    update_min_d: 0.2  # Min distance to update
    update_min_a: 0.2  # Min angle to update

    # Laser model
    laser_max_beams: 60  # Reduce for faster processing
```

## Troubleshooting

### "No /scan topic found"

**Problem:** Bag doesn't contain scan data or publisher not running.

**Solution:**
```bash
# Check bag contents
ros2 bag info your_bag

# Check live topics
ros2 topic list | grep scan
```

### "TF lookup failed"

**Problem:** Missing TF transforms or simulation time not enabled.

**Solution:**
```bash
# Ensure bag playback uses --clock
ros2 bag play your_bag --clock

# Check all nodes have use_sim_time:=true
ros2 param get /slam_toolbox use_sim_time
```

### "Map server failed to activate"

**Problem:** Map file not found or invalid.

**Solution:**
```bash
# Verify map file exists
ls -l path/to/map.yaml
ls -l path/to/map.pgm

# Check map file format
cat path/to/map.yaml
```

### "SLAM produces poor quality map"

**Problem:** Insufficient features, wrong parameters, or bad odometry.

**Solutions:**
- Increase recording duration
- Drive robot slower for better scan matching
- Adjust SLAM parameters (resolution, search space)
- Check odometry quality in bag

### "Synthetic data not realistic enough"

**Problem:** Default synthetic publisher is too simple.

**Solutions:**
- Use Gazebo simulation instead
- Customize obstacles in `synthetic_nav_data_publisher.py`
- Record from real robot for most realistic data

## Tips for Best Results

### Recording Tips

1. **Good Coverage**: Drive robot to cover entire area
2. **Slow Motion**: Don't move too fast (< 0.5 m/s for best results)
3. **Overlap**: Revisit same areas for loop closure
4. **Features**: Ensure environment has distinguishable features
5. **Duration**: Record at least 2-3 minutes per room

### Testing Tips

1. **Start Simple**: Test with synthetic data first
2. **Check Logs**: Always review log files for errors
3. **Visualize**: Use RViz to visualize results
4. **Iterate**: Tune parameters based on results
5. **Document**: Keep notes on test conditions and results

### Performance Tips

1. **Reduce Bag Size**: Record only essential topics
2. **Use MCAP**: Better performance than SQLite for large bags
3. **Enable Compression**: Saves disk space
4. **Playback Rate**: Use `--rate 0.5` for slower, more stable playback
5. **Close Other Apps**: Free up CPU/memory during testing

## Integration with CI/CD

These scripts can be used in automated testing pipelines:

```yaml
# Example GitHub Actions workflow
- name: Test SLAM
  run: |
    ./scripts/testing/generate_test_bag.sh ci_test 30 circular
    ./scripts/testing/test_slam_with_bag.sh ci_test
    python3 ./scripts/testing/analyze_slam_quality.py slam_test_*/final_map.yaml
```

## Further Reading

- [Main Testing Guide](../../findings/2026-01-11-rosbag-testing-guide.md) - Comprehensive documentation
- [Nav2 Documentation](https://navigation.ros.org/) - Official Nav2 docs
- [rosbag2 Documentation](https://github.com/ros2/rosbag2) - rosbag2 tools

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review logs in output directories
3. Consult the main testing guide
4. Check Nav2 and rosbag2 documentation

---

**Version:** 1.0
**Last Updated:** 2026-01-11
**Author:** WayfindR Development Team
