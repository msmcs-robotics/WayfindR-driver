# Quick Start Guide - Rosbag Testing

Fast track guide to get started with Nav2 testing without hardware.

## 5-Minute Quick Start

### Option 1: Test with Synthetic Data (Easiest)

```bash
cd scripts/testing

# Generate synthetic test data (30 seconds)
./generate_test_bag.sh quick_test 30 circular

# Test SLAM (creates map)
./test_slam_with_bag.sh quick_test

# Analyze map quality
python3 analyze_slam_quality.py slam_test_*/final_map.yaml
```

**Expected Output:**
- Synthetic rosbag in `quick_test/`
- Map files in `slam_test_TIMESTAMP/final_map.{pgm,yaml}`
- Quality report with score

### Option 2: Record from Gazebo Simulation

```bash
# Terminal 1: Start Gazebo
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Record
cd scripts/testing
./record_nav_data.sh gazebo_test 60

# Terminal 3: Drive robot
ros2 run turtlebot3_teleop teleop_keyboard
# Drive around for 60 seconds

# Terminal 2: After recording, test SLAM
./test_slam_with_bag.sh gazebo_test
```

## Common Commands Cheat Sheet

### Generate Test Data
```bash
# 30-second circular motion
./generate_test_bag.sh test1 30 circular

# 60-second straight line
./generate_test_bag.sh test2 60 straight

# 45-second square pattern
./generate_test_bag.sh test3 45 square
```

### Record Real Data
```bash
# Record until Ctrl+C
./record_nav_data.sh my_recording

# Record for 2 minutes
./record_nav_data.sh my_recording 120
```

### Test SLAM
```bash
# Basic SLAM test
./test_slam_with_bag.sh <bag_name>

# SLAM with custom config
./test_slam_with_bag.sh <bag_name> my_config.yaml
```

### Test Localization
```bash
# Basic localization test
./test_localization_with_bag.sh <map.yaml> <bag_name>

# With initial pose
./test_localization_with_bag.sh <map.yaml> <bag_name> 1.0 2.0 0.0
```

### Analyze Map
```bash
# Analyze saved map
python3 analyze_slam_quality.py <map.yaml>

# Monitor live SLAM
python3 analyze_slam_quality.py --live
```

## Troubleshooting Quick Fixes

### No scan data
```bash
# Check what's in the bag
ros2 bag info <bag_name>

# List live topics
ros2 topic list
```

### TF errors
```bash
# Always use --clock for playback
ros2 bag play <bag_name> --clock

# Verify sim_time
ros2 param get /node_name use_sim_time
```

### SLAM not working
```bash
# Check SLAM is running
ros2 node list | grep slam

# Check scan topic
ros2 topic echo /scan --once
```

## What Each Script Does

| Script | Purpose | Input | Output |
|--------|---------|-------|--------|
| `generate_test_bag.sh` | Create synthetic data | Duration, pattern | Rosbag |
| `record_nav_data.sh` | Record from robot/sim | Duration | Rosbag |
| `test_slam_with_bag.sh` | Test SLAM mapping | Rosbag | Map files |
| `test_localization_with_bag.sh` | Test AMCL | Map + Rosbag | Pose data |
| `analyze_slam_quality.py` | Analyze map | Map file | Quality report |

## Next Steps

1. **Read full guides:**
   - [Complete Testing Guide](../../findings/2026-01-11-rosbag-testing-guide.md)
   - [Scripts README](README.md)

2. **Try different scenarios:**
   - Different motion patterns
   - Various durations
   - Custom configurations

3. **Test with your robot:**
   - Record real data
   - Build real maps
   - Validate in real environment

## Need Help?

- Check [README.md](README.md) for detailed documentation
- See [Troubleshooting Guide](../../findings/2026-01-11-rosbag-testing-guide.md#troubleshooting)
- Review script logs in output directories

---

**Last Updated:** 2026-01-11
