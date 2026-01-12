# LiDAR Data Recording and Replay Workflow for SLAM Testing

## Overview

This document provides a comprehensive guide to recording, replaying, and analyzing LiDAR data from the RP LIDAR C1M1 for SLAM testing and development. It covers the complete workflow from initial LiDAR connection through map creation and algorithm tuning.

**Target System:**
- ROS2 Humble
- RP LIDAR C1M1 (RPLidar C1)
- 2D SLAM (Cartographer, SLAM Toolbox, etc.)

---

## Table of Contents

1. [LiDAR Connection and Setup](#1-lidar-connection-and-setup)
2. [Understanding Essential SLAM Topics](#2-understanding-essential-slam-topics)
3. [Recording LiDAR Data](#3-recording-lidar-data)
4. [Best Practices for Data Collection](#4-best-practices-for-data-collection)
5. [Replaying Recorded Data](#5-replaying-recorded-data)
6. [Quality Assessment](#6-quality-assessment)
7. [SLAM Tuning with Recorded Data](#7-slam-tuning-with-recorded-data)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. LiDAR Connection and Setup

### Hardware Connection

1. **Connect via USB**
   ```bash
   # Check USB device detection
   lsusb | grep -i silicon
   # Expected: Silicon Labs CP210x UART Bridge

   # Check serial port
   ls -la /dev/ttyUSB*
   # Expected: /dev/ttyUSB0 (or similar)
   ```

2. **Create udev Rules for Persistent Naming**
   ```bash
   sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
   # Slamtec RPLidar C1M1
   KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
   EOF

   # Reload rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger

   # Verify symlink
   ls -la /dev/rplidar
   ```

3. **Set Permissions**
   ```bash
   # Add user to dialout group (requires logout/reboot)
   sudo usermod -a -G dialout $USER

   # Or set temporary permissions (for testing)
   sudo chmod 666 /dev/ttyUSB0
   ```

### RP LIDAR C1M1 Specifications

| Parameter | Value |
|-----------|-------|
| **Sample Rate** | 5000 Hz (5000 samples/sec) |
| **Scan Frequency** | 8-12 Hz (typical 10 Hz @ 600 RPM) |
| **Angular Resolution** | 0.72° (@10Hz) or 0.225° (high-res mode) |
| **Range** | 0.05m - 12m |
| **Accuracy** | ±30mm (@25°C) |
| **Interface** | TTL UART (3.3V Level) |
| **Baud Rate** | 460800 |
| **Power** | 5V DC ±0.2V (230mA @10Hz) |

### Scan Modes

The C1M1 supports multiple scan modes:

- **Standard**: Balanced range/speed, good for general use
- **DenseBoost**: High sample rate (5000Hz), ideal for indoor SLAM
- **Sensitivity**: Maximum range mode
- **Express**: Fast scanning for dynamic environments

**Recommendation for SLAM**: Use **DenseBoost** mode for indoor mapping to maximize point density.

### Start LiDAR Node

```bash
source /opt/ros/humble/setup.bash

ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/rplidar \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser \
    -p angle_compensate:=true \
    -p scan_mode:=DenseBoost
```

### Verify Data Stream

```bash
# Check topic exists
ros2 topic list | grep scan

# Check data rate
ros2 topic hz /scan
# Expected: ~10 Hz for C1M1

# View single scan
ros2 topic echo /scan --once

# Check scan statistics
ros2 topic echo /scan --once | grep -E "range_min|range_max|angle_increment"
```

---

## 2. Understanding Essential SLAM Topics

### Required Topics for SLAM

Different SLAM algorithms require different topic combinations:

#### Minimal SLAM Setup (LiDAR-only)
- **/scan** - LaserScan data from LiDAR
- **/tf** and **/tf_static** - Transform tree

#### Full SLAM Setup (with Odometry)
- **/scan** - LaserScan data
- **/odom** - Odometry from wheel encoders or visual odometry
- **/tf** and **/tf_static** - Transform tree
- **/imu** (optional) - IMU data for better motion estimation

### Topic Descriptions

#### /scan (sensor_msgs/LaserScan)

Contains the LiDAR scan data:
```
header:
  stamp: <timestamp>
  frame_id: "laser"
angle_min: -3.14159        # Start angle (radians)
angle_max: 3.14159         # End angle (radians)
angle_increment: 0.0126    # Angular resolution (~0.72°)
time_increment: 0.0        # Time between readings
scan_time: 0.1             # Time for full 360° scan (10Hz)
range_min: 0.05            # Minimum valid range (m)
range_max: 12.0            # Maximum valid range (m)
ranges: [...]              # Array of ~500-800 distances (m)
intensities: [...]         # Signal strength values
```

#### /odom (nav_msgs/Odometry)

Provides robot pose estimate from wheel encoders or other sources:
```
header:
  stamp: <timestamp>
  frame_id: "odom"
child_frame_id: "base_link"
pose:
  position: {x, y, z}
  orientation: {x, y, z, w}  # Quaternion
twist:
  linear: {x, y, z}          # Linear velocity
  angular: {x, y, z}         # Angular velocity
```

#### /tf and /tf_static (tf2_msgs/TFMessage)

Transform relationships between coordinate frames:
```
odom -> base_link -> laser
```

**Critical**: The TF tree must be valid and complete for SLAM to work.

### Generating Odometry from LiDAR

If you don't have wheel encoders, you can generate odometry from LiDAR scans:

```bash
# Install rf2o_laser_odometry
sudo apt install ros-humble-rf2o-laser-odometry

# Run laser odometry node
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
    -p laser_scan_topic:=/scan \
    -p odom_topic:=/odom \
    -p base_frame_id:=base_link
```

**Note**: Laser-based odometry works best with feature-rich environments and moderate movement speeds.

---

## 3. Recording LiDAR Data

### Basic Recording Commands

#### Record Specific Topics
```bash
# Minimal SLAM recording (LiDAR only)
ros2 bag record /scan /tf /tf_static -o minimal_slam_test

# Full SLAM recording (with odometry)
ros2 bag record /scan /odom /tf /tf_static -o full_slam_test

# With IMU data
ros2 bag record /scan /odom /imu /tf /tf_static -o imu_slam_test
```

#### Record All Topics
```bash
# Record everything (not recommended - large files)
ros2 bag record -a -o complete_test
```

### Recording Options

```bash
# Set compression (reduces file size significantly)
ros2 bag record /scan /odom /tf /tf_static \
    --compression-mode file \
    --compression-format zstd \
    -o compressed_test

# Set max bag file size (creates multiple files)
ros2 bag record /scan /odom /tf /tf_static \
    --max-bag-size 500000000 \
    -o split_test
# Creates: split_test_0.db3, split_test_1.db3, etc.

# Set storage format
ros2 bag record /scan /odom /tf /tf_static \
    --storage sqlite3 \
    -o sqlite_test

# Use simulation time (when replaying from another bag)
ros2 bag record /scan /odom /tf /tf_static \
    --use-sim-time \
    -o sim_time_test
```

### Recommended Recording Setup

For best SLAM testing results:

```bash
#!/bin/bash
# Record session with optimal settings

SESSION_NAME="slam_session_$(date +%Y%m%d_%H%M%S)"

ros2 bag record \
    /scan \
    /odom \
    /tf \
    /tf_static \
    --compression-mode file \
    --compression-format zstd \
    --max-bag-size 1000000000 \
    -o "$SESSION_NAME"
```

**File Size Estimate**:
- /scan @ 10Hz: ~100-200 KB/sec
- /odom @ 30Hz: ~10-20 KB/sec
- /tf: ~5-10 KB/sec
- **Total**: ~115-230 KB/sec (~7-14 MB/min uncompressed)
- **With compression**: ~50-60% size reduction

---

## 4. Best Practices for Data Collection

### Movement Patterns for Good SLAM

#### Loop Closure Requirements

Loop closures are essential for reducing accumulated drift in SLAM:

1. **Perform loop closures every 60-120 meters** of travel
2. **Overlap loops by at least 5 meters** for reliable detection
3. **Smaller loops = less accumulated error** = better accuracy
4. **Return through previously mapped areas** periodically

#### Recommended Movement Strategy

```
Start → Explore → Loop Back → Explore → Loop Back → End at Start
  ↓                  ↑             ↓           ↑
  └──────────────────┘             └───────────┘
     Primary Loop               Secondary Loop
```

**Example Session Plan**:
1. Start at a distinctive feature (corner, doorway)
2. Move forward 30-50 meters
3. Loop back through starting area
4. Explore different direction
5. Loop back again
6. End at starting point

#### Coverage Strategy

- **Break large areas into sections**: Map room-by-room or zone-by-zone
- **Practice loop closure between sections**: Overlap boundaries
- **Ensure feature-rich paths**: Avoid long, featureless corridors

### Movement Speed

#### Speed Guidelines

| Movement Type | Max Speed | Scan Coverage | Notes |
|--------------|-----------|---------------|-------|
| **Slow & Steady** | 0.2-0.3 m/s | Excellent | Best for high-quality maps |
| **Normal Walking** | 0.4-0.6 m/s | Good | Acceptable for most SLAM |
| **Fast Walking** | 0.7-1.0 m/s | Fair | May have gaps/distortion |
| **Running** | >1.0 m/s | Poor | Not recommended |

#### Why Speed Matters

The C1M1 scans at **10 Hz** (0.1 seconds per full rotation):

- At **0.5 m/s**: Robot moves **5 cm** during one scan
- At **1.0 m/s**: Robot moves **10 cm** during one scan
- At **2.0 m/s**: Robot moves **20 cm** during one scan

**Problem**: Movement during scanning creates distortion in the point cloud. The scan assumes the robot is stationary, but it's actually moving, causing alignment errors.

**Solution**:
- Keep speed under 0.6 m/s for good results
- Use IMU data for motion compensation if available
- Some SLAM algorithms compensate for this automatically

### Scan Frequency Considerations

#### C1M1 Scan Frequency: 10 Hz

- **1 full scan every 0.1 seconds**
- **~500-800 points per scan** in DenseBoost mode
- **Angular resolution: 0.72°** (or better in high-res modes)

#### Matching Movement to Scan Rate

For **10 cm spatial resolution** (typical SLAM requirement):
- At 10 Hz, robot can move up to **1.0 m/s**
- But this is theoretical maximum
- **Practical limit: 0.5-0.6 m/s** for reliable SLAM

#### When to Adjust Scan Frequency

Some LiDARs allow frequency adjustment:
- **Lower frequency (5-8 Hz)**: Better point density per scan
- **Higher frequency (15-20 Hz)**: Better for faster movement
- **C1M1 default (10 Hz)**: Good balance for typical indoor robots

### Avoiding SLAM Failure Scenarios

#### Scenarios That Cause Poor SLAM Results

1. **Featureless Environments**
   - Long, empty hallways
   - Pointing at blank walls
   - Looking at the sky/ceiling
   - Small, empty rooms

   **Solution**: Ensure environment has distinguishable features (corners, furniture, obstacles)

2. **Dynamic Objects**
   - Moving people/robots in scan area
   - Opening/closing doors during mapping

   **Solution**: Map when environment is static, or filter dynamic objects

3. **Reflective/Transparent Surfaces**
   - Glass walls/windows
   - Mirrors
   - Polished floors

   **Solution**: Avoid or mark these areas; check for scan anomalies

4. **Insufficient Overlap**
   - Moving too fast between scans
   - Rotating too quickly

   **Solution**: Maintain slow, steady movement

5. **Lighting Issues**
   - Direct sunlight (interferes with infrared)
   - Very dark or very bright areas

   **Solution**: Maintain moderate lighting; avoid windows with direct sun

### Data Quality Checklist

Before ending a recording session, verify:

- [ ] Loop closures completed (returned to start)
- [ ] Movement speed was appropriate (0.3-0.6 m/s)
- [ ] All areas have sufficient overlap (>30%)
- [ ] No prolonged featureless sections
- [ ] LiDAR was functioning throughout (check /scan topic)
- [ ] TF transforms were valid (no discontinuities)
- [ ] File recorded successfully (check .db3 file size)

---

## 5. Replaying Recorded Data

### Basic Replay Commands

#### Simple Playback
```bash
# Play bag at normal speed
ros2 bag play slam_session_20260111_143022

# Play at half speed (better for visualization)
ros2 bag play slam_session_20260111_143022 --rate 0.5

# Play at double speed
ros2 bag play slam_session_20260111_143022 --rate 2.0

# Loop playback continuously
ros2 bag play slam_session_20260111_143022 --loop

# Start paused (manually unpause with spacebar)
ros2 bag play slam_session_20260111_143022 --start-paused
```

#### Advanced Playback Options

```bash
# Publish /clock for simulation time
ros2 bag play slam_session_20260111_143022 --clock 100
# Publishes /clock at 100 Hz

# Play only specific topics
ros2 bag play slam_session_20260111_143022 \
    --topics /scan /odom

# Remap topics during playback
ros2 bag play slam_session_20260111_143022 \
    --remap /scan:=/lidar_scan

# Delay start by 5 seconds
ros2 bag play slam_session_20260111_143022 --delay 5.0

# Play from specific timestamp offset (10 seconds in)
ros2 bag play slam_session_20260111_143022 --start-offset 10.0
```

### Replay for SLAM Map Creation

#### Workflow

1. **Start bag playback with clock**
   ```bash
   ros2 bag play slam_session_20260111_143022 \
       --clock 100 \
       --rate 1.0
   ```

2. **Start SLAM node with sim time** (in another terminal)
   ```bash
   # Cartographer example
   ros2 launch cartographer_ros cartographer.launch.py \
       use_sim_time:=true

   # SLAM Toolbox example
   ros2 launch slam_toolbox online_async_launch.py \
       use_sim_time:=true
   ```

3. **Monitor map creation**
   ```bash
   # Start RViz to visualize
   ros2 run rviz2 rviz2

   # Add displays: /map, /scan
   ```

4. **Save map when complete**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

### Replay for Localization Testing

#### Test Localization Against Existing Map

1. **Load existing map**
   ```bash
   ros2 run nav2_map_server map_server --ros-args \
       -p yaml_filename:=/path/to/my_map.yaml \
       -p use_sim_time:=true
   ```

2. **Start localization node**
   ```bash
   ros2 run nav2_amcl amcl --ros-args \
       -p use_sim_time:=true
   ```

3. **Replay bag with clock**
   ```bash
   ros2 bag play slam_session_20260111_143022 \
       --clock 100
   ```

4. **Compare localization to ground truth**
   - Visualize in RViz: /amcl_pose vs /odom
   - Check convergence time
   - Verify particle cloud spread

### Replay for Algorithm Tuning

#### Iterative Tuning Workflow

```bash
# 1. Record a challenging scenario once
ros2 bag record /scan /odom /tf /tf_static -o tuning_test

# 2. Test parameter variations by replaying
# Test 1: Default parameters
ros2 bag play tuning_test --clock 100 &
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Test 2: Modified parameters
ros2 bag play tuning_test --clock 100 &
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    resolution:=0.025 \
    max_laser_range:=10.0

# Compare resulting maps
```

**Benefits of bag-based tuning**:
- **Repeatability**: Exact same input data every time
- **Speed**: No need to drive robot again
- **Safety**: Can test aggressive parameters without risk
- **Comparison**: Easy to compare different parameter sets

### Inspecting Bag Contents

```bash
# Show bag information
ros2 bag info slam_session_20260111_143022

# Output includes:
# - Duration
# - Start/end times
# - Message count per topic
# - Topic types
# - Storage format

# Example output:
# Files:             slam_session_20260111_143022_0.db3
# Bag size:          45.2 MB
# Storage id:        sqlite3
# Duration:          120.5s
# Start:             Jan 11 2026 14:30:22.156 (1736609422.156)
# End:               Jan 11 2026 14:32:22.671 (1736609542.671)
# Messages:          4825
# Topic information: Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 1205 | Serialization Format: cdr
#                    Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 3610 | Serialization Format: cdr
#                    Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 10 | Serialization Format: cdr
```

---

## 6. Quality Assessment

### Real-Time Quality Checks During Recording

#### Check Scan Data Rate
```bash
# Should be ~10 Hz for C1M1
ros2 topic hz /scan

# If lower than expected:
# - Check LiDAR connection
# - Check CPU usage
# - Check for USB interference
```

#### Check Scan Data Content
```bash
# View single scan
ros2 topic echo /scan --once

# Check for valid ranges
ros2 topic echo /scan --once | grep -A 5 "ranges:"

# Monitor scan statistics
watch -n 1 'ros2 topic echo /scan --once | grep -E "range_min|range_max|scan_time"'
```

#### Check Transform Tree
```bash
# Verify TF tree is valid
ros2 run tf2_tools view_frames

# Generates frames_YYYY-MM-DD_HH.MM.SS.pdf
# Open PDF to verify: odom -> base_link -> laser

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link

# Check for transform errors
ros2 topic echo /tf --once
```

### Post-Recording Quality Analysis

#### Bag File Validation

```bash
# 1. Check bag file integrity
ros2 bag info slam_session_20260111_143022

# Verify:
# - Duration is as expected
# - Message counts are reasonable
# - No errors in output

# 2. Verify topic presence
# Should see: /scan, /odom, /tf, /tf_static

# 3. Check message rates
# /scan: ~10 Hz
# /odom: typically 20-50 Hz
# /tf: varies (static after initial frames)
```

#### LiDAR Scan Quality Analysis

Use the provided `check_lidar_quality.py` script:

```bash
# Analyze bag file
python3 scripts/lidar_tools/check_lidar_quality.py slam_session_20260111_143022

# Output includes:
# - Valid range percentage
# - Average points per scan
# - Range distribution
# - Scan rate statistics
# - Quality warnings
```

**Quality Metrics**:

| Metric | Good | Acceptable | Poor |
|--------|------|------------|------|
| Valid range % | >90% | 80-90% | <80% |
| Points per scan | >500 | 300-500 | <300 |
| Scan rate | 9.5-10.5 Hz | 8-11 Hz | <8 or >11 Hz |
| Max range readings | <20% | 20-40% | >40% |

**Interpreting Results**:

- **High invalid range %**: Check for obstacles too close/far, reflective surfaces
- **Low points per scan**: LiDAR may be obstructed or in wrong mode
- **Inconsistent scan rate**: CPU overload or USB issues
- **Too many max range readings**: Open spaces (normal) or sensor issue

### Visual Quality Assessment

#### RViz Playback Inspection

```bash
# 1. Start RViz
ros2 run rviz2 rviz2

# 2. Play bag with clock
ros2 bag play slam_session_20260111_143022 --clock 100 --rate 0.5

# 3. In RViz:
# - Add LaserScan display for /scan
# - Set Fixed Frame to "laser" or "base_link"
# - Watch for:
#   - Consistent scan patterns
#   - No sudden jumps
#   - Good environment coverage
#   - Minimal noise
```

#### What to Look For

- **Scan Consistency**: Scans should be smooth and consistent
- **No Gaps**: All 360° should have data (except occlusions)
- **Stable Mounting**: Scans shouldn't vibrate or shift erratically
- **Good Features**: Environment should have distinguishable features
- **Loop Closure**: When revisiting areas, scans should align

---

## 7. SLAM Tuning with Recorded Data

### Why Use Recorded Data for Tuning?

**Advantages**:
1. **Repeatability**: Test with identical conditions
2. **Speed**: No need to re-drive robot for each test
3. **Safety**: Tune aggressive parameters without robot risk
4. **Comparison**: Easy to benchmark parameter changes
5. **Debugging**: Pause and inspect at problem points

### Iterative Tuning Process

#### 1. Record a Challenging Dataset

```bash
# Record a session with known challenges:
# - Loop closures
# - Long corridors
# - Feature-poor areas
# - Tight corners

ros2 bag record /scan /odom /tf /tf_static \
    -o tuning_challenge_dataset
```

#### 2. Create Baseline Map

```bash
# Test with default parameters
ros2 bag play tuning_challenge_dataset --clock 100 --rate 1.0 &

ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true

# Save baseline map
ros2 run nav2_map_server map_saver_cli -f baseline_map
```

#### 3. Modify Parameters and Re-run

**Example: SLAM Toolbox Parameter Tuning**

```bash
# Test different resolution
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    resolution:=0.025

# Test different loop search parameters
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    minimum_travel_distance:=0.3 \
    minimum_travel_heading:=0.3

# Test different correlation search space
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    correlation_search_space_dimension:=0.8 \
    correlation_search_space_resolution:=0.02
```

#### 4. Compare Results

**Qualitative Comparison**:
- Visual inspection in RViz
- Loop closure success rate
- Map consistency
- Feature preservation

**Quantitative Metrics**:
- Map size (MB)
- Processing time
- Loop closure count
- Pose graph size

### Common SLAM Parameters to Tune

#### SLAM Toolbox

| Parameter | Default | Effect | Tuning Advice |
|-----------|---------|--------|---------------|
| `resolution` | 0.05 | Map cell size (m) | Lower = more detail, higher CPU |
| `minimum_travel_distance` | 0.5 | Min movement for scan matching | Lower = more scans, better detail |
| `minimum_travel_heading` | 0.5 | Min rotation for scan matching | Lower = more orientation data |
| `max_laser_range` | 20.0 | Max range to use | Match LiDAR specs (12m for C1M1) |
| `minimum_time_interval` | 0.5 | Min time between updates | Lower = higher rate, more CPU |

**Tuning for C1M1**:
```yaml
resolution: 0.03              # Good detail for 12m range
minimum_travel_distance: 0.2  # Capture at ~0.5 m/s movement
minimum_travel_heading: 0.2   # Good rotational updates
max_laser_range: 11.5         # Slightly below max for reliability
minimum_time_interval: 0.3    # 3 Hz updates
```

#### Cartographer

| Parameter | Default | Effect | Tuning Advice |
|-----------|---------|--------|---------------|
| `resolution` | 0.05 | Submaps resolution | Lower = better detail |
| `num_range_data` | 90 | Points per scan to use | Higher = more data, slower |
| `missing_data_ray_length` | 5.0 | Fill-in for max range | Match environment size |
| `max_range` | 30.0 | Max range to use | Match LiDAR (12m for C1M1) |

### Parameter Tuning Workflow Example

```bash
#!/bin/bash
# Automated parameter sweep

DATASET="tuning_challenge_dataset"
PARAMS=(
    "resolution:=0.025"
    "resolution:=0.05"
    "resolution:=0.075"
)

for i in "${!PARAMS[@]}"; do
    echo "Testing: ${PARAMS[$i]}"

    # Play bag
    ros2 bag play $DATASET --clock 100 --rate 2.0 &
    PLAY_PID=$!

    # Run SLAM with parameters
    ros2 launch slam_toolbox online_async_launch.py \
        use_sim_time:=true \
        ${PARAMS[$i]} &
    SLAM_PID=$!

    # Wait for completion
    wait $PLAY_PID
    sleep 5

    # Save map
    ros2 run nav2_map_server map_saver_cli -f "test_$i"

    # Stop SLAM
    kill $SLAM_PID
    sleep 2
done

echo "Parameter sweep complete. Compare test_*.pgm files."
```

### Debugging SLAM Issues with Bags

#### Common Issues and Diagnosis

**1. Poor Loop Closure**

```bash
# Replay slowly and watch for alignment
ros2 bag play problem_bag --clock 100 --rate 0.2

# Check loop closure parameters
# Increase correlation search space
# Decrease loop closure thresholds
```

**2. Map Drift**

```bash
# Check odometry quality
ros2 bag play problem_bag --topics /odom
ros2 topic echo /odom

# Verify TF tree consistency
ros2 run tf2_tools view_frames

# Consider using scan-matching odometry if wheel odom is poor
```

**3. Missing Features**

```bash
# Check scan quality during problem areas
ros2 bag play problem_bag --start-offset 45.0

# Adjust laser range filters
# Increase minimum_travel_distance to skip redundant scans
```

---

## 8. Troubleshooting

### Recording Issues

#### Problem: "Failed to open database"

**Cause**: Permissions or disk space issue

**Solution**:
```bash
# Check disk space
df -h

# Check write permissions
ls -la .

# Try different directory
cd /tmp
ros2 bag record /scan -o test_bag
```

#### Problem: "No messages recorded"

**Cause**: Topics not publishing or wrong topic names

**Solution**:
```bash
# Verify topics exist
ros2 topic list

# Check topic types
ros2 topic info /scan

# Test topic data
ros2 topic echo /scan --once

# Record with verbose output
ros2 bag record /scan -o test_bag -v
```

#### Problem: Bag file size growing too fast

**Cause**: Recording unnecessary topics or too much data

**Solution**:
```bash
# Use compression
ros2 bag record /scan /odom \
    --compression-mode file \
    --compression-format zstd

# Limit bag size
ros2 bag record /scan /odom \
    --max-bag-size 500000000

# Record only needed topics (not -a)
```

### Playback Issues

#### Problem: "Failed to open database"

**Cause**: Corrupted bag file or wrong storage format

**Solution**:
```bash
# Check bag integrity
ros2 bag info problem_bag

# Try different storage plugin
ros2 bag play problem_bag --storage sqlite3

# If corrupted, no easy fix - re-record
```

#### Problem: Topics not appearing during playback

**Cause**: Nodes not subscribing or clock issues

**Solution**:
```bash
# Check what's being published
ros2 topic list

# Verify playback is running
ros2 bag info --verbose problem_bag

# Check for clock issues (use --clock)
ros2 bag play problem_bag --clock 100

# Ensure SLAM node uses sim time
ros2 param get /slam_toolbox use_sim_time
# Should return: true
```

#### Problem: SLAM node not processing bag data

**Cause**: Simulation time mismatch

**Solution**:
```bash
# Play bag WITH clock
ros2 bag play bag_name --clock 100

# Launch SLAM WITH use_sim_time
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true

# Verify sim time parameter
ros2 param set /slam_toolbox use_sim_time true
```

### LiDAR Data Quality Issues

#### Problem: High percentage of invalid ranges

**Symptoms**: >20% invalid, inf, or nan values

**Causes and Solutions**:

1. **Too close to obstacles**
   ```bash
   # Check range_min in scans
   ros2 topic echo /scan --once | grep range_min
   # Ensure robot not closer than 0.05m to walls
   ```

2. **Reflective/transparent surfaces**
   ```bash
   # Glass, mirrors, polished floors cause issues
   # Solution: Avoid or mark these areas
   ```

3. **LiDAR malfunction**
   ```bash
   # Check hardware connection
   ls -la /dev/rplidar

   # Restart LiDAR node
   # Clean LiDAR lens
   ```

#### Problem: Low scan rate (<9 Hz)

**Causes and Solutions**:

1. **CPU overload**
   ```bash
   # Check CPU usage
   top

   # Reduce other processes
   # Lower SLAM update rate
   ```

2. **USB interference**
   ```bash
   # Try different USB port
   # Use USB 2.0 port (not 3.0 hub)
   # Check cable quality
   ```

3. **Serial communication issues**
   ```bash
   # Verify baud rate
   ros2 param get /rplidar_node serial_baudrate
   # Should be 460800 for C1M1
   ```

#### Problem: Noisy/unstable scans

**Symptoms**: Scans vibrating, flickering, or inconsistent

**Causes and Solutions**:

1. **Vibration/mounting issues**
   ```bash
   # Ensure rigid mounting
   # Check for loose connections
   # Dampen robot vibrations
   ```

2. **Environmental factors**
   ```bash
   # Avoid direct sunlight
   # Check for moving objects
   # Verify stable surfaces
   ```

3. **Electrical interference**
   ```bash
   # Separate LiDAR USB from motor controllers
   # Use shielded USB cable
   # Check power supply quality
   ```

### SLAM-Specific Issues

#### Problem: Map drift over time

**Diagnosis**:
```bash
# Play bag and watch map in RViz
ros2 bag play problem_bag --clock 100 --rate 0.5

# Check odometry quality
ros2 topic echo /odom

# Monitor TF transforms
ros2 run tf2_ros tf2_echo odom base_link
```

**Solutions**:
- Increase loop closure frequency (move slower)
- Improve odometry source (use scan-matching)
- Tune SLAM parameters (correlation search space)
- Add more features to environment

#### Problem: Failed loop closures

**Diagnosis**:
```bash
# Watch SLAM output for loop closure messages
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    | grep -i "loop"

# Check scan overlap when returning to start
```

**Solutions**:
- Ensure sufficient overlap (>5 meters)
- Move slower through loop closure zones
- Increase correlation search space
- Add distinctive features to environment

#### Problem: Maps don't align between runs

**Diagnosis**:
```bash
# Record same path multiple times
ros2 bag record /scan /odom /tf /tf_static -o run1
# ... drive same path ...
ros2 bag record /scan /odom /tf /tf_static -o run2

# Generate maps from each
# Compare in image viewer
```

**Solutions**:
- Check odometry consistency
- Verify TF tree stability
- Improve feature-rich environment
- Increase scan matching frequency

---

## Appendix A: Quick Reference Commands

### Recording
```bash
# Minimal SLAM recording
ros2 bag record /scan /tf /tf_static -o session_name

# Full SLAM recording
ros2 bag record /scan /odom /tf /tf_static -o session_name

# With compression
ros2 bag record /scan /odom /tf /tf_static \
    --compression-mode file \
    --compression-format zstd \
    -o session_name
```

### Playback
```bash
# Normal playback
ros2 bag play session_name

# With clock (for SLAM)
ros2 bag play session_name --clock 100

# Slow motion
ros2 bag play session_name --rate 0.5 --clock 100

# Loop continuously
ros2 bag play session_name --loop --clock 100
```

### Inspection
```bash
# Bag information
ros2 bag info session_name

# Topic data rate
ros2 topic hz /scan

# Single message
ros2 topic echo /scan --once

# TF tree visualization
ros2 run tf2_tools view_frames
```

### Quality Check
```bash
# Scan rate
ros2 topic hz /scan

# Scan content
ros2 topic echo /scan --once | grep -E "range|angle"

# Quality analysis
python3 scripts/lidar_tools/check_lidar_quality.py session_name
```

---

## Appendix B: File Size Calculator

**Uncompressed Sizes** (per minute):

| Topic | Rate | Size/msg | Total/min |
|-------|------|----------|-----------|
| /scan | 10 Hz | 1-2 KB | 0.6-1.2 MB |
| /odom | 30 Hz | 400 B | 0.7 MB |
| /tf | varies | 200 B | 0.1 MB |
| **Total** | - | - | **1.4-2.0 MB** |

**Compressed Sizes** (zstd):
- Compression ratio: ~40-60%
- **Total/min**: 0.6-1.2 MB
- **Total/hour**: 36-72 MB

**Recommendation**:
- 10-minute session: ~10 MB compressed
- 30-minute session: ~30 MB compressed
- 1-hour session: ~60 MB compressed

---

## Appendix C: Recommended Session Templates

### Template 1: Quick Test Session (2-3 minutes)
```bash
# Short loop for testing SLAM parameters
# - Single room
# - One loop closure
# - Fast iteration
```

### Template 2: Full Mapping Session (15-30 minutes)
```bash
# Complete floor/building map
# - Multiple rooms/zones
# - Several loop closures
# - High coverage
```

### Template 3: Localization Test Session (5-10 minutes)
```bash
# Test localization with existing map
# - Revisit all areas
# - Multiple loop closures
# - Different paths than mapping
```

### Template 4: Algorithm Tuning Session (10 minutes)
```bash
# Challenging scenarios for parameter tuning
# - Long corridors
# - Feature-poor areas
# - Tight spaces
# - Multiple loop closures
```

---

**Last Updated**: 2026-01-11

**See Also**:
- `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/docs/LIDAR_SETUP.md`
- `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/docs/SLAM_MAPPING_FINDINGS.md`
- Helper scripts in `/home/devel/Desktop/WayfindR-driver/scripts/lidar_tools/`
