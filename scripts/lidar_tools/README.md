# LiDAR Tools for SLAM Testing

This directory contains helper scripts for recording, replaying, and analyzing LiDAR data for SLAM testing with the RP LIDAR C1M1.

## Scripts Overview

### 1. record_lidar_session.sh
Records LiDAR data from the robot for later SLAM processing.

**Features:**
- Automatic session naming with timestamps
- Pre-flight checks for LiDAR connection and topic health
- Multiple recording modes (minimal, full, custom topics)
- Compression enabled by default
- Best practices guidance during recording

**Usage:**
```bash
# Quick start - record with defaults
./record_lidar_session.sh

# Named session
./record_lidar_session.sh my_test_session

# Minimal recording (LiDAR only, no odometry)
./record_lidar_session.sh my_test minimal

# Custom topics
./record_lidar_session.sh my_test custom "/scan /odom /imu /tf /tf_static"
```

**Prerequisites:**
- LiDAR node must be running
- Required topics must be publishing

**Output:**
- Creates a compressed rosbag2 directory with recorded data
- Displays recording summary and next steps

---

### 2. replay_for_slam.sh
Replays recorded bag files for SLAM mapping or localization testing.

**Features:**
- Three replay modes: mapping, localization, playback
- Automatic simulation time handling
- Interactive setup and guidance
- Speed control for detailed inspection

**Usage:**
```bash
# Create map from recording
./replay_for_slam.sh my_session

# Create map at half speed (better for visualization)
./replay_for_slam.sh my_session mapping 0.5

# Test localization with existing map
./replay_for_slam.sh my_session localization

# Just replay data without SLAM
./replay_for_slam.sh my_session playback

# Fast replay at 2x speed
./replay_for_slam.sh my_session playback 2.0
```

**Modes:**

1. **mapping** (default)
   - Replays bag with SLAM Toolbox
   - Creates new map from recorded data
   - Interactive map saving

2. **localization**
   - Tests localization against existing map
   - Launches map server and AMCL
   - Useful for testing localization accuracy

3. **playback**
   - Just replays topics without SLAM
   - Good for visual inspection and quality checks
   - Options for looping and pausing

**Prerequisites:**
- For mapping: `ros-humble-slam-toolbox`
- For localization: `ros-humble-nav2-amcl`, `ros-humble-nav2-map-server`

---

### 3. check_lidar_quality.py
Analyzes recorded LiDAR scan data quality for SLAM suitability.

**Features:**
- Comprehensive quality metrics
- Range validity analysis
- Scan rate consistency checks
- Quality score and recommendations
- Detailed statistics and warnings

**Usage:**
```bash
# Basic analysis
python3 check_lidar_quality.py my_session

# Analyze first 100 scans only
python3 check_lidar_quality.py my_session --max-samples 100

# Verbose output
python3 check_lidar_quality.py my_session --verbose

# Different topic name
python3 check_lidar_quality.py my_session --topic /custom_scan
```

**Quality Metrics:**

| Metric | Good | Acceptable | Poor |
|--------|------|------------|------|
| Valid range % | >90% | 80-90% | <80% |
| Points per scan | >500 | 300-500 | <300 |
| Scan rate | 9.5-10.5 Hz | 8-11 Hz | <8 or >11 Hz |
| Max range readings | <20% | 20-40% | >40% |

**Output Includes:**
- Scan statistics (rate, point count)
- Range validity percentages
- Invalid range breakdown (inf, nan, zero)
- Range distribution histogram
- Quality warnings and recommendations
- Overall quality score (0-100)
- SLAM usage recommendations

---

## Complete Workflow Example

### 1. Start LiDAR Node
```bash
# In terminal 1
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/rplidar \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser \
    -p scan_mode:=DenseBoost
```

### 2. Start Odometry (if using full mode)
```bash
# In terminal 2
# Option A: Wheel odometry (if available)
ros2 run your_robot odometry_node

# Option B: Laser odometry
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node
```

### 3. Record Session
```bash
# In terminal 3
cd /home/devel/Desktop/WayfindR-driver
./scripts/lidar_tools/record_lidar_session.sh my_mapping_session

# Follow best practices:
# - Move slowly (0.3-0.6 m/s)
# - Return to starting point (loop closure)
# - Cover all areas thoroughly
# - Avoid featureless environments
```

### 4. Check Data Quality
```bash
# After recording completes
python3 scripts/lidar_tools/check_lidar_quality.py my_mapping_session

# Review quality score and warnings
# If score < 70, consider re-recording with improvements
```

### 5. Create Map
```bash
# If quality is good, create map
./scripts/lidar_tools/replay_for_slam.sh my_mapping_session mapping

# In another terminal, visualize
ros2 run rviz2 rviz2
# Add displays: /map, /scan

# When replay completes, save map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 6. Test Localization (Optional)
```bash
# Test localization with recorded data
./scripts/lidar_tools/replay_for_slam.sh my_mapping_session localization

# When prompted, enter map file path:
# /path/to/my_map.yaml

# Visualize in RViz:
# - Add /map display
# - Add /scan display
# - Add /amcl_pose display
# - Add /particlecloud display
```

---

## Tips and Best Practices

### Recording Tips

1. **Pre-flight Checks**
   - Always verify LiDAR is spinning and publishing
   - Check topic rates before recording
   - Ensure sufficient disk space

2. **Movement Patterns**
   - Keep speed under 0.6 m/s
   - Perform loop closures every 60-120 meters
   - Overlap loops by at least 5 meters
   - Return to starting point at end

3. **Environment Considerations**
   - Avoid featureless areas (blank walls, open spaces)
   - Watch for reflective surfaces (glass, mirrors)
   - Maintain moderate lighting (avoid direct sunlight)
   - Map when environment is static (no moving people)

4. **Session Length**
   - Test sessions: 2-3 minutes
   - Room mapping: 5-10 minutes
   - Full floor mapping: 15-30 minutes
   - Keep sessions focused and manageable

### Quality Check Tips

1. **Interpreting Scores**
   - 85-100: Excellent, use for high-accuracy mapping
   - 70-84: Good, suitable for general SLAM
   - 50-69: Acceptable, but may need tuning
   - <50: Poor, recommend re-recording

2. **Common Issues**
   - **Low valid range %**: Obstacles too close/far, or reflective surfaces
   - **Low scan rate**: CPU overload or USB issues
   - **High max range readings**: Open spaces (normal) or sensor problem
   - **Low point count**: Wrong scan mode or obstruction

### Replay Tips

1. **Replay Speed**
   - 0.5x: Better for visualization and debugging
   - 1.0x: Normal speed, faster processing
   - 2.0x: Quick testing and iteration

2. **Simulation Time**
   - Always use `--clock` when replaying for SLAM
   - Ensure SLAM nodes have `use_sim_time:=true`
   - Check with: `ros2 param get /slam_toolbox use_sim_time`

3. **Parameter Tuning**
   - Use same bag for all parameter tests (repeatability)
   - Change one parameter at a time
   - Save maps with descriptive names for comparison
   - Document parameter changes and results

---

## Troubleshooting

### "Topic /scan not found"
**Problem**: LiDAR not publishing data
**Solution**:
```bash
# Check if node is running
ros2 node list | grep rplidar

# Check topics
ros2 topic list | grep scan

# Restart LiDAR node
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/rplidar \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser \
    -p scan_mode:=DenseBoost
```

### "Failed to open database"
**Problem**: Bag file corrupted or wrong path
**Solution**:
```bash
# Verify bag exists
ls -la my_session/

# Check bag info
ros2 bag info my_session

# If corrupted, no easy fix - must re-record
```

### Low Quality Score
**Problem**: Poor scan quality for SLAM
**Solution**:
1. Check LiDAR connection (USB cable, power)
2. Clean LiDAR lens
3. Verify DenseBoost mode is active
4. Check for environmental issues (reflective surfaces, sunlight)
5. Ensure rigid LiDAR mounting (no vibration)
6. Re-record with slower, more careful movement

### SLAM Not Processing Bag Data
**Problem**: Simulation time mismatch
**Solution**:
```bash
# Replay WITH clock
ros2 bag play my_session --clock 100

# Launch SLAM WITH sim time
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Verify parameter
ros2 param get /slam_toolbox use_sim_time
# Should show: true
```

---

## File Organization

**Recommended directory structure:**
```
WayfindR-driver/
├── scripts/
│   └── lidar_tools/
│       ├── README.md (this file)
│       ├── record_lidar_session.sh
│       ├── replay_for_slam.sh
│       └── check_lidar_quality.py
├── bags/  (create this directory for recordings)
│   ├── test_session_20260111_143022/
│   ├── mapping_session_20260111_150000/
│   └── tuning_session_20260111_160000/
└── maps/  (create this directory for saved maps)
    ├── my_map.yaml
    ├── my_map.pgm
    └── test_map.yaml
```

**Create directories:**
```bash
cd /home/devel/Desktop/WayfindR-driver
mkdir -p bags maps
```

---

## Additional Resources

**Related Documentation:**
- `/home/devel/Desktop/WayfindR-driver/findings/lidar-data-workflow.md` - Comprehensive workflow guide
- `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/docs/LIDAR_SETUP.md` - LiDAR hardware setup
- `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/docs/SLAM_MAPPING_FINDINGS.md` - SLAM configuration

**ROS2 Documentation:**
- [rosbag2 Recording and Playback](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)

**Hardware Documentation:**
- [RP LIDAR C1 Product Page](https://www.slamtec.com/en/C1)
- [RP LIDAR ROS2 Package](https://github.com/Slamtec/rplidar_ros)

---

**Last Updated**: 2026-01-11
