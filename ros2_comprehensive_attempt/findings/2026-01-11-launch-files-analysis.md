# Launch Files Analysis: SLAM and Localization
**Date**: 2026-01-11
**Files Analyzed**:
- `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/launch/slam.launch.py`
- `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/launch/localization.launch.py`
- `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/slam_params.yaml`
- `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml`

---

## Executive Summary

Both launch files are well-structured and can be partially tested without physical LiDAR hardware using recorded ROS2 bag files. The critical dependency on `/scan` topic data means:
- **With bag files**: SLAM Toolbox and AMCL can be fully tested
- **Without data**: Only TF transforms and node configuration can be verified

---

## 1. SLAM Launch File Analysis

### File: `slam.launch.py`

#### 1.1 Components Overview

| Component | Purpose | Hardware Dependent | Testable Without LiDAR |
|-----------|---------|-------------------|----------------------|
| RPLidar Node | Publishes `/scan` topic | YES | NO (but can skip) |
| Static TF: odom→base_link | Robot coordinate frame | NO | YES |
| Static TF: base_link→laser | LiDAR mounting position | NO | YES |
| SLAM Toolbox | Creates map from scans | YES (/scan topic) | YES (with bag file) |
| RViz | Visualization | NO | YES |

#### 1.2 Component Details

**1. RPLidar Node** (Lines 46-57)
```python
rplidar_node = Node(
    package='rplidar_ros',
    executable='rplidar_node',
    name='rplidar_node',
    parameters=[{
        'serial_port': LaunchConfiguration('serial_port'),
        'serial_baudrate': 460800,
        'frame_id': 'laser',
        'scan_mode': 'DenseBoost',
    }],
)
```
- **Purpose**: Communicates with physical RPLidar C1 sensor
- **Publishes**: `/scan` topic (sensor_msgs/LaserScan)
- **Hardware Required**: YES - needs `/dev/ttyUSB0` or specified serial port
- **Configuration**:
  - Baudrate: 460800 (standard for RPLidar C1)
  - Frame ID: `laser` (must match TF tree)
  - Scan Mode: `DenseBoost` (high-density mode for better mapping)

**2. Static TF: odom → base_link** (Lines 60-65)
```python
static_tf_odom = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
)
```
- **Purpose**: Defines robot position in odometry frame
- **Transform**: Identity (0,0,0) at origin - assumes stationary robot
- **Hardware Required**: NO
- **Note**: For moving robot, this would come from wheel encoders/odometry

**3. Static TF: base_link → laser** (Lines 67-73)
```python
static_tf_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
)
```
- **Purpose**: Defines LiDAR mounting position on robot
- **Transform**: (x=0, y=0, z=0.1m) - LiDAR is 10cm above base_link
- **Hardware Required**: NO
- **Testable**: YES - can verify with `ros2 run tf2_ros tf2_echo base_link laser`

**4. SLAM Toolbox** (Lines 76-83)
```python
slam_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    parameters=[slam_params_file],
)
```
- **Purpose**: Generates 2D occupancy grid map from LiDAR scans
- **Subscribes**: `/scan` topic
- **Publishes**: `/map` topic, TF map→odom
- **Hardware Required**: NO (needs data source)
- **Testable with bag file**: YES

**5. RViz** (Lines 86-94)
- **Purpose**: Visualization of map building process
- **Hardware Required**: NO
- **Testable**: YES (can run standalone)

#### 1.3 SLAM Toolbox Configuration

**File**: `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/slam_params.yaml`

##### Key Parameters Explained

**Frame Configuration** (Lines 13-16)
```yaml
odom_frame: odom
map_frame: map
base_frame: base_link
scan_topic: /scan
```
- Defines coordinate frame hierarchy: map → odom → base_link → laser
- `scan_topic`: Must match LiDAR publisher topic

**Map Resolution** (Line 26)
```yaml
resolution: 0.05  # 5cm per pixel
```
- Controls map detail vs. memory usage
- 0.05m (5cm) is good balance for indoor navigation
- Smaller = more detail but more memory/processing

**LiDAR Range** (Line 34)
```yaml
max_laser_range: 12.0  # meters
```
- Maximum range to use from LiDAR data
- Should match your LiDAR's reliable range
- RPLidar C1: 12m indoor, 16m outdoor

**Movement Thresholds** (Lines 44-45)
```yaml
minimum_travel_distance: 0.5  # meters
minimum_travel_heading: 0.5   # radians
```
- Minimum robot movement before processing new scan
- Prevents redundant processing when stationary
- 0.5m = ~1.6 feet, 0.5 rad = ~28.6 degrees

**Loop Closure** (Lines 66-77)
```yaml
do_loop_closing: true
loop_search_maximum_distance: 3.0
loop_match_minimum_chain_size: 10
```
- **Loop Closure**: Detects when robot returns to previously visited location
- Corrects accumulated drift by recognizing known areas
- Critical for large-scale mapping accuracy
- `loop_search_maximum_distance`: How far to look for loop closures (3m)
- `loop_match_minimum_chain_size`: Minimum matched scans needed (10)

**Scan Matching** (Lines 83-98)
```yaml
use_scan_matching: true
correlation_search_space_dimension: 0.5
correlation_search_space_resolution: 0.01
```
- **Scan Matching**: Aligns current scan with map
- `correlation_search_space_dimension`: Search area size (±0.5m)
- `correlation_search_space_resolution`: Search precision (1cm steps)
- Trades accuracy vs. processing time

**Solver Configuration** (Lines 117-122)
```yaml
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
```
- Uses Google's Ceres optimizer for graph optimization
- SPARSE_NORMAL_CHOLESKY: Fast solver for sparse problems
- Optimizes entire map pose graph when loop closures detected

#### 1.4 What Requires LiDAR vs. What Doesn't

**Requires Physical LiDAR**:
- `/scan` topic generation (RPLidar node)
- Real-time mapping

**Works Without LiDAR**:
- TF tree setup and validation
- SLAM Toolbox configuration loading
- RViz visualization (can visualize recorded data)
- All processing with bag file replay

---

## 2. Localization Launch File Analysis

### File: `localization.launch.py`

#### 2.1 Components Overview

| Component | Purpose | Hardware Dependent | Testable Without LiDAR |
|-----------|---------|-------------------|----------------------|
| RPLidar Node | Publishes `/scan` topic | YES | NO (but can skip) |
| Static TF: odom→base_link | Robot position | NO | YES |
| Static TF: base_link→laser | LiDAR position | NO | YES |
| Map Server | Loads saved map | NO (needs map file) | YES |
| AMCL | Localizes on map | YES (/scan topic) | YES (with bag file) |
| Lifecycle Manager | Manages node lifecycle | NO | YES |
| RViz | Visualization | NO | YES |

#### 2.2 Component Details

**1. RPLidar Node** (Lines 52-63)
- Same configuration as SLAM launch file
- See Section 1.2 Component #1

**2. Static TF Publishers** (Lines 66-79)
- Same as SLAM launch file
- See Section 1.2 Components #2 and #3

**3. Map Server** (Lines 82-91)
```python
map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    parameters=[{
        'yaml_filename': LaunchConfiguration('map'),
        'use_sim_time': False,
    }],
)
```
- **Purpose**: Loads and publishes saved map from SLAM
- **Publishes**: `/map` topic (nav_msgs/OccupancyGrid)
- **Requires**: Map YAML file (from SLAM save)
- **Hardware Required**: NO
- **Testable**: YES (if you have a saved map)

**4. AMCL Node** (Lines 94-100)
```python
amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    parameters=[amcl_params],
)
```
- **Purpose**: Localizes robot pose on known map using particle filter
- **Subscribes**: `/scan` (from LiDAR), `/map` (from map_server)
- **Publishes**: `/particle_cloud`, TF map→odom
- **Hardware Required**: NO (needs `/scan` data)
- **Testable with bag file**: YES

**5. Lifecycle Manager** (Lines 103-114)
```python
lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    parameters=[{
        'autostart': True,
        'node_names': ['map_server', 'amcl'],
        'bond_timeout': 4.0,
    }],
)
```
- **Purpose**: Manages startup/shutdown of Nav2 lifecycle nodes
- **Manages**: map_server and AMCL activation sequence
- **Autostart**: Automatically activates nodes on launch
- **Bond Timeout**: 4s to wait for node heartbeat
- **Hardware Required**: NO
- **Testable**: YES

#### 2.3 AMCL Configuration Analysis

**File**: `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml`

##### What is AMCL?

**AMCL (Adaptive Monte Carlo Localization)** is a probabilistic localization algorithm that:
1. Uses a **particle filter** to track robot pose
2. Each particle represents a possible robot position hypothesis
3. Particles are weighted based on how well LiDAR scans match the map
4. Over time, particles converge to robot's true position
5. Adapts number of particles based on localization confidence

##### Key Parameters Explained

**Frame Configuration** (Lines 18-27)
```yaml
global_frame_id: map       # World frame
odom_frame_id: odom        # Odometry frame
base_frame_id: base_link   # Robot frame
scan_topic: /scan          # LiDAR topic
```
- **TF Tree**: map → odom → base_link → laser
- AMCL computes and publishes map→odom transform
- Odometry provides odom→base_link (from wheels/IMU)

**Particle Filter** (Lines 34-39)
```yaml
min_particles: 500
max_particles: 2000
pf_err: 0.05      # Resampling error threshold
pf_z: 0.99        # Confidence level
```
- **min_particles**: Minimum particle count (500)
  - Fewer = faster but less robust
  - More = slower but more accurate
- **max_particles**: Maximum when uncertain (2000)
  - System adds particles when localization is poor
- **pf_err/pf_z**: Controls resampling trigger
  - Resamples when effective particle count drops
  - pf_err=0.05 means resample at 5% error
  - pf_z=0.99 means 99% confidence requirement

**Motion Model** (Lines 46-53)
```yaml
robot_model_type: nav2_amcl::DifferentialMotionModel
alpha1: 0.2  # Rotation noise from rotation
alpha2: 0.2  # Rotation noise from translation
alpha3: 0.2  # Translation noise from translation
alpha4: 0.2  # Translation noise from rotation
```
- **Differential Drive Model**: For 2-wheeled robots
- **Alpha Parameters**: Model odometry noise
  - Higher values = less trust in odometry
  - alpha1: Rotation causes rotation error
  - alpha2: Forward motion causes rotation error (wheel slip)
  - alpha3: Forward motion causes translation error
  - alpha4: Rotation causes translation error
- **Tuning**: Start at 0.2, increase if localization drifts

**Laser Model** (Lines 60-75)
```yaml
laser_model_type: likelihood_field
laser_min_range: 0.1   # meters
laser_max_range: 12.0  # meters
max_beams: 60          # Subset of beams to use

z_hit: 0.95      # Weight for correct measurements
z_short: 0.1     # Weight for short readings (obstacles)
z_max: 0.05      # Weight for max range readings
z_rand: 0.05     # Weight for random noise
sigma_hit: 0.2   # Std dev for measurement noise
```
- **likelihood_field**: Fast method, computes probability of scan given map
  - Alternative: "beam" model (more accurate but slower)
- **max_beams**: Uses 60 of ~360 beams for performance
  - More beams = more accurate but slower
- **Z-weights**: Probability model for different reading types
  - z_hit (0.95): Most readings are accurate
  - z_short (0.1): Some blocked by dynamic obstacles
  - z_max (0.05): Some max-range readings (windows, mirrors)
  - z_rand (0.05): Some random noise
  - **Must sum to ~1.0**
- **sigma_hit**: Measurement noise standard deviation (20cm)

**Update Thresholds** (Lines 82-89)
```yaml
update_min_d: 0.1      # meters
update_min_a: 0.2      # radians
resample_interval: 1   # every N updates
transform_tolerance: 1.0  # seconds
```
- **update_min_d**: Minimum distance to move before filter update (10cm)
- **update_min_a**: Minimum rotation before update (0.2 rad ≈ 11.5°)
- **Prevents**: Excessive updates when stationary
- **resample_interval**: Resample particles every 1 update
- **transform_tolerance**: Accept TF up to 1 second old

**Recovery** (Lines 96-97)
```yaml
recovery_alpha_slow: 0.001
recovery_alpha_fast: 0.1
```
- **Recovery Mode**: Triggered when localization is lost
- Compares long-term vs. short-term likelihood averages
- If short-term drops below long-term: add random particles
- **alpha_slow**: Slow average decay (0.001 = very slow)
- **alpha_fast**: Fast average decay (0.1 = quick response)
- **Effect**: Helps recover from kidnapped robot problem

**Initial Pose** (Lines 105-110)
```yaml
set_initial_pose: false
initial_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
```
- **set_initial_pose**: Auto-set pose on startup
- **false**: Must use RViz "2D Pose Estimate" tool
- **true**: Would start at specified coordinates
- **Best Practice**: Set false, manually initialize in RViz

**TF Broadcasting** (Line 124)
```yaml
tf_broadcast: true
```
- **true**: Publish map→odom transform
- **Required**: For navigation stack to work
- **Output**: Corrects odometry drift using map matching

#### 2.4 Parameter Tuning Guide

##### When to Adjust Particle Counts

**Increase particles (→2000) if**:
- Localization is unstable or jumps around
- Environment has symmetry (hallways, repetitive features)
- Multiple similar-looking locations

**Decrease particles (→500) if**:
- CPU usage too high
- Environment is unique/distinctive
- Good odometry (low drift)

##### When to Adjust Alpha Values

**Increase alphas (→0.5) if**:
- Odometry is poor (wheel slip, rough terrain)
- Robot drifts away from true position
- Particles don't spread enough

**Decrease alphas (→0.1) if**:
- Particles spread too much
- Good odometry from encoders
- Smooth floors, no wheel slip

##### When to Adjust Laser Model

**Increase z_hit (→0.98) if**:
- LiDAR is very accurate
- Clean indoor environment
- No dynamic obstacles

**Increase z_short (→0.2) if**:
- Many dynamic obstacles (people, chairs)
- Short readings are common

**Increase z_rand (→0.1) if**:
- Noisy LiDAR
- Reflective surfaces (glass, mirrors)

---

## 3. Testing Strategy

### 3.1 Without LiDAR Hardware (No Data Source)

**What You CAN Test**:

1. **TF Tree Structure**
   ```bash
   # Terminal 1: Launch without RPLidar node (modify launch file)
   ros2 launch localization.launch.py use_rviz:=false

   # Terminal 2: Check TF tree
   ros2 run tf2_tools view_frames
   ros2 run tf2_ros tf2_echo odom base_link
   ros2 run tf2_ros tf2_echo base_link laser
   ```

2. **Node Configuration Loading**
   ```bash
   # Check if parameters load correctly
   ros2 param list
   ros2 param get /slam_toolbox resolution
   ros2 param get /amcl min_particles
   ```

3. **Launch File Syntax**
   ```bash
   # Validate launch file syntax
   ros2 launch --show-args slam.launch.py
   ros2 launch --show-args localization.launch.py
   ```

4. **Map Server** (if you have a map)
   ```bash
   # Test map loading
   ros2 launch localization.launch.py \
     map:=/path/to/map.yaml \
     use_rviz:=true

   # Verify map published
   ros2 topic echo /map
   ```

**What You CANNOT Test**:
- SLAM map building (no `/scan` data)
- AMCL localization (no `/scan` data)
- Loop closure detection
- Scan matching algorithms
- Actual pose estimation

### 3.2 With Recorded Bag Files (RECOMMENDED)

**Available Bag File**:
```
Location: /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/testing/test_run/
Duration: 115.6 seconds (~2 minutes)
Topics:
  - /scan: 2313 messages (sensor_msgs/LaserScan)
  - /odom: 4625 messages (nav_msgs/Odometry)
  - /tf: 10043 messages (tf2_msgs/TFMessage)
  - /tf_static: 1 message (tf2_msgs/TFMessage)
```

**Full SLAM Testing Workflow**:

```bash
# 1. Launch SLAM (without RPLidar node - comment it out or create modified launch)
ros2 launch slam.launch.py use_rviz:=true

# 2. In another terminal, play bag file
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/testing
ros2 bag play test_run --rate 0.5  # Half speed for observation

# 3. Watch map build in RViz
# - Add Map display
# - Topic: /map
# - Should see map being built from scan data

# 4. Save map when complete
ros2 run nav2_map_server map_saver_cli -f my_test_map
```

**Full Localization Testing Workflow**:

```bash
# 1. Ensure you have a saved map from SLAM
ls -l ~/maps/my_test_map.yaml
ls -l ~/maps/my_test_map.pgm

# 2. Launch localization (without RPLidar node)
ros2 launch localization.launch.py \
  map:=$HOME/maps/my_test_map.yaml \
  use_rviz:=true

# 3. Set initial pose in RViz
# - Click "2D Pose Estimate" tool
# - Click and drag on map at robot's starting position

# 4. Play bag file
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/testing
ros2 bag play test_run --rate 0.5

# 5. Observe in RViz
# - Watch particle cloud converge
# - Green arrow shows estimated pose
# - Red dots show particle distribution
```

**What This Tests**:
- Complete SLAM pipeline (scan matching, loop closure, map building)
- Complete localization pipeline (particle filter, pose estimation)
- Parameter configurations
- TF tree correctness
- Map quality and resolution
- Localization accuracy and convergence speed
- Recovery from poor initial pose

### 3.3 With Physical LiDAR Hardware

**When you have actual hardware**:

```bash
# 1. Connect RPLidar to /dev/ttyUSB0

# 2. Test LiDAR only
ros2 launch rplidar_ros rplidar.launch.py

# 3. Check scan data
ros2 topic echo /scan --once
ros2 topic hz /scan  # Should be ~10 Hz

# 4. Test SLAM with live data
ros2 launch slam.launch.py

# 5. Move robot around environment
# - Can push manually or use teleop
# - Watch map build in real-time

# 6. Save map
ros2 run nav2_map_server map_saver_cli -f live_map

# 7. Test localization with saved map
ros2 launch localization.launch.py map:=$HOME/maps/live_map.yaml
```

---

## 4. Configuration Verification Checklist

### 4.1 Pre-Launch Checks

**File Existence**:
- [ ] `slam_params.yaml` exists and readable
- [ ] `amcl_params.yaml` exists and readable
- [ ] `rviz_config.rviz` exists (optional but recommended)
- [ ] Map files exist (for localization only)

**Parameter Validation**:
```bash
# Check SLAM parameters
grep -A 5 "resolution:" config/slam_params.yaml
grep -A 5 "max_laser_range:" config/slam_params.yaml

# Check AMCL parameters
grep -A 5 "min_particles:" config/amcl_params.yaml
grep -A 5 "laser_model_type:" config/amcl_params.yaml
```

### 4.2 Runtime Verification

**After launching SLAM**:
```bash
# 1. Check all nodes running
ros2 node list
# Expected: /rplidar_node, /slam_toolbox, /rviz2, /static_tf_*

# 2. Check topics published
ros2 topic list
# Expected: /scan, /map, /tf, /tf_static

# 3. Verify TF tree
ros2 run tf2_tools view_frames
# Expected: map → odom → base_link → laser

# 4. Check SLAM parameters loaded
ros2 param get /slam_toolbox resolution
# Expected: 0.05

# 5. Monitor map updates
ros2 topic hz /map
# Expected: ~0.2 Hz (every 5 seconds per config)
```

**After launching Localization**:
```bash
# 1. Check all nodes running
ros2 node list
# Expected: /rplidar_node, /map_server, /amcl, /lifecycle_manager_localization

# 2. Check topics
ros2 topic list
# Expected: /scan, /map, /particle_cloud, /tf

# 3. Verify map loaded
ros2 topic echo /map --once
# Should show map metadata

# 4. Check AMCL particles
ros2 topic echo /particle_cloud
# Should show particle poses

# 5. Verify TF published
ros2 run tf2_ros tf2_echo map odom
# Should show transform (may be identity until localized)
```

### 4.3 Troubleshooting Common Issues

**Problem**: SLAM Toolbox not publishing map
```bash
# Check if receiving scan data
ros2 topic hz /scan
ros2 topic echo /scan --once

# Check SLAM parameters
ros2 param list | grep slam_toolbox
ros2 param get /slam_toolbox scan_topic

# Check for errors
ros2 node info /slam_toolbox
```

**Problem**: AMCL particles not converging
```bash
# Increase particle count
ros2 param set /amcl max_particles 5000

# Check if map is loaded
ros2 topic echo /map --once

# Verify scan topic
ros2 topic hz /scan

# Set better initial pose in RViz
```

**Problem**: TF tree broken
```bash
# List all TF frames
ros2 run tf2_tools view_frames

# Check for missing transforms
ros2 run tf2_ros tf2_monitor

# Verify static publishers running
ros2 node list | grep static_transform
```

---

## 5. Summary: Hardware Dependencies

### Components That Require Physical LiDAR

1. **RPLidar Node**
   - Needs `/dev/ttyUSB0` serial connection
   - Cannot run without hardware
   - **Solution**: Comment out or replace with bag file replay

### Components That Work Without Hardware

1. **Static TF Publishers** - Always testable
2. **SLAM Toolbox** - Testable with bag file
3. **Map Server** - Testable if you have a saved map
4. **AMCL** - Testable with bag file + saved map
5. **Lifecycle Manager** - Always testable
6. **RViz** - Always testable

### Testing Recommendations

**Priority 1 - Do First (No Hardware Needed)**:
1. Verify TF tree structure with static publishers only
2. Test map server with existing map from other attempts
3. Validate parameter files load correctly
4. Check launch file syntax and arguments

**Priority 2 - Do Next (Needs Bag File)**:
Use existing bag file at:
`/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/testing/test_run/`

1. Test complete SLAM pipeline with recorded data
2. Save map from SLAM
3. Test AMCL localization with saved map + recorded data
4. Tune parameters and observe effects

**Priority 3 - Do Last (Needs Hardware)**:
1. Test with live LiDAR when available
2. Collect new bag files in your actual environment
3. Create production maps
4. Validate real-time performance

---

## 6. Next Steps

### Immediate Actions (No Hardware Required)

1. **Create Modified Launch Files**
   - Copy existing launch files
   - Comment out RPLidar node
   - Test with bag file replay

2. **Test with Existing Bag Data**
   ```bash
   # Run SLAM on recorded data
   ros2 bag play test_run
   # Save resulting map
   ```

3. **Verify Configuration Loading**
   ```bash
   # Check all parameters load correctly
   ros2 launch slam.launch.py --show-args
   ```

### When Hardware Available

1. **Calibrate LiDAR Position**
   - Measure actual base_link → laser transform
   - Update static TF publisher

2. **Tune SLAM Parameters**
   - Test different resolutions
   - Adjust loop closure thresholds
   - Optimize for your environment

3. **Tune AMCL Parameters**
   - Adjust alpha values for your odometry
   - Tune particle counts
   - Optimize laser model weights

### Documentation Needs

1. **Create Testing Procedures**
   - Step-by-step test scripts
   - Expected outputs
   - Troubleshooting guide

2. **Parameter Tuning Guide**
   - When to adjust each parameter
   - Expected effects
   - Environment-specific recommendations

---

## Appendix: Quick Reference

### SLAM Launch Arguments
```bash
# Default launch
ros2 launch slam.launch.py

# Custom serial port
ros2 launch slam.launch.py serial_port:=/dev/rplidar

# No visualization
ros2 launch slam.launch.py use_rviz:=false
```

### Localization Launch Arguments
```bash
# Default launch (must provide map)
ros2 launch localization.launch.py map:=/path/to/map.yaml

# Custom serial port
ros2 launch localization.launch.py \
  map:=/path/to/map.yaml \
  serial_port:=/dev/rplidar

# No visualization
ros2 launch localization.launch.py \
  map:=/path/to/map.yaml \
  use_rviz:=false
```

### Useful Commands
```bash
# Save map during SLAM
ros2 run nav2_map_server map_saver_cli -f map_name

# Play bag file at half speed
ros2 bag play bag_name --rate 0.5

# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo frame1 frame2

# Monitor topic rate
ros2 topic hz /scan

# Get parameter value
ros2 param get /slam_toolbox resolution
```

---

**End of Analysis**
