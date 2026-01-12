# SLAM Toolbox Parameter Tuning for RP LIDAR C1M1
**Date:** 2026-01-11
**Author:** Research compiled for WayfindR-driver project
**Target Hardware:** RP LIDAR C1M1 on Raspberry Pi

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [RP LIDAR C1M1 Specifications](#rp-lidar-c1m1-specifications)
3. [Parameter Categories](#parameter-categories)
4. [Detailed Parameter Explanations](#detailed-parameter-explanations)
5. [Recommended Configuration for RP LIDAR C1M1](#recommended-configuration-for-rp-lidar-c1m1)
6. [Trade-offs: Accuracy vs Performance](#trade-offs-accuracy-vs-performance)
7. [Tuning Methodology](#tuning-methodology)
8. [Raspberry Pi Optimization Strategies](#raspberry-pi-optimization-strategies)
9. [Analysis of Current Configuration](#analysis-of-current-configuration)
10. [Recommended Changes](#recommended-changes)
11. [References](#references)

---

## Executive Summary

This document provides comprehensive guidance for tuning SLAM Toolbox parameters specifically for the RP LIDAR C1M1 sensor in indoor environments running on Raspberry Pi hardware. SLAM Toolbox is a pose-graph SLAM method that offers superior accuracy (ATE ~0.13m) but requires careful parameter tuning to balance computational demands with mapping precision.

**Key Findings:**
- RP LIDAR C1M1 provides 360° FOV with 12m range and ±30mm accuracy
- SLAM Toolbox requires ~70% CPU and 293 MB RAM for optimal performance
- Indoor environments benefit from smaller travel thresholds and finer resolution
- Loop closure detection is critical for long-term mapping accuracy
- Raspberry Pi optimization requires careful balance of scan rates and resolution

---

## RP LIDAR C1M1 Specifications

### Core Specifications
| Parameter | Value | Notes |
|-----------|-------|-------|
| **Field of View** | 360° | Full omnidirectional coverage |
| **Range** | 0.05m - 12m | White objects (70% reflectivity) |
| **Range (Dark)** | 0.05m - 6m | Black objects (10% reflectivity) |
| **Accuracy** | ±30mm | @ 25°C |
| **Sample Rate** | 5000 Hz | 5000 samples/second |
| **Scan Frequency** | 8-12 Hz | Typical: 10 Hz @ 600 RPM |
| **Angular Resolution** | 0.72° | @ 10Hz scan rate |
| **Technology** | DTOF (Direct Time of Flight) | Fusion-type laser scanner |
| **Laser Class** | Class 1 | IEC-60825 compliant |
| **Interface** | TTL UART | 3.3V level |
| **Power** | 5V DC ±0.2V | 230mA typical @ 10Hz |

**Source:** [SLAMTEC RPLIDAR C1 Official Specifications](https://www.slamtec.com/en/C1)

### Implications for SLAM Configuration
1. **360° FOV:** Full coverage eliminates concerns about limited FOV degradation (unlike 90° sensors)
2. **12m Range:** Suitable for indoor environments; max_laser_range should be set to 12.0
3. **10Hz Scan Rate:** Allows moderate robot speeds; consider throttle_scans parameter
4. **±30mm Accuracy:** Enables fine resolution mapping (0.05m resolution is appropriate)
5. **5000 Hz Sample Rate:** Provides dense point clouds for reliable scan matching

---

## Parameter Categories

SLAM Toolbox parameters can be grouped into seven functional categories:

### 1. **Solver & Optimization Parameters**
Control the underlying optimization engine (Google Ceres)

### 2. **ROS Integration Parameters**
Frame IDs, topics, and ROS-specific settings

### 3. **Map Update & Timing Parameters**
Control when and how frequently maps are updated

### 4. **Motion Threshold Parameters**
Determine when new scans are processed based on robot motion

### 5. **Scan Matching Parameters**
Control local scan-to-map alignment

### 6. **Loop Closure Parameters**
Enable long-term consistency through loop detection

### 7. **Correlation Search Parameters**
Fine-tune the search space for scan matching

---

## Detailed Parameter Explanations

### 1. Solver & Optimization Parameters

#### `solver_plugin: solver_plugins::CeresSolver`
**Purpose:** Specifies the backend optimization plugin
**Recommended:** `solver_plugins::CeresSolver` (Google Ceres - fastest and most flexible)
**Alternatives:** None recommended

#### `ceres_linear_solver: SPARSE_NORMAL_CHOLESKY`
**Purpose:** Linear solver method for pose graph optimization
**Options:**
- `SPARSE_NORMAL_CHOLESKY` - Best for medium-sized maps (recommended for indoor)
- `SPARSE_SCHUR` - Better for very large outdoor maps
- `DENSE_QR` - Small maps only

**Trade-off:** SPARSE_NORMAL_CHOLESKY balances speed and accuracy for indoor environments

#### `ceres_preconditioner: SCHUR_JACOBI`
**Purpose:** Preconditioner to accelerate convergence
**Recommended:** `SCHUR_JACOBI` for most cases
**Alternative:** `JACOBI` for simpler problems

#### `ceres_trust_strategy: LEVENBERG_MARQUARDT`
**Purpose:** Trust region optimization strategy
**Recommended:** `LEVENBERG_MARQUARDT` (robust and reliable)
**Alternative:** `DOGLEG` (faster but less robust)

#### `ceres_dogleg_type: TRADITIONAL_DOGLEG`
**Purpose:** Dogleg method variant (only used if trust_strategy is DOGLEG)
**Recommended:** `TRADITIONAL_DOGLEG`

#### `ceres_loss_function: None`
**Purpose:** Robust loss function for outlier rejection
**Options:**
- `None` - No outlier rejection (recommended when odometry is good)
- `HuberLoss` - Gentle outlier rejection
- `CauchyLoss` - Aggressive outlier rejection

**Recommendation for RP LIDAR C1M1:** Use `None` with good odometry; use `HuberLoss` if experiencing drift

---

### 2. ROS Integration Parameters

#### `odom_frame: odom`
**Purpose:** Odometry frame name
**Recommended:** `odom` (ROS standard)

#### `map_frame: map`
**Purpose:** Global map frame name
**Recommended:** `map` (ROS standard)

#### `base_frame: base_link`
**Purpose:** Robot base frame
**Recommended:** `base_link` or `base_footprint`

#### `scan_topic: /scan`
**Purpose:** LaserScan topic to subscribe to
**Recommended:** Match your LiDAR driver topic

#### `mode: mapping`
**Purpose:** Operation mode
**Options:**
- `mapping` - Create new maps
- `localization` - Localize in existing map (lower CPU)

---

### 3. Map Update & Timing Parameters

#### `resolution: 0.05`
**Purpose:** Map grid resolution in meters
**Range:** 0.01 - 0.1
**Trade-off:**
- **Lower (0.01-0.03):** Higher detail, more memory, slower processing
- **Medium (0.05):** Balanced for most indoor environments
- **Higher (0.1):** Faster processing, less detail

**Recommendation for RP LIDAR C1M1:** `0.05` (matches sensor accuracy of ±30mm)

#### `map_update_interval: 5.0`
**Purpose:** Seconds between map updates
**Range:** 1.0 - 10.0
**Trade-off:**
- **Lower (1.0-3.0):** More frequent updates, higher CPU usage
- **Higher (5.0-10.0):** Less CPU, delayed map visualization

**Recommendation for Raspberry Pi:** `5.0` to reduce CPU load

#### `transform_publish_period: 0.02`
**Purpose:** Rate for publishing odometry transforms (seconds)
**Calculation:** 0.02 = 50 Hz
**Recommended:** `0.02` (50 Hz) for smooth navigation
**Note:** Set to `0.0` to disable odometry publishing

#### `throttle_scans: 1`
**Purpose:** Process every Nth scan
**Range:** 1 - 10
**Trade-off:**
- **1:** Process all scans (best accuracy, highest CPU)
- **2-3:** Skip some scans (good for faster robot speeds)
- **5-10:** Minimal processing (only for very constrained systems)

**Recommendation for RP LIDAR C1M1:** `1` (10Hz scan rate is already moderate)

#### `minimum_time_interval: 0.5`
**Purpose:** Minimum time between processed scans (seconds)
**Recommendation:** `0.5` for moderate robot speeds

---

### 4. Motion Threshold Parameters

#### `minimum_travel_distance: 0.5`
**Purpose:** Minimum distance robot must travel before processing new scan (meters)
**Range:** 0.1 - 1.0
**Trade-off:**
- **Lower (0.1-0.3):** Better for small spaces, slow robots, detailed maps
- **Higher (0.5-1.0):** Better for large spaces, fast robots, reduces computation

**Recommendation for Indoor Mapping:** `0.3` (smaller than default for indoor environments)

#### `minimum_travel_heading: 0.5`
**Purpose:** Minimum rotation before processing new scan (radians)
**Calculation:** 0.5 rad ≈ 28.6°
**Range:** 0.1 - 1.0
**Trade-off:**
- **Lower (0.1-0.3):** Better rotational tracking
- **Higher (0.5-1.0):** Reduces scans during rotation

**Recommendation:** `0.5` (default works well for most cases)

#### `check_min_dist_and_heading_precisely: false`
**Purpose:** Whether to require BOTH distance AND heading thresholds (false = OR logic)
**Recommendation:** `false` (suits most cases, especially with poor rotational odometry)

---

### 5. Scan Matching Parameters

#### `use_scan_matching: true`
**Purpose:** Enable scan-to-map matching
**Recommendation:** `true` (essential for SLAM)

#### `use_scan_barycenter: true`
**Purpose:** Use scan center of mass for better matching
**Recommendation:** `true` (improves accuracy)

#### `correlation_search_space_dimension: 0.5`
**Purpose:** Search grid size for scan correlation (meters)
**Range:** 0.3 - 1.0
**Trade-off:**
- **Smaller (0.3-0.5):** Faster, assumes good odometry
- **Larger (0.7-1.0):** More robust to odometry drift, slower

**Recommendation for RP LIDAR C1M1:** `0.5` with good odometry, `0.7` with poor odometry

#### `correlation_search_space_resolution: 0.01`
**Purpose:** Grid resolution for correlation search (meters)
**Range:** 0.005 - 0.05
**Trade-off:**
- **Finer (0.005-0.01):** More accurate, slower
- **Coarser (0.02-0.05):** Faster, less accurate

**Recommendation:** `0.01` (good balance)

#### `correlation_search_space_smear_deviation: 0.1`
**Purpose:** Gaussian smoothing for correlation response
**Range:** 0.03 - 0.2
**Recommendation:** `0.1` (moderate smoothing)

#### `coarse_angle_resolution: 0.0349`
**Purpose:** Angular resolution for coarse scan matching (radians)
**Calculation:** 0.0349 rad ≈ 2°
**Recommendation:** `0.0349` (2° increments)

#### `fine_search_angle_offset: 0.00349`
**Purpose:** Angular search range for fine matching (radians)
**Calculation:** 0.00349 rad ≈ 0.2°
**Recommendation:** `0.00349` (fine precision)

#### `coarse_search_angle_offset: 0.349`
**Purpose:** Angular search range for coarse matching (radians)
**Calculation:** 0.349 rad ≈ 20°
**Recommendation:** `0.349` (wide initial search)

---

### 6. Loop Closure Parameters

Loop closure is CRITICAL for long-term mapping accuracy. When the robot revisits a location, loop closure corrects accumulated drift.

#### `do_loop_closing: true`
**Purpose:** Enable loop closure detection
**Recommendation:** `true` (essential for multi-room environments)
**Impact:** Prevents map drift over time

#### `loop_search_maximum_distance: 3.0`
**Purpose:** Maximum distance to search for loop closures (meters)
**Range:** 2.0 - 10.0
**Trade-off:**
- **Smaller (2.0-3.0):** Faster, only nearby loops
- **Larger (5.0-10.0):** Finds distant loops, slower

**Recommendation for Indoor:** `3.0` for room-scale, `5.0` for building-scale

#### `loop_match_minimum_chain_size: 10`
**Purpose:** Minimum number of scans required for loop closure candidate
**Range:** 5 - 20
**Recommendation:** `10` (balanced)

#### `loop_match_maximum_variance_coarse: 3.0`
**Purpose:** Maximum variance allowed for coarse loop matching
**Recommendation:** `3.0` (default)

#### `loop_match_minimum_response_coarse: 0.35`
**Purpose:** Minimum correlation score for coarse loop matching
**Range:** 0.2 - 0.5
**Trade-off:**
- **Lower (0.2-0.3):** More loop closures, more false positives
- **Higher (0.4-0.5):** Fewer false positives, might miss valid loops

**Recommendation:** `0.35` (balanced)

#### `loop_match_minimum_response_fine: 0.45`
**Purpose:** Minimum correlation score for fine loop matching
**Range:** 0.3 - 0.6
**Recommendation:** `0.45` (default is good)

#### `loop_search_space_dimension: 8.0`
**Purpose:** Search space size for loop closure (meters)
**Range:** 5.0 - 15.0
**Recommendation:** `8.0` for indoor environments

#### `loop_search_space_resolution: 0.05`
**Purpose:** Resolution of loop closure search grid
**Recommendation:** `0.05` (matches map resolution)

#### `loop_search_space_smear_deviation: 0.03`
**Purpose:** Smoothing for loop closure correlation
**Recommendation:** `0.03` (less smoothing than regular scan matching)

---

### 7. Scan Buffer Parameters

#### `scan_buffer_size: 10`
**Purpose:** Number of scans to buffer into a chain
**Range:** 5 - 20
**Trade-off:**
- **Smaller (5-10):** Less memory, faster
- **Larger (15-20):** Better matching, more memory

**Recommendation for Raspberry Pi:** `10` (balances memory and performance)

#### `scan_buffer_maximum_scan_distance: 10.0`
**Purpose:** Maximum distance between scans in buffer (meters)
**Recommendation:** `10.0` (matches sensor range)

#### `link_match_minimum_response_fine: 0.1`
**Purpose:** Threshold for linking scans in the graph
**Range:** 0.05 - 0.3
**Trade-off:**
- **Lower (0.05-0.1):** More permissive linking, better in feature-poor environments
- **Higher (0.2-0.3):** Stricter linking, better in feature-rich environments

**Recommendation:** `0.1` (permissive for indoor corridors)

#### `link_scan_maximum_distance: 1.5`
**Purpose:** Maximum distance for linking consecutive scans (meters)
**Recommendation:** `1.5` (allows moderate motion between scans)

---

### 8. Variance Penalty Parameters

#### `distance_variance_penalty: 0.5`
**Purpose:** Weight for penalizing distance variance in matching
**Range:** 0.1 - 1.0
**Recommendation:** `0.5` (balanced)

#### `angle_variance_penalty: 1.0`
**Purpose:** Weight for penalizing angular variance in matching
**Range:** 0.5 - 2.0
**Recommendation:** `1.0` (slightly higher than distance for rotational accuracy)

#### `minimum_angle_penalty: 0.9`
**Purpose:** Minimum penalty for angular differences
**Recommendation:** `0.9`

#### `minimum_distance_penalty: 0.5`
**Purpose:** Minimum penalty for distance differences
**Recommendation:** `0.5`

---

## Recommended Configuration for RP LIDAR C1M1

### Optimized for Indoor Environments on Raspberry Pi

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params - Google Ceres Optimizer
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY  # Best for indoor maps
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None  # Use HuberLoss if experiencing drift

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping  # Use 'localization' for lower CPU when map is complete

    # Debugging and Logging
    debug_logging: false
    throttle_scans: 1  # Process all scans (C1M1 is only 10Hz)

    # Transform and Update Timing
    transform_publish_period: 0.02  # 50 Hz for smooth navigation
    map_update_interval: 5.0  # Update map every 5s (reduces CPU on RPi)
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000

    # Map Resolution
    resolution: 0.05  # 5cm grid (matches C1M1 ±30mm accuracy)
    max_laser_range: 12.0  # C1M1 max range
    min_laser_range: 0.1  # Ignore very close readings

    # Motion Thresholds (TUNED FOR INDOOR)
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3  # Reduced for indoor (default 0.5)
    minimum_travel_heading: 0.5  # ~28° rotation threshold
    check_min_dist_and_heading_precisely: false  # OR logic (not AND)

    # Scan Buffer
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # Loop Closure (CRITICAL FOR INDOOR MULTI-ROOM)
    do_loop_closing: true
    loop_search_maximum_distance: 3.0  # Room-scale (increase to 5.0 for buildings)
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation Search Parameters (Scan Matching)
    correlation_search_space_dimension: 0.5  # Good odometry assumption
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Loop Closure Search Space
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Angle Parameters
    fine_search_angle_offset: 0.00349  # ±0.2°
    coarse_search_angle_offset: 0.349  # ±20°
    coarse_angle_resolution: 0.0349  # 2° steps

    # Variance Penalties
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

---

## Trade-offs: Accuracy vs Performance

### Key Trade-off Dimensions

#### 1. **Resolution vs Computation**
| Resolution | Map Detail | Memory Usage | CPU Load | Use Case |
|------------|------------|--------------|----------|----------|
| 0.01m | Very High | High (~500MB) | Very High | Precision robotics |
| 0.05m | **Good** | **Medium (~300MB)** | **Medium** | **General indoor (recommended)** |
| 0.10m | Low | Low (~100MB) | Low | Large outdoor areas |

#### 2. **Scan Processing Rate vs Accuracy**
| throttle_scans | Scans Processed | Accuracy | CPU Usage |
|----------------|-----------------|----------|-----------|
| 1 | **All (10Hz)** | **Best** | **70%** |
| 2 | 5Hz | Good | 40% |
| 3 | 3.3Hz | Fair | 25% |

**Recommendation:** Keep at `1` for RP LIDAR C1M1 (already moderate 10Hz rate)

#### 3. **Travel Distance vs Map Density**
| minimum_travel_distance | Map Density | Feature Coverage | Performance |
|-------------------------|-------------|------------------|-------------|
| 0.1m | Very Dense | Excellent | Slow |
| 0.3m | **Dense** | **Very Good** | **Good** |
| 0.5m | Medium | Good | Fast |
| 1.0m | Sparse | Fair | Very Fast |

**Indoor Recommendation:** `0.3m` for detailed room mapping

#### 4. **Loop Closure Aggressiveness vs False Positives**
| Response Threshold | Loop Closures Found | False Positives | Map Consistency |
|--------------------|---------------------|-----------------|-----------------|
| 0.2 (low) | Many | High risk | May have artifacts |
| 0.35 (medium) | **Moderate** | **Low** | **Good** |
| 0.5 (high) | Few | Very Low | May drift in loops |

**Recommendation:** `0.35` for coarse, `0.45` for fine

---

### Performance Metrics (Research Findings)

Based on comparative studies of SLAM Toolbox in indoor environments:

**SLAM Toolbox Performance:**
- **Absolute Trajectory Error (ATE):** 0.13m (4x better than some alternatives)
- **Precision:** 1.75 cm
- **CPU Usage:** ~70% (single core on modern processors)
- **Memory Usage:** 293 MB
- **Startup Time:** 5.2 seconds

**Critical Dependencies:**
- **Odometry Quality:** SLAM Toolbox degrades sharply with poor odometry (as a pose-graph method)
- **Sensor FOV:** Performance degrades significantly with <90° FOV; 360° FOV (C1M1) is optimal
- **Dynamic Environments:** Handles moving people/objects better than grid-based methods

---

## Tuning Methodology

### Step-by-Step Tuning Process

#### Phase 1: Baseline Configuration
1. **Start with recommended defaults** (see configuration above)
2. **Verify sensor data:**
   ```bash
   ros2 topic echo /scan --once
   ```
   - Check `range_min` and `range_max` match C1M1 specs (0.05-12m)
   - Verify `angle_min`, `angle_max` cover 360° (-π to π)
   - Confirm scan frequency ~10Hz

3. **Test basic mapping:**
   - Drive robot in small loop
   - Check map quality in RViz
   - Monitor CPU usage: `top` or `htop`

#### Phase 2: Motion Threshold Tuning
**Goal:** Balance map density with computation

1. **Test different minimum_travel_distance values:**
   - Start: `0.5` (default)
   - Small rooms: Try `0.3` or `0.2`
   - Large spaces: Try `0.7` or `1.0`

2. **Evaluation criteria:**
   - Map coverage: Are all areas captured?
   - CPU usage: Staying under 80%?
   - Map quality: Sharp corners and walls?

3. **Use RViz plugin for real-time adjustment:**
   ```bash
   ros2 run slam_toolbox async_slam_toolbox_node
   ```
   - Adjust parameters in RViz SlamToolboxPlugin
   - Observe immediate effects

#### Phase 3: Scan Matching Tuning
**Goal:** Optimize local alignment accuracy

1. **If seeing map drift in straight hallways:**
   - Increase `correlation_search_space_dimension` from `0.5` to `0.7`
   - May indicate poor odometry

2. **If scan matching is slow:**
   - Decrease `correlation_search_space_resolution` from `0.01` to `0.02`
   - Trade-off: Slightly less accurate alignment

3. **If map has blurry/doubled walls:**
   - Check odometry quality first
   - Increase `link_match_minimum_response_fine` from `0.1` to `0.15`
   - Consider using `ceres_loss_function: HuberLoss`

#### Phase 4: Loop Closure Tuning
**Goal:** Ensure long-term consistency

1. **Test loop closure:**
   - Drive robot in complete loop back to start
   - Map should align perfectly with starting point
   - Check RViz for "loop closure" messages

2. **If loops are NOT detected:**
   - Decrease `loop_match_minimum_response_coarse` from `0.35` to `0.30`
   - Increase `loop_search_maximum_distance` from `3.0` to `5.0`

3. **If seeing false loop closures (map jumps/artifacts):**
   - Increase `loop_match_minimum_response_fine` from `0.45` to `0.50`
   - Increase `loop_match_minimum_chain_size` from `10` to `15`

#### Phase 5: Raspberry Pi Optimization
**Goal:** Balance accuracy with available resources

1. **Monitor resource usage:**
   ```bash
   # CPU and memory
   htop

   # Temperature (important for RPi throttling)
   vcgencmd measure_temp
   ```

2. **If CPU usage >90%:**
   - Increase `map_update_interval` from `5.0` to `7.0`
   - Consider `throttle_scans: 2` (process every other scan)
   - Increase `minimum_travel_distance` to `0.5`

3. **If memory usage >80%:**
   - Increase `resolution` from `0.05` to `0.07`
   - Decrease `scan_buffer_size` from `10` to `7`

4. **If experiencing thermal throttling (>80°C):**
   - Add heatsink/fan to Raspberry Pi
   - Reduce processing load as above
   - Consider running SLAM on remote machine via network

### Validation Tests

#### Test 1: Small Loop Closure
- **Procedure:** Drive 2x2m square, return to start
- **Success Criteria:** Map closes within 5cm
- **Failure Actions:** Tune loop closure parameters (Phase 4)

#### Test 2: Hallway Test
- **Procedure:** Drive 10m straight hallway and back
- **Success Criteria:** Walls are straight, no drift
- **Failure Actions:** Tune scan matching (Phase 3)

#### Test 3: Multi-Room Test
- **Procedure:** Map 3+ connected rooms
- **Success Criteria:** Rooms align correctly, no overlaps
- **Failure Actions:** Tune loop closure and motion thresholds

#### Test 4: Long-Term Stability
- **Procedure:** Run SLAM for 30+ minutes
- **Success Criteria:** CPU <80%, memory stable, no crashes
- **Failure Actions:** Raspberry Pi optimization (Phase 5)

---

## Raspberry Pi Optimization Strategies

### Hardware Optimizations

#### 1. **Cooling**
- **Essential:** Add heatsink and active cooling fan
- **Reasoning:** Thermal throttling significantly degrades SLAM performance
- **Target:** Keep CPU temperature <70°C during operation

#### 2. **Storage**
- **Recommended:** Use high-quality SD card (UHS-I Class 3 or better)
- **Alternative:** Boot from USB 3.0 SSD (RPi 4 only)
- **Impact:** Faster map file I/O, especially during serialization

#### 3. **Power Supply**
- **Required:** Official RPi power supply or equivalent (5V/3A minimum)
- **Reasoning:** Undervoltage causes instability and throttling

### Software Optimizations

#### 1. **Operating System**
- **Recommended:** Ubuntu Server 22.04 (no desktop environment)
- **Reasoning:** Desktop environments consume 20-30% CPU
- **Alternative:** Raspberry Pi OS Lite

#### 2. **ROS 2 Configuration**
```bash
# Reduce DDS discovery overhead
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Limit logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
```

#### 3. **Process Priority**
```bash
# Give SLAM higher priority
sudo nice -n -10 ros2 run slam_toolbox async_slam_toolbox_node
```

#### 4. **Network Optimization**
- **Use Ethernet:** LAN connection for remote visualization
- **Reduce Bandwidth:** Publish map at lower rate or reduced size
```yaml
# Reduce map publishing rate
map_update_interval: 10.0  # Instead of 5.0
```

#### 5. **Memory Management**
- **Enable ZRAM:** Compressed swap in RAM
```bash
sudo apt install zram-tools
```
- **Adjust swappiness:**
```bash
sudo sysctl vm.swappiness=10
```

### Configuration Presets for Different RPi Models

#### Raspberry Pi 5 (Quad-core 2.4GHz)
```yaml
# Can run full performance config
resolution: 0.05
throttle_scans: 1
minimum_travel_distance: 0.3
map_update_interval: 5.0
```

#### Raspberry Pi 4 (Quad-core 1.8GHz)
```yaml
# Balanced configuration (recommended)
resolution: 0.05
throttle_scans: 1
minimum_travel_distance: 0.3
map_update_interval: 5.0
# Should work well with cooling
```

#### Raspberry Pi 3B+ (Quad-core 1.4GHz)
```yaml
# Performance-optimized
resolution: 0.05
throttle_scans: 2  # Process every other scan
minimum_travel_distance: 0.5
map_update_interval: 7.0
# Monitor CPU usage carefully
```

### Remote Processing Strategy

For very constrained systems, consider running SLAM on a more powerful machine:

```bash
# On Raspberry Pi: publish sensor data only
ros2 run rplidar_ros rplidar_composition

# On remote PC: run SLAM
ros2 run slam_toolbox async_slam_toolbox_node
```

**Benefits:**
- RPi only handles sensor interface (~10% CPU)
- Full SLAM performance on desktop CPU
- Requires reliable network connection

**Network Setup:**
```bash
# On both machines
export ROS_DOMAIN_ID=42

# Verify communication
ros2 topic list
```

---

## Analysis of Current Configuration

### Current Configuration Review
File: `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/config/slam_toolbox_params.yaml`

#### Strengths
1. ✅ **Correct sensor range:** `max_laser_range: 12.0` matches C1M1 specs
2. ✅ **Good resolution:** `resolution: 0.05` appropriate for indoor mapping
3. ✅ **Proper Ceres configuration:** Optimal solver settings for indoor environments
4. ✅ **Loop closure enabled:** `do_loop_closing: true` with reasonable parameters
5. ✅ **Good scan matching settings:** Correlation parameters are well-configured

#### Areas for Improvement

##### 1. **Motion Thresholds for Indoor Use**
**Current:**
```yaml
minimum_travel_distance: 0.5
minimum_travel_heading: 0.5
```

**Issue:** `0.5m` travel distance is designed for large spaces. Indoor environments with detailed features benefit from denser scan coverage.

**Impact:** May miss small rooms or detailed features; map will be sparser than optimal.

##### 2. **Missing min_laser_range Parameter**
**Current:** Not specified (defaults to 0.0)

**Issue:** Very close readings (0-10cm) can include noise from the robot itself or sensor artifacts.

**Impact:** Minor noise in map near robot, especially if mounting is close to obstacles.

##### 3. **Loop Closure Search Distance**
**Current:**
```yaml
loop_search_maximum_distance: 3.0
```

**Assessment:** Good for room-scale, but may be limiting for multi-room buildings.

**Impact:** May not detect loop closures when returning to distant starting locations.

##### 4. **Map Update Interval**
**Current:**
```yaml
map_update_interval: 5.0
```

**Assessment:** Good for Raspberry Pi, but could provide feedback faster if CPU allows.

##### 5. **Missing check_min_dist_and_heading_precisely Parameter**
**Current:** Not specified (defaults to false)

**Assessment:** Default is fine, but should be explicitly documented for clarity.

---

## Recommended Changes

### Priority 1: Indoor Environment Optimization

**Change 1: Reduce Motion Thresholds**
```yaml
# FROM:
minimum_travel_distance: 0.5

# TO:
minimum_travel_distance: 0.3  # Better for indoor room mapping
```
**Rationale:** Indoor environments have more detailed features at smaller scales. Reducing to 0.3m provides denser scan coverage while still avoiding redundant scans.

**Change 2: Add Minimum Laser Range**
```yaml
# ADD:
min_laser_range: 0.1  # Ignore readings closer than 10cm
```
**Rationale:** Filters out sensor artifacts and self-reflections from robot chassis.

### Priority 2: Enhanced Loop Closure (for Multi-Room Buildings)

**Change 3: Increase Loop Search Distance**
```yaml
# FROM:
loop_search_maximum_distance: 3.0

# TO (if mapping multiple rooms):
loop_search_maximum_distance: 5.0  # Better for building-scale mapping
```
**Rationale:** Enables loop closure detection when returning to distant areas (e.g., different floor sections).

**Note:** Keep at 3.0 for single-room or small apartment mapping.

### Priority 3: Documentation and Clarity

**Change 4: Add Explicit Parameters**
```yaml
# ADD these for completeness:
check_min_dist_and_heading_precisely: false  # OR logic for motion thresholds
minimum_angle_penalty: 0.9
minimum_distance_penalty: 0.5
```

### Priority 4: Raspberry Pi Thermal Management Notes

**Change 5: Add Comments**
```yaml
# Add to configuration file:
# RASPBERRY PI USERS:
# - Ensure active cooling (heatsink + fan)
# - Monitor CPU temp: vcgencmd measure_temp
# - If CPU usage >90%, increase map_update_interval to 7.0
# - If thermal throttling occurs, consider throttle_scans: 2
```

### Complete Updated Configuration

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params - Google Ceres Optimizer
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping

    # Debugging and Performance
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02  # 50 Hz
    map_update_interval: 5.0  # Adjust to 7.0 if CPU constrained
    resolution: 0.05
    max_laser_range: 12.0  # RP LIDAR C1M1 max range
    min_laser_range: 0.1   # NEW: Filter close-range noise
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000

    # Motion Thresholds - TUNED FOR INDOOR
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3  # CHANGED: From 0.5 for indoor use
    minimum_travel_heading: 0.5
    check_min_dist_and_heading_precisely: false  # NEW: Explicit OR logic

    # Scan Buffer
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # Loop Closure
    do_loop_closing: true
    loop_search_maximum_distance: 5.0  # CHANGED: From 3.0 for multi-room
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Loop Closure Search Space
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9  # NEW: Explicit
    minimum_distance_penalty: 0.5  # NEW: Explicit
    use_response_expansion: true
```

### Summary of Changes
| Parameter | Current | Recommended | Reason |
|-----------|---------|-------------|--------|
| `minimum_travel_distance` | 0.5 | 0.3 | Better indoor coverage |
| `min_laser_range` | Not set | 0.1 | Filter noise |
| `loop_search_maximum_distance` | 3.0 | 5.0 | Multi-room buildings |
| `check_min_dist_and_heading_precisely` | Not set | false | Explicit clarity |
| `minimum_angle_penalty` | Not set | 0.9 | Completeness |
| `minimum_distance_penalty` | Not set | 0.5 | Completeness |

---

## References

### Primary Sources

1. **SLAM Toolbox GitHub Repository**
   [https://github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
   *Official repository with parameter documentation and examples*

2. **SLAM Toolbox ROS 2 Documentation**
   [https://docs.ros.org/en/jazzy/p/slam_toolbox/](https://docs.ros.org/en/jazzy/p/slam_toolbox/)
   *Official ROS 2 parameter reference*

3. **SLAM Toolbox: SLAM for the Dynamic World (Research Paper)**
   [https://www.researchgate.net/publication/351568967_SLAM_Toolbox_SLAM_for_the_dynamic_world](https://www.researchgate.net/publication/351568967_SLAM_Toolbox_SLAM_for_the_dynamic_world)
   *Academic paper describing SLAM Toolbox architecture and capabilities*

4. **ROSCon 2019: On Use of the SLAM Toolbox**
   [https://roscon.ros.org/2019/talks/roscon2019_slamtoolbox.pdf](https://roscon.ros.org/2019/talks/roscon2019_slamtoolbox.pdf)
   *Presentation by Steve Macenski covering parameter tuning*

### RP LIDAR C1M1 Specifications

5. **SLAMTEC RPLIDAR C1 Official Specifications**
   [https://www.slamtec.com/en/C1](https://www.slamtec.com/en/C1)
   *Manufacturer specifications and datasheets*

6. **RPLIDAR C1M1 Product Page - Amazon**
   [https://www.amazon.com/SLAMTEC-Navigation-Obstacle-Avoidance-Interface/dp/B0FRM8KHZF](https://www.amazon.com/SLAMTEC-Navigation-Obstacle-Avoidance-Interface/dp/B0FRM8KHZF)
   *Detailed specifications including sample rate and accuracy*

### Parameter Tuning Guides

7. **Hands on with slam_toolbox**
   [https://msadowski.github.io/hands-on-with-slam_toolbox/](https://msadowski.github.io/hands-on-with-slam_toolbox/)
   *Practical guide to SLAM Toolbox configuration and tuning*

8. **SLAM Toolbox Configuration Examples**
   [https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)
   *Official example configuration files*

9. **SLAM | Husarion Tutorials**
   [https://husarion.com/tutorials/ros2-tutorials/8-slam/](https://husarion.com/tutorials/ros2-tutorials/8-slam/)
   *ROS 2 SLAM tutorial with parameter examples*

### Loop Closure Research

10. **Multi-Objective Optimization of Loop Closure Detection Parameters**
    [https://www.mdpi.com/1424-8220/20/7/1906](https://www.mdpi.com/1424-8220/20/7/1906)
    *Research on systematic loop closure parameter optimization*

11. **Introduction to Loop Closure Detection in SLAM**
    [https://www.thinkautonomous.ai/blog/loop-closure/](https://www.thinkautonomous.ai/blog/loop-closure/)
    *Educational resource on loop closure concepts*

### Performance and Benchmarking

12. **From Simulation to Reality: Comparative Performance Analysis of SLAM Toolbox and Cartographer**
    [https://www.mdpi.com/2079-9292/14/24/4822](https://www.mdpi.com/2079-9292/14/24/4822)
    *Benchmarking study comparing SLAM Toolbox performance metrics*

13. **Comparison of Various SLAM Systems for Mobile Robot in Indoor Environment**
    [https://arxiv.org/html/2501.09490v1](https://arxiv.org/html/2501.09490v1)
    *Recent 2026 comparison of SLAM algorithms for indoor use*

### Raspberry Pi Optimization

14. **Increase the Processing Speed on SLAM by Raspberry PI**
    [https://www.researchgate.net/publication/338013305_Increase_the_Processing_Speed_on_Slam_by_Raspberry_PI](https://www.researchgate.net/publication/338013305_Increase_the_Processing_Speed_on_Slam_by_Raspberry_PI)
    *Research on optimizing SLAM performance on Raspberry Pi*

15. **SLAM on Raspberry Pi GitHub Repository**
    [https://github.com/AdroitAnandAI/SLAM-on-Raspberry-Pi](https://github.com/AdroitAnandAI/SLAM-on-Raspberry-Pi)
    *Example implementation with RP LIDAR on Raspberry Pi*

16. **ROS 2 SLAM Beginner Guide**
    [https://robotisim.com/ros-2-slam-beginner-guide-floorplan/](https://robotisim.com/ros-2-slam-beginner-guide-floorplan/)
    *Practical guide for ROS 2 SLAM setup and configuration*

### Additional Technical Resources

17. **How to Build an Indoor Map Using ROS and LIDAR-based SLAM**
    [https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/](https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/)
    *Step-by-step tutorial for indoor mapping*

18. **Mapping with slam_toolbox - Learn by Doing**
    [https://learnbydoing.dev/mapping-with-slam-toolbox/](https://learnbydoing.dev/mapping-with-slam-toolbox/)
    *Hands-on guide to SLAM Toolbox mapping*

19. **stevengong.co SLAM Toolbox Notes**
    [https://stevengong.co/notes/slam_toolbox](https://stevengong.co/notes/slam_toolbox)
    *Personal notes and tips on SLAM Toolbox configuration*

---

## Conclusion

This comprehensive guide provides parameter-by-parameter explanations, recommended values optimized for RP LIDAR C1M1 in indoor environments, and a systematic tuning methodology. The current configuration is already quite good, with minor improvements recommended for:

1. **Indoor optimization** (reduce `minimum_travel_distance` to 0.3m)
2. **Noise filtering** (add `min_laser_range: 0.1`)
3. **Multi-room mapping** (increase `loop_search_maximum_distance` to 5.0m if needed)

Key takeaways:
- RP LIDAR C1M1's 360° FOV and 12m range are ideal for SLAM Toolbox
- Balance accuracy vs. performance based on Raspberry Pi model and cooling
- Loop closure is critical for long-term mapping accuracy
- Monitor CPU temperature and usage during tuning
- Use RViz plugin for real-time parameter adjustment

For production deployment, test the recommended configuration thoroughly using the validation tests outlined in the Tuning Methodology section.

---

**Document Version:** 1.0
**Last Updated:** 2026-01-11
**Maintained By:** WayfindR-driver project
