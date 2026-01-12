# ROS2 Localization Attempt - Scope Document

## Purpose

This folder contains a complete implementation of ROS2-based robot localization using pre-built maps and LiDAR sensor data. The system determines the robot's position on an existing map and provides waypoint-based navigation capabilities with A* pathfinding.

**Primary Goal:** Answer the question "Where am I on this map?" using sensor fusion and probabilistic localization.

**Status:** ✅ Tested and working on remote system (devel@192.168.0.7)

---

## Localization Strategies & Algorithms

### 1. AMCL (Adaptive Monte Carlo Localization) - PRIMARY METHOD

**Algorithm Type:** Particle Filter-based Probabilistic Localization

**How It Works:**
- Maintains 500-2000 "particles" representing possible robot poses
- Each particle is a hypothesis of robot position (x, y, theta)
- Motion model spreads particles based on expected movement + noise
- Sensor model compares LiDAR scans to expected readings at each particle location
- Resampling: High-likelihood particles are duplicated, low-likelihood particles die
- Over time, particles converge around the true robot position

**Key Parameters:**
- `min_particles`: 500
- `max_particles`: 2000
- `laser_max_range`: 12.0m (Slamtec C1M1RP LiDAR)
- `laser_model_type`: likelihood_field (best for indoor environments)
- `robot_model_type`: DifferentialMotionModel
- `update_min_d`: 0.1m (minimum movement before update)
- `update_min_a`: 0.2 rad (minimum rotation before update)

**Pros:**
- Robust to ambiguity and symmetrical environments
- Can recover from "kidnapped robot" problem
- Handles multi-modal distributions (multiple possible positions)
- Well-tested in Nav2 ecosystem

**Cons:**
- Computationally intensive with many particles
- Requires good initial pose estimate for fast convergence
- Performance degrades in featureless environments

**ROS2 Package:** `nav2_amcl`

### 2. SLAM Toolbox Localization Mode - ALTERNATIVE (Configured but not primary)

**Algorithm Type:** Graph-based SLAM in localization-only mode

**How It Works:**
- Uses saved pose graph from previous SLAM session
- Performs scan matching against known map
- Updates robot pose based on correlation between current and expected scans
- Does not update the map (localization only, not mapping)

**Key Parameters:**
- `mode`: localization
- `solver_plugin`: CeresSolver (non-linear optimization)
- `use_scan_matching`: true
- `minimum_travel_distance`: 0.2m
- `do_loop_closing`: false (disabled for pure localization)

**Pros:**
- Smooth pose updates
- Can use pose graph information
- Lower computational overhead than AMCL in some cases

**Cons:**
- Less robust to large initial pose errors
- Requires .posegraph file from SLAM Toolbox
- May not handle kidnapped robot scenario as well

**ROS2 Package:** `slam_toolbox`

### 3. A* Pathfinding Algorithm

**Purpose:** Plan collision-free paths between waypoints on occupancy grid

**Algorithm Type:** Heuristic search with cost function

**How It Works:**
1. Converts world coordinates to grid indices
2. Maintains open set (frontier) and closed set (visited)
3. Expands nodes with lowest f-score: f(n) = g(n) + h(n)
   - g(n): Actual cost from start to node n
   - h(n): Heuristic estimate from n to goal (Euclidean distance)
4. Uses 8-connected grid (cardinal + diagonal movement)
5. Movement costs: 1.0 for cardinal, 1.414 (√2) for diagonal
6. Avoids obstacles with inflation radius (2 cells margin)
7. Simplifies path using Ramer-Douglas-Peucker algorithm

**Obstacle Detection:**
- Reads PGM map: 0 = occupied, 205 = unknown, 254 = free
- Treats values < 100 as obstacles
- Applies 2-pixel inflation radius for safety margin

**Path Simplification:**
- Reduces 80+ waypoints to 2-4 key turning points
- Uses perpendicular distance tolerance of 0.15m

---

## Key Files and Their Purposes

### Configuration Files

| File | Purpose |
|------|---------|
| `config/amcl_params.yaml` | AMCL node configuration: particle filter settings, laser model parameters, motion model, transform settings, recovery behavior |
| `config/slam_localization_params.yaml` | SLAM Toolbox localization mode configuration (alternative to AMCL) |
| `config/localization_view.rviz` | RViz visualization config showing map, LiDAR scans, particle cloud, transforms |

### Launch Files

| File | Purpose |
|------|---------|
| `launch/localization.launch.py` | AMCL-only launch (requires external LiDAR driver) - includes map server, AMCL, lifecycle manager, static transforms, RViz |
| `launch/localization_with_lidar.launch.py` | Complete launch with RPLidar driver + AMCL + Map Server + RViz - ready-to-run full stack |

### Python Scripts

| File | Purpose | Key Features |
|------|---------|--------------|
| `scripts/monitor_pose.py` | Real-time AMCL pose monitoring | Subscribes to `/amcl_pose`, displays position/orientation/uncertainty, quality rating (★★★), movement tracking |
| `scripts/set_initial_pose.py` | Programmatic initial pose setting | Publishes to `/initialpose` topic, converts Euler to quaternion, alternative to RViz "2D Pose Estimate" tool |
| `scripts/simple_pathfinder.py` | A* pathfinding implementation | Loads PGM maps, world↔grid coordinate conversion, obstacle detection with inflation, path simplification (RDP), CLI for testing |
| `scripts/localization_with_waypoints.py` | Waypoint-aware pose monitor | Combines AMCL pose with waypoint distances, calculates bearings, shows direction arrows (→↗↑), plans paths to nearest waypoint |
| `scripts/navigate_to_office.py` | Waypoint navigation demo | Plans paths to named waypoints, supports route execution, dry-run mode, integrates localization + pathfinding |
| `scripts/add_office_waypoints.py` | Waypoint file generator | Analyzes map to find free space, creates 3 office waypoints, defines routes (tour, reverse, patrol), saves YAML format |
| `scripts/start_localization.sh` | Convenience launcher script | Bash script to start full localization stack with error checking |

### Documentation

| File | Purpose |
|------|---------|
| `README.md` | Quick start guide, usage instructions, file overview, troubleshooting |
| `docs/LOCALIZATION_FINDINGS.md` | Technical deep-dive: AMCL algorithm, transform tree, lifecycle nodes, performance metrics |
| `docs/WAYPOINT_NAVIGATION.md` | Waypoint system documentation: file format, A* algorithm, coordinate transformations, usage examples |

---

## Current State of Implementation

### ✅ Completed & Tested

1. **AMCL Localization:**
   - Fully configured and tuned for Slamtec C1M1RP LiDAR
   - Particle filter working with 500-2000 particles
   - Reliable pose estimation in tested environment
   - Transform tree properly configured (map→odom→base_link→laser)

2. **Sensor Integration:**
   - RPLidar driver integrated (460800 baud, DenseBoost scan mode)
   - Static transforms for testing without odometry
   - LiDAR frame correctly aligned with base_link

3. **Map Server:**
   - Loads pre-built maps (YAML + PGM format)
   - Publishes occupancy grid on `/map` topic
   - Lifecycle management with auto-start

4. **Waypoint System:**
   - YAML-based waypoint storage
   - Automatic waypoint placement in free space
   - Named routes (office_tour, office_patrol, etc.)
   - Position and orientation tolerance specification

5. **Pathfinding:**
   - A* algorithm with 8-connected grid
   - Obstacle inflation for safety
   - Path simplification (80+ points → 2-4 points)
   - World↔grid coordinate conversion

6. **Monitoring Tools:**
   - Real-time pose quality assessment
   - Distance/bearing to waypoints
   - Direction indicators (arrows)
   - Covariance-based uncertainty display

7. **Visualization:**
   - RViz integration with particle cloud
   - Map overlay
   - Laser scan display
   - Transform tree visualization

### 🚧 Partially Implemented

1. **Route Execution:**
   - Planning complete
   - Visualization available
   - Actual robot motion control not implemented (dry-run only)
   - Needs Nav2 controller integration

2. **Odometry Integration:**
   - Currently using static transforms for testing
   - Real wheel odometry not connected
   - Motion model configured but not receiving real odometry data

3. **SLAM Toolbox Localization:**
   - Configuration file present
   - Not actively used (AMCL is primary)
   - Would need pose graph file to test

### ❌ Not Implemented

1. **Nav2 Controller:**
   - No actual robot motion commands
   - DWB/TEB local planner not configured
   - No velocity command publishing

2. **Global/Local Costmaps:**
   - No dynamic obstacle avoidance
   - Static map only
   - No inflation layer for navigation

3. **Recovery Behaviors:**
   - No stuck detection
   - No rotation recovery
   - No backup maneuvers

4. **Multi-floor Support:**
   - Single map only
   - No elevator waypoints
   - No map switching

5. **Pose Persistence:**
   - No pose saving between runs
   - Must set initial pose each session
   - No automatic relocalization

---

## Dependencies & Requirements

### Hardware Requirements

- **LiDAR:** Slamtec RPLidar C1M1RP
  - Serial connection: `/dev/rplidar` or `/dev/ttyUSB0`
  - Baud rate: 460800
  - Max range: 12 meters
  - Scan mode: DenseBoost

- **Robot Platform:** Differential drive (configured for differential motion model)

- **Computer:** Tested on Ubuntu 22.04 desktop
  - CPU: ~5-10% per core for AMCL
  - Memory: ~50-100 MB for localization stack
  - Update rate: ~10 Hz

### Software Requirements

**ROS2 Distribution:** Humble Hawksbill

**Required ROS2 Packages:**
```bash
ros-humble-nav2-amcl          # AMCL localization
ros-humble-nav2-map-server    # Map loading
ros-humble-nav2-lifecycle-manager  # Node lifecycle management
ros-humble-rplidar-ros        # RPLidar driver
ros-humble-tf2-ros            # Transform library
ros-humble-rviz2              # Visualization
ros-humble-slam-toolbox       # Alternative localization (optional)
```

**Python Dependencies:**
```python
rclpy                         # ROS2 Python client
geometry_msgs                 # Pose messages
nav_msgs                      # Map, path messages
sensor_msgs                   # LaserScan messages
pyyaml                        # YAML file parsing
```

**System Requirements:**
- Ubuntu 22.04 LTS
- Python 3.10+
- ROS2 Humble installed (`/opt/ros/humble`)

### Map Requirements

- **Format:** YAML metadata + PGM image
- **Required Files:**
  - `*.yaml` - Map metadata (resolution, origin, occupied thresholds)
  - `*.pgm` - Grayscale occupancy grid image

- **Map Properties:**
  - Resolution: Typically 0.05 m/pixel (5 cm)
  - Origin: (x, y, theta) offset of lower-left pixel
  - Free space: Pixel value 254
  - Occupied: Pixel value 0
  - Unknown: Pixel value 205

- **Default Map Location:** `/home/devel/ros2_ws/maps/first_map.yaml`

### Network Requirements

- **Tested Environment:** Local network (192.168.0.x)
- **Remote System:** devel@192.168.0.7
- **RViz Display:** Requires X11 display or SSH X-forwarding

---

## Transform Tree (TF2)

The localization system requires this transform hierarchy:

```
map (world reference frame)
 └── odom (odometry frame - drift over time)
      └── base_link (robot center)
           └── laser (LiDAR sensor)
```

**Transform Sources:**
- `map → odom`: Published by AMCL (corrects odometry drift)
- `odom → base_link`: Should come from wheel encoders (currently static for testing)
- `base_link → laser`: Static transform (LiDAR mounting position: 0, 0, 0.1m)

---

## ROS2 Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | ~10 Hz | LiDAR data input |
| `/map` | nav_msgs/OccupancyGrid | Latched | Pre-built map |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | ~10 Hz | Localization output |
| `/particlecloud` | nav2_msgs/ParticleCloud | ~1 Hz | Particle distribution |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | On-demand | Set initial pose estimate |
| `/tf` | tf2_msgs/TFMessage | ~30 Hz | Transform tree |
| `/tf_static` | tf2_msgs/TFMessage | Latched | Static transforms |

---

## Usage Workflow

1. **Ensure map exists:** `~/ros2_ws/maps/first_map.yaml`
2. **Start localization:** `~/start_localization.sh`
3. **Set initial pose:** Use RViz "2D Pose Estimate" tool or run `set_initial_pose.py`
4. **Monitor convergence:** Watch particle cloud in RViz or run `monitor_pose.py`
5. **Add waypoints (optional):** `python3 add_office_waypoints.py`
6. **Plan navigation (optional):** `python3 navigate_to_office.py office1 --dry-run`

---

## Testing Status

**Tested Environment:**
- System: Ubuntu 22.04, ROS2 Humble
- LiDAR: Slamtec C1M1RP at `/dev/rplidar`
- Location: Remote system at 192.168.0.7
- Date: December 21-22, 2025

**Verified Functionality:**
- ✅ LiDAR data acquisition
- ✅ Map loading and visualization
- ✅ AMCL particle filter convergence
- ✅ Pose estimation with <10cm accuracy
- ✅ Transform tree publication
- ✅ Waypoint pathfinding (A*)
- ✅ Distance/bearing calculation
- ✅ Path simplification

**Known Limitations:**
- Requires manual initial pose setting each session
- No actual robot motion (dry-run only)
- Static odometry (no wheel encoder integration)
- Single map only (no multi-floor support)

---

## Next Steps for Full Navigation

To complete the navigation pipeline, the following would be needed:

1. **Integrate Nav2 Controller:**
   - Configure DWB or TEB local planner
   - Set up global/local costmap parameters
   - Implement velocity command publisher

2. **Add Real Odometry:**
   - Connect wheel encoders
   - Publish odom→base_link transform
   - Tune motion model parameters

3. **Implement Safety:**
   - Emergency stop on obstacle detection
   - Velocity limits
   - Watchdog timers

4. **Enhance Waypoint System:**
   - Pose saving/loading
   - Dynamic waypoint editing
   - Route optimization

5. **Add Recovery Behaviors:**
   - Rotation in place for relocalization
   - Backward movement when stuck
   - Global relocalization after kidnapping

---

**Document Created:** 2026-01-11
**Last Updated:** 2026-01-11
**Maintained By:** WayfindR Project
