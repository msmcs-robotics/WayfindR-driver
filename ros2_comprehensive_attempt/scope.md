# ROS2 Comprehensive Attempt - Project Scope

## Purpose

This folder contains a **complete, self-contained ROS2 Humble navigation system** designed for Ubuntu 22.04 with Python 3.10. It provides a drag-and-drop solution for 2D LiDAR-based SLAM mapping, localization, and autonomous navigation without requiring a full ROS2 workspace setup.

The system is designed to be:
- **Portable**: Can be copied to any Ubuntu 22.04 system and run immediately after dependency installation
- **Self-documenting**: Includes comprehensive configuration files, launch files, and helper scripts
- **Beginner-friendly**: Provides bash scripts for common operations instead of requiring ROS2 command knowledge
- **Production-ready**: Includes advanced features like A* pathfinding, waypoint management, and route execution

---

## ROS2 Features & Packages Integrated

### Core ROS2 Packages

1. **SLAM Toolbox** (`ros-humble-slam-toolbox`)
   - Async SLAM for real-time map building
   - Loop closure detection for drift correction
   - Pose graph optimization using Ceres solver
   - Map serialization for later localization

2. **Navigation2 (Nav2)** (`ros-humble-navigation2`)
   - Complete navigation stack
   - Path planning and execution
   - Costmap generation
   - Recovery behaviors

3. **AMCL (Adaptive Monte Carlo Localization)** (`ros-humble-nav2-amcl`)
   - Particle filter-based localization
   - Probabilistic position estimation on known maps
   - Configurable motion and sensor models

4. **Map Server** (`ros-humble-nav2-map-server`)
   - Loads and serves saved maps
   - PGM image format support
   - YAML metadata handling

5. **Lifecycle Manager** (`ros-humble-nav2-lifecycle-manager`)
   - Manages node lifecycles
   - Coordinated startup/shutdown
   - Automatic recovery

6. **TF2** (`ros-humble-tf2-ros`, `ros-humble-tf2-tools`)
   - Transform tree management
   - Static and dynamic transforms
   - Coordinate frame relationships (map → odom → base_link → laser)

7. **RPLidar Driver** (`ros-humble-rplidar-ros`)
   - Slamtec RPLidar support (C1, A1, A2, A3)
   - Configurable scan modes (Standard, DenseBoost, Sensitivity)
   - Serial communication over USB

### Custom Python Components

1. **Pathfinder** (`scripts/pathfinder.py`)
   - A* pathfinding algorithm
   - PGM map parsing
   - Obstacle inflation for safety margins
   - Path simplification using Ramer-Douglas-Peucker algorithm
   - Grid-to-world coordinate conversion

2. **Waypoint Manager** (`scripts/waypoint_manager.py`)
   - Create/edit/save waypoints programmatically
   - Map coordinate calculations
   - Quaternion/Euler angle conversions
   - Auto-generation of center and corner waypoints
   - Route definitions (sequences of waypoints)

3. **Navigator** (`scripts/navigator.py`)
   - Integration with Nav2 Simple Commander API
   - Waypoint-based navigation
   - Route execution
   - Dry-run mode for testing without hardware
   - Position monitoring

---

## Key Files and Their Purposes

### Configuration Files (`config/`)

| File | Purpose |
|------|---------|
| `slam_params.yaml` | SLAM Toolbox configuration: map resolution (0.05m), laser range (12m), loop closure settings, scan matching parameters |
| `amcl_params.yaml` | AMCL localization settings: particle filter (500-2000 particles), motion model (differential drive), laser likelihood field model |
| `lidar_params.yaml` | RPLidar driver configuration: serial port (/dev/ttyUSB0), baud rate (460800), scan mode (DenseBoost), frame ID (laser) |

### Launch Files (`launch/`)

| File | Purpose |
|------|---------|
| `slam.launch.py` | Launches SLAM mapping: RPLidar driver, SLAM Toolbox, static TF publishers (odom→base_link→laser), optional RViz |
| `localization.launch.py` | Launches localization: RPLidar driver, map server, AMCL, lifecycle manager, static TFs, optional RViz |

### Python Scripts (`scripts/`)

| File | Lines | Purpose |
|------|-------|---------|
| `pathfinder.py` | 433 | A* pathfinding on occupancy grids, PGM loading, path simplification, waypoint-to-waypoint planning |
| `waypoint_manager.py` | 474 | Waypoint creation/management, map info parsing, coordinate conversions, YAML serialization |
| `navigator.py` | 292 | ROS2 navigation interface, Nav2 integration, route execution, position monitoring, dry-run testing |

### Bash Helper Scripts (`scripts/`)

| File | Purpose |
|------|---------|
| `install_dependencies.sh` | Installs ROS2 Humble Desktop, SLAM Toolbox, Nav2, RPLidar driver, Python dependencies |
| `setup_lidar.sh` | Creates udev rules for RPLidar, adds user to dialout group, sets up /dev/rplidar symlink |
| `start_mapping.sh` | Launches SLAM mapping session with configurable serial port and RViz |
| `start_localization.sh` | Starts AMCL localization with a saved map |
| `save_map.sh` | Saves current SLAM map to PGM/YAML format plus pose graph |

### Documentation (`docs/`)

| File | Purpose |
|------|---------|
| `QUICKSTART.md` | 10-minute getting started guide with step-by-step setup |
| `TROUBLESHOOTING.md` | Common issues and solutions for LiDAR, SLAM, localization, pathfinding, and RViz |

### Data Directories

| Directory | Purpose |
|-----------|---------|
| `maps/` | Storage for saved maps (YAML metadata + PGM images) - currently empty |
| `waypoints/` | Storage for waypoint definition files - currently empty |

---

## Current State of Implementation

### Completed Components

1. **SLAM Mapping Pipeline**
   - ✓ RPLidar driver integration
   - ✓ SLAM Toolbox configuration optimized for indoor 2D mapping
   - ✓ TF tree setup (map → odom → base_link → laser)
   - ✓ Map saving functionality (PGM + YAML + pose graph)
   - ✓ Launch file with configurable parameters

2. **Localization Pipeline**
   - ✓ AMCL configuration with differential drive motion model
   - ✓ Map server integration
   - ✓ Lifecycle management for coordinated node startup
   - ✓ Initial pose estimation support (via RViz 2D Pose Estimate)
   - ✓ Launch file with map parameter

3. **Waypoint System**
   - ✓ Waypoint data structure with position + orientation
   - ✓ YAML-based waypoint storage
   - ✓ Map coordinate system integration
   - ✓ Auto-generation utilities (center, corners)
   - ✓ Route definitions (named sequences)

4. **Pathfinding**
   - ✓ A* implementation on occupancy grids
   - ✓ PGM map parser
   - ✓ Obstacle inflation for safety
   - ✓ Path simplification algorithm
   - ✓ Cost calculation (Euclidean distance heuristic)
   - ✓ 8-connected grid (diagonal movement allowed)

5. **Navigation Interface**
   - ✓ Nav2 Simple Commander integration
   - ✓ Waypoint-based goal setting
   - ✓ Route execution with sequential waypoints
   - ✓ Dry-run mode (no ROS2 required)
   - ✓ Command-line interface

6. **Documentation**
   - ✓ README with overview and examples
   - ✓ Quick start guide
   - ✓ Comprehensive troubleshooting guide
   - ✓ Inline code comments
   - ✓ Configuration file documentation

### Missing/Incomplete Components

1. **RViz Configuration**
   - ✗ No `rviz_config.rviz` file exists (referenced but not created)
   - Launch files check for it but fall back to default RViz config

2. **Full Nav2 Integration**
   - ✗ No `navigation.launch.py` file (mentioned in README but not implemented)
   - ✗ No `nav2_params.yaml` file (mentioned in README but not implemented)
   - Would provide full Nav2 stack with costmaps, planners, controllers

3. **Sample Data**
   - ✗ No example maps in `maps/` directory
   - ✗ No example waypoints in `waypoints/` directory

4. **Additional Launch Files**
   - ✗ No `lidar.launch.py` (LiDAR-only mode mentioned in README)

5. **Advanced Documentation**
   - ✗ No `CONFIGURATION.md` (mentioned in README)
   - ✗ No `API_REFERENCE.md` (mentioned in README)

### Implementation Status: **~85% Complete**

The core functionality is **fully operational**:
- Can build maps using SLAM
- Can localize on saved maps using AMCL
- Can define and manage waypoints
- Can plan collision-free paths
- Can navigate to waypoints (requires Nav2 Simple Commander)

Missing components are primarily:
- Advanced Nav2 features (full navigation stack)
- Visual configuration (RViz config file)
- Sample/demo data
- Extended documentation

---

## Dependencies and Requirements

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Architecture**: amd64 (x86_64) or arm64
- **ROS2 Distribution**: Humble Hawksbill
- **Python Version**: 3.10.x (Ubuntu 22.04 default)

### Hardware Requirements

- **LiDAR**: 2D laser scanner (tested with Slamtec C1/RPLidar)
  - USB connection (creates /dev/ttyUSB* device)
  - Silicon Labs CP210x UART Bridge chipset
  - Range: 0.1-12m (indoor) recommended
  - Scan rate: 5-10 Hz

- **Computer**:
  - CPU: 2+ cores recommended for SLAM
  - RAM: 4GB minimum, 8GB recommended
  - Storage: 500MB for ROS2 + packages

### ROS2 Package Dependencies

```bash
# Core ROS2
ros-humble-desktop              # Includes RViz2, rqt tools
ros-dev-tools                   # Build tools
python3-colcon-common-extensions

# SLAM & Navigation
ros-humble-slam-toolbox         # Async SLAM, loop closure
ros-humble-navigation2          # Nav2 stack
ros-humble-nav2-bringup         # Nav2 launch files
ros-humble-nav2-map-server      # Map loading/serving
ros-humble-nav2-amcl            # Localization
ros-humble-nav2-lifecycle-manager

# Transforms & Tools
ros-humble-robot-localization   # EKF, UKF filters
ros-humble-tf2-ros              # Transform library
ros-humble-tf2-tools            # TF debugging (view_frames)

# LiDAR
ros-humble-rplidar-ros          # RPLidar driver
ros-humble-laser-filters        # Scan filtering
```

### Python Dependencies

```bash
pyyaml    # YAML parsing for configs/waypoints
numpy     # Numerical operations (optional, for advanced path processing)
```

### Optional Dependencies

- **Nav2 Simple Commander**: For programmatic navigation (Python API)
  - Typically included with `ros-humble-navigation2`
  - Used by `navigator.py` for goal execution

---

## Workflow Overview

### Phase 1: Map Building
1. Run `install_dependencies.sh` to install ROS2 and packages
2. Run `setup_lidar.sh` to configure LiDAR permissions
3. Run `start_mapping.sh` to launch SLAM
4. Move LiDAR around environment to scan
5. Run `save_map.sh <name>` to save map to `maps/`

### Phase 2: Waypoint Definition
1. Use `waypoint_manager.py --map maps/<name>.yaml --add-waypoint ...`
2. Add multiple waypoints at key locations
3. Optionally define routes as sequences of waypoints
4. Save to `waypoints/<name>.yaml`

### Phase 3: Navigation
1. Run `start_localization.sh maps/<name>.yaml` to load map
2. Set initial pose in RViz using "2D Pose Estimate" tool
3. Run `navigator.py --waypoints waypoints/<name>.yaml --goto <waypoint>`
4. Robot navigates autonomously to goal

---

## Technical Architecture

### Transform Tree (TF)
```
map
 └─ odom (published by AMCL)
     └─ base_link (static, identity transform)
         └─ laser (static, +0.1m Z offset)
```

### ROS2 Topics
- `/scan` - LiDAR scan data (sensor_msgs/LaserScan)
- `/map` - Occupancy grid map (nav_msgs/OccupancyGrid)
- `/map_metadata` - Map metadata
- `/particle_cloud` - AMCL particle visualization
- `/amcl_pose` - Estimated robot pose

### ROS2 Services
- `/slam_toolbox/save_map` - Save current map
- `/slam_toolbox/serialize_map` - Save pose graph
- Various lifecycle services (configure, activate, cleanup, shutdown)

### Coordinate Systems
- **Map Frame**: Fixed world frame, origin at map bottom-left
- **Odom Frame**: Odometry frame (continuous, may drift)
- **Base Link Frame**: Robot center
- **Laser Frame**: LiDAR sensor position (0.1m above base)

### File Formats
- **Maps**: PGM (Portable GrayMap) + YAML metadata
  - PGM: 8-bit grayscale, 0=occupied, 254=free, 205=unknown
  - YAML: resolution, origin, thresholds
- **Waypoints**: Custom YAML format
  - Position: x, y, z in meters
  - Orientation: quaternion (x, y, z, w)
  - Additional: name, description, tolerances

---

## Design Philosophy

1. **Minimal Configuration**: Works out-of-box with sensible defaults
2. **No Workspace Required**: Operates independently, no `colcon build` needed
3. **Script-Driven**: Bash scripts hide ROS2 complexity from users
4. **Portable**: Self-contained, can be copied between systems
5. **Educational**: Well-documented code for learning ROS2 concepts
6. **Production-Ready**: Includes error handling, logging, recovery

---

## Version Information

- **Version**: 1.0.0
- **ROS2 Distribution**: Humble Hawksbill
- **Target Platform**: Ubuntu 22.04 LTS
- **Python Version**: 3.10.x
- **License**: MIT (as stated in README)
- **Creation Date**: December 2024 (based on file timestamps)

---

## Future Enhancements (Potential)

1. Full Nav2 stack integration with costmap configuration
2. Dynamic obstacle avoidance
3. Multi-robot support
4. Web-based waypoint editor/visualizer
5. Automatic map merging for multi-floor buildings
6. Integration with robot base motor controllers
7. GPS waypoint fusion for outdoor navigation
8. REST API for remote control
9. RViz plugin for waypoint management
10. Docker containerization for easier deployment

---

## Notes

- This system assumes a **stationary robot** (no wheel odometry) for SLAM/localization
  - Static TF from odom → base_link (identity transform)
  - Suitable for handheld LiDAR mapping or static sensor platforms
  - For mobile robots, add odometry publishers and update TF tree

- **No Nav2 velocity commands**: Currently missing motor controller integration
  - Pathfinding and waypoint management work standalone
  - Full autonomous navigation requires Nav2 with cmd_vel output connected to robot base

- Designed for **indoor environments** with 2D planar navigation
  - Not suitable for 3D environments, stairs, or rough terrain
  - LiDAR must be mounted horizontally

- **Development Status**: Functional prototype, suitable for learning and prototyping
  - Not tested in production environments
  - May require tuning for specific LiDAR models or environments
