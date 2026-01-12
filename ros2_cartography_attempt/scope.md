# ROS2 Cartography Attempt - Scope Document

**Last Updated:** 2026-01-11
**Status:** Functional - Successfully tested with Slamtec C1 LiDAR
**Platform:** Ubuntu 22.04.5 LTS, ROS2 Humble

---

## Purpose

This folder contains a complete implementation for **2D SLAM (Simultaneous Localization and Mapping)** using ROS2 Humble and the Slamtec C1M1RP LiDAR sensor. The implementation enables:

1. **Real-time mapping** of indoor environments using 2D laser scans
2. **Map persistence** for later localization and navigation
3. **Waypoint-based navigation** with programmatic waypoint management
4. **Automated workflows** via helper scripts for rapid deployment

This serves as the foundation for autonomous navigation in the WayfindR robot project, providing spatial awareness and path planning capabilities.

---

## Cartography Strategies & Algorithms

### Primary Strategy: Graph-Based SLAM

**Package:** `slam_toolbox` (async mode)
**Algorithm:** Karto SLAM with Ceres Solver optimization

#### How It Works

1. **Scan Matching**: Aligns consecutive laser scans to estimate robot motion
2. **Graph Construction**: Builds a pose graph where nodes are robot poses and edges are spatial constraints
3. **Loop Closure Detection**: Identifies when the robot revisits previously mapped areas
4. **Global Optimization**: Uses Ceres Solver to minimize pose graph errors and reduce drift
5. **Occupancy Grid Generation**: Converts pose graph and laser scans into 2D occupancy grid map

#### Key Algorithm Components

- **Solver**: Ceres Solver with Sparse Normal Cholesky linear solver
- **Trust Strategy**: Levenberg-Marquardt optimization
- **Scan Matching**: Correlative scan matching with configurable search space
- **Loop Closure**: Chain-based matching with 3-meter search radius
- **Map Representation**: Probabilistic occupancy grid (trinary mode: free/occupied/unknown)

#### Configuration Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `resolution` | 0.05 m/pixel | Map detail level (5cm cells) |
| `max_laser_range` | 12.0 m | Match C1 LiDAR indoor range |
| `minimum_travel_distance` | 0.5 m | Movement threshold for new scan |
| `minimum_travel_heading` | 0.5 rad | Rotation threshold (~28°) |
| `map_update_interval` | 5.0 sec | Map publish frequency |
| `do_loop_closing` | true | Enable drift correction |
| `loop_search_maximum_distance` | 3.0 m | Loop closure search radius |

### Map Output Format

**Occupancy Grid Map:**
- **Format**: PGM (Portable Gray Map) + YAML metadata
- **Resolution**: 5cm per pixel (configurable)
- **Color Encoding**:
  - White (254): Free navigable space
  - Black (0): Occupied by obstacles
  - Gray (205): Unknown/unexplored regions
- **Coordinate System**: Right-handed with origin at bottom-left corner

**Pose Graph Files:**
- `.posegraph`: Binary serialized pose graph for localization
- `.data`: Pose data for SLAM Toolbox state recovery

---

## Key Files and Their Purposes

### Launch Files

#### `rplidar_slam.launch.py`
**Purpose:** Master launch file for complete SLAM system
**Components Launched:**
- RPLidar node (configured for Slamtec C1 with DenseBoost mode)
- SLAM Toolbox async node
- Static TF transforms (base_link → laser, odom → base_link)
- RViz2 visualization

**Usage:**
```bash
ros2 launch rplidar_slam.launch.py serial_port:=/dev/rplidar
```

### Python Scripts

#### `waypoint_manager.py` (474 lines)
**Purpose:** Programmatic waypoint creation and management

**Features:**
- Parse map YAML files and extract metadata
- Calculate map dimensions, center, and world coordinates
- Add waypoints programmatically (center, corners, custom positions)
- Convert between yaw angles (degrees) and quaternions
- Export/import waypoints to YAML format
- Pixel-to-world and world-to-pixel coordinate conversion

**Classes:**
- `Waypoint`: Represents a navigation waypoint with position, orientation, tolerance
- `MapInfo`: Parses and stores map metadata (resolution, origin, dimensions)
- `WaypointManager`: High-level waypoint management interface

**Usage Examples:**
```bash
# Add center waypoint to map
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-center

# Add custom waypoint
python3 waypoint_manager.py --map-yaml maps/first_map.yaml \
    --add-waypoint "kitchen" 2.5 1.0 90

# List all waypoints
python3 waypoint_manager.py --waypoints maps/first_map_waypoints.yaml --list
```

#### `navigate_waypoints.py` (302 lines)
**Purpose:** Execute waypoint-based navigation using Nav2

**Features:**
- Load waypoints from YAML files
- Navigate to single waypoints or execute multi-waypoint routes
- Monitor navigation progress and provide feedback
- Support for looping routes (patrol mode)
- Dry-run mode for testing without ROS2/Nav2

**Navigation Modes:**
- Single waypoint navigation
- Sequential route execution
- Continuous loop mode
- Dry-run validation

**Dependencies:**
- `nav2_simple_commander` (BasicNavigator API)
- Running Nav2 navigation stack
- Active localization in existing map

**Usage Examples:**
```bash
# Navigate to single waypoint
python3 navigate_waypoints.py --waypoints waypoints.yaml --waypoint map_center

# Execute named route
python3 navigate_waypoints.py --waypoints waypoints.yaml --route patrol

# Navigate all waypoints in sequence
python3 navigate_waypoints.py --waypoints waypoints.yaml --all
```

### Configuration Files

#### `config/slam_toolbox_params.yaml`
**Purpose:** SLAM Toolbox configuration parameters

**Critical Settings:**
- Solver configuration (Ceres parameters)
- Frame IDs (map, odom, base_link)
- Scan matching thresholds
- Loop closure parameters
- Correlation search space settings

**Optimizations for C1 LiDAR:**
- `max_laser_range: 12.0` (matches C1 indoor range)
- `scan_mode: DenseBoost` (5 KHz sample rate)
- `resolution: 0.05` (balanced detail vs. performance)

### Maps Directory

#### `maps/first_map.pgm` (~30 KB)
**Format:** Grayscale PGM image
**Content:** 2D occupancy grid (212 x 144 pixels, 10.6 x 7.2 meters)

#### `maps/first_map.yaml` (127 bytes)
**Content:** Map metadata
```yaml
resolution: 0.05         # 5cm per pixel
origin: [-4.88, -4.09, 0]  # World coords of bottom-left
occupied_thresh: 0.65    # Occupied cell probability
free_thresh: 0.25        # Free cell probability
```

#### `maps/first_map_waypoints.yaml`
**Content:** 5 waypoints (center + 4 corners)
**Format:** Structured YAML with position, orientation, and tolerance

#### `maps/first_map_posegraph.posegraph` (~6.6 MB)
**Purpose:** Serialized pose graph for localization mode
**Usage:** Load in SLAM Toolbox for localization-only operation

### Helper Scripts (scripts/)

#### `start_all.sh`
**Purpose:** Launch all SLAM components in separate terminal windows
**Components:** TF publisher, LiDAR node, SLAM Toolbox, RViz2
**Terminal Support:** gnome-terminal, xterm, tmux fallback

#### `start_tf.sh`
**Purpose:** Publish static TF transforms
**Transforms:**
- `map → odom` (SLAM Toolbox publishes this dynamically)
- `odom → base_link` (static for testing; wheel odometry in production)
- `base_link → laser` (LiDAR mounting position: 0.1m above base)

#### `start_lidar.sh`
**Purpose:** Start RPLidar node
**Configuration:**
- Serial port: `/dev/rplidar`
- Baud rate: 460800
- Scan mode: DenseBoost (5 KHz sample rate)
- Frame ID: `laser`

#### `start_slam.sh`
**Purpose:** Start SLAM Toolbox with custom parameters
**Pre-checks:** Verifies `/scan` topic exists before launching

#### `start_rviz.sh`
**Purpose:** Launch RViz2 with custom configuration (if available)

#### `save_map.sh`
**Purpose:** Save current map using `nav2_map_server`
**Output:** PGM image + YAML metadata
**Usage:** `./save_map.sh my_map_name`

#### `check_status.sh`
**Purpose:** Diagnostic script to verify system state
**Checks:**
- LiDAR device presence (`/dev/rplidar`)
- Active ROS2 nodes
- Topic publication rates (`/scan`, `/map`, `/tf`)
- TF frame connectivity

### Documentation Files

#### `README.md`
**Content:** Quick start guide, map specifications, workflow overview
**Audience:** First-time users setting up SLAM

#### `WAYPOINT_WORKFLOW.md` (9.4 KB)
**Content:** Complete waypoint creation and navigation tutorial
**Topics:**
- Waypoint file format explanation
- Coordinate system primer
- Orientation (quaternion) conversion
- Python script usage examples
- Best practices for waypoint placement

#### `VIEW_MAP_INSTRUCTIONS.md`
**Content:** Guide for visualizing saved maps in RViz
**Methods:** Map server approach, static visualization, direct PGM viewing

#### `docs/TESTING_CARTOGRAPHY.md`
**Content:** Step-by-step testing guide with checklists
**Stages:**
1. LiDAR connection verification
2. Scan data validation
3. TF tree inspection
4. Map building verification
5. Map saving procedure

#### `docs/SLAM_MAPPING_FINDINGS.md`
**Content:** Technical deep-dive into SLAM concepts
**Topics:**
- SLAM theory and principles
- SLAM Toolbox architecture
- Map file formats
- ROS2 topic/service reference
- Performance benchmarks
- Known limitations

#### `docs/LIDAR_SETUP.md`
**Content:** Hardware setup guide for Slamtec C1 LiDAR
**Topics:**
- Hardware specifications
- USB/udev configuration
- ROS2 driver installation
- LaserScan message format
- Troubleshooting common issues

---

## Current State of Implementation

### Completed Features

- **2D Mapping**: Fully functional SLAM-based mapping
- **Map Persistence**: Save/load maps in standard ROS2 format
- **Waypoint Management**: Programmatic waypoint creation with geometric utilities
- **Launch Infrastructure**: Modular launch files for flexible deployment
- **Helper Scripts**: Complete automation toolkit for common operations
- **Visualization**: RViz2 integration with pre-configured displays
- **Documentation**: Comprehensive guides for setup, testing, and operation

### Tested Configurations

**Hardware:**
- Slamtec C1M1RP LiDAR (RPLidar C1)
- USB connection (CP210x UART Bridge)
- DenseBoost scan mode (5 KHz, 10 Hz scan rate)

**Software:**
- Ubuntu 22.04.5 LTS
- ROS2 Humble
- SLAM Toolbox (async mode)
- Nav2 Map Server

**Test Results:**
- Successfully created 2D occupancy grid map
- Map dimensions: 212 x 144 pixels (10.6 x 7.2 meters)
- Resolution: 5cm per pixel
- 5 waypoints created and validated
- LiDAR operating at 10 Hz scan rate

### Known Limitations

**From Testing:**
1. **Stationary Mapping**: Test map was created with stationary LiDAR (single viewpoint)
2. **No Odometry**: Uses static TF transforms instead of wheel encoders
3. **Single Pose Graph**: Limited pose graph without robot movement
4. **No Obstacle Validation**: Waypoint scripts don't verify free space

**General Limitations:**
1. **2D Only**: Cannot map multi-floor environments or vertical features
2. **Static Environment Assumption**: Moving objects cause map artifacts
3. **Drift Accumulation**: Errors grow without loop closure
4. **Indoor Focus**: LiDAR optimized for 12m indoor range

### Not Yet Implemented

1. **Full Navigation Stack**: Nav2 autonomous navigation integration
2. **Wheel Odometry**: Integration with robot encoders/IMU
3. **Localization Mode**: SLAM Toolbox localization-only operation
4. **Route Planning**: Automated path planning between waypoints
5. **Fleet Management**: Multi-robot coordination
6. **3D Mapping**: Elevation/stairs detection
7. **Waypoint Visualization**: Overlay waypoints on map images
8. **Obstacle Detection**: Runtime collision avoidance

---

## Dependencies and Requirements

### Hardware Requirements

- **LiDAR**: Slamtec C1M1RP or compatible RPLidar (A1, A2, A3, S1)
- **Computer**: x86_64 PC running Linux (tested on Ubuntu 22.04)
- **USB Port**: For LiDAR connection (CP210x driver required)
- **Optional**: Mobile robot platform with wheel encoders

### Software Dependencies

#### ROS2 Packages (Required)

```bash
# ROS2 Humble base
ros-humble-desktop

# LiDAR driver
ros-humble-rplidar-ros

# SLAM package
ros-humble-slam-toolbox

# Navigation stack (for waypoint navigation)
ros-humble-navigation2
ros-humble-nav2-bringup

# Utilities
ros-humble-tf2-tools
ros-humble-rviz2
```

#### Python Packages

```bash
# Core ROS2 Python
rclpy
geometry-msgs
sensor-msgs
nav-msgs

# Navigation (optional, for navigate_waypoints.py)
nav2-simple-commander

# Standard library (no installation needed)
yaml
math
argparse
```

#### System Utilities

```bash
# Terminal multiplexer (optional)
tmux

# Image viewer (optional, for map visualization)
imagemagick
eog (Eye of GNOME)
```

### udev Rules (LiDAR)

File: `/etc/udev/rules.d/99-rplidar.rules`
```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
```

### Permissions

```bash
# Option 1: Add user to dialout group
sudo usermod -a -G dialout $USER
# (requires logout/login)

# Option 2: Set device permissions
sudo chmod 666 /dev/ttyUSB0
```

---

## Coordinate Systems and Transforms

### TF Tree Structure

```
map (global frame)
 └── odom (odometry frame)
      └── base_link (robot body center)
           └── laser (LiDAR sensor)
```

### Transform Publishers

| Transform | Publisher | Type | Values |
|-----------|-----------|------|--------|
| `map → odom` | slam_toolbox | Dynamic | Corrected pose |
| `odom → base_link` | wheel_odometry | Dynamic | Dead reckoning |
| `base_link → laser` | static_tf_publisher | Static | (0, 0, 0.1) |

**Note:** In test setup, `odom → base_link` is static (0,0,0). In production, this comes from wheel encoders.

### Coordinate Frame Conventions

- **X-axis**: Forward (robot front)
- **Y-axis**: Left (robot left side)
- **Z-axis**: Up (vertical)
- **Rotation**: Right-hand rule (counter-clockwise positive)

---

## Performance Characteristics

**Tested on Intel Desktop (specs not specified):**
- SLAM update rate: ~2 Hz
- Map publish rate: 0.2 Hz (every 5 seconds)
- CPU usage: 10-15%
- Memory usage: 200-500 MB (depends on map size)
- LiDAR scan rate: 10 Hz
- LiDAR sample rate: 5 KHz (DenseBoost mode)

**Map File Sizes:**
- PGM image: ~30 KB (for 10.6 x 7.2m map)
- YAML metadata: ~127 bytes
- Pose graph data: ~19 KB
- Full pose graph: ~6.6 MB

---

## Integration with WayfindR Project

This cartography implementation provides the **spatial awareness foundation** for the WayfindR autonomous navigation robot. It enables:

1. **Environment Mapping**: Create persistent 2D maps of operating environments
2. **Localization**: Determine robot position within known maps
3. **Path Planning**: Navigate between waypoints autonomously (via Nav2)
4. **Obstacle Awareness**: Detect static obstacles via occupancy grid

### Next Steps for Integration

1. Mount LiDAR on WayfindR robot chassis
2. Integrate wheel odometry from robot encoders
3. Configure Nav2 for autonomous navigation
4. Implement higher-level navigation logic (patrol routes, charging station return)
5. Add dynamic obstacle avoidance
6. Develop fleet management for multi-robot coordination

---

## Quick Reference Commands

```bash
# Start complete SLAM system
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./start_all.sh

# Check system status
./check_status.sh

# Save current map
./save_map.sh my_environment

# Add waypoints to map
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-center --add-corners

# List waypoints
python3 waypoint_manager.py --waypoints maps/first_map_waypoints.yaml --list

# Navigate to waypoint (requires Nav2 running)
python3 navigate_waypoints.py --waypoints maps/first_map_waypoints.yaml --waypoint map_center

# Dry-run navigation (test without ROS2)
python3 navigate_waypoints.py --waypoints maps/first_map_waypoints.yaml --all --dry-run
```

---

## References and Resources

**ROS2 Documentation:**
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [RPLidar ROS2 Package](https://github.com/Slamtec/rplidar_ros)

**LiDAR Specifications:**
- [Slamtec C1 Product Page](https://www.slamtec.com/en/Lidar/C1)

**ROS2 Concepts:**
- [Understanding TF2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Occupancy Grid Maps](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)
- [LaserScan Messages](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)

---

**Document Created:** 2026-01-11
**Project:** WayfindR Autonomous Robot
**Maintainer:** Development Team
