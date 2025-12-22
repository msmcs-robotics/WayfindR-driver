# SLAM Mapping Findings

## Overview

This document captures findings from implementing SLAM-based 2D mapping using SLAM Toolbox on ROS2 Humble with a Slamtec C1M1RP LiDAR.

**Test System:** Ubuntu 22.04, ROS2 Humble
**LiDAR:** Slamtec C1M1RP (RPLidar C1)
**SLAM Package:** slam_toolbox (async mode)

---

## What is SLAM?

**SLAM** = Simultaneous Localization And Mapping

The robot simultaneously:
1. **Builds a map** of the environment
2. **Localizes itself** within that map

This solves the chicken-and-egg problem: you need a map to know where you are, but you need to know where you are to build a map.

---

## SLAM Toolbox

### Why SLAM Toolbox?

- Designed specifically for ROS2
- Efficient graph-based SLAM
- Supports real-time operation
- Can save/load pose graphs
- Loop closure detection
- Works with 2D LiDAR

### Operating Modes

| Mode | Purpose |
|------|---------|
| `mapping` | Build new map from scratch |
| `localization` | Localize in existing map (no map updates) |
| `lifelong` | Continuous mapping with global optimization |

---

## Map Output Files

SLAM Toolbox produces four files:

| File | Size | Purpose |
|------|------|---------|
| `map.pgm` | ~30 KB | Grayscale occupancy grid image |
| `map.yaml` | ~127 B | Map metadata (resolution, origin) |
| `map_posegraph.data` | ~19 KB | Pose data for localization |
| `map_posegraph.posegraph` | ~6.6 MB | Full pose graph |

### Occupancy Grid (PGM)

```
Pixel Value   Meaning           Color
-----------   -------           -----
0             Occupied          Black
205           Unknown           Gray
254           Free              White
```

### Map YAML Format

```yaml
image: first_map.pgm
mode: trinary
resolution: 0.05          # meters per pixel
origin: [-4.88, -4.09, 0] # world coords of bottom-left corner
negate: 0
occupied_thresh: 0.65     # probability > this = occupied
free_thresh: 0.25         # probability < this = free
```

---

## LiDAR Configuration

### Slamtec C1M1RP Specifications

| Parameter | Value |
|-----------|-------|
| Max Range | 12-40 m (mode dependent) |
| Scan Frequency | 10 Hz |
| Sample Rate | 5 KHz (DenseBoost mode) |
| Baud Rate | 460800 |
| Interface | USB (CP210x UART) |

### Scan Modes

| Mode | Range | Sample Rate | Best For |
|------|-------|-------------|----------|
| Standard | 12m | 4 KHz | General use |
| DenseBoost | 12m | 5 KHz | Indoor mapping |
| Sensitivity | 40m | 2 KHz | Long range |

We use **DenseBoost** for indoor mapping.

---

## SLAM Toolbox Parameters

### Critical Parameters

```yaml
# Resolution - map detail level
resolution: 0.05  # 5cm per pixel (good balance)

# LiDAR range - match your sensor
max_laser_range: 12.0  # Slamtec C1 indoor range

# Movement thresholds - when to add new scans
minimum_travel_distance: 0.5  # meters
minimum_travel_heading: 0.5   # radians (~28 degrees)

# Map updates
map_update_interval: 5.0  # seconds between map publishes
```

### Loop Closure Settings

```yaml
do_loop_closing: true
loop_search_maximum_distance: 3.0
loop_match_minimum_chain_size: 10
```

Loop closure corrects accumulated drift when the robot returns to a previously visited area.

### Solver Settings (Ceres)

```yaml
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
ceres_trust_strategy: LEVENBERG_MARQUARDT
```

Ceres Solver optimizes the pose graph for global consistency.

---

## Transform Tree

SLAM Toolbox requires and produces:

```
map (world frame)
 └── odom (odometry frame)
      └── base_link (robot body)
           └── laser (LiDAR sensor)
```

### Who Publishes What

| Transform | Publisher |
|-----------|-----------|
| map → odom | SLAM Toolbox |
| odom → base_link | Wheel odometry or static |
| base_link → laser | Static (robot geometry) |

---

## Mapping Best Practices

### For Good Maps

1. **Move slowly**: 0.2-0.3 m/s maximum
2. **Rotate gradually**: Smooth turns, avoid spinning
3. **Cover all areas**: Don't leave large unmapped regions
4. **Close loops**: Return to starting point
5. **Avoid dynamic objects**: Wait for people to move

### Common Problems

| Problem | Cause | Solution |
|---------|-------|----------|
| Ghosting | Multiple occupancy | Move slower, better odometry |
| Drift | No loop closure | Return to start, revisit areas |
| Missing walls | LiDAR occluded | Map from multiple angles |
| Noisy map | Poor scan matching | Better odometry, slower motion |

---

## ROS2 Topics

### Input Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/scan` | sensor_msgs/LaserScan | LiDAR data |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

### Output Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/map` | nav_msgs/OccupancyGrid | Built map |
| `/map_metadata` | nav_msgs/MapMetaData | Map info |
| `/pose` | geometry_msgs/PoseStamped | Current pose |

### Service Calls

```bash
# Save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/path/to/map'}}"

# Serialize pose graph
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/path/to/map'}"
```

---

## Helper Scripts

Located at `~/scripts/` on remote system:

| Script | Purpose |
|--------|---------|
| `start_mapping.sh` | Launch SLAM + LiDAR + RViz |
| `save_map.sh <name>` | Save current map |
| `view_map.sh <name>` | View saved map in RViz |
| `test_lidar.sh` | Test LiDAR only |

---

## Performance

On Intel desktop:
- SLAM update rate: ~2 Hz
- Map publish rate: 0.2 Hz (every 5 seconds)
- CPU usage: 10-15%
- Memory: 200-500 MB (depends on map size)

---

## Known Limitations

1. **2D Only**: Creates flat occupancy grids, not 3D
2. **Static Environment**: Assumes walls don't move
3. **Requires Motion**: Need to move robot to build map
4. **Drift Accumulates**: Without loop closure, errors grow

---

## Files Created

| File | Purpose |
|------|---------|
| `config/slam_toolbox_params.yaml` | SLAM configuration |
| `config/slam_view.rviz` | RViz display settings |
| `rplidar_slam.launch.py` | Launch file for mapping |
| `maps/first_map.*` | Generated map files |

---

## Next Steps After Mapping

1. **Verify map quality** in RViz
2. **Save map** with pose graph
3. **Switch to localization mode**
4. **Add waypoints** for navigation
5. **Start Nav2** for autonomous navigation

---

**Last Updated:** 2025-12-22
