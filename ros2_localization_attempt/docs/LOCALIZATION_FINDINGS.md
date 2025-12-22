# Localization System Findings

## Overview

This document captures findings from implementing AMCL-based localization with waypoint navigation on ROS2 Humble.

**Test System:** Ubuntu 22.04, ROS2 Humble, Slamtec C1M1RP LiDAR
**Remote Host:** devel@192.168.0.7

---

## What is Localization?

Localization answers: **"Where am I on this map?"**

Given a pre-built map and current LiDAR scans, the system estimates:
- Robot position (x, y) in meters
- Robot orientation (yaw/theta) in radians
- Confidence/uncertainty of the estimate

---

## AMCL (Adaptive Monte Carlo Localization)

### How It Works

1. **Particle Filter**: Maintains thousands of "particles" - each representing a possible robot pose
2. **Motion Model**: When robot moves, particles spread based on expected motion + noise
3. **Sensor Model**: LiDAR scans are compared to expected readings at each particle's position
4. **Resampling**: Particles with high likelihood are duplicated; low likelihood particles die
5. **Convergence**: Over time, particles cluster around the true robot position

### Key Parameters (amcl_params.yaml)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `min_particles` | 500 | Minimum particle count |
| `max_particles` | 2000 | Maximum particle count |
| `laser_max_range` | 12.0 | Slamtec C1 max range |
| `laser_model_type` | likelihood_field | Best for indoor environments |
| `odom_model_type` | diff-corrected | For differential drive robots |
| `update_min_d` | 0.1 | Min movement before update (meters) |
| `update_min_a` | 0.2 | Min rotation before update (radians) |

### Tuning Tips

- **More particles** = more accurate but slower
- **Fewer particles** = faster but may lose track
- **recovery_alpha_slow/fast** = controls when to add random particles (kidnapped robot)

---

## Transform Tree (TF2)

AMCL requires a proper transform tree:

```
map
 └── odom
      └── base_link
           └── laser
```

### Transforms Needed

| From | To | Source |
|------|-----|--------|
| `map` → `odom` | Published by AMCL | Corrects odometry drift |
| `odom` → `base_link` | Wheel encoders or static | Robot's local odometry |
| `base_link` → `laser` | Static | LiDAR mounting position |

For stationary testing (no wheel encoders), we use static transforms:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser
```

---

## Lifecycle Nodes

Nav2 uses lifecycle-managed nodes for controlled startup:

1. **Unconfigured** → `configure()` → **Inactive**
2. **Inactive** → `activate()` → **Active**
3. **Active** → `deactivate()` → **Inactive**

The `lifecycle_manager_localization` handles this automatically:
```yaml
node_names: ['map_server', 'amcl']
autostart: true
```

---

## Integration with Waypoints

### Waypoint Storage Format

Waypoints are stored in YAML:
```yaml
waypoints:
- name: office1
  position: {x: -1.58, y: -0.49, z: 0.0}
  orientation: {x: 0, y: 0, z: 0, w: 1}
  yaw_degrees: 0
  tolerance: {position: 0.3, orientation: 0.2}
```

### A* Pathfinding

The `simple_pathfinder.py` implements A* on the occupancy grid:

1. Load map image (PGM) as 2D grid
2. Convert world coordinates to grid indices
3. Run A* with 8-connected neighbors
4. Simplify path using Ramer-Douglas-Peucker algorithm
5. Convert back to world coordinates

**Key insight**: Map pixels < 100 are treated as obstacles (0 = occupied, 205 = unknown, 254 = free).

---

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LiDAR data input |
| `/map` | nav_msgs/OccupancyGrid | Loaded map |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | Current pose estimate |
| `/particlecloud` | nav2_msgs/ParticleCloud | Particle distribution |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | Set initial pose |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

---

## Common Issues & Solutions

### Issue: "Transform from odom to base_link was unavailable"
**Solution**: Add static transform publisher for odom→base_link

### Issue: Particles scattered across entire map
**Solution**: Set initial pose using RViz "2D Pose Estimate" tool or `/initialpose` topic

### Issue: Robot position jumps erratically
**Causes**:
- Map doesn't match environment (furniture moved)
- LiDAR data quality issues
- Too few particles

### Issue: RViz won't start over SSH
**Solution**: Use physical display or X11 forwarding (`ssh -X`)

---

## Performance Metrics

On test system (Intel desktop):
- AMCL update rate: ~10 Hz
- Particle count: 500-2000
- CPU usage: ~5-10% per core
- Memory: ~50-100 MB

---

## Files Created

| File | Purpose |
|------|---------|
| `config/amcl_params.yaml` | AMCL configuration |
| `launch/localization_with_lidar.launch.py` | Full launch file |
| `scripts/monitor_pose.py` | Pose quality monitor |
| `scripts/localization_with_waypoints.py` | Waypoint-aware monitor |
| `scripts/navigate_to_office.py` | Pathfinding demo |
| `scripts/simple_pathfinder.py` | A* implementation |

---

## Next Steps

1. Integrate with Nav2 controller for actual robot motion
2. Add wheel odometry for better motion model
3. Implement global/local costmaps for dynamic obstacle avoidance
4. Add waypoint persistence and editing UI

---

**Last Updated:** 2025-12-22
