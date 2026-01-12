# Map Server & AMCL Quick Reference Guide
**Generated:** 2026-01-11

## Quick Start Commands

### Launch Map Server
```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml
```

### Activate Map Server
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### Launch AMCL
```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_amcl amcl --ros-args \
  --params-file /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml
```

### Activate AMCL
```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

## Full Stack Launch (One-liner)
```bash
# Terminal 1: Map Server
source /opt/ros/humble/setup.bash && \
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml

# Terminal 2: AMCL
source /opt/ros/humble/setup.bash && \
ros2 run nav2_amcl amcl --ros-args \
  --params-file /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml

# Terminal 3: Activate both
source /opt/ros/humble/setup.bash && \
ros2 lifecycle set /map_server configure && \
ros2 lifecycle set /map_server activate && \
ros2 lifecycle set /amcl configure && \
ros2 lifecycle set /amcl activate
```

## Useful Commands

### Check Lifecycle States
```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

### List Topics
```bash
ros2 topic list | grep -E "map|amcl|particle|pose"
```

### Monitor Map Publication
```bash
ros2 topic echo /map --once | head -50
```

### Monitor AMCL Pose
```bash
ros2 topic echo /amcl_pose
```

### Monitor Particle Cloud
```bash
ros2 topic echo /particle_cloud
```

### Check Parameters
```bash
ros2 param list /amcl
ros2 param get /amcl laser_max_range
```

### Topic Information
```bash
ros2 topic info /map
ros2 topic info /particle_cloud
ros2 topic info /amcl_pose
```

## File Locations

### Maps
- **Directory:** `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/`
- **Map File:** `first_map.yaml`
- **Map Image:** `first_map.pgm`
- **Waypoints:** `first_map_waypoints.yaml`

### Configurations
- **AMCL Config 1:** `/home/devel/Desktop/WayfindR-driver/ros2_localization_attempt/config/amcl_params.yaml`
- **AMCL Config 2 (Recommended):** `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml`

### Documentation
- **Full Test Report:** `/home/devel/Desktop/WayfindR-driver/findings/2026-01-11-map-server-test.md`
- **Test Scripts:** `/home/devel/Desktop/WayfindR-driver/findings/map-server-test-scripts.sh`
- **This Guide:** `/home/devel/Desktop/WayfindR-driver/findings/map-server-quick-reference.md`

## Test Scripts Usage

### Interactive Menu
```bash
/home/devel/Desktop/WayfindR-driver/findings/map-server-test-scripts.sh
```

### Run Specific Test
```bash
# Test map server lifecycle
./map-server-test-scripts.sh map-lifecycle

# Test AMCL basic
./map-server-test-scripts.sh amcl-basic

# Test full stack
./map-server-test-scripts.sh full

# Run all tests
./map-server-test-scripts.sh all
```

## Key Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/map` | nav_msgs/OccupancyGrid | map_server | Static map data |
| `/map_metadata` | nav_msgs/MapMetaData | map_server | Map metadata |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | amcl | Robot pose estimate |
| `/particle_cloud` | nav2_msgs/ParticleCloud | amcl | Particle filter visualization |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | (user) | Initial pose input |
| `/scan` | sensor_msgs/LaserScan | (lidar) | LiDAR scan data |

## Map Specifications

**first_map.yaml:**
- Resolution: 0.05m/pixel (5cm)
- Size: 212 x 144 pixels
- Physical: ~10.6m x 7.2m
- Origin: (-4.88, -4.09, 0)
- Format: Trinary (free/occupied/unknown)

## AMCL Key Parameters

**Frame Configuration:**
- global_frame_id: `map`
- odom_frame_id: `odom`
- base_frame_id: `base_link`
- scan_topic: `/scan`

**Particle Filter:**
- min_particles: `500`
- max_particles: `2000`

**Laser Model:**
- laser_max_range: `12.0` meters (RPLiDAR C1)
- laser_min_range: `0.1` meters
- max_beams: `60`
- laser_model_type: `likelihood_field`

**Motion Model:**
- robot_model_type: `nav2_amcl::DifferentialMotionModel`

## Troubleshooting

### Map Server Won't Configure
**Issue:** "Unknown transition requested"
**Solution:** Check state with `ros2 lifecycle get /map_server` - may already be configured

### AMCL Not Publishing Pose
**Issue:** No output on `/amcl_pose`
**Solution:** AMCL needs `/scan` data from LiDAR. This is expected without hardware.

### Multiple Node Warnings
**Issue:** "Be aware that there are nodes that share an exact name"
**Solution:** Normal in development. Check with `ros2 node list` to see all nodes.

### Can't Find Map File
**Issue:** "Failed to load map"
**Solution:** Use absolute path: `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml`

## Status Summary

| Component | Status | Notes |
|-----------|--------|-------|
| Map Server | ✓ Working | Loads and serves maps correctly |
| Map Files | ✓ Available | first_map validated and ready |
| AMCL Launch | ✓ Working | Starts and configures correctly |
| AMCL Config | ✓ Valid | Two configurations available |
| Integration | ✓ Working | Map server → AMCL communication OK |
| Localization | ⚠ Pending | Requires LiDAR hardware |

## Next Steps

1. **Connect LiDAR Hardware**
   - RPLiDAR C1 to USB port
   - Launch LiDAR driver
   - Verify `/scan` topic

2. **Test Localization**
   - Set initial pose via RViz or `/initialpose`
   - Drive robot to generate odometry
   - Verify AMCL pose estimates

3. **Create New Maps (if needed)**
   - Use SLAM Toolbox: `ros2 launch rplidar_slam.launch.py`
   - Drive around area to map
   - Save map for future use

4. **Tune Parameters**
   - Adjust particle count based on performance
   - Tune laser model for scan matching
   - Optimize update thresholds

## References

- [Nav2 Map Server Documentation](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
- [Nav2 AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [ROS2 Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- Full Test Report: `2026-01-11-map-server-test.md`
