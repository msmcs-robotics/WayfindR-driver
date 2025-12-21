# ROS2 Cartography Attempt

**Date:** 2025-12-20
**Remote System:** devel@192.168.0.7 (Ubuntu 22.04.5 LTS Desktop)
**LiDAR:** Slamtec C1M1RP (RPLidar C1)
**SLAM Package:** slam_toolbox (async mode)

---

## Summary

Successfully created a 2D occupancy grid map using the Slamtec C1 LiDAR and SLAM Toolbox on ROS2 Humble. The LiDAR was stationary during this test, capturing the immediate environment.

---

## Map Details

### Output Files

| File | Size | Description |
|------|------|-------------|
| `first_map.pgm` | 30 KB | Occupancy grid image (grayscale) |
| `first_map.yaml` | 127 B | Map metadata |
| `first_map_posegraph.data` | 19 KB | SLAM Toolbox pose data |
| `first_map_posegraph.posegraph` | 6.6 MB | Full pose graph for localization |

### Map Specifications

```yaml
image: first_map.pgm
mode: trinary
resolution: 0.05          # 5cm per pixel
origin: [-4.88, -4.09, 0] # Map origin in meters
negate: 0
occupied_thresh: 0.65     # Probability threshold for occupied
free_thresh: 0.25         # Probability threshold for free
```

### Map Dimensions
- **Width:** 212 pixels (10.6 meters)
- **Height:** 144 pixels (7.2 meters)
- **Resolution:** 0.05 m/pixel (5 cm)

---

## How to View the Map in RViz

### Option 1: View Saved Map (Recommended)

On the remote machine (192.168.0.7), open a terminal and run:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Start the map server with the saved map
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=/home/devel/ros2_ws/maps/first_map.yaml &

# Wait for it to start
sleep 2

# Bring up the lifecycle node
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p node_names:=['map_server'] \
    -p autostart:=true &

# Or simpler - just configure and activate manually:
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# Start RViz
rviz2
```

In RViz:
1. Click **Add** in the Displays panel
2. Select **By topic** tab
3. Find `/map` and select **Map**
4. Click **OK**
5. Set **Fixed Frame** to `map` in Global Options

### Option 2: Use the Helper Script

```bash
~/scripts/view_map.sh first_map
```

### Option 3: View While Mapping Live

```bash
~/scripts/start_mapping.sh
```

This opens RViz automatically and shows:
- Live laser scan (colorful dots)
- Map building in real-time
- TF frames

---

## RViz Configuration Tips

### Display Settings for Map Viewing

| Display | Topic | Settings |
|---------|-------|----------|
| Map | `/map` | Color scheme: map, Alpha: 0.7 |
| LaserScan | `/scan` | Style: Points, Size: 0.03m |
| TF | - | Show names: true |
| Grid | - | Cell size: 1m |

### Recommended View
- **View Type:** TopDownOrtho (top-down 2D view)
- **Fixed Frame:** map
- **Background:** Dark (48, 48, 48)

---

## LiDAR Performance

### Slamtec C1 Specifications (Observed)

| Parameter | Value |
|-----------|-------|
| Serial Number | C117E0F6C0E292CDB5E099F044C7400A |
| Firmware | 1.02 |
| Hardware Rev | 18 |
| Scan Mode | DenseBoost |
| Sample Rate | 5 KHz |
| Max Range | 40 m |
| Scan Frequency | 10 Hz |
| Baud Rate | 460800 |

### Topics Published

| Topic | Type | Rate |
|-------|------|------|
| `/scan` | sensor_msgs/LaserScan | 10 Hz |
| `/map` | nav_msgs/OccupancyGrid | ~0.2 Hz |
| `/pose` | geometry_msgs/PoseStamped | 10 Hz |
| `/tf` | tf2_msgs/TFMessage | 10 Hz |

---

## SLAM Toolbox Configuration

Key parameters used (from `slam_toolbox_params.yaml`):

```yaml
# Core settings
mode: mapping
resolution: 0.05              # 5cm map resolution
max_laser_range: 12.0         # Indoor-optimized range

# Loop closure
do_loop_closing: true
loop_search_maximum_distance: 3.0

# Update rates
map_update_interval: 5.0      # Seconds between map updates
transform_publish_period: 0.02 # 50 Hz TF updates

# Quality settings
use_scan_matching: true
minimum_travel_distance: 0.5  # Meters before new scan
minimum_travel_heading: 0.5   # Radians before new scan
```

---

## Known Limitations

### This Test Session

1. **Stationary LiDAR:** The LiDAR was not moved during mapping, so the map only shows the immediate surroundings from one viewpoint.

2. **No Odometry:** Used static transforms instead of actual wheel odometry. For a moving robot, you need:
   - Wheel encoders publishing `/odom`
   - IMU data for orientation
   - Proper `odom → base_link` transform

3. **Single Pose:** Without movement, the pose graph only contains one position.

### General Limitations

1. **2D Only:** SLAM Toolbox creates 2D occupancy grids. For 3D mapping, use OctoMap or RTAB-Map.

2. **Loop Closure:** Requires revisiting previously seen areas. Won't work with a single stationary scan.

3. **Dynamic Objects:** Moving objects (people, pets) will appear as noise or phantom obstacles.

---

## Next Steps

### For Better Maps

1. **Mount LiDAR on Robot:** Attach to a mobile platform with wheel encoders
2. **Add IMU:** For better orientation tracking
3. **Drive Slowly:** 0.2-0.3 m/s for best SLAM results
4. **Cover Entire Area:** Drive through all spaces you want mapped
5. **Close Loops:** Return to starting position to improve map accuracy

### For Navigation

After creating a good map:

1. Save the map with both `.pgm/.yaml` and `.posegraph` files
2. Switch SLAM Toolbox to localization mode
3. Start Nav2 navigation stack
4. Set waypoints for autonomous navigation

---

## File Locations on Remote System

```
~/ros2_ws/
├── maps/
│   ├── first_map.pgm           # Map image
│   ├── first_map.yaml          # Map metadata
│   ├── first_map_posegraph.data
│   └── first_map_posegraph.posegraph
├── src/
│   └── lidar_mapping/
│       ├── config/
│       │   ├── slam_toolbox_params.yaml
│       │   └── slam_view.rviz
│       └── launch/
│           └── rplidar_slam.launch.py
└── install/

~/scripts/
├── start_mapping.sh
├── save_map.sh
├── test_lidar.sh
└── view_map.sh
```

---

## Quick Reference Commands

```bash
# Source ROS2 (do this in every new terminal)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start mapping
~/scripts/start_mapping.sh

# Save map (while mapping is running)
~/scripts/save_map.sh <map_name>

# View saved map
~/scripts/view_map.sh <map_name>

# Test LiDAR only
~/scripts/test_lidar.sh

# Check active topics
ros2 topic list

# Check scan frequency
ros2 topic hz /scan

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf  # Opens the TF tree visualization
```

---

## Troubleshooting

### LiDAR Not Detected

```bash
# Check USB device
lsusb | grep -i silicon
# Should show: Silicon Labs CP210x UART Bridge

# Check serial port
ls -la /dev/ttyUSB* /dev/rplidar
# Should show device with 666 permissions
```

### Map Not Building

1. Check that `/scan` topic is publishing:
   ```bash
   ros2 topic hz /scan
   ```

2. Check TF tree is complete:
   ```bash
   ros2 run tf2_tools view_frames
   ```
   Should show: `map → odom → base_link → laser`

3. Check SLAM Toolbox is receiving scans:
   ```bash
   ros2 topic echo /slam_toolbox/scan_visualization --once
   ```

### RViz Shows No Map

1. Set Fixed Frame to `map`
2. Add Map display with topic `/map`
3. Wait for map update (up to 5 seconds)

---

**Created:** 2025-12-20
**Author:** Claude Code (Opus 4.5)
**Status:** SUCCESS - Map created and saved
