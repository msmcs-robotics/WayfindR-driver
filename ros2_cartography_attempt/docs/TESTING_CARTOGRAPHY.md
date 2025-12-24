# Cartography (SLAM Mapping) Testing Guide

## Overview

This guide walks you through testing SLAM mapping with the Slamtec C1M1RP LiDAR. You'll create a 2D occupancy grid map of your environment that can later be used for localization and navigation.

## Prerequisites

- Remote server (192.168.0.7) with Ubuntu 22.04
- ROS2 Humble installed
- Slamtec C1M1RP LiDAR connected via USB
- Monitor/keyboard on remote server (for RViz visualization)

## Quick Start

### Option 1: All-in-One Launch

On the remote server terminal (with display):

```bash
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./start_all.sh
```

This opens 4 terminal windows:
1. TF transforms
2. LiDAR driver
3. SLAM Toolbox
4. RViz

### Option 2: Step-by-Step (Recommended for Debugging)

Open 4 terminal windows/tabs on the remote server:

**Terminal 1 - TF Transforms:**
```bash
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./start_tf.sh
```

**Terminal 2 - LiDAR (wait 2 seconds):**
```bash
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./start_lidar.sh
```

**Terminal 3 - SLAM Toolbox (wait 2 seconds):**
```bash
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./start_slam.sh
```

**Terminal 4 - RViz:**
```bash
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./start_rviz.sh
```

## What You Should See in RViz

1. **Grid** - Reference grid on the floor (1m cells)
2. **TF Frames** - Coordinate frames (map, odom, base_link, laser)
3. **LaserScan** - Red/rainbow points showing LiDAR readings
4. **Map** - Grey occupancy grid building as you move the LiDAR

### Initial View
- Set "Fixed Frame" to `map` (should be default)
- You should see laser scan points immediately
- Map will be mostly grey (unknown) initially

### As You Scan
- Move the LiDAR around the room
- Walls and obstacles appear as black cells
- Open areas become white cells
- Grey areas are unexplored

## Testing Checklist

### Stage 1: Verify LiDAR Connection
```bash
# On remote server
ls -la /dev/rplidar
# Should show: /dev/rplidar -> ttyUSB0
```
- [ ] LiDAR device exists at /dev/rplidar
- [ ] Green LED on LiDAR is spinning

### Stage 2: Verify Scan Data
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /scan --once
```
- [ ] /scan topic is publishing
- [ ] Data shows ranges (distance values)
- [ ] No "inf" values for nearby objects

### Stage 3: Verify TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing TF tree
```
- [ ] map → odom → base_link → laser chain exists
- [ ] No TF errors in terminal output

### Stage 4: Verify Map Building
In RViz:
- [ ] Fixed Frame set to "map"
- [ ] LaserScan display shows points
- [ ] Map display shows grid being built
- [ ] Moving LiDAR updates the map

### Stage 5: Save the Map
When satisfied with coverage:
```bash
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./save_map.sh my_room_map
```
- [ ] Map saved to `maps/my_room_map.pgm`
- [ ] Metadata saved to `maps/my_room_map.yaml`

## Troubleshooting

### "No /scan topic"
- Check LiDAR USB connection
- Verify device: `ls /dev/ttyUSB*`
- Check permissions: `sudo chmod 666 /dev/ttyUSB0`
- Restart LiDAR node

### "TF transform not found"
- Make sure start_tf.sh is running
- Check for TF errors: `ros2 run tf2_ros tf2_echo map laser`
- Restart TF transforms

### "Map not updating"
- Ensure all nodes are running (use check_status.sh)
- Move the LiDAR more - it needs motion to build map
- Check SLAM Toolbox output for errors

### RViz Display Issues
- Set Fixed Frame to "map"
- Add displays manually:
  - Add → By Topic → /scan → LaserScan
  - Add → By Topic → /map → Map
  - Add → TF

## Status Check Script

```bash
cd ~/Desktop/WayfindR-driver/ros2_cartography_attempt/scripts
./check_status.sh
```

This shows:
- LiDAR device status
- Active ROS2 nodes
- Key topic status (/scan, /map, /tf)
- TF frame availability

## Expected Output Examples

### Successful LiDAR Start
```
==========================================
  Starting RPLidar Node
  Device: /dev/rplidar (/dev/ttyUSB0)
==========================================
Using device: /dev/rplidar

[INFO] [rplidar_node]: RPLIDAR S/N: ...
[INFO] [rplidar_node]: Firmware Ver: ...
[INFO] [rplidar_node]: Hardware Rev: ...
```

### Successful SLAM Start
```
==========================================
  Starting SLAM Toolbox (Mapping Mode)
==========================================
[INFO] [slam_toolbox]: Using solver: CeresSolver
[INFO] [slam_toolbox]: Message Filter connecting...
```

### Successful Topic List
```bash
$ ros2 topic list
/map
/map_metadata
/parameter_events
/rosout
/scan
/slam_toolbox/graph_visualization
/tf
/tf_static
```

## Next Steps

After successful mapping:

1. **Save your map** using `save_map.sh`
2. **Test localization** - See `docs/TESTING_LOCALIZATION.md`
3. **Set up waypoints** - See `WAYPOINT_WORKFLOW.md`
4. **Navigate** - See navigation testing docs

## Files Created

| File | Purpose |
|------|---------|
| `maps/your_map.pgm` | Occupancy grid image |
| `maps/your_map.yaml` | Map metadata (resolution, origin) |
| `maps/your_map.posegraph` | SLAM pose graph (for localization) |

## Configuration Reference

Key parameters in `config/slam_toolbox_params.yaml`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| resolution | 0.05 | Map cell size (5cm) |
| max_laser_range | 12.0 | LiDAR max range (12m for C1) |
| map_update_interval | 5.0 | Seconds between map publishes |
| minimum_travel_distance | 0.5 | Min movement for new scan |

---

*Last tested: December 2024*
*Platform: Ubuntu 22.04, ROS2 Humble, Slamtec C1M1RP*
