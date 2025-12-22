# Remote System Status Report

**Date:** 2025-12-21
**Remote System:** devel@192.168.0.7
**OS:** Ubuntu 22.04.5 LTS (Desktop)

---

## System Status: ALL SYSTEMS GO

| Component | Status | Details |
|-----------|--------|---------|
| ROS2 Humble | ✅ Installed | Full desktop installation |
| RViz2 | ✅ Installed | `/opt/ros/humble/bin/rviz2` |
| SLAM Toolbox | ✅ Installed | v2.6.10 |
| Nav2 | ✅ Installed | v1.1.20 |
| RPLidar Driver | ✅ Installed | v2.1.4 |
| LiDAR Hardware | ✅ Connected | Slamtec C1 on `/dev/ttyUSB0` |
| Project Sync | ✅ Complete | `~/Desktop/WayfindR-driver/` |

---

## LiDAR Hardware

```
Device: Silicon Labs CP210x UART Bridge
USB Bus: 003, Device: 004
Vendor ID: 10c4
Product ID: ea60
Serial Port: /dev/ttyUSB0
Symlink: /dev/rplidar
Permissions: 666 (read/write for all)
```

---

## Map Files Location

### Primary Maps (created during testing):
```
~/ros2_ws/maps/
├── first_map.pgm              # Occupancy grid image (30KB)
├── first_map.yaml             # Map metadata
├── first_map_waypoints.yaml   # Waypoints with center + corners
├── first_map_posegraph.data   # SLAM pose data
└── first_map_posegraph.posegraph  # Full pose graph (6.6MB)
```

### Synced Project Maps:
```
~/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/
├── first_map.pgm
├── first_map.yaml
└── first_map_waypoints.yaml
```

---

## How to View Maps in RViz

### Quick Method (use the script I created):

On the remote system's monitor, open a terminal and run:

```bash
~/view_map_rviz.sh
```

Or to view a specific map:
```bash
~/view_map_rviz.sh first_map
```

### Manual Method:

```bash
# Terminal 1: Source ROS2 and start map server
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=/home/devel/ros2_ws/maps/first_map.yaml &

# Wait a moment, then activate
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# Terminal 2: Start RViz
rviz2
```

### In RViz:
1. Set **Fixed Frame** to `map` (in Global Options, left panel)
2. Click **Add** button
3. Select **By topic** tab
4. Expand `/map` and select **Map**
5. Click **OK**
6. You should see the occupancy grid (white=free, black=obstacles, gray=unknown)

---

## How to Create New Maps

### Start Live Mapping:

```bash
# On the remote system
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Use the helper script
~/scripts/start_mapping.sh

# OR launch manually
ros2 launch lidar_mapping rplidar_slam.launch.py
```

This will:
- Start the LiDAR
- Start SLAM Toolbox
- Open RViz with live view

### Save a Map:

While mapping is running:
```bash
~/scripts/save_map.sh my_new_map
```

Or manually:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_new_map
```

---

## Helper Scripts on Remote System

| Script | Purpose |
|--------|---------|
| `~/scripts/start_mapping.sh` | Start SLAM mapping with RViz |
| `~/scripts/save_map.sh <name>` | Save current map |
| `~/scripts/test_lidar.sh` | Test LiDAR connection |
| `~/scripts/view_map.sh <name>` | View saved map |
| `~/view_map_rviz.sh <name>` | New! Simple RViz map viewer |
| `~/system_info.sh` | Show system information |
| `~/test_lidar.sh` | Test LiDAR only |

---

## Project Files Synced

The following were synced from local to `~/Desktop/WayfindR-driver/`:

- `ros2_cartography_attempt/` - Maps, waypoints, launch files
- `ros2_install_attempt/` - Installation scripts and docs
- `new_bakery/` - Raspberry Pi SD card baking scripts
- `old_bakery/` - Previous baking attempts
- `pi-fleet-manager/` - Fleet management web UI
- All other project files

---

## What Works

1. **LiDAR Detection**: Slamtec C1 detected and accessible
2. **ROS2 Stack**: Full Humble installation with SLAM/Nav2
3. **RViz**: Available for visualization
4. **Maps**: Previously created maps available in `~/ros2_ws/maps/`
5. **Rsync**: Project syncs from local to remote successfully

---

## What to Do Next

### To View Existing Maps:

1. Go to the remote system's monitor
2. Open terminal
3. Run: `~/view_map_rviz.sh`

### To Create Better Maps:

1. Mount LiDAR on mobile platform
2. Run: `~/scripts/start_mapping.sh`
3. Physically move the LiDAR around the space
4. Save: `~/scripts/save_map.sh room_name`

### To Test Navigation:

```bash
# Start localization with existing map
ros2 launch nav2_bringup localization_launch.py \
    map:=/home/devel/ros2_ws/maps/first_map.yaml

# Start navigation
ros2 launch nav2_bringup navigation_launch.py
```

---

## Network Access

- **SSH**: `ssh devel@192.168.0.7`
- **mDNS**: May work as `devel@err0r.local` (hostname is "err0r")
- **Same network**: 192.168.0.x subnet

---

**Last Updated:** 2025-12-21
**Verified By:** Claude Code (Opus 4.5)
