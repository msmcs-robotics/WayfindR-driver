# How to View the Map in RViz

## Quick Start (On Remote Server 192.168.0.7)

### Step 1: Open a Terminal on the Remote Machine

Either:
- Use the physical monitor attached to the server
- SSH with X11 forwarding: `ssh -X devel@192.168.0.7`

### Step 2: Start Map Server and RViz

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Method A: Use the helper script
~/scripts/view_map.sh first_map

# Method B: Manual startup
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=/home/devel/ros2_ws/maps/first_map.yaml &
sleep 2
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
rviz2 &
```

### Step 3: Configure RViz

1. In the **Displays** panel on the left, click **Add**
2. Go to the **By topic** tab
3. Expand `/map` and select **Map**
4. Click **OK**

5. In **Global Options** (top of Displays panel):
   - Set **Fixed Frame** to: `map`

6. You should now see the occupancy grid map!

---

## Understanding the Map

### Color Legend

| Color | Meaning | Value |
|-------|---------|-------|
| White | Free space (navigable) | 0 |
| Black | Occupied (obstacles) | 100 |
| Gray | Unknown (not scanned) | -1 |

### Map Properties

- **Resolution:** 5cm per pixel
- **Size:** 212 x 144 pixels (10.6m x 7.2m)
- **Origin:** (-4.88, -4.09) meters

---

## Troubleshooting

### "Fixed frame [map] does not exist"

The map server isn't publishing TF. Run:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### Map appears all gray

1. Check map server is running:
   ```bash
   ros2 node list | grep map_server
   ```

2. Check map topic:
   ```bash
   ros2 topic echo /map --once
   ```

3. Make sure you activated the lifecycle node:
   ```bash
   ros2 lifecycle set /map_server activate
   ```

### RViz shows "No transform from [laser] to [map]"

Add static transforms:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link laser &
```

---

## Alternative: View the PGM File Directly

The map is saved as a standard PGM image. You can view it with any image viewer:

```bash
# On the remote machine
eog ~/ros2_ws/maps/first_map.pgm

# Or with ImageMagick
display ~/ros2_ws/maps/first_map.pgm

# Or convert to PNG
convert ~/ros2_ws/maps/first_map.pgm ~/ros2_ws/maps/first_map.png
```

---

## Files Location

On remote server (192.168.0.7):
```
~/ros2_ws/maps/
├── first_map.pgm           # Map image
├── first_map.yaml          # Metadata
├── first_map_posegraph.data
└── first_map_posegraph.posegraph
```

Local copy in this project:
```
ros2_cartography_attempt/
├── maps/
│   ├── first_map.pgm
│   └── first_map.yaml
├── config/
│   ├── slam_toolbox_params.yaml
│   └── slam_view.rviz
└── rplidar_slam.launch.py
```
