# Waypoint Workflow Documentation

**Date:** 2025-12-20
**Status:** SUCCESS - Waypoints created and validated

---

## Summary

This document describes how to create, manage, and navigate to waypoints on ROS2 maps using the tools developed in this project.

---

## Quick Start

### 1. Add Waypoints to a Map (Automatic)

```bash
# Add center waypoint
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-center

# Add center + corners
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-center --add-corners

# Add a custom waypoint (name, x, y, yaw_degrees)
python3 waypoint_manager.py --map-yaml maps/first_map.yaml \
    --add-waypoint "kitchen" 2.5 1.0 90
```

### 2. View Waypoints

```bash
# List all waypoints
python3 waypoint_manager.py --waypoints maps/first_map_waypoints.yaml --list

# Show map info
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --info
```

### 3. Navigate to Waypoints (requires running Nav2)

```bash
# Navigate to a specific waypoint
python3 navigate_waypoints.py --waypoints maps/first_map_waypoints.yaml \
    --waypoint map_center

# Navigate to all waypoints
python3 navigate_waypoints.py --waypoints maps/first_map_waypoints.yaml --all

# Dry run (no ROS2 required)
python3 navigate_waypoints.py --waypoints maps/first_map_waypoints.yaml --all --dry-run
```

---

## Waypoint File Format

Waypoints are stored in YAML format:

```yaml
metadata:
  created_by: waypoint_manager.py
  map_yaml: maps/first_map.yaml
  frame_id: map

waypoints:
- name: map_center
  description: Center of map (auto-generated)
  position:
    x: 0.42
    y: -0.49
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  yaw_degrees: 0.0
  tolerance:
    position: 0.3      # meters
    orientation: 0.2   # radians

routes:
  patrol:
    - map_center
    - corner_top_left
    - corner_top_right
    - map_center
```

---

## Understanding Coordinates

### Map Coordinate System

```
                    Y+
                    ^
                    |
        (-,+)       |       (+,+)
                    |
    ----------------+----------------> X+
                    |
        (-,-)       |       (+,-)
                    |
```

### How Coordinates Work

1. **Origin**: Defined in map YAML as `origin: [x, y, z]`
   - This is the world coordinate of the map's bottom-left corner

2. **Resolution**: Meters per pixel (e.g., 0.05 = 5cm per pixel)

3. **Map Center**: Calculated as:
   ```
   center_x = origin_x + (width_pixels * resolution / 2)
   center_y = origin_y + (height_pixels * resolution / 2)
   ```

### Our Map (first_map)

| Property | Value |
|----------|-------|
| Resolution | 0.05 m/pixel |
| Dimensions | 212 x 144 pixels |
| Size | 10.60 x 7.20 meters |
| Origin | (-4.88, -4.09) |
| Center | (0.42, -0.49) |

---

## Orientation (Yaw Angle)

Orientation is specified in degrees, then converted to quaternion:

| Yaw (degrees) | Robot Facing | Quaternion (z, w) |
|---------------|--------------|-------------------|
| 0 | Right (+X) | (0.0, 1.0) |
| 90 | Up (+Y) | (0.707, 0.707) |
| 180 | Left (-X) | (1.0, 0.0) |
| 270 / -90 | Down (-Y) | (-0.707, 0.707) |

### Conversion Formula

```python
import math

def yaw_to_quaternion(yaw_degrees):
    yaw_rad = math.radians(yaw_degrees)
    z = math.sin(yaw_rad / 2)
    w = math.cos(yaw_rad / 2)
    return {'x': 0.0, 'y': 0.0, 'z': z, 'w': w}
```

---

## Python Scripts

### waypoint_manager.py

**Purpose:** Create and manage waypoints for maps

**Features:**
- Parse map YAML files
- Calculate map dimensions and center
- Add waypoints programmatically
- Convert between degrees and quaternions
- Export/import waypoints to/from YAML

**Usage Examples:**

```bash
# Show map information
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --info

# Add center waypoint
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-center

# Add all corners
python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-corners

# Add custom waypoint
python3 waypoint_manager.py --map-yaml maps/first_map.yaml \
    --add-waypoint "door" 1.5 2.0 180

# Specify output file
python3 waypoint_manager.py --map-yaml maps/first_map.yaml \
    --add-center -o custom_waypoints.yaml

# Load existing waypoints and add more
python3 waypoint_manager.py --waypoints existing.yaml \
    --add-waypoint "new_point" 3.0 1.0 0
```

### navigate_waypoints.py

**Purpose:** Navigate robot to waypoints using Nav2

**Features:**
- Load waypoints from YAML
- Navigate to single waypoint
- Execute waypoint routes
- Loop routes continuously
- Dry-run mode for testing

**Usage Examples:**

```bash
# List all waypoints
python3 navigate_waypoints.py --waypoints waypoints.yaml --list

# Navigate to single waypoint
python3 navigate_waypoints.py --waypoints waypoints.yaml --waypoint map_center

# Navigate all waypoints in order
python3 navigate_waypoints.py --waypoints waypoints.yaml --all

# Execute a named route
python3 navigate_waypoints.py --waypoints waypoints.yaml --route patrol

# Loop a route continuously
python3 navigate_waypoints.py --waypoints waypoints.yaml --route patrol --loop

# Dry run (no Nav2 required)
python3 navigate_waypoints.py --waypoints waypoints.yaml --all --dry-run
```

---

## Adding Waypoints Manually in RViz

If you prefer visual waypoint selection:

1. **Start mapping or localization**:
   ```bash
   ~/scripts/start_mapping.sh
   ```

2. **In RViz**, click the **"2D Nav Goal"** button

3. **Click on the map** where you want the waypoint

4. **Record the coordinates** from the terminal:
   ```bash
   # In another terminal
   ros2 topic echo /goal_pose
   ```

5. **Add to waypoint file** using the script:
   ```bash
   python3 waypoint_manager.py --waypoints existing.yaml \
       --add-waypoint "clicked_point" <x> <y> <yaw>
   ```

---

## Complete Navigation Workflow

### Prerequisites
1. Map created and saved
2. Waypoints defined
3. Robot with wheel encoders (for real navigation)
4. Nav2 stack running

### Steps

```bash
# Terminal 1: Start localization with map
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/ros2_ws/maps/first_map.yaml

# Terminal 2: Start Nav2 navigation
ros2 launch nav2_bringup navigation_launch.py

# Terminal 3: Navigate to waypoints
cd ~/ros2_ws/src/lidar_mapping/lidar_mapping
python3 navigate_waypoints.py \
    --waypoints ~/ros2_ws/maps/first_map_waypoints.yaml \
    --all
```

---

## Waypoint Best Practices

### Naming Conventions
- Use descriptive names: `kitchen_entrance`, `office_desk_5`
- Include zone/area in name: `warehouse_dock_a`
- Avoid generic names: `waypoint_1`, `point_a`

### Placement Tips
- Place waypoints in open areas (not near walls)
- Avoid placing waypoints inside obstacles
- Leave 0.5m buffer from walls for robot clearance
- Place docking/charging waypoints with precise orientation

### Tolerance Settings
- **Open areas**: position=0.5m, orientation=0.3rad
- **Tight spaces**: position=0.2m, orientation=0.1rad
- **Docking**: position=0.05m, orientation=0.05rad

---

## Troubleshooting

### Waypoint script fails to load map

```bash
# Verify map file exists and is valid
python3 -c "import yaml; print(yaml.safe_load(open('maps/first_map.yaml')))"

# Check PGM file exists
ls -la maps/first_map.pgm
```

### Navigation fails to reach waypoint

1. **Check waypoint is in free space**:
   - Open map in image viewer
   - Verify waypoint coordinates are in white (free) area

2. **Check Nav2 is running**:
   ```bash
   ros2 node list | grep nav
   ```

3. **Check TF tree**:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Wrong orientation at waypoint

- Verify yaw_degrees value in waypoint file
- Use online quaternion calculator to verify
- Test with simple angles first (0, 90, 180, 270)

---

## Files Location

### Local Project
```
ros2_cartography_attempt/
├── waypoint_manager.py        # Waypoint creation tool
├── navigate_waypoints.py      # Navigation script
├── maps/
│   ├── first_map.pgm          # Map image
│   ├── first_map.yaml         # Map metadata
│   └── first_map_waypoints.yaml  # Waypoints
└── config/
    └── slam_toolbox_params.yaml
```

### Remote Server (192.168.0.7)
```
~/ros2_ws/
├── maps/
│   ├── first_map.pgm
│   ├── first_map.yaml
│   └── first_map_waypoints.yaml
└── src/lidar_mapping/lidar_mapping/
    ├── waypoint_manager.py
    └── navigate_waypoints.py
```

---

## What Worked

1. **Map Parsing**: Successfully reading map YAML and PGM files
2. **Coordinate Calculation**: Correctly calculating map center and dimensions
3. **Quaternion Conversion**: Proper yaw to quaternion conversion
4. **YAML Export**: Clean, readable waypoint file format
5. **Dry Run Mode**: Testing without ROS2 environment

## Limitations

1. **Stationary Map**: Our test map was created with stationary LiDAR
2. **No Obstacle Checking**: Scripts don't verify waypoints are in free space
3. **No Route Validation**: Routes aren't checked for traversability

## Future Improvements

1. Add obstacle detection for waypoint validation
2. Visualize waypoints on map image
3. Integrate with fleet management dashboard
4. Add waypoint grouping by zones/areas
5. Support for 3D waypoints (multi-floor)

---

**Created:** 2025-12-20
**Author:** Claude Code (Opus 4.5)
