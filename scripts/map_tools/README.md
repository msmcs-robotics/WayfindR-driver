# Map Tools for ROS2

A collection of Python tools for editing ROS2 occupancy grid maps and managing waypoints.

## Tools

### 1. map_viewer.py
**Purpose:** Inspect and visualize ROS2 maps

**Features:**
- Interactive map viewing with coordinate display
- Grid overlay for planning
- Map metadata display
- Coordinate conversion (pixel ↔ world)
- Export grid points for waypoint planning

**Quick Start:**
```bash
python3 map_viewer.py --map-yaml /path/to/map.yaml
```

### 2. map_editor.py
**Purpose:** Command-line waypoint management

**Features:**
- Add/edit/remove waypoints programmatically
- Load and save waypoint YAML files
- Visualize waypoints on map
- Automatic coordinate conversion
- Quaternion calculation from yaw angles

**Quick Start:**
```bash
# Add a waypoint
python3 map_editor.py --map-yaml map.yaml --add "goal1" 2.0 1.0 90

# Visualize
python3 map_editor.py --map-yaml map.yaml --waypoints waypoints.yaml --visualize
```

### 3. waypoint_annotator.py
**Purpose:** Interactive GUI for waypoint annotation

**Features:**
- Click-based waypoint creation
- Visual position and orientation setting
- Real-time waypoint list
- Delete/undo functionality
- Automatic YAML export

**Quick Start:**
```bash
python3 waypoint_annotator.py --map-yaml /path/to/map.yaml
```

**Usage:**
1. Click to set waypoint position
2. Click again to set orientation
3. Enter waypoint name
4. Save when done

## Installation

### Dependencies

```bash
# System packages
sudo apt-get install python3-tk python3-pil python3-yaml

# Python packages
pip3 install numpy matplotlib pillow pyyaml
```

### Make Scripts Executable

```bash
chmod +x map_viewer.py map_editor.py waypoint_annotator.py
```

## Documentation

See `/findings/map-editing-guide.md` for:
- Complete map editing guide
- Waypoint management workflows
- RViz visualization
- Integration with Nav2
- Troubleshooting

## Examples

### Workflow 1: View and Inspect Map
```bash
# View map with info
python3 map_viewer.py --map-yaml maps/first_map.yaml --info

# Interactive view with grid
python3 map_viewer.py --map-yaml maps/first_map.yaml --grid
```

### Workflow 2: Create Waypoints Interactively
```bash
# Launch interactive annotator
python3 waypoint_annotator.py --map-yaml maps/first_map.yaml

# Click on map to add waypoints
# Save when done
```

### Workflow 3: Programmatic Waypoint Management
```bash
# Add waypoints via command line
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --add "kitchen" 2.5 1.0 90 "Kitchen area" \
    --add "hallway" 0.0 0.0 0 "Main hallway" \
    --output waypoints.yaml

# Edit existing waypoint
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --edit "kitchen" x=2.6 yaw_degrees=85 \
    --output waypoints.yaml

# Visualize result
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --visualize
```

### Workflow 4: Export Grid for Planning
```bash
# Export 0.5m grid of free space points
python3 map_viewer.py --map-yaml maps/first_map.yaml \
    --export-grid 0.5 \
    --output grid_points.yaml

# Use grid_points.yaml to identify good waypoint locations
```

## File Formats

### Map YAML Format
```yaml
image: map.pgm
resolution: 0.05
origin: [-4.88, -4.09, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### Waypoint YAML Format
```yaml
metadata:
  created_by: map_tools
  map_yaml: maps/first_map.yaml
  frame_id: map

waypoints:
  - name: goal1
    description: First goal
    position:
      x: 2.5
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707107
      w: 0.707107
    yaw_degrees: 90.0
    tolerance:
      position: 0.3
      orientation: 0.2

routes:
  tour:
    - goal1
    - goal2
```

## Coordinate Systems

**Pixel Coordinates:**
- Origin at top-left (0, 0)
- X right, Y down

**World Coordinates:**
- Origin from map YAML
- X right, Y up
- In meters

**Conversion:**
```python
# Pixel to World
world_x = origin_x + (pixel_x * resolution)
world_y = origin_y + ((height - pixel_y) * resolution)

# World to Pixel
pixel_x = (world_x - origin_x) / resolution
pixel_y = height - ((world_y - origin_y) / resolution)
```

## Tips

1. **Always inspect map first** with `map_viewer.py --info`
2. **Use grid overlay** to plan waypoint positions
3. **Interactive annotation** is fastest for most use cases
4. **Command-line editing** is best for automation
5. **Visualize before deploying** to verify waypoints
6. **Name waypoints descriptively** (e.g., "kitchen_entrance" not "wp1")
7. **Check coordinates** are within map bounds
8. **Test in RViz** before deploying to robot

## Troubleshooting

### tkinter not found (waypoint_annotator)
```bash
sudo apt-get install python3-tk
```

### matplotlib display issues
```bash
export MPLBACKEND=TkAgg
```

### Permission denied
```bash
chmod +x *.py
```

### Waypoint out of bounds
Check map bounds with:
```bash
python3 map_viewer.py --map-yaml map.yaml --info
```

## Integration

These tools work with:
- Existing `waypoint_manager.py` in the project
- ROS2 Nav2 navigation stack
- RViz for visualization
- Any ROS2 map created with map_saver

## License

Part of the WayfindR-driver project.
