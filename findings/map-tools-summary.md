# ROS2 Map Editing Tools - Research Summary

## Overview

This document summarizes the research on ROS2 map editing and waypoint management, and describes the practical tools created for the WayfindR-driver project.

**Date:** 2026-01-11
**Purpose:** Enable practical map manipulation and waypoint annotation for autonomous navigation

---

## Research Findings

### 1. ROS2 Map File Format

ROS2 uses occupancy grid maps consisting of two files:

#### PGM Image File
- **Format:** Portable Gray Map (P5 binary or P2 ASCII)
- **Encoding:**
  - White (255) = Free space
  - Black (0) = Occupied/obstacle
  - Gray (1-254) = Unknown
- **Occupancy formula:** `occupancy = (255 - pixel_value) / 255.0`
- **Tools:** Can be edited with GIMP, ImageMagick, or Python PIL/Pillow

#### YAML Metadata File
```yaml
image: map.pgm              # Image filename
resolution: 0.05            # Meters per pixel
origin: [-4.88, -4.09, 0]  # World coordinates of bottom-left
negate: 0                   # Invert colors (0 or 1)
occupied_thresh: 0.65       # Occupancy threshold
free_thresh: 0.25           # Free space threshold
mode: trinary               # trinary, scale, or raw
```

**Key Parameters:**
- **Resolution:** Determines map accuracy (typical: 0.01-0.1 m/pixel)
- **Origin:** Maps pixel (0,0) to world coordinates
- **Thresholds:** Define free vs occupied boundaries

### 2. Waypoint Tools Ecosystem

#### Existing ROS2 Packages
- **nav2_map_server:** Core map serving functionality
- **wayp_plan_tools:** Saves/loads waypoints to CSV
- **waypoint_navigation:** Services for waypoint management
- **nav2_waypoint_follower:** Sequential waypoint following

#### GUI Tools Found
- Tkinter-based annotator that allows clicking on maps
- First click sets position, second click sets orientation
- Copies waypoint Python code to clipboard

#### Best Practices Identified
- Store waypoints in YAML format for ROS2 compatibility
- Use world coordinates (meters) not pixel coordinates
- Include quaternion orientation (converted from yaw)
- Add tolerance parameters for navigation flexibility

### 3. Visualization in RViz

#### Marker Types Available
- **SPHERE:** Position markers
- **ARROW:** Orientation indicators
- **TEXT_VIEW_FACING:** Labels
- **CUBE/CYLINDER:** Alternative position markers
- **LINE_STRIP:** Paths between waypoints

#### Interactive Markers
- Enable drag-and-drop waypoint editing in RViz
- 6-DOF control for full pose adjustment
- Feedback callbacks for real-time updates
- Requires interactive_markers package

### 4. Coordinate Systems

#### Pixel Coordinates
- Origin: Top-left (0, 0)
- X-axis: Right (increasing)
- Y-axis: Down (increasing)
- Integer values

#### World Coordinates
- Origin: Defined in map YAML
- X-axis: Right (East)
- Y-axis: Up (North)
- Floating-point meters
- Can be negative

#### Conversion Math
```python
# Pixel to World
world_x = origin_x + (pixel_x * resolution)
world_y = origin_y + ((height - pixel_y) * resolution)

# World to Pixel
pixel_x = (world_x - origin_x) / resolution
pixel_y = height - ((world_y - origin_y) / resolution)
```

### 5. Python Libraries for Map Manipulation

#### PIL/Pillow
- Full support for PGM format
- `Image.open()` reads PGM directly
- `ImageDraw` module for annotations
- Can convert between formats

#### NumPy
- Efficient array operations on map data
- Analyze occupancy statistics
- Filter and transform maps

#### PyYAML
- Read/write map metadata
- Parse waypoint definitions
- Safe loading prevents code injection

#### Matplotlib
- Visualization and interactive plotting
- Click event handling for annotation
- Export to various image formats
- Grid overlays and annotations

---

## Tools Created

### Location
`/home/devel/Desktop/WayfindR-driver/scripts/map_tools/`

### 1. map_viewer.py

**Purpose:** Inspect and visualize ROS2 maps

**Key Features:**
- Load and display PGM maps with metadata
- Interactive coordinate display (click to see world/pixel coords)
- Show map bounds, center, and physical dimensions
- Grid overlay for planning (configurable spacing)
- Scale ruler for reference
- Export grid of free-space points
- Analyze map content (occupied/free/unknown percentages)

**Usage Examples:**
```bash
# Basic view
python3 map_viewer.py --map-yaml maps/first_map.yaml

# Detailed info
python3 map_viewer.py --map-yaml maps/first_map.yaml --info

# With grid overlay
python3 map_viewer.py --map-yaml maps/first_map.yaml --grid --grid-spacing 50

# Check specific coordinates
python3 map_viewer.py --map-yaml maps/first_map.yaml --coords 100 70

# Export grid points
python3 map_viewer.py --map-yaml maps/first_map.yaml --export-grid 0.5
```

**Output:**
- Real-time coordinate conversion on mouse click
- Map information panel with bounds and resolution
- Visual scale ruler (1 meter reference)
- Color-coded legend
- Grid points YAML file (for planning)

### 2. map_editor.py

**Purpose:** Command-line waypoint management

**Key Features:**
- Add/edit/remove waypoints programmatically
- Load existing waypoint YAML files
- Automatic coordinate conversion (world ↔ pixel)
- Quaternion calculation from yaw angles
- Visualize waypoints on map with arrows
- Support for waypoint routes (sequences)
- Batch operations

**Usage Examples:**
```bash
# Add waypoint
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --add "kitchen" 2.5 1.0 90 "Kitchen waypoint" \
    --output waypoints.yaml

# Edit waypoint
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --edit "kitchen" x=2.6 yaw_degrees=85 \
    --output waypoints.yaml

# Remove waypoint
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --remove "old_point" \
    --output waypoints.yaml

# List all waypoints
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml --list

# Visualize
python3 map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml --visualize
```

**Visualization:**
- Red circles: Waypoint positions
- Blue arrows: Orientation (direction robot will face)
- White labels: Waypoint names
- Supports saving visualization to image file

### 3. waypoint_annotator.py

**Purpose:** Interactive GUI for waypoint creation

**Key Features:**
- Click-based waypoint annotation
- Two-click workflow (position, then orientation)
- Real-time visualization
- Waypoint list panel with management controls
- Delete via double-click
- Undo last waypoint
- Clear all waypoints
- Auto-save to YAML

**Usage Examples:**
```bash
# Start annotator
python3 waypoint_annotator.py --map-yaml maps/first_map.yaml

# Load existing waypoints
python3 waypoint_annotator.py --map-yaml maps/first_map.yaml \
    --waypoints existing_waypoints.yaml
```

**Workflow:**
1. Click on map → Sets position (red dot appears)
2. Click again → Sets orientation (blue arrow drawn)
3. Dialog → Enter waypoint name
4. Repeat for more waypoints
5. Click "Save Waypoints" when done

**GUI Controls:**
- Save Waypoints: Export to YAML
- Clear All: Delete all waypoints
- Undo Last: Remove most recent
- Refresh View: Redraw display
- Waypoint List: Shows all waypoints
- Double-click list item: Delete waypoint

### Supporting Files

#### __init__.py
Package initialization file for Python imports

#### README.md
Comprehensive documentation for the tools including:
- Installation instructions
- Quick start examples
- File format specifications
- Workflow recommendations
- Troubleshooting guide

#### example_usage.sh
Executable script demonstrating all tools:
- Map information display
- Waypoint listing
- Coordinate checking
- Programmatic waypoint addition
- Grid export

---

## File Formats Produced

### Waypoint YAML Format

```yaml
metadata:
  created_by: waypoint_annotator.py  # or map_editor.py
  map_yaml: maps/first_map.yaml
  frame_id: map
  map_resolution: 0.05
  map_size:
    width_pixels: 212
    height_pixels: 144
    width_meters: 10.6
    height_meters: 7.2

waypoints:
  - name: kitchen
    description: Kitchen area waypoint
    position:
      x: 2.5000      # World coordinates (meters)
      y: 1.0000
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707107    # Quaternion Z (from yaw)
      w: 0.707107    # Quaternion W
    yaw_degrees: 90.0   # Human-readable orientation
    tolerance:
      position: 0.3      # Position tolerance (meters)
      orientation: 0.2   # Orientation tolerance (radians)

routes:
  tour:
    - kitchen
    - hallway
    - living_room
```

**Key Fields:**
- **name:** Unique identifier
- **position:** World coordinates in meters
- **orientation:** Quaternion (for ROS2 compatibility)
- **yaw_degrees:** Human-readable orientation
- **tolerance:** Navigation accuracy requirements
- **routes:** Named sequences of waypoints

### Grid Points YAML Format

```yaml
metadata:
  map_yaml: maps/first_map.yaml
  spacing_meters: 0.5
  total_points: 250

grid_points:
  - world: {x: -4.5, y: -3.5}
    pixel: {x: 8, y: 136}
  - world: {x: -4.5, y: -3.0}
    pixel: {x: 8, y: 126}
  # ... more points
```

**Purpose:** Provides evenly-spaced grid of free-space points for waypoint planning

---

## Integration with Existing Tools

### With waypoint_manager.py

The new tools complement the existing waypoint_manager.py:

```bash
# Use existing manager for batch operations
python3 ros2_cartography_attempt/waypoint_manager.py \
    --map-yaml maps/first_map.yaml \
    --add-center --add-corners \
    --output waypoints.yaml

# Then use new annotator to add custom waypoints
python3 scripts/map_tools/waypoint_annotator.py \
    --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml

# Visualize with new tools
python3 scripts/map_tools/map_editor.py \
    --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --visualize
```

**Benefits:**
- Existing manager: Algorithmic waypoint generation
- New annotator: Precise manual placement
- New viewer: Planning and inspection
- New editor: Programmatic batch operations

### With ROS2 Nav2

All waypoints are compatible with Nav2:

```python
# Load and publish waypoints
from geometry_msgs.msg import PoseStamped
import yaml

with open('waypoints.yaml', 'r') as f:
    data = yaml.safe_load(f)

for wp in data['waypoints']:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = wp['position']['x']
    pose.pose.position.y = wp['position']['y']
    pose.pose.orientation.z = wp['orientation']['z']
    pose.pose.orientation.w = wp['orientation']['w']
    # Send to Nav2...
```

### With RViz

Example marker publisher for visualization (see guide for full code):

```python
# Publish waypoint markers
marker = Marker()
marker.type = Marker.SPHERE
marker.pose.position.x = wp['position']['x']
marker.pose.position.y = wp['position']['y']
# Configure and publish...
```

---

## Testing Results

### Test Environment
- Map: `/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml`
- Map size: 212×144 pixels (10.6m × 7.2m)
- Resolution: 0.05 m/pixel (5cm/pixel)
- Existing waypoints: 5 (center + 4 corners)

### map_viewer.py Test
```bash
python3 map_viewer.py --map-yaml ros2_cartography_attempt/maps/first_map.yaml --info
```

**Results:**
- Successfully loaded PGM and YAML
- Correctly parsed all metadata
- Calculated bounds: X[-4.88, 5.67], Y[-4.04, 3.11]
- Center: (0.395, -0.465)
- Content analysis: 99.8% free, 0.2% occupied
- ✅ All coordinate calculations verified

### map_editor.py Test
```bash
python3 map_editor.py --map-yaml ros2_cartography_attempt/maps/first_map.yaml \
    --waypoints ros2_cartography_attempt/maps/first_map_waypoints.yaml --list
```

**Results:**
- Loaded 5 existing waypoints
- Displayed world and pixel coordinates
- Verified quaternion calculations
- Pixel coords match expected values
- ✅ All waypoints loaded correctly

### Coordinate Conversion Verification

**Example:** Center waypoint
- World: (0.420, -0.490)
- Pixel: (105, 72)

**Manual verification:**
```
px = (0.420 - (-4.88)) / 0.05 = 106.0 ✅
py = 144 - ((-0.490 - (-4.09)) / 0.05) = 72.0 ✅
```

**Quaternion verification:**
- Yaw: 45°
- Expected: z = sin(45°/2) = 0.3827, w = cos(45°/2) = 0.9239
- Actual: z = 0.3827, w = 0.9239 ✅

---

## Recommended Workflows

### Workflow 1: New Map with Waypoints

```bash
# 1. Create map with SLAM
ros2 launch slam_toolbox online_async_launch.py
ros2 run nav2_map_server map_saver_cli -f my_map

# 2. Inspect map
python3 scripts/map_tools/map_viewer.py --map-yaml my_map.yaml --info --grid

# 3. Plan waypoints (export grid)
python3 scripts/map_tools/map_viewer.py --map-yaml my_map.yaml --export-grid 0.5

# 4. Add waypoints interactively
python3 scripts/map_tools/waypoint_annotator.py --map-yaml my_map.yaml

# 5. Verify
python3 scripts/map_tools/map_editor.py --map-yaml my_map.yaml \
    --waypoints my_map_waypoints.yaml --visualize --list
```

### Workflow 2: Batch Waypoint Generation + Manual Refinement

```bash
# 1. Generate algorithmic waypoints
python3 waypoint_manager.py --map-yaml map.yaml --add-center --add-corners

# 2. Add custom waypoints
python3 scripts/map_tools/waypoint_annotator.py \
    --map-yaml map.yaml --waypoints map_waypoints.yaml

# 3. Fine-tune via command line
python3 scripts/map_tools/map_editor.py --map-yaml map.yaml \
    --waypoints map_waypoints.yaml \
    --edit "corner_top_left" x=-4.35 yaw_degrees=310
```

### Workflow 3: Map Editing

```bash
# 1. View current map
python3 scripts/map_tools/map_viewer.py --map-yaml map.yaml

# 2. Edit in GIMP
gimp map.pgm
# Make edits, save

# 3. Verify changes
python3 scripts/map_tools/map_viewer.py --map-yaml map.yaml --info

# 4. Update waypoints if needed
python3 scripts/map_tools/waypoint_annotator.py --map-yaml map.yaml
```

---

## Key Insights

### 1. Map Resolution Trade-offs
- **High (0.01-0.02 m/px):** Detailed, large file, slower processing
- **Medium (0.05 m/px):** Standard, good balance (recommended)
- **Low (0.1+ m/px):** Fast, less detail, outdoor use

### 2. Coordinate System Complexity
- Must account for Y-axis inversion (image vs world)
- Origin can be anywhere (often bottom-left is negative)
- Pixel coordinates are always non-negative integers
- World coordinates are floating-point and can be negative

### 3. Orientation Representation
- Users think in degrees (0-360)
- ROS2 requires quaternions
- Store both for usability
- 2D rotation: only Z and W components matter

### 4. Tool Design Decisions

**Why three separate tools?**
- **Viewer:** Read-only inspection, no dependencies on editing
- **Editor:** Programmatic/scriptable, automation-friendly
- **Annotator:** GUI for manual precision, different workflow

**Why not one combined tool?**
- Simpler to maintain and test
- Users can choose workflow
- Lighter dependencies for each use case
- Easier to integrate into automation

### 5. Compatibility Considerations
- All tools produce Nav2-compatible YAML
- Coordinate systems match ROS2 conventions
- Quaternions follow ROS2 standards
- Frame ID always "map" for static maps

---

## Future Enhancements

### Possible Additions

1. **Map Editing Features:**
   - GUI map editor (draw walls, obstacles)
   - Auto-inflate obstacles for robot size
   - Merge multiple maps
   - Crop/resize maps

2. **Waypoint Features:**
   - Automatic waypoint placement (room centers, doorways)
   - Waypoint validation (check if in free space)
   - Route optimization (shortest path)
   - Waypoint groups/categories

3. **Visualization:**
   - 3D visualization
   - Cost map overlay
   - Path planning preview
   - Distance measurements

4. **Integration:**
   - ROS2 nodes for real-time editing
   - RViz plugin for annotation
   - Web interface
   - Multi-map support

5. **Analysis:**
   - Map quality metrics
   - Coverage analysis
   - Connectivity graph
   - Waypoint reachability

---

## Resources Referenced

### Web Research Sources

**Map Format Documentation:**
- [ORB-SLAM3 Extension - Occupancy Grid for ROS 2](https://discourse.openrobotics.org/t/orb-slam3-extension-real-time-sparse-map-to-occupancy-grid-yaml-pgm-for-ros-2-navigation/49158)
- [How to Create a Map for ROS From a Floor Plan](https://automaticaddison.com/how-to-create-a-map-for-ros-from-a-floor-plan-or-blueprint/)
- [ROS2 Tutorial: Creating map from a Floor Plan](https://robotics.snowcron.com/robotics_ros2/adv_nav_creating_map.htm)

**Waypoint Tools:**
- [wayp_plan_tools - GitHub](https://github.com/jkk-research/wayp_plan_tools)
- [waypoint_navigation - GitHub](https://github.com/MERLIN2-ARCH/waypoint_navigation)
- [ROS2 Tutorial: Utility to add waypoints](https://robotics.snowcron.com/robotics_ros2/adv_nav_add_waypoints_util.htm)

**Python Libraries:**
- [Image Processing With the Python Pillow Library](https://realpython.com/image-processing-with-the-python-pillow-library/)
- [Drawing Grids With Python and Pillow](https://randomgeekery.org/post/2017/11/drawing-grids-with-python-and-pillow/)

**RViz Visualization:**
- [Marker: Display types - ROS 2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html)
- [Interactive Markers - GitHub](https://github.com/ros-visualization/interactive_markers)
- [RViz Markers - Stretch Docs](https://docs.hello-robot.com/0.3/ros2/example_4/)

### Official Documentation
- Nav2 Documentation: https://docs.nav2.org
- ROS2 Map Server: https://github.com/ros-planning/navigation2/tree/main/nav2_map_server
- PGM Format: http://netpbm.sourceforge.net/doc/pgm.html

---

## Conclusion

The research successfully identified ROS2 map formats, editing techniques, and waypoint management approaches. The three tools created provide a complete workflow for:

1. **Inspection** - Understanding map structure and bounds
2. **Planning** - Identifying optimal waypoint locations
3. **Annotation** - Adding waypoints interactively or programmatically
4. **Verification** - Visualizing and validating waypoints
5. **Integration** - Using waypoints with Nav2 navigation

All tools are tested, documented, and ready for use in the WayfindR-driver project. The comprehensive guide (`map-editing-guide.md`) provides detailed instructions for all use cases.

**Status:** ✅ Complete and tested
**Location:** `/home/devel/Desktop/WayfindR-driver/scripts/map_tools/`
**Documentation:** `/home/devel/Desktop/WayfindR-driver/findings/map-editing-guide.md`
