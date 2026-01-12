# ROS2 Map Editing and Waypoint Annotation Guide

This guide covers everything you need to know about editing ROS2 occupancy grid maps and adding waypoints for navigation.

## Table of Contents
1. [Map File Formats](#map-file-formats)
2. [Manual Map Editing](#manual-map-editing)
3. [Using the Map Tools](#using-the-map-tools)
4. [Waypoint Management](#waypoint-management)
5. [RViz Visualization](#rviz-visualization)
6. [Integration Guide](#integration-guide)

---

## Map File Formats

ROS2 navigation uses occupancy grid maps consisting of two files:

### 1. Map Metadata File (.yaml)

The YAML file contains map metadata:

```yaml
image: first_map.pgm          # Path to image file (relative or absolute)
mode: trinary                 # Mode: trinary, scale, or raw
resolution: 0.05              # Meters per pixel
origin: [-4.88, -4.09, 0]    # World coordinates of bottom-left pixel [x, y, theta]
negate: 0                     # 0 or 1, inverts black/white
occupied_thresh: 0.65         # Threshold for occupied (0.0-1.0)
free_thresh: 0.25             # Threshold for free (0.0-1.0)
```

**Key Parameters:**
- **resolution**: Map resolution in meters/pixel (e.g., 0.05 = 5cm per pixel)
- **origin**: Position of the map's lower-left corner in world coordinates
- **occupied_thresh**: Pixels darker than this are considered occupied
- **free_thresh**: Pixels lighter than this are considered free
- **negate**: Set to 1 to invert colors (white becomes occupied, black becomes free)

### 2. Map Image File (.pgm)

PGM (Portable Gray Map) format represents occupancy:

**Pixel Values:**
- **White (255)**: Free space - robot can navigate here
- **Black (0)**: Occupied space - obstacles, walls
- **Gray (1-254)**: Unknown space - not yet mapped

**Occupancy Calculation:**
```
occupancy_probability = (255 - pixel_value) / 255.0
```

**PGM File Structure:**
```
P5                    # Magic number (P5 = binary grayscale)
# Comment line       # Optional comments
640 480              # Width Height (pixels)
255                  # Maximum pixel value
<binary data>        # Raw pixel data
```

---

## Manual Map Editing

### Using GIMP (Recommended)

1. **Open the map:**
   ```bash
   gimp /path/to/map.pgm
   ```

2. **Edit the map:**
   - Use white (255) to mark free space
   - Use black (0) to mark obstacles
   - Use gray for unknown areas
   - Use selection tools to draw walls, rooms, etc.

3. **Export:**
   - File → Export As
   - Choose PGM format
   - Ensure "Raw" encoding is selected
   - Save as `.pgm`

### Using ImageMagick

```bash
# Convert to PNG for easier editing
convert map.pgm map.png

# Edit map.png with any image editor

# Convert back to PGM
convert map.png -type Grayscale map.pgm
```

### Using Python/PIL

```python
from PIL import Image, ImageDraw

# Load map
img = Image.open('map.pgm')

# Create drawing context
draw = ImageDraw.Draw(img)

# Add obstacles (black rectangles)
draw.rectangle([100, 100, 150, 150], fill=0)

# Add free space (white)
draw.rectangle([200, 200, 300, 300], fill=255)

# Save
img.save('map_edited.pgm')
```

### Creating Maps from Floor Plans

1. **Prepare floor plan:**
   - Scan or obtain digital floor plan
   - Convert to grayscale
   - Scale to desired resolution

2. **Convert to occupancy grid:**
   ```bash
   # Resize to match desired physical size
   # If map should be 10m x 10m at 0.05m/pixel = 200x200 pixels
   convert floorplan.png -resize 200x200 -type Grayscale map.pgm
   ```

3. **Clean up:**
   - Threshold to remove gray values
   - Manually fix any issues in GIMP

4. **Create YAML file:**
   ```yaml
   image: map.pgm
   resolution: 0.05
   origin: [0.0, 0.0, 0.0]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.25
   ```

---

## Using the Map Tools

The `scripts/map_tools/` directory contains three powerful tools for map manipulation.

### 1. Map Viewer (`map_viewer.py`)

**Purpose:** Inspect maps, view metadata, and plan waypoint locations

**Basic Usage:**
```bash
# View map with interactive coordinates
python3 scripts/map_tools/map_viewer.py --map-yaml maps/first_map.yaml

# Show detailed info
python3 scripts/map_tools/map_viewer.py --map-yaml maps/first_map.yaml --info

# View with grid overlay (helpful for planning)
python3 scripts/map_tools/map_viewer.py --map-yaml maps/first_map.yaml --grid

# Check specific pixel coordinates
python3 scripts/map_tools/map_viewer.py --map-yaml maps/first_map.yaml --coords 100 200

# Export grid of free space points (for waypoint planning)
python3 scripts/map_tools/map_viewer.py --map-yaml maps/first_map.yaml --export-grid 0.5
```

**Features:**
- Interactive coordinate display (click on map)
- Shows map bounds and center point
- Grid overlay for planning
- Scale ruler
- Export grid points in free space

**Output Example:**
```
Map loaded successfully:
  Image: /path/to/first_map.pgm
  Size: 193 x 124 pixels
  Resolution: 0.05 m/pixel (5.0 cm/pixel)
  Physical size: 9.65m x 6.20m
  Origin: (-4.880, -4.090, 0.000)
```

### 2. Map Editor (`map_editor.py`)

**Purpose:** Programmatically add, edit, and remove waypoints

**Basic Usage:**
```bash
# Load map and list info
python3 scripts/map_tools/map_editor.py --map-yaml maps/first_map.yaml

# Add a waypoint (name, x, y, yaw_degrees)
python3 scripts/map_tools/map_editor.py --map-yaml maps/first_map.yaml \
    --add "kitchen" 2.5 1.0 90 "Kitchen area" \
    --output waypoints.yaml

# Load existing waypoints and add more
python3 scripts/map_tools/map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --add "hallway" 0.0 0.0 0 \
    --output waypoints.yaml

# Edit existing waypoint
python3 scripts/map_tools/map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --edit "kitchen" x=2.6 y=1.1 yaw_degrees=85 \
    --output waypoints.yaml

# Remove a waypoint
python3 scripts/map_tools/map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --remove "old_waypoint" \
    --output waypoints.yaml

# Visualize map with waypoints
python3 scripts/map_tools/map_editor.py --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --visualize
```

**Features:**
- Add/remove/edit waypoints by command line
- Automatic coordinate conversion (world ↔ pixel)
- Quaternion calculation from yaw angle
- Visualization of waypoints on map
- YAML export/import

**Visualization:**
- Red circles show waypoint positions
- Blue arrows show orientation
- Labels show waypoint names

### 3. Waypoint Annotator (`waypoint_annotator.py`)

**Purpose:** Interactive GUI for adding waypoints by clicking on the map

**Basic Usage:**
```bash
# Start interactive annotator
python3 scripts/map_tools/waypoint_annotator.py --map-yaml maps/first_map.yaml

# Load with existing waypoints
python3 scripts/map_tools/waypoint_annotator.py \
    --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml
```

**Interactive Workflow:**
1. **Click #1**: Click on map to set waypoint position (red dot appears)
2. **Click #2**: Click again to set orientation direction (blue arrow)
3. **Name**: Enter waypoint name in dialog box
4. **Repeat**: Add more waypoints
5. **Save**: Click "Save Waypoints" button

**GUI Features:**
- Real-time visualization
- Waypoint list on right panel
- Double-click waypoint to delete
- "Undo Last" button
- "Clear All" button
- Auto-saves to `<map>_waypoints.yaml`

**Tips:**
- The second click determines the direction the robot should face
- Use the grid in map_viewer first to plan positions
- Waypoints are saved in ROS2-compatible YAML format

---

## Waypoint Management

### Waypoint File Format

Waypoints are stored in YAML format:

```yaml
metadata:
  created_by: waypoint_annotator.py
  map_yaml: maps/first_map.yaml
  frame_id: map

waypoints:
  - name: kitchen
    description: Kitchen area waypoint
    position:
      x: 2.5000
      y: 1.0000
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707107    # sin(yaw/2)
      w: 0.707107    # cos(yaw/2)
    yaw_degrees: 90.0
    tolerance:
      position: 0.3      # meters
      orientation: 0.2   # radians

  - name: hallway
    description: Hallway junction
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
    yaw_degrees: 0.0
    tolerance:
      position: 0.3
      orientation: 0.2

routes:
  tour:
    - kitchen
    - hallway
    - kitchen
```

### Coordinate Systems

**Pixel Coordinates:**
- Origin (0, 0) at top-left of image
- X increases right, Y increases down
- Integer values

**World Coordinates:**
- Origin defined by `origin` in map YAML
- X increases right, Y increases up
- In meters, can be negative
- Matches ROS2 coordinate frame

**Conversion Formulas:**
```python
# Pixel to World
world_x = origin_x + (pixel_x * resolution)
world_y = origin_y + ((height - pixel_y) * resolution)

# World to Pixel
pixel_x = (world_x - origin_x) / resolution
pixel_y = height - ((world_y - origin_y) / resolution)
```

### Orientation (Yaw)

**Degrees vs Radians:**
- Tools accept degrees (0-360)
- ROS2 uses radians and quaternions
- 0° = East, 90° = North, 180° = West, 270° = South

**Yaw to Quaternion:**
```python
yaw_rad = math.radians(yaw_degrees)
qz = math.sin(yaw_rad / 2)
qw = math.cos(yaw_rad / 2)
# qx = 0, qy = 0 for 2D rotation
```

**Common Directions:**
- East (→): 0°
- North (↑): 90°
- West (←): 180° or -180°
- South (↓): 270° or -90°

---

## RViz Visualization

### Publishing Waypoint Markers

Create a ROS2 node to publish waypoint markers for visualization in RViz:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml

class WaypointMarkerPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_marker_publisher')

        self.publisher = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_markers)
        self.waypoints = self.load_waypoints('waypoints.yaml')

    def load_waypoints(self, filepath):
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', [])

    def publish_markers(self):
        marker_array = MarkerArray()

        for i, wp in enumerate(self.waypoints):
            # Position marker (sphere)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = wp['position']['x']
            marker.pose.position.y = wp['position']['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

            # Orientation arrow
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "waypoints"
            arrow.id = i * 2 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            arrow.pose.position.x = wp['position']['x']
            arrow.pose.position.y = wp['position']['y']
            arrow.pose.position.z = 0.0
            arrow.pose.orientation.x = wp['orientation']['x']
            arrow.pose.orientation.y = wp['orientation']['y']
            arrow.pose.orientation.z = wp['orientation']['z']
            arrow.pose.orientation.w = wp['orientation']['w']

            arrow.scale.x = 0.5  # Length
            arrow.scale.y = 0.05  # Width
            arrow.scale.z = 0.05  # Height

            arrow.color.r = 0.0
            arrow.color.g = 0.0
            arrow.color.b = 1.0
            arrow.color.a = 1.0

            marker_array.markers.append(arrow)

            # Text label
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "waypoint_labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = wp['position']['x']
            text.pose.position.y = wp['position']['y']
            text.pose.position.z = 0.5

            text.text = wp['name']
            text.scale.z = 0.3

            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0

            marker_array.markers.append(text)

        self.publisher.publish(marker_array)

def main():
    rclpy.init()
    node = WaypointMarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### RViz Configuration

1. **Add Marker Display:**
   - Click "Add" in RViz
   - Select "MarkerArray"
   - Set topic to `/waypoint_markers`

2. **Configure Display:**
   - Ensure "Fixed Frame" is set to "map"
   - Check that markers are visible

3. **Save Configuration:**
   - File → Save Config As
   - Save as `waypoint_viz.rviz`

### Interactive Markers (Advanced)

For interactive waypoint editing in RViz:

```python
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl

# Create interactive marker server
server = InteractiveMarkerServer(self, 'waypoint_editor')

# Create interactive marker
int_marker = InteractiveMarker()
int_marker.header.frame_id = "map"
int_marker.name = wp['name']
int_marker.description = wp['name']
int_marker.pose.position.x = wp['position']['x']
int_marker.pose.position.y = wp['position']['y']

# Add 6-DOF control
control = InteractiveMarkerControl()
control.orientation.w = 1
control.orientation.x = 0
control.orientation.y = 1
control.orientation.z = 0
control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
int_marker.controls.append(control)

server.insert(int_marker, feedback_callback)
server.applyChanges()
```

---

## Integration Guide

### Integration with Existing waypoint_manager.py

The existing `waypoint_manager.py` can be used alongside the new tools:

```bash
# Use existing waypoint_manager for batch operations
python3 ros2_cartography_attempt/waypoint_manager.py \
    --map-yaml maps/first_map.yaml \
    --add-center \
    --add-corners \
    --output waypoints.yaml

# Then use waypoint_annotator to add custom waypoints
python3 scripts/map_tools/waypoint_annotator.py \
    --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml

# Visualize the result
python3 scripts/map_tools/map_editor.py \
    --map-yaml maps/first_map.yaml \
    --waypoints waypoints.yaml \
    --visualize
```

### Workflow Recommendations

**1. Initial Map Creation:**
```bash
# Create map with SLAM
ros2 launch slam_toolbox online_async_launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f my_map
```

**2. Map Inspection:**
```bash
# View and analyze map
python3 scripts/map_tools/map_viewer.py --map-yaml my_map.yaml --info --grid
```

**3. Waypoint Planning:**
```bash
# Export grid of free space points
python3 scripts/map_tools/map_viewer.py \
    --map-yaml my_map.yaml \
    --export-grid 0.5

# Review grid_points.yaml to plan waypoint locations
```

**4. Waypoint Creation:**
```bash
# Interactive annotation (recommended)
python3 scripts/map_tools/waypoint_annotator.py --map-yaml my_map.yaml

# OR programmatic addition
python3 scripts/map_tools/map_editor.py \
    --map-yaml my_map.yaml \
    --add "goal1" 2.0 1.0 90 "First goal"
```

**5. Verification:**
```bash
# Visualize waypoints on map
python3 scripts/map_tools/map_editor.py \
    --map-yaml my_map.yaml \
    --waypoints my_map_waypoints.yaml \
    --visualize

# List all waypoints
python3 scripts/map_tools/map_editor.py \
    --map-yaml my_map.yaml \
    --waypoints my_map_waypoints.yaml \
    --list
```

**6. Navigation Testing:**
```bash
# Publish markers for RViz
python3 waypoint_marker_publisher.py

# Use with Nav2
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, \
    pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
    orientation: {z: 0.707, w: 0.707}}}}"
```

### Using with Nav2 Waypoint Follower

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import yaml

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def navigate_waypoints(self, waypoint_file, route_name=None):
        # Load waypoints
        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)

        waypoints = data['waypoints']

        # If route specified, filter waypoints
        if route_name and route_name in data.get('routes', {}):
            route = data['routes'][route_name]
            waypoints = [wp for wp in waypoints if wp['name'] in route]

        # Create goal
        goal_msg = FollowWaypoints.Goal()

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = wp['position']['x']
            pose.pose.position.y = wp['position']['y']
            pose.pose.position.z = wp['position']['z']

            pose.pose.orientation.x = wp['orientation']['x']
            pose.pose.orientation.y = wp['orientation']['y']
            pose.pose.orientation.z = wp['orientation']['z']
            pose.pose.orientation.w = wp['orientation']['w']

            goal_msg.poses.append(pose)

        # Send goal
        self.client.wait_for_server()
        return self.client.send_goal_async(goal_msg)

# Usage
navigator = WaypointNavigator()
navigator.navigate_waypoints('waypoints.yaml', route_name='tour')
```

---

## Troubleshooting

### Common Issues

**1. Waypoints out of bounds:**
```bash
# Check map bounds first
python3 scripts/map_tools/map_viewer.py --map-yaml map.yaml --info

# Ensure waypoints are within bounds:
# min_x ≤ waypoint.x ≤ max_x
# min_y ≤ waypoint.y ≤ max_y
```

**2. Incorrect orientation:**
- Remember: 0° = East, 90° = North
- Use visualizer to verify arrow direction
- ROS2 uses right-hand rule (counter-clockwise positive)

**3. Map not loading in RViz:**
- Ensure map_server is running
- Check topic names (`/map`)
- Verify frame_id is "map"
- Check that YAML path to PGM is correct

**4. Coordinate mismatch:**
- Verify resolution matches between tools
- Check origin is correctly set
- Use map_viewer to validate coordinate conversion

### Best Practices

1. **Always backup maps before editing**
   ```bash
   cp map.pgm map.pgm.backup
   cp map.yaml map.yaml.backup
   ```

2. **Use consistent resolution**
   - Standard: 0.05m/pixel (5cm/pixel) for indoor
   - Use 0.01-0.02 for high detail
   - Use 0.1 for large outdoor areas

3. **Name waypoints descriptively**
   - Use: "kitchen_entrance", "hallway_junction"
   - Avoid: "wp1", "point2"

4. **Test waypoints in simulation first**
   - Use RViz to visualize before deploying
   - Test navigation paths
   - Adjust tolerances as needed

5. **Document waypoint purposes**
   - Use the description field
   - Maintain a separate route list
   - Keep waypoint coordinates in version control

---

## Additional Resources

### ROS2 Documentation
- [Nav2 Documentation](https://docs.nav2.org)
- [Map Server](https://github.com/ros-planning/navigation2/tree/main/nav2_map_server)
- [Waypoint Follower](https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html)

### External Tools
- **GIMP**: https://www.gimp.org
- **ImageMagick**: https://imagemagick.org
- **ROS2 Nav2**: https://github.com/ros-planning/navigation2

### Research Resources
- [ROS Wiki - Map Format](http://wiki.ros.org/map_server#Map_format)
- [Occupancy Grid Mapping](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)
- [PGM Format Specification](http://netpbm.sourceforge.net/doc/pgm.html)

---

## Summary

This guide provided comprehensive coverage of:
- Map file formats (PGM/YAML structure)
- Manual editing techniques (GIMP, ImageMagick, Python)
- Using the three map tools (viewer, editor, annotator)
- Waypoint management and coordinate systems
- RViz visualization with markers
- Integration with existing tools and Nav2

The tools in `scripts/map_tools/` provide a complete workflow for:
1. **Inspecting** maps (map_viewer.py)
2. **Planning** waypoint locations (map_viewer.py with grid)
3. **Adding** waypoints interactively (waypoint_annotator.py)
4. **Managing** waypoints programmatically (map_editor.py)
5. **Visualizing** results (all tools + RViz)

For practical navigation, combine these tools with the existing waypoint_manager.py and ROS2 Nav2 stack for a complete autonomous navigation solution.
