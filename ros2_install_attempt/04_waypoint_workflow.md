# Waypoint Management Workflow

This guide explains how to add waypoints to saved maps and use them for autonomous navigation.

## Overview

Waypoints are specific locations (x, y, theta) on your map that the robot can navigate to. The workflow is:

1. **Create a map** using SLAM (see [05_map_saving_loading.md](05_map_saving_loading.md))
2. **Add waypoints** by clicking on the map in RViz2
3. **Save waypoints** to a YAML file
4. **Load waypoints** and send navigation goals programmatically

## Method 1: Using RViz2 (GUI Method)

This is the easiest method for adding waypoints visually.

### Prerequisites
- A saved map (`.pgm` + `.yaml` files)
- A computer with GUI (not WSL - use another Ubuntu machine or remote desktop)
- RViz2 installed

### Step 1: Launch Map Server

```bash
source ~/.bashrc

# Terminal 1: Load your map
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/maps/my_indoor_map.yaml
```

### Step 2: Launch RViz2

```bash
# Terminal 2: Open RViz2
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Or start with a blank config:
```bash
rviz2
```

### Step 3: Configure RViz2 Displays

In RViz2, add these displays:

1. **Map**
   - Add → By topic → `/map` → Map
   - This shows your occupancy grid

2. **TF**
   - Add → By display type → TF
   - Shows coordinate frames

3. **RobotModel** (optional)
   - Add → By display type → RobotModel
   - Shows your robot if URDF is loaded

### Step 4: Set Initial Pose

Before adding waypoints, set where the robot currently is:

1. Click **"2D Pose Estimate"** tool in RViz2 toolbar
2. Click and drag on the map:
   - Click = position
   - Drag direction = orientation
3. This publishes to `/initialpose` topic

### Step 5: Add Waypoints

Now you can mark waypoints on the map:

1. Click **"2D Nav Goal"** tool in RViz2 toolbar (or **"Publish Point"**)
2. Click on the map where you want a waypoint
3. Drag to set the orientation (direction robot should face)
4. **Record the coordinates** shown in the terminal or RViz status

### Step 6: Record Waypoint Coordinates

When you click, RViz publishes to `/goal_pose`. You can echo this to see coordinates:

```bash
# Terminal 3: Echo goal poses
ros2 topic echo /goal_pose
```

Output will look like:
```yaml
pose:
  position:
    x: 2.5
    y: 1.3
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707
```

**Manually record these** for each waypoint you want to save.

### Step 7: Create Waypoint File

Create `~/waypoints/my_waypoints.yaml`:

```yaml
waypoints:
  - name: "charging_station"
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

  - name: "warehouse_entrance"
    position:
      x: 2.5
      y: 1.3
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707

  - name: "loading_dock"
    position:
      x: 5.8
      y: -2.1
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.383
      w: 0.924
```

## Method 2: Programmatic Waypoint Recording

If you don't have GUI access, you can record waypoints programmatically.

### Step 1: Run Waypoint Recorder Node

Create a Python script to record waypoints from `/goal_pose` topic:

```python
#!/usr/bin/env python3
# waypoint_recorder.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.waypoints = []
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.get_logger().info('Waypoint recorder started. Publish to /goal_pose to add waypoints.')
        self.get_logger().info('Press Ctrl+C to save waypoints to file.')

    def goal_callback(self, msg):
        waypoint = {
            'position': {
                'x': float(msg.pose.position.x),
                'y': float(msg.pose.position.y),
                'z': float(msg.pose.position.z),
            },
            'orientation': {
                'x': float(msg.pose.orientation.x),
                'y': float(msg.pose.orientation.y),
                'z': float(msg.pose.orientation.z),
                'w': float(msg.pose.orientation.w),
            }
        }
        self.waypoints.append(waypoint)
        self.get_logger().info(f'Recorded waypoint {len(self.waypoints)}: '
                             f'x={waypoint["position"]["x"]:.2f}, '
                             f'y={waypoint["position"]["y"]:.2f}')

    def save_waypoints(self, filename):
        data = {'waypoints': self.waypoints}
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {filename}')

def main():
    rclpy.init()
    recorder = WaypointRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        filename = f'{os.path.expanduser("~")}/waypoints/recorded_waypoints.yaml'
        recorder.save_waypoints(filename)
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import os
    main()
```

Save this to your workspace, build, and run:

```bash
chmod +x waypoint_recorder.py
python3 waypoint_recorder.py
```

Then publish waypoints manually:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'},
    pose: {position: {x: 2.0, y: 1.0, z: 0.0},
           orientation: {w: 1.0}}}"
```

## Method 3: Manual Entry (No RViz)

If you know the coordinates from your map, manually create the YAML file:

### Understanding Coordinates

- **Origin**: Bottom-left of your map (or where you started SLAM)
- **X-axis**: Forward/right on the map
- **Y-axis**: Left/up on the map
- **Orientation**: Quaternion (use online calculator or Python)

### Converting Angle to Quaternion

For a robot facing a specific angle (in degrees):

```python
import math

def yaw_to_quaternion(yaw_degrees):
    """Convert yaw angle in degrees to quaternion."""
    yaw_rad = math.radians(yaw_degrees)
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw_rad / 2),
        'w': math.cos(yaw_rad / 2)
    }

# Examples:
# 0° (facing right):   w=1.0, z=0.0
# 90° (facing up):     w=0.707, z=0.707
# 180° (facing left):  w=0.0, z=1.0
# 270° (facing down):  w=0.707, z=-0.707
```

## Waypoint File Format

### Full Example

```yaml
# ~/waypoints/warehouse_waypoints.yaml

map_name: "warehouse_main_floor"
map_file: "/home/pi/maps/warehouse.yaml"
frame_id: "map"

waypoints:
  - name: "home_base"
    description: "Charging station near entrance"
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
    tolerance:
      position: 0.5  # meters
      orientation: 0.3  # radians

  - name: "checkpoint_1"
    description: "First patrol point"
    position:
      x: 3.2
      y: 1.5
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707
    tolerance:
      position: 0.3
      orientation: 0.3

  - name: "inspection_area"
    description: "Equipment inspection zone"
    position:
      x: 5.8
      y: -1.2
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
    tolerance:
      position: 0.2
      orientation: 0.2

routes:
  patrol_route:
    - "home_base"
    - "checkpoint_1"
    - "inspection_area"
    - "home_base"

  delivery_route:
    - "home_base"
    - "inspection_area"
    - "home_base"
```

## Using Waypoints for Navigation

### Load and Navigate to Waypoint

Python script using Nav2 Simple Commander:

```python
#!/usr/bin/env python3
# navigate_to_waypoint.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml

def load_waypoints(filepath):
    """Load waypoints from YAML file."""
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    return data['waypoints']

def create_pose_stamped(waypoint, frame_id='map'):
    """Convert waypoint dict to PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()

    pose.pose.position.x = waypoint['position']['x']
    pose.pose.position.y = waypoint['position']['y']
    pose.pose.position.z = waypoint['position']['z']

    pose.pose.orientation.x = waypoint['orientation']['x']
    pose.pose.orientation.y = waypoint['orientation']['y']
    pose.pose.orientation.z = waypoint['orientation']['z']
    pose.pose.orientation.w = waypoint['orientation']['w']

    return pose

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Load waypoints
    waypoints = load_waypoints('/home/pi/waypoints/warehouse_waypoints.yaml')

    # Navigate to each waypoint
    for wp in waypoints:
        print(f"Navigating to: {wp.get('name', 'unnamed')}")
        goal_pose = create_pose_stamped(wp)
        navigator.goToPose(goal_pose)

        # Wait for navigation to complete
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # Could add progress monitoring here
            rclpy.spin_once(navigator, timeout_sec=0.1)

        result = navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            print(f"✓ Reached {wp.get('name')}")
        else:
            print(f"✗ Failed to reach {wp.get('name')}")

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run Navigation Script

```bash
python3 navigate_to_waypoint.py
```

## Waypoint Patrol Routes

For continuous patrol between waypoints:

```python
def patrol_route(navigator, waypoints, route_name):
    """Execute a patrol route continuously."""
    route = waypoints['routes'][route_name]
    waypoint_list = waypoints['waypoints']

    # Create a map of waypoint names to waypoint data
    wp_map = {wp['name']: wp for wp in waypoint_list}

    while True:
        for wp_name in route:
            wp = wp_map[wp_name]
            print(f"Patrolling to: {wp_name}")

            goal_pose = create_pose_stamped(wp)
            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                rclpy.spin_once(navigator, timeout_sec=0.1)

            # Optional: wait at waypoint
            time.sleep(2.0)
```

## Waypoint Management Best Practices

### 1. Naming Convention
- Use descriptive names: `loading_dock_a` not `waypoint_1`
- Include zone/area: `warehouse_entrance`, `office_desk_5`
- Add direction if needed: `hallway_north_entrance`

### 2. Tolerance Settings
- **Tight spaces**: `position: 0.2` (20cm), `orientation: 0.1` (5.7°)
- **Open areas**: `position: 0.5` (50cm), `orientation: 0.3` (17°)
- **Charging docks**: Very tight tolerances needed

### 3. Organize by Map
```
~/waypoints/
├── warehouse_waypoints.yaml
├── office_floor1_waypoints.yaml
├── office_floor2_waypoints.yaml
└── test_area_waypoints.yaml
```

### 4. Version Control
- Keep waypoints in git repository
- Document changes when updating waypoints
- Test new waypoints before deploying to production

### 5. Safety Waypoints
Always include:
- **Home/Charging station**
- **Emergency stop points**
- **Manual control handoff locations**

## Troubleshooting

### Robot doesn't reach waypoint
- Check waypoint coordinates are on the map
- Verify tolerance settings aren't too tight
- Ensure path to waypoint is clear (no obstacles)

### Wrong orientation
- Double-check quaternion conversion
- Use online quaternion calculator
- Test with simple angles first (0°, 90°, 180°, 270°)

### Can't load waypoint file
```bash
# Verify YAML syntax
python3 -c "import yaml; yaml.safe_load(open('waypoints.yaml'))"
```

### Navigation fails
- Verify `/map` topic is publishing
- Check `/tf` transforms are correct
- Ensure Nav2 is running: `ros2 node list | grep nav`

## Next Steps

1. Create your first waypoint file
2. Test navigation to individual waypoints
3. Create patrol routes
4. Integrate with your fleet management dashboard
5. Add waypoint monitoring and logging

## Resources

- [Nav2 Simple Commander API](https://navigation.ros.org/commander_api/index.html)
- [Quaternion Calculator](https://quaternions.online/)
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)
