# TF Tree and Coordinate Frames for Mobile Robots

**Document:** Transform Trees and Coordinate Systems in ROS2
**Date:** 2026-01-11
**Focus:** Differential Drive Navigation Stack
**Author:** WayfindR Development Team

---

## Table of Contents

1. [Introduction to TF](#introduction-to-tf)
2. [WayfindR TF Tree Structure](#wayfindr-tf-tree-structure)
3. [Frame Responsibilities](#frame-responsibilities)
4. [Transform Publishers](#transform-publishers)
5. [Navigation Stack Integration](#navigation-stack-integration)
6. [Common TF Issues and Debugging](#common-tf-issues-and-debugging)
7. [Best Practices](#best-practices)

---

## Introduction to TF

### What is TF?

**TF (Transform)** is ROS's distributed system for tracking and managing coordinate frame transformations. It allows any node in the system to:

- Query the relationship between any two coordinate frames at any time
- Transform data (points, vectors, poses) between coordinate frames
- Track moving coordinate frames over time

### Why TF is Critical for Navigation

Navigation requires understanding:

1. **Where is the robot in the world?** (map → base_link)
2. **Where are sensors relative to robot?** (base_link → laser)
3. **Where is the goal relative to robot?** (base_link → goal)
4. **How accurate is our position estimate?** (map vs. odom)

Without proper TF tree, navigation components cannot function.

### TF vs TF2

- **TF**: Original implementation (ROS 1)
- **TF2**: Rewritten, more efficient (ROS 1 Hydro+, all ROS2)
- **Key improvements**: Better performance, cleaner API, static transforms

**In ROS2:** Always use TF2 (it's the default).

### Static vs Dynamic Transforms

**Static Transforms:**
- Never change (e.g., sensor mounted on robot)
- Published to `/tf_static` topic (latched)
- Lower rate (1 Hz or one-time)
- Use: `static_transform_publisher` node

**Dynamic Transforms:**
- Change over time (e.g., robot moving)
- Published to `/tf` topic
- Higher rate (10-50 Hz typical)
- Use: Odometry nodes, localization nodes, SLAM nodes

---

## WayfindR TF Tree Structure

### Complete TF Tree

```
map (world-fixed global frame)
 │
 │ [Published by: AMCL or SLAM Toolbox]
 │ [Update rate: 10-20 Hz]
 │ [Purpose: Correct odometry drift using sensor data]
 │
 └─ odom (odometry frame, drifts over time)
     │
     │ [Published by: Odometry node or Gazebo diff_drive plugin]
     │ [Update rate: 20-50 Hz]
     │ [Purpose: Track robot motion via wheel encoders/IMU]
     │
     └─ base_footprint (ground-level projection)
         │
         │ [Published by: static_transform_publisher]
         │ [Static transform: Z = wheel_radius]
         │ [Purpose: Reference point at ground level]
         │
         └─ base_link (robot's main reference frame)
             │
             │ [Published by: robot_state_publisher from URDF]
             │ [Static transforms to all child frames]
             │ [Purpose: Central reference for all robot components]
             │
             ├─ laser (LiDAR sensor frame)
             │   [Offset: X=0, Y=0, Z=0.125m]
             │   [Purpose: Origin of laser scan measurements]
             │
             ├─ left_wheel (left drive wheel)
             │   [Offset: X=0, Y=0.075m, Z=0]
             │   [Purpose: Wheel position for kinematics]
             │
             ├─ right_wheel (right drive wheel)
             │   [Offset: X=0, Y=-0.075m, Z=0]
             │   [Purpose: Wheel position for kinematics]
             │
             └─ caster_wheel (rear caster)
                 [Offset: X=-0.07m, Y=0, Z=-0.04m]
                 [Purpose: Physical model completeness]
```

### Frame Hierarchy Explained

**Level 1: map**
- Top-level global reference
- Fixed in world, does not move
- Origin: Arbitrary point (typically where robot starts or map origin)
- Used by: Global path planner, waypoint manager

**Level 2: odom**
- Child of map
- Tracks robot motion via odometry
- Drifts over time due to wheel slip, encoder errors
- Transform from map corrects this drift

**Level 3: base_footprint**
- Child of odom
- Projection of robot on ground (Z=0 at ground level)
- Useful for 2D navigation (ignores small height changes)
- Common reference for Nav2 stack

**Level 4: base_link**
- Child of base_footprint
- Robot's physical center (typically center of rotation)
- All robot components defined relative to this
- Main frame used by most ROS nodes

**Level 5: Sensor and component frames**
- Children of base_link
- Fixed offsets defined in URDF
- Examples: laser, camera, imu, wheels

---

## Frame Responsibilities

### map Frame

**Purpose:** Global reference frame that doesn't move.

**Characteristics:**
- Origin: Arbitrary point in the world (often top-left corner of map image)
- Coordinates: Global X, Y position in meters
- Rotation: Typically aligned with world (0° = North or East)

**Who uses it:**
- SLAM systems (to build map)
- Global path planner (to plan paths through map)
- Waypoint manager (to store waypoint positions)
- AMCL (to correct odometry using map)

**Published by:**
- **During mapping**: SLAM Toolbox publishes map → odom
- **During localization**: AMCL publishes map → odom
- **During simulation**: Can be identity or published by localization

**When it exists:**
- Only when SLAM or localization is running
- Not present when only using odometry

### odom Frame

**Purpose:** Continuous local frame that tracks robot motion.

**Characteristics:**
- Origin: Where robot was when odometry started
- Drifts over time (unbounded error accumulation)
- Smooth, continuous (no jumps)
- Short-term accuracy: High
- Long-term accuracy: Low (drift)

**Who uses it:**
- Local planner (for short-term path planning)
- Obstacle avoidance (for reactive navigation)
- Velocity controller (for smooth motion)

**Published by:**
- **Real robot**: Odometry publisher (fuses wheel encoders + IMU)
- **Simulation**: Gazebo differential drive plugin
- **Fallback**: Dead reckoning from cmd_vel integration

**Properties:**
- Smooth: Never jumps (even when localization corrects)
- Continuous: Always available (even without map)
- Local: Good for short distances (< 10m)

### base_footprint Frame

**Purpose:** Robot's reference at ground level.

**Characteristics:**
- Z = 0 at ground level
- X, Y same as base_link
- Simplifies 2D navigation

**Who uses it:**
- Nav2 costmaps (for obstacle projection)
- Path planners (for 2D planning)
- Goal positions (for navigation targets)

**Published by:**
- Static transform publisher: odom → base_footprint → base_link

**Why needed:**
- Separates ground-level reference from robot center
- Allows robot center to be above ground (more natural)
- Compatible with Nav2 defaults

### base_link Frame

**Purpose:** Robot's main reference frame.

**Characteristics:**
- Origin: Typically at robot's center of rotation
- X-axis: Forward (robot front)
- Y-axis: Left (robot left side)
- Z-axis: Up (vertical)

**Who uses it:**
- All robot components (sensors, actuators)
- Collision checking
- Visualization (RViz RobotModel)
- Kinematics calculations

**Published by:**
- robot_state_publisher (from URDF joints)

**Important:**
- This is THE reference frame for the robot
- All sensor data should ultimately relate to this frame
- Controller commands are relative to this frame

### laser Frame (LiDAR)

**Purpose:** Origin and orientation of laser scanner.

**Characteristics:**
- Origin: At laser's measurement origin
- X-axis: 0° beam direction (forward)
- Y-axis: Left
- Z-axis: Up (scan plane perpendicular)

**Who uses it:**
- SLAM algorithms (to process scan data)
- Obstacle detection (to transform scans to map)
- Collision avoidance (to detect nearby obstacles)

**Published by:**
- robot_state_publisher (static transform from base_link)

**Critical:**
- Must be accurate (affects map quality)
- Height above ground determines what is seen
- Orientation affects scan interpretation

---

## Transform Publishers

### 1. robot_state_publisher

**Publishes:** base_link → sensor frames (from URDF)

**Source:** Reads URDF file, publishes all static and joint-based transforms

**Launch:**
```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
)
```

**Publishes to:** `/tf_static` (for fixed joints), `/tf` (for moving joints)

**Critical:** Must be running for robot model to appear in RViz

### 2. static_transform_publisher

**Publishes:** Any static transform not in URDF

**Common uses:**
- odom → base_footprint (when no real odometry)
- base_footprint → base_link (can be in URDF instead)

**Command line:**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.0325 0 0 0 odom base_footprint
# Args: x y z yaw pitch roll parent_frame child_frame
```

**Launch file:**
```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.0325', '0', '0', '0', 'odom', 'base_footprint']
)
```

### 3. Odometry Publisher

**Publishes:** odom → base_footprint (or base_link)

**Options:**

**A) Real Robot - Wheel Encoders + IMU:**
```python
class OdometryPublisher(Node):
    def __init__(self):
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # Subscribe to encoder and IMU data
        # Calculate position and velocity
        # Publish odometry message and TF

    def publish_odometry(self, x, y, theta, vx, vy, vth):
        # Create Odometry message
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        # ... set orientation, velocity, covariance

        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = quaternion_from_euler(0, 0, theta)

        self.tf_broadcaster.sendTransform(t)
```

**B) Simulation - Gazebo Plugin:**
```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_footprint</robotBaseFrame>
    <publishOdomTF>true</publishOdomTF>
  </plugin>
</gazebo>
```

**C) Fallback - Dead Reckoning:**
- Subscribe to `/cmd_vel`
- Integrate velocity commands over time
- Publish estimated position
- Low accuracy but better than nothing

### 4. SLAM Toolbox

**Publishes:** map → odom

**Purpose:** Build map and localize simultaneously

**Configuration:**
```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
```

**Behavior:**
- Reads `/scan` topic (laser data)
- Reads `odom → base_footprint` transform
- Builds map
- Corrects odometry drift by publishing `map → odom`

**When active:** During mapping phase

### 5. AMCL (Adaptive Monte Carlo Localization)

**Publishes:** map → odom

**Purpose:** Localize robot on existing map

**Configuration:**
```yaml
amcl:
  ros__parameters:
    odom_frame_id: odom
    base_frame_id: base_footprint
    global_frame_id: map
    scan_topic: /scan
```

**Behavior:**
- Loads pre-built map
- Reads `/scan` topic
- Reads `odom → base_footprint` transform
- Estimates robot position in map
- Corrects odometry by publishing `map → odom`

**When active:** During localization/navigation phase

---

## Navigation Stack Integration

### Nav2 Frame Requirements

**Nav2 expects this TF tree:**

```
map → odom → base_footprint → base_link → sensors
```

**Configuration (nav2_params.yaml):**
```yaml
controller_server:
  ros__parameters:
    odom_topic: /odom
    robot_base_frame: base_footprint

planner_server:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_footprint

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_footprint

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
```

### Frame Usage by Nav2 Components

**Global Costmap:**
- Frame: `map`
- Uses static map layer
- Plans global paths in map frame
- Requires: map → odom transform

**Local Costmap:**
- Frame: `odom`
- Uses obstacle detection layer
- Plans local paths in odom frame
- Smooth (no jumps when map → odom updates)
- Requires: odom → base_footprint transform

**Controller:**
- Frame: `base_footprint` or `base_link`
- Generates velocity commands
- Requires: Path in odom frame, robot pose in map frame

**Planner:**
- Frame: `map`
- Computes global path from start to goal
- Requires: map → odom → base_footprint chain

### Transform Lookup Example

**Nav2 needs to know:** Where is the robot in the map?

**Query:**
```python
# Lookup transform from map to base_link
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

try:
    transform = tf_buffer.lookup_transform(
        'map',              # Target frame
        'base_link',        # Source frame
        rclpy.time.Time(),  # Get latest available
        timeout=Duration(seconds=1.0)
    )
    # transform.transform.translation gives robot position in map
    # transform.transform.rotation gives robot orientation in map
except TransformException as ex:
    node.get_logger().error(f'Could not transform: {ex}')
```

**TF automatically computes:**
```
map → odom (from AMCL)
odom → base_footprint (from odometry)
base_footprint → base_link (static)

Combined: map → base_link
```

---

## Common TF Issues and Debugging

### Issue 1: Frame Does Not Exist

**Error:**
```
[WARN] Could not transform from 'map' to 'base_link': frame 'map' does not exist
```

**Cause:** Required transform publisher not running

**Solutions:**
1. Check if AMCL or SLAM is running (publishes map → odom)
2. Check if odometry publisher is running (publishes odom → base_footprint)
3. Check if robot_state_publisher is running (publishes base_footprint → base_link)

**Debug:**
```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf - shows which frames exist
```

### Issue 2: Transform Timeout

**Error:**
```
[WARN] Could not get transform from 'odom' to 'base_link': lookup would require extrapolation into the past
```

**Cause:** Transform not publishing fast enough or timing issue

**Solutions:**
1. Check publish rate: `ros2 topic hz /tf`
2. Increase timeout in lookup
3. Verify `use_sim_time` parameter matches (simulation vs real)
4. Check for clock synchronization

**Debug:**
```bash
ros2 topic hz /tf
ros2 topic hz /odom
ros2 run tf2_ros tf2_monitor
```

### Issue 3: Multiple Publishers for Same Transform

**Warning:**
```
TF_REPEATED_DATA ignoring data with redundant timestamp for frame odom
```

**Cause:** Two nodes publishing same transform

**Common culprits:**
- Both Gazebo plugin and separate odometry node publishing odom → base_link
- Both robot_state_publisher and manual static_transform_publisher

**Solution:**
- Identify duplicate publishers: `ros2 node list`, check each node's published topics
- Disable one publisher
- In simulation: Let Gazebo handle it
- On real robot: Use separate odometry node

**Debug:**
```bash
ros2 topic info /tf
ros2 topic info /tf_static
# Check which nodes are publishing
```

### Issue 4: Incorrect Frame Parent/Child

**Symptom:** TF tree disconnected, frames floating

**Cause:** Wrong parent or child in transform

**Solution:**
- Verify TF tree structure matches expected hierarchy
- Check URDF joint definitions
- Check static_transform_publisher arguments (order: parent then child)

**Debug:**
```bash
ros2 run tf2_tools view_frames
# Check tree structure in frames.pdf
```

### Issue 5: Use Sim Time Mismatch

**Error (in simulation):**
```
Lookup would require extrapolation into the future
```

**Cause:** Some nodes using wall time, some using sim time

**Solution:**
Set `use_sim_time: true` for ALL nodes when using Gazebo:

```python
# In launch file
parameters=[{'use_sim_time': True}]
```

**Debug:**
```bash
ros2 param get /node_name use_sim_time
# Check each node
```

### Issue 6: Transforms Not Updating

**Symptom:** Robot appears frozen in RViz

**Cause:** Transform publisher stopped or paused

**Solutions:**
1. Check if publisher node crashed: `ros2 node list`
2. Check topic activity: `ros2 topic hz /odom`
3. Restart publisher node

**Debug:**
```bash
ros2 topic echo /odom --once
ros2 topic echo /tf --once
# Verify data is being published
```

---

## Best Practices

### 1. Follow Standard Frame Hierarchy

**Always use:**
```
map → odom → base_footprint → base_link → sensors
```

**Why:**
- Nav2 expects this structure
- Standard in ROS community
- Most tools and tutorials assume this

### 2. Never Break the Chain

**Rule:** Every frame must have exactly one parent.

**Bad:**
```
map → odom → base_link
map → base_link  (ERROR: base_link has two parents!)
```

**Good:**
```
map → odom → base_link
```

### 3. Publish at Appropriate Rates

**Guidelines:**

| Transform | Rate | Publisher |
|-----------|------|-----------|
| map → odom | 10-20 Hz | AMCL/SLAM |
| odom → base_footprint | 20-50 Hz | Odometry |
| base_footprint → base_link | Static | robot_state_publisher |
| base_link → sensors | Static | robot_state_publisher |

**Why:**
- Too slow: Localization lags, navigation suffers
- Too fast: Unnecessary CPU/network load
- Match sensor rates (e.g., LiDAR at 10 Hz → localization at 10-20 Hz)

### 4. Use use_sim_time Correctly

**Rule:** All nodes must agree on time source.

**In simulation:**
```python
parameters=[{'use_sim_time': True}]
```

**On real robot:**
```python
parameters=[{'use_sim_time': False}]
```

**Or use launch argument:**
```python
use_sim_time = LaunchConfiguration('use_sim_time')
parameters=[{'use_sim_time': use_sim_time}]
```

### 5. Separate Static and Dynamic Transforms

**Static transforms:**
- Publish to `/tf_static` (latched topic)
- Use static_transform_publisher or URDF
- Low rate (1 Hz or one-time)

**Dynamic transforms:**
- Publish to `/tf`
- Match sensor/odometry rate
- Never publish static transforms here (wastes bandwidth)

### 6. Validate TF Tree Regularly

**During development:**
```bash
ros2 run tf2_tools view_frames
```

**In launch file (for debugging):**
```python
Node(
    package='tf2_ros',
    executable='echo',
    arguments=['map', 'base_link'],
    output='screen'
)
```

**In RViz:**
- Add TF display
- Enable "Show Axes" for all frames
- Verify orientations match REP 103

### 7. Handle Missing Transforms Gracefully

**In code:**
```python
try:
    transform = tf_buffer.lookup_transform(
        target_frame,
        source_frame,
        rclpy.time.Time(),
        timeout=Duration(seconds=1.0)
    )
except TransformException as ex:
    self.get_logger().warn(f'Transform not available: {ex}')
    return None  # Handle gracefully
```

**Don't:**
- Let exceptions crash your node
- Assume transforms are always available
- Use infinite timeout (causes hanging)

### 8. Document Your TF Tree

**In README or documentation:**
- Diagram of expected TF tree
- List of publishers for each transform
- Required nodes for complete tree
- Troubleshooting guide for common issues

**Example:**
```markdown
## TF Tree Structure

map → odom → base_footprint → base_link → laser

Publishers:
- map → odom: AMCL (amcl node)
- odom → base_footprint: Odometry (odometry_publisher node)
- base_footprint → base_link: Static (robot_state_publisher)
- base_link → laser: Static (robot_state_publisher from URDF)
```

---

## WayfindR TF Configuration

### Launch File Setup

**SLAM Mode (Mapping):**
```python
# slam.launch.py

# 1. Robot state publisher (base_link → sensors)
robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
)

# 2. Static transform (odom → base_footprint) - placeholder for real odometry
static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.0325', '0', '0', '0', 'odom', 'base_footprint']
)

# 3. SLAM Toolbox (publishes map → odom)
slam_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    parameters=[slam_config]
)

# Result: map → odom → base_footprint → base_link → laser
```

**Localization Mode (Navigation):**
```python
# localization.launch.py

# 1. Robot state publisher
robot_state_publisher_node = Node(...)  # Same as above

# 2. Odometry publisher (odom → base_footprint)
# Replace static TF with real odometry
odometry_node = Node(
    package='my_robot',
    executable='odometry_publisher'
)

# 3. Map server (loads map)
map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    parameters=[{'yaml_filename': map_file}]
)

# 4. AMCL (publishes map → odom)
amcl_node = Node(
    package='nav2_amcl',
    executable='amcl',
    parameters=[amcl_config]
)

# Result: map → odom → base_footprint → base_link → laser
```

### Verification Checklist

**Before running navigation:**

- [ ] robot_state_publisher is running
- [ ] URDF is loaded (check /robot_description topic)
- [ ] base_link → laser transform exists
- [ ] Odometry is publishing to /odom
- [ ] odom → base_footprint transform exists
- [ ] AMCL or SLAM is running
- [ ] map → odom transform exists
- [ ] All frames update at appropriate rates
- [ ] No TF errors in logs
- [ ] TF tree is fully connected (view_frames shows single tree)

---

## Conclusion

Understanding and correctly configuring the TF tree is fundamental to successful navigation in ROS2. The WayfindR robot follows standard conventions and best practices to ensure compatibility with Nav2 and other ROS packages.

**Key takeaways:**

1. TF tree must follow standard hierarchy: map → odom → base_footprint → base_link
2. Each transform has a specific publisher and purpose
3. Debugging TF issues is critical skill for ROS development
4. Follow REP 103 and REP 105 for coordinate frame conventions
5. Test TF tree thoroughly before deploying navigation

The provided URDF and launch files implement these best practices, providing a solid foundation for autonomous navigation.

---

**References:**

- REP 103: Standard Units and Coordinate Conventions: https://www.ros.org/reps/rep-0103.html
- REP 105: Coordinate Frames for Mobile Platforms: https://www.ros.org/reps/rep-0105.html
- TF2 Tutorials: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- Nav2 Coordinate Frames: https://navigation.ros.org/setup_guides/transformation/setup_transforms.html

---

**Last Updated:** 2026-01-11
**Author:** WayfindR Development Team
**Version:** 1.0.0
