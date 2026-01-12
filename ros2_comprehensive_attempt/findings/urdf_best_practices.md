# URDF Best Practices for Differential Drive Robots

**Document:** URDF Design Patterns and Best Practices
**Date:** 2026-01-11
**Robot:** WayfindR Differential Drive Platform
**Author:** WayfindR Development Team

---

## Table of Contents

1. [URDF Overview](#urdf-overview)
2. [Best Practices for Robot Description](#best-practices-for-robot-description)
3. [Coordinate Frame Conventions](#coordinate-frame-conventions)
4. [TF Tree Structure](#tf-tree-structure)
5. [Inertial Properties](#inertial-properties)
6. [Joint Types and Usage](#joint-types-and-usage)
7. [Gazebo Integration](#gazebo-integration)
8. [Common Pitfalls and Solutions](#common-pitfalls-and-solutions)
9. [WayfindR Robot Specification](#wayfindr-robot-specification)
10. [Testing and Validation](#testing-and-validation)

---

## URDF Overview

### What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based format for representing the physical structure of a robot in ROS. It describes:

- **Links**: Rigid bodies that make up the robot (chassis, wheels, sensors)
- **Joints**: Connections between links (fixed, revolute, continuous, prismatic)
- **Visual geometry**: How the robot appears in visualization tools (RViz, Gazebo)
- **Collision geometry**: Simplified shapes for collision detection
- **Inertial properties**: Mass, center of mass, moment of inertia for physics simulation

### URDF vs XACRO

**XACRO (XML Macros)** is a preprocessor for URDF that adds:

- **Variables/Properties**: Define reusable constants
- **Macros**: Create reusable components (e.g., wheel macro)
- **Mathematical expressions**: Calculate values dynamically
- **Conditionals**: Include/exclude elements based on conditions
- **File inclusion**: Split large descriptions into multiple files

**Best Practice:** Always use XACRO for any non-trivial robot description.

---

## Best Practices for Robot Description

### 1. Modular Design with Macros

**Principle:** Use macros for repeated components.

**Example: Wheel Macro**
```xml
<xacro:macro name="wheel" params="prefix reflect">
  <link name="${prefix}_wheel">
    <!-- Visual, collision, inertial definitions -->
  </link>
  <joint name="${prefix}_wheel_joint" type="continuous">
    <origin xyz="0 ${reflect * wheel_separation / 2} 0" rpy="0 0 0"/>
    <!-- Joint properties -->
  </joint>
</xacro:macro>

<!-- Usage -->
<xacro:wheel prefix="left" reflect="1"/>
<xacro:wheel prefix="right" reflect="-1"/>
```

**Benefits:**
- Reduces code duplication
- Ensures consistency between symmetric components
- Simplifies maintenance (change once, apply everywhere)

### 2. Parameterization with Properties

**Principle:** Define all physical dimensions as properties at the top of the file.

**Example:**
```xml
<!-- Robot dimensions -->
<xacro:property name="base_radius" value="0.1" />
<xacro:property name="wheel_radius" value="0.0325" />
<xacro:property name="wheel_separation" value="0.15" />
<xacro:property name="lidar_offset_z" value="0.1" />
```

**Benefits:**
- Easy to adjust robot dimensions
- Self-documenting code
- Enables design variations
- Simplifies calculations

### 3. Clear Naming Conventions

**Principle:** Use consistent, descriptive names for links and joints.

**Naming Standards:**
- **Links**: Describe the physical component (`base_link`, `left_wheel`, `laser`)
- **Joints**: Link name + `_joint` suffix (`left_wheel_joint`, `laser_joint`)
- **Frames**: Follow ROS conventions (`base_footprint`, `odom`, `map`)

**Avoid:**
- Generic names like `link1`, `link2`
- Inconsistent naming between similar components
- Names that don't describe function

### 4. Separate Visual and Collision Geometry

**Principle:** Visual geometry can be detailed; collision geometry should be simple.

**Visual Geometry:**
- Can use complex meshes (.stl, .dae files)
- For rendering in RViz and Gazebo
- Aesthetics matter

**Collision Geometry:**
- Should use primitive shapes (box, cylinder, sphere) when possible
- Simpler = faster collision detection
- Trade accuracy for performance

**Example:**
```xml
<link name="base_link">
  <visual>
    <geometry>
      <!-- Detailed mesh -->
      <mesh filename="package://my_robot/meshes/chassis.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <!-- Simple approximation -->
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </collision>
</link>
```

### 5. Proper Inertial Properties

**Principle:** Accurate inertial properties are critical for realistic simulation.

**Components:**
- **Mass**: Total mass of the link (kg)
- **Center of mass**: Often at origin, but can be offset
- **Inertia tensor**: 3x3 matrix describing rotational inertia

**Use Macros for Standard Shapes:**
```xml
<!-- Cylinder inertia (common for wheels, base) -->
<xacro:macro name="cylinder_inertia" params="mass radius height">
  <inertia
    ixx="${mass * (3 * radius * radius + height * height) / 12}"
    iyy="${mass * (3 * radius * radius + height * height) / 12}"
    izz="${mass * radius * radius / 2}"
    ixy="0.0" ixz="0.0" iyz="0.0"
  />
</xacro:macro>

<!-- Sphere inertia (common for caster wheels) -->
<xacro:macro name="sphere_inertia" params="mass radius">
  <inertia
    ixx="${2 * mass * radius * radius / 5}"
    iyy="${2 * mass * radius * radius / 5}"
    izz="${2 * mass * radius * radius / 5}"
    ixy="0.0" ixz="0.0" iyz="0.0"
  />
</xacro:macro>

<!-- Box inertia -->
<xacro:macro name="box_inertia" params="mass x y z">
  <inertia
    ixx="${mass * (y*y + z*z) / 12}"
    iyy="${mass * (x*x + z*z) / 12}"
    izz="${mass * (x*x + y*y) / 12}"
    ixy="0.0" ixz="0.0" iyz="0.0"
  />
</xacro:macro>
```

**Important Notes:**
- **Never use zero mass or inertia** - causes simulation instability
- Minimum mass: 0.01 kg (10g) for small components
- Use realistic values based on actual hardware when possible
- For complex shapes, use CAD software to calculate inertia

### 6. Material Definitions

**Principle:** Define materials once, reuse throughout.

**Example:**
```xml
<material name="blue">
  <color rgba="0 0 0.8 1"/>
</material>

<material name="black">
  <color rgba="0 0 0 1"/>
</material>

<!-- Usage in link -->
<link name="base_link">
  <visual>
    <geometry>...</geometry>
    <material name="blue"/>
  </visual>
</link>
```

**Gazebo Materials:**
```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>
```

### 7. Joint Limits and Dynamics

**Principle:** Always specify realistic joint limits and dynamics.

**For Continuous Joints (wheels):**
```xml
<joint name="left_wheel_joint" type="continuous">
  <axis xyz="0 1 0"/>  <!-- Rotation around Y axis -->
  <limit effort="10" velocity="10"/>  <!-- 10 Nm, 10 rad/s -->
  <dynamics damping="0.1" friction="0.1"/>
</joint>
```

**For Revolute Joints (e.g., camera pan/tilt):**
```xml
<joint name="camera_pan_joint" type="revolute">
  <axis xyz="0 0 1"/>  <!-- Rotation around Z axis -->
  <limit lower="-1.57" upper="1.57" effort="1" velocity="1"/>  <!-- ±90 degrees -->
  <dynamics damping="0.5" friction="0.5"/>
</joint>
```

**Parameters:**
- **effort**: Maximum torque (Nm)
- **velocity**: Maximum angular velocity (rad/s)
- **damping**: Resistance to motion (higher = slower response)
- **friction**: Static/dynamic friction

---

## Coordinate Frame Conventions

### ROS REP 103: Standard Units of Measure and Coordinate Conventions

**Reference:** https://www.ros.org/reps/rep-0103.html

**Key Conventions:**

#### Units
- **Length**: meters (m)
- **Mass**: kilograms (kg)
- **Time**: seconds (s)
- **Angle**: radians (rad)
- **Velocity**: m/s or rad/s
- **Force**: Newtons (N)
- **Torque**: Newton-meters (Nm)

#### Coordinate Frame Orientation

**For mobile robots (like WayfindR):**

- **X-axis**: Forward direction (robot's front)
- **Y-axis**: Left direction (robot's left side)
- **Z-axis**: Upward direction (vertical, away from ground)

**Right-hand rule applies:**
- Curl fingers from X to Y
- Thumb points in Z direction

**Rotation about axes:**
- **Roll**: Rotation about X-axis
- **Pitch**: Rotation about Y-axis
- **Yaw**: Rotation about Z-axis

#### ROS Standard Frames

1. **map**: Fixed global frame
   - Origin: Arbitrary point in the world
   - Does not move
   - Used for global path planning

2. **odom**: Odometry frame
   - Origin: Robot's starting position
   - Drifts over time (due to wheel slip, etc.)
   - Used for local navigation

3. **base_link**: Robot's main reference frame
   - Origin: Typically center of rotation
   - Moves with the robot
   - All robot components defined relative to this

4. **base_footprint**: Projection of robot on ground plane
   - Z = 0 at ground level
   - Useful for 2D navigation
   - Often used as parent of base_link

5. **sensor frames**: Individual sensor reference frames
   - Examples: `laser`, `camera`, `imu`
   - Defined relative to base_link

### Sensor Frame Conventions (REP 105)

**Reference:** https://www.ros.org/reps/rep-0105.html

**LiDAR Frame (laser):**
- **X**: Forward (0° beam direction)
- **Y**: Left
- **Z**: Up
- Origin: At sensor's measurement origin

**Camera Frame:**
- **Z**: Forward (optical axis, direction camera is looking)
- **X**: Right (in image plane)
- **Y**: Down (in image plane)
- Origin: At camera lens

**IMU Frame:**
- **X**: Forward
- **Y**: Left
- **Z**: Up
- Origin: At IMU sensor location

---

## TF Tree Structure

### What is TF?

**TF (Transform)** is ROS's system for tracking coordinate frame transformations over time.

**Key Concepts:**
- **Transform**: 3D transformation (translation + rotation) between two frames
- **TF Tree**: Hierarchical structure of all coordinate frames
- **Static Transform**: Never changes (e.g., sensor mounted on robot)
- **Dynamic Transform**: Changes over time (e.g., robot moving in world)

### Standard TF Tree for Differential Drive Robot

```
map (global frame, fixed in world)
 │
 └─ odom (odometry frame, drifts over time)
     │
     └─ base_footprint (ground projection of robot)
         │
         └─ base_link (robot center)
             │
             ├─ laser (LiDAR sensor)
             │
             ├─ left_wheel
             │
             ├─ right_wheel
             │
             ├─ caster_wheel
             │
             └─ imu (if present)
```

### Transform Publishers

**Who publishes what?**

1. **map → odom**: Published by localization system (AMCL, Cartographer)
   - Corrects odometry drift
   - Updated when robot localizes itself on map

2. **odom → base_footprint**: Published by odometry source
   - Wheel encoders + IMU
   - Or Gazebo (in simulation)
   - Or dead reckoning from cmd_vel

3. **base_footprint → base_link**: Static transform publisher
   - Fixed relationship (height of robot center above ground)
   - Usually just a Z-offset

4. **base_link → sensor frames**: Static transform publisher or robot_state_publisher
   - Defined in URDF
   - Published by robot_state_publisher node

### Best Practices for TF

1. **Never have multiple publishers for same transform**
   - Causes conflicts and errors
   - Use `tf2_echo` to debug

2. **Keep tree connected**
   - Every frame must have path to every other frame
   - No disconnected branches

3. **Maintain correct direction**
   - Transforms go from parent to child
   - Cannot have loops in tree

4. **Use appropriate update rates**
   - Static transforms: Low rate (1 Hz) or use `/tf_static`
   - Dynamic transforms: Match sensor rate (10-50 Hz typical)
   - Odometry: 20-50 Hz recommended

5. **Timestamp correctly**
   - Use `use_sim_time` parameter for simulation
   - Ensure all nodes use same time source

### Debugging TF

**Check TF tree structure:**
```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf with TF tree visualization
```

**Monitor specific transform:**
```bash
ros2 run tf2_ros tf2_echo map base_link
# Shows transform from map to base_link
```

**List all frames:**
```bash
ros2 run tf2_ros tf2_monitor
# Shows all transforms and their update rates
```

**Check for problems:**
```bash
ros2 topic echo /tf
ros2 topic echo /tf_static
# Look for missing transforms or errors
```

---

## Inertial Properties

### Why Inertial Properties Matter

**In Simulation (Gazebo):**
- Physics engine needs mass and inertia to simulate realistic motion
- Incorrect values → unrealistic behavior (too fast, too slow, unstable)

**In Real Robot:**
- Some controllers use inertial properties for feedforward control
- Helpful for model-based control algorithms

### Guidelines for Setting Inertial Properties

1. **Measure or estimate actual mass**
   - Weigh components if possible
   - Use manufacturer datasheets
   - Reasonable estimate better than random guess

2. **Calculate inertia for standard shapes**
   - Use macros (cylinder, sphere, box)
   - For complex shapes, use CAD software (SolidWorks, Fusion 360)

3. **Never use zero**
   - Minimum mass: 0.01 kg
   - Zero inertia causes division by zero in physics engine

4. **Sum component masses**
   - Base link: Include all components mounted on chassis
   - Total robot mass should match actual robot

5. **Reasonable defaults for unknown values**
   - Small sensor: 0.1 - 0.5 kg
   - Wheel: 0.05 - 0.2 kg
   - Base chassis: 1 - 5 kg (depending on robot size)

### WayfindR Robot Inertial Properties

**From our URDF:**

| Component | Mass (kg) | Inertia Calculation |
|-----------|-----------|---------------------|
| Base link | 2.0 | Solid cylinder: r=0.1m, h=0.05m |
| Wheel | 0.1 | Solid cylinder: r=0.0325m, h=0.025m |
| Caster | 0.05 | Solid sphere: r=0.015m |
| LiDAR | 0.2 | Solid cylinder: r=0.02m, h=0.04m |

**Total robot mass:** ~2.5 kg (reasonable for small mobile robot)

---

## Joint Types and Usage

### Available Joint Types in URDF

1. **fixed**: No motion, rigidly connected
   - Use for: Sensor mounts, chassis components
   - Example: LiDAR attached to base

2. **revolute**: Rotation around axis with limits
   - Use for: Servo motors, limited rotation joints
   - Example: Camera pan/tilt, robot arm joints
   - Requires: `<limit lower="..." upper="..."/>`

3. **continuous**: Rotation around axis without limits
   - Use for: Wheels, continuous rotation motors
   - Example: Drive wheels
   - No angle limits (can rotate forever)

4. **prismatic**: Linear sliding motion
   - Use for: Linear actuators, elevators
   - Example: Lifting mechanism
   - Requires: `<limit lower="..." upper="..."/>`

5. **floating**: 6-DOF unconstrained motion
   - Rarely used in URDF
   - Mainly for simulation of free-flying objects

6. **planar**: 2D motion in a plane
   - Rarely used
   - Alternative: Use fixed base_footprint

### Joint Selection for Differential Drive Robot

**WayfindR joints:**

| Joint | Type | Reason |
|-------|------|--------|
| base_footprint_joint | fixed | Base always at fixed height above footprint |
| left_wheel_joint | continuous | Wheel rotates continuously |
| right_wheel_joint | continuous | Wheel rotates continuously |
| caster_wheel_joint | fixed* | Passive caster, no actuation |
| laser_joint | fixed | LiDAR mounted rigidly to base |

*Note: Caster could be `continuous` with very low friction for more realistic simulation

### Joint Axis Convention

**Important:** Axis direction affects rotation direction.

**Wheel rotating forward:**
- If axis is `0 1 0` (Y-axis): Positive velocity = forward
- If axis is `0 -1 0` (-Y-axis): Negative velocity = forward

**Standard for differential drive:**
- Left wheel axis: `0 1 0`
- Right wheel axis: `0 1 0`
- Both same direction (controller handles differential)

---

## Gazebo Integration

### Gazebo-Specific URDF Elements

**Material override:**
```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>
```

**Friction coefficients:**
```xml
<gazebo reference="left_wheel">
  <mu1>1.0</mu1>  <!-- Friction in primary direction -->
  <mu2>1.0</mu2>  <!-- Friction in secondary direction -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>100.0</kd>  <!-- Contact damping -->
</gazebo>
```

**Sensor plugins:**
```xml
<gazebo reference="laser">
  <sensor type="ray" name="rplidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <min_angle>0.0</min_angle>
          <max_angle>6.28319</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.15</min>
        <max>12.0</max>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
      <topicName>/scan</topicName>
      <frameName>laser</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### Differential Drive Plugin

**Critical for simulation:**

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <updateRate>50</updateRate>

    <leftJoint>left_wheel_joint</leftJoint>
    <rightJoint>right_wheel_joint</rightJoint>

    <wheelSeparation>${wheel_separation}</wheelSeparation>
    <wheelDiameter>${2 * wheel_radius}</wheelDiameter>

    <commandTopic>/cmd_vel</commandTopic>
    <odometryTopic>/odom</odometryTopic>

    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_footprint</robotBaseFrame>

    <publishWheelTF>true</publishWheelTF>
    <publishOdomTF>true</publishOdomTF>
  </plugin>
</gazebo>
```

**Parameters:**
- **updateRate**: Control loop frequency (Hz)
- **wheelSeparation**: Distance between wheels (m)
- **wheelDiameter**: Wheel diameter (m)
- **commandTopic**: Velocity command topic (default: `/cmd_vel`)
- **odometryTopic**: Odometry output topic (default: `/odom`)
- **odometryFrame**: Parent frame for odometry (usually `odom`)
- **robotBaseFrame**: Child frame for odometry (usually `base_link` or `base_footprint`)

---

## Common Pitfalls and Solutions

### 1. TF Tree Disconnected

**Problem:** Frames not connected in TF tree

**Symptoms:**
- `frame [X] does not exist` errors in RViz
- Navigation fails
- Localization doesn't work

**Solution:**
- Use `ros2 run tf2_tools view_frames` to visualize tree
- Check that all joints have parent and child defined
- Ensure static transforms are published
- Verify robot_state_publisher is running

### 2. Incorrect Wheel Axis

**Problem:** Wheels rotate in wrong direction or wrong axis

**Symptoms:**
- Robot moves backward when commanded forward
- Robot doesn't move or moves erratically

**Solution:**
- Check wheel joint axis definition: `<axis xyz="0 1 0"/>`
- Verify wheel origin and orientation
- Test with: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`

### 3. Missing or Zero Inertia

**Problem:** Physics simulation unstable or crashes

**Symptoms:**
- Gazebo crashes or freezes
- Robot falls through floor
- Unrealistic motion

**Solution:**
- Ensure all links have `<inertial>` block
- Use minimum mass of 0.01 kg
- Use inertia macros for accurate values
- Never use zero for mass or inertia components

### 4. Collision Geometry Too Complex

**Problem:** Simulation runs very slowly

**Symptoms:**
- Low FPS in Gazebo
- High CPU usage
- Delayed response to commands

**Solution:**
- Simplify collision geometry (use primitives: box, cylinder, sphere)
- Visual geometry can be detailed, collision should be simple
- Reduce number of collision elements

### 5. Ground Clearance Issues

**Problem:** Robot floating above ground or sinking through it

**Symptoms:**
- Gap visible between wheels and ground
- Robot falls through floor
- Unstable motion

**Solution:**
- Set `base_footprint` at ground level (z=0)
- Raise `base_link` by `wheel_radius`: `<origin xyz="0 0 ${wheel_radius}" .../>`
- Check wheel and caster positions relative to base_link
- Verify caster is slightly below or at same level as wheels

### 6. LiDAR Height or Orientation Wrong

**Problem:** SLAM doesn't work, obstacles not detected

**Symptoms:**
- Map is blank or distorted
- Obstacles detected at wrong height
- Scan shows ceiling instead of walls

**Solution:**
- Check LiDAR Z-offset: Should be elevated above obstacles
- Verify LiDAR orientation (typically upright, scanning horizontal plane)
- For 2D LiDAR: Scan plane should be parallel to ground
- Use RViz to visualize `/scan` topic and verify beam directions

### 7. Multiple TF Publishers for Same Transform

**Problem:** TF conflicts, navigation fails

**Symptoms:**
- Warning: `TF_REPEATED_DATA ignoring data with redundant timestamp`
- Jittery robot pose in RViz
- Localization unstable

**Solution:**
- Check which nodes publish to `/tf` and `/tf_static`
- Common culprit: Both `robot_state_publisher` and differential drive plugin publishing `odom → base_link`
- Solution: Configure differential drive plugin to publish or not based on setup
- In simulation: Let Gazebo plugin publish `odom → base_link`
- On real robot: Create separate odometry publisher node

### 8. Use Sim Time Not Set Correctly

**Problem:** Timestamps mismatch in simulation

**Symptoms:**
- TF transform timeout errors
- SLAM doesn't work in simulation
- "Message from [X] has a non-fully-specified time" warnings

**Solution:**
- Set `use_sim_time: true` parameter for ALL nodes when using Gazebo
- Launch Gazebo with `use_sim_time:=true` argument
- Verify with: `ros2 param get /node_name use_sim_time`

---

## WayfindR Robot Specification

### Physical Dimensions

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Base** |
| Diameter | 200 mm (0.2 m) | Circular chassis |
| Height | 50 mm (0.05 m) | Main body thickness |
| Mass | 2.0 kg | Includes electronics, battery |
| **Wheels** |
| Diameter | 65 mm (0.065 m) | Drive wheels |
| Radius | 32.5 mm (0.0325 m) | |
| Width | 25 mm (0.025 m) | Wheel thickness |
| Mass | 0.1 kg each | Per wheel |
| Separation | 150 mm (0.15 m) | Between left and right wheels |
| **Caster** |
| Diameter | 30 mm (0.03 m) | Passive rear caster |
| Radius | 15 mm (0.015 m) | |
| Mass | 0.05 kg | |
| Offset | -70 mm (-0.07 m) | Behind robot center |
| **LiDAR** |
| Model | RP LIDAR C1M1 | Slamtec 2D LiDAR |
| Size | 40 x 40 x 40 mm | Approximated as cube/cylinder |
| Mass | 0.2 kg | Including mount |
| Height | 100 mm (0.1 m) | Above base_link center |
| Position | Center (0, 0) | Aligned with robot center |
| Range | 0.15 - 12.0 m | Minimum and maximum range |
| FOV | 360° | Full rotation |

### Calculated Parameters

**Wheel circumference:**
```
C = π × d = π × 0.065 m = 0.204 m
```

**Maximum linear velocity** (assuming 100 RPM = 10.47 rad/s):
```
v = ω × r = 10.47 rad/s × 0.0325 m = 0.34 m/s
```

**Maximum angular velocity** (differential drive):
```
ω = v / (wheel_separation / 2) = 0.34 / 0.075 = 4.53 rad/s
```

**Minimum turning radius:**
```
r_min = wheel_separation / 2 = 0.075 m = 75 mm
```
(Can turn in place)

### Frame Positions (relative to base_link)

| Frame | X (m) | Y (m) | Z (m) | Notes |
|-------|-------|-------|-------|-------|
| base_link | 0 | 0 | 0 | Robot center |
| left_wheel | 0 | 0.075 | 0 | Left side |
| right_wheel | 0 | -0.075 | 0 | Right side |
| caster_wheel | -0.07 | 0 | -0.04 | Behind and below |
| laser | 0 | 0 | 0.125 | Elevated at center |
| base_footprint | 0 | 0 | -0.0325 | Ground level |

---

## Testing and Validation

### 1. URDF Syntax Validation

**Check URDF is valid:**
```bash
check_urdf /path/to/wayfinder_robot.urdf.xacro
```

**Process xacro to URDF:**
```bash
xacro /path/to/wayfinder_robot.urdf.xacro > robot.urdf
check_urdf robot.urdf
```

**Expected output:**
```
robot name is: wayfinder
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  left_wheel
        child(2):  right_wheel
        child(3):  caster_wheel
        child(4):  laser
```

### 2. Visualize in RViz

**Launch robot state publisher:**
```bash
ros2 launch robot_state_publisher.launch.py use_rviz:=true
```

**In RViz:**
- Add RobotModel display
- Set Fixed Frame to `base_link`
- Verify robot appears correctly
- Check all links are visible
- Rotate view to inspect from all angles

**Check TF tree:**
- Add TF display
- Verify all frames are present
- Check frame orientations (axes should follow REP 103)

### 3. Test in Gazebo

**Create simple Gazebo world:**
```bash
gazebo --verbose
```

**Spawn robot:**
```bash
ros2 run gazebo_ros spawn_entity.py -file robot.urdf -entity wayfinder_robot
```

**Send test commands:**
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# Rotate in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

**Verify:**
- Robot moves forward when commanded
- Robot rotates when commanded
- Wheels rotate correctly
- No instability or jittering
- LiDAR publishes to `/scan` topic

### 4. TF Verification

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# Open frames.pdf to see tree structure
```

**Monitor transforms:**
```bash
# Check odom to base_link
ros2 run tf2_ros tf2_echo odom base_link

# Check base_link to laser
ros2 run tf2_ros tf2_echo base_link laser
```

**Verify all expected transforms exist and update:**
- `map → odom` (if localization running)
- `odom → base_footprint`
- `base_footprint → base_link`
- `base_link → laser`
- `base_link → left_wheel`
- `base_link → right_wheel`

### 5. Joint State Verification

**Monitor joint states:**
```bash
ros2 topic echo /joint_states
```

**Expected output:**
```yaml
name: [left_wheel_joint, right_wheel_joint]
position: [0.0, 0.0]  # Wheel angles
velocity: [0.0, 0.0]  # Angular velocities
effort: [0.0, 0.0]    # Torques
```

**While moving:**
- Positions should change
- Velocities should be non-zero
- Effort may show torque values

### 6. Sensor Data Verification

**Check LiDAR scan:**
```bash
ros2 topic echo /scan --once
```

**Verify:**
- `angle_min`, `angle_max` cover 0 to 2π (360°)
- `range_min` = 0.15 m
- `range_max` = 12.0 m
- `ranges` array has 360 elements
- Valid ranges for detected objects

**Visualize in RViz:**
- Add LaserScan display
- Topic: `/scan`
- Verify scan shows environment correctly

### 7. Odometry Verification

**Check odometry topic:**
```bash
ros2 topic echo /odom
```

**Send movement command and monitor odometry:**
```bash
# In terminal 1: Monitor odometry
ros2 topic echo /odom --field pose.pose.position

# In terminal 2: Send command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --rate 10
```

**Verify:**
- Position X increases when moving forward
- Position Y changes when rotating
- Orientation (quaternion) changes with rotation
- Velocity in twist matches command

### 8. Performance Benchmarks

**Expected performance in Gazebo:**

| Metric | Target | Acceptable |
|--------|--------|------------|
| Update rate | 50 Hz | > 20 Hz |
| TF latency | < 20 ms | < 100 ms |
| Command response | < 50 ms | < 200 ms |
| Real-time factor | 1.0 | > 0.5 |

**Monitor performance:**
```bash
# Check update rates
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /joint_states

# Check Gazebo performance
gazebo --verbose  # Look for real-time factor in output
```

---

## Additional Resources

### Official ROS Documentation

- **URDF Tutorials**: http://wiki.ros.org/urdf/Tutorials
- **URDF XML Specification**: http://wiki.ros.org/urdf/XML
- **XACRO Documentation**: http://wiki.ros.org/xacro
- **REP 103 (Coordinate Frames)**: https://www.ros.org/reps/rep-0103.html
- **REP 105 (Mobile Robot Frames)**: https://www.ros.org/reps/rep-0105.html
- **TF2 Documentation**: http://wiki.ros.org/tf2

### Gazebo Integration

- **Gazebo ROS2 Integration**: https://github.com/ros-simulation/gazebo_ros_pkgs
- **Gazebo Tutorial: Using URDF**: http://gazebosim.org/tutorials?tut=ros_urdf
- **Differential Drive Plugin**: http://gazebosim.org/tutorials?tut=ros_gzplugins#DifferentialDrive

### Tools and Utilities

- **check_urdf**: Validates URDF syntax
- **urdf_to_graphiz**: Visualizes URDF structure
- **xacro**: Processes xacro macros to URDF
- **tf2_tools**: TF tree visualization and debugging
- **RViz**: Robot visualization
- **Gazebo**: Physics simulation

### Example URDFs

- **TurtleBot3**: https://github.com/ROBOTIS-GIT/turtlebot3
- **Fetch Robot**: https://github.com/fetchrobotics/fetch_ros
- **Mobile Base Examples**: https://github.com/ros-planning/navigation2/tree/main/nav2_bringup

---

## Conclusion

This document provides comprehensive best practices for creating URDF robot descriptions, specifically tailored for differential drive robots like WayfindR. Key takeaways:

1. **Use XACRO** for modularity and maintainability
2. **Follow REP 103/105** for coordinate frame conventions
3. **Set realistic inertial properties** for accurate simulation
4. **Keep TF tree simple and connected**
5. **Test thoroughly** using check_urdf, RViz, and Gazebo
6. **Document your design** for future reference and debugging

The WayfindR URDF implementation follows all these best practices, providing a solid foundation for navigation, SLAM, and autonomous operation.

---

**Last Updated:** 2026-01-11
**Author:** WayfindR Development Team
**Version:** 1.0.0
