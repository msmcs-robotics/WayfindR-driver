# WayfindR Robot URDF Description

This directory contains the URDF (Unified Robot Description Format) files for the WayfindR differential drive robot.

---

## Files

### `wayfinder_robot.urdf.xacro`

Complete robot description using XACRO (XML Macros) for the WayfindR platform.

**Contents:**
- Base link (chassis)
- Two drive wheels (differential drive)
- One caster wheel (passive rear support)
- RP LIDAR C1M1 sensor
- Inertial properties for all links
- Gazebo simulation plugins

---

## Robot Specifications

### Physical Dimensions

| Component | Value | Notes |
|-----------|-------|-------|
| **Base** | Ø200mm × 50mm | Circular chassis |
| **Base Mass** | 2.0 kg | Including electronics |
| **Wheels** | Ø65mm × 25mm | Standard small robot wheels |
| **Wheel Mass** | 0.1 kg each | Per wheel |
| **Wheel Separation** | 150mm | Distance between left/right |
| **Caster** | Ø30mm | Passive support |
| **Caster Mass** | 0.05 kg | Small ball caster |
| **LiDAR** | 40×40×40mm | RP LIDAR C1M1 |
| **LiDAR Mass** | 0.2 kg | Including mount |
| **LiDAR Height** | 100mm above base | Mounted at center |

### Performance

| Parameter | Value |
|-----------|-------|
| **Total Mass** | ~2.5 kg |
| **Wheel Circumference** | 204 mm |
| **Max Linear Velocity** | 0.34 m/s |
| **Max Angular Velocity** | 4.53 rad/s |
| **Minimum Turning Radius** | 75 mm (can turn in place) |
| **LiDAR Range** | 0.15 - 12.0 m |
| **LiDAR FOV** | 360° |

---

## Coordinate Frames

The URDF defines the following coordinate frames:

```
base_footprint (ground level, Z=0)
 └─ base_link (robot center, Z=wheel_radius)
     ├─ laser (LiDAR sensor, +100mm above base_link)
     ├─ left_wheel (+75mm left of center)
     ├─ right_wheel (+75mm right of center)
     └─ caster_wheel (-70mm behind center, -40mm below base)
```

### Frame Details

| Frame | Position (X, Y, Z) | Orientation | Type |
|-------|-------------------|-------------|------|
| base_footprint | 0, 0, 0 | 0, 0, 0 | Root |
| base_link | 0, 0, 0.0325 | 0, 0, 0 | Fixed to footprint |
| laser | 0, 0, 0.125 | 0, 0, 0 | Fixed to base_link |
| left_wheel | 0, 0.075, 0 | 0, 0, 0 | Continuous rotation |
| right_wheel | 0, -0.075, 0 | 0, 0, 0 | Continuous rotation |
| caster_wheel | -0.07, 0, -0.04 | 0, 0, 0 | Fixed to base_link |

**Note:** All distances in meters, angles in radians.

---

## Usage

### 1. Process XACRO to URDF

```bash
# Generate URDF from XACRO
xacro wayfinder_robot.urdf.xacro > wayfinder_robot.urdf
```

### 2. Validate URDF

```bash
# Check URDF syntax
check_urdf wayfinder_robot.urdf

# Or use the validation script
cd ../scripts
./validate_urdf.sh
```

### 3. Visualize in RViz

```bash
# Launch robot state publisher with RViz
ros2 launch ../launch/robot_state_publisher.launch.py use_rviz:=true

# Or use the validation script
cd ../scripts
./validate_urdf.sh --visualize
```

### 4. Use in Gazebo

```bash
# The URDF includes Gazebo plugins and can be spawned directly
gazebo
# Then in another terminal:
ros2 run gazebo_ros spawn_entity.py \
    -file wayfinder_robot.urdf \
    -entity wayfinder_robot
```

---

## Integration with Navigation Stack

### In SLAM Launch File

```python
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/path/to/urdf/wayfinder_robot.urdf.xacro'
    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # ... rest of SLAM nodes

    return LaunchDescription([robot_state_publisher, ...])
```

### In Localization Launch File

Same pattern - include robot_state_publisher node to publish TF transforms from URDF.

### In Navigation Launch File

```python
# Include robot description
# Configure Nav2 to use correct frames
nav2_params = {
    'robot_base_frame': 'base_footprint',  # Defined in URDF
    'global_frame': 'map',
    'odom_topic': '/odom',
    # ...
}
```

---

## Modifying the URDF

### Changing Robot Dimensions

All dimensions are defined as properties at the top of the file:

```xml
<!-- Edit these values to change robot size -->
<xacro:property name="base_radius" value="0.1" />     <!-- Base size -->
<xacro:property name="wheel_radius" value="0.0325" /> <!-- Wheel size -->
<xacro:property name="wheel_separation" value="0.15" /> <!-- Width -->
<xacro:property name="lidar_offset_z" value="0.1" />  <!-- LiDAR height -->
```

After editing, regenerate URDF and test:
```bash
xacro wayfinder_robot.urdf.xacro > wayfinder_robot.urdf
check_urdf wayfinder_robot.urdf
```

### Adding Sensors

1. Define sensor link with visual, collision, inertial properties
2. Create joint connecting sensor to base_link
3. Add Gazebo plugin for sensor (if simulating)

Example:
```xml
<!-- Camera link -->
<link name="camera">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<!-- Camera joint -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera"/>
  <origin xyz="0.08 0 0.05" rpy="0 0 0"/>  <!-- Front of robot -->
</joint>
```

---

## Troubleshooting

### URDF Validation Fails

**Problem:** `check_urdf` reports errors

**Solutions:**
1. Check XML syntax (matched tags, proper nesting)
2. Verify all properties are defined before use
3. Ensure all joints have parent and child
4. Check for duplicate link or joint names

### Robot Doesn't Appear in RViz

**Problem:** RobotModel display shows nothing

**Solutions:**
1. Verify robot_state_publisher is running: `ros2 node list`
2. Check robot_description topic: `ros2 topic echo /robot_description --once`
3. Set Fixed Frame to `base_link` in RViz
4. Check TF tree: `ros2 run tf2_tools view_frames`

### Robot Falls Through Floor in Gazebo

**Problem:** Robot drops through ground plane

**Solutions:**
1. Check collision geometry is defined for all links
2. Verify inertial properties (no zero mass)
3. Ensure base_footprint is at ground level (Z=0)
4. Check contact parameters in Gazebo elements

### Wheels Don't Rotate

**Problem:** cmd_vel commands don't move robot

**Solutions:**
1. Verify differential drive plugin is loaded
2. Check joint names in plugin match URDF
3. Verify wheel_separation and wheel_diameter are correct
4. Test with: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`

### TF Errors

**Problem:** "frame does not exist" errors

**Solutions:**
1. Check robot_state_publisher is running
2. Verify URDF has no disconnected links
3. Use `ros2 run tf2_tools view_frames` to debug
4. Ensure all joints connect to base_footprint or base_link

---

## Best Practices

### When Modifying URDF

1. **Always validate** after changes: `check_urdf file.urdf`
2. **Test in RViz** before Gazebo (faster iteration)
3. **Commit working versions** to version control
4. **Document changes** in comments
5. **Keep backup** of working URDF

### For Real Robot

1. **Measure actual dimensions** and update URDF
2. **Weigh components** for accurate masses
3. **Test and calibrate**:
   - Wheel separation (affects rotation accuracy)
   - LiDAR position (affects SLAM quality)
   - Sensor orientations

### For Simulation

1. **Use realistic inertia** for stable simulation
2. **Set appropriate friction** for wheels and caster
3. **Configure Gazebo plugins** properly
4. **Match sensor specs** to real hardware

---

## Related Documentation

- **URDF Best Practices**: `../findings/urdf_best_practices.md`
- **TF Tree Guide**: `../findings/tf_tree_and_coordinate_frames.md`
- **Summary**: `../findings/URDF_AND_ROBOT_DESCRIPTION_SUMMARY.md`
- **Launch Files**: `../launch/robot_state_publisher.launch.py`
- **Validation Script**: `../scripts/validate_urdf.sh`

---

## Standards and References

### ROS REPs (ROS Enhancement Proposals)

- **REP 103**: Standard Units of Measure and Coordinate Conventions
  - Units: meters, kilograms, seconds, radians
  - Coordinates: X forward, Y left, Z up

- **REP 105**: Coordinate Frames for Mobile Platforms
  - Frame naming: map, odom, base_link, base_footprint
  - Transform chain: map → odom → base_link

### External Resources

- **URDF Tutorials**: http://wiki.ros.org/urdf/Tutorials
- **URDF XML Spec**: http://wiki.ros.org/urdf/XML
- **XACRO Documentation**: http://wiki.ros.org/xacro
- **Gazebo URDF**: http://gazebosim.org/tutorials?tut=ros_urdf

---

## Version History

**Version 1.0.0** (2026-01-11)
- Initial URDF creation
- Differential drive configuration
- RP LIDAR C1M1 integration
- Gazebo plugins configured
- Full documentation

---

## Contributing

When modifying the URDF:

1. Update version history above
2. Test thoroughly (validation, RViz, Gazebo)
3. Update documentation if changing specifications
4. Ensure backwards compatibility with existing launch files

---

**Last Updated:** 2026-01-11
**Maintainer:** WayfindR Development Team
**ROS2 Version:** Humble
**Status:** Production Ready
