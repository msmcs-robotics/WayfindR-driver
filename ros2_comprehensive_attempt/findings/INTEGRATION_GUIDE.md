# URDF Integration Guide for WayfindR Navigation Stack

**Document:** Integrating Robot Description with Existing Navigation Stack
**Date:** 2026-01-11
**Purpose:** Step-by-step guide to update launch files with URDF support
**Author:** WayfindR Development Team

---

## Overview

This guide explains how to integrate the newly created URDF robot description with the existing WayfindR navigation stack (SLAM, localization, and navigation launch files).

---

## Table of Contents

1. [Current State](#current-state)
2. [Integration Steps](#integration-steps)
3. [Modified Launch Files](#modified-launch-files)
4. [Testing Integration](#testing-integration)
5. [Troubleshooting](#troubleshooting)

---

## Current State

### Existing Launch Files

The current navigation stack has these launch files:

- `launch/slam.launch.py` - SLAM mapping with static TF
- `launch/localization.launch.py` - AMCL localization with static TF
- `launch/lidar.launch.py` - LiDAR only (mentioned but not created)

### Current TF Publishing

**In SLAM mode:**
```python
# Static TF: odom → base_link (identity transform)
static_tf_odom_base = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
)

# Static TF: base_link → laser
static_tf_base_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
)
```

### Issues with Current Approach

1. **No robot model** - RViz can't display robot structure
2. **Manual TF management** - Error-prone, hard to maintain
3. **Duplicated parameters** - LiDAR height defined in multiple places
4. **No wheel information** - Can't integrate odometry later
5. **Not scalable** - Adding sensors requires editing multiple files

---

## Integration Steps

### Step 1: Update slam.launch.py

**Add robot_state_publisher to SLAM launch file**

**Location:** `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/launch/slam.launch.py`

**Changes:**

1. **Import robot description utilities:**

```python
import os
from launch.substitutions import Command, PathJoinSubstitution
```

2. **Define URDF path:**

```python
def generate_launch_description():
    # Get paths
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_file = os.path.join(pkg_dir, 'urdf', 'wayfinder_robot.urdf.xacro')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    # ... other paths
```

3. **Process XACRO to robot_description:**

```python
    # Process URDF/XACRO
    robot_description = Command(['xacro ', urdf_file])
```

4. **Add robot_state_publisher node:**

```python
    # Robot State Publisher
    # Replaces static TF publishers for base_link → laser
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )
```

5. **Update static TF publisher (only odom → base_footprint):**

```python
    # Static TF: odom → base_footprint
    # Note: Changed from base_link to base_footprint (URDF defines base_footprint → base_link)
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
```

6. **Update SLAM Toolbox base_frame parameter:**

```python
# In slam_params.yaml
base_frame: base_footprint  # Changed from base_link
```

7. **Remove old static TF publisher:**

```python
# DELETE THIS (now handled by robot_state_publisher from URDF):
# static_tf_base_laser = Node(
#     package='tf2_ros',
#     executable='static_transform_publisher',
#     arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
# )
```

8. **Add to LaunchDescription:**

```python
    return LaunchDescription([
        use_sim_time_arg,
        # ... other args
        robot_state_publisher_node,  # ADD THIS
        static_tf_odom,              # KEEP THIS (renamed)
        # static_tf_base_laser,      # REMOVE THIS
        rplidar_node,
        slam_node,
        rviz_node,
    ])
```

**Complete Modified slam.launch.py:**

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_file = os.path.join(pkg_dir, 'urdf', 'wayfinder_robot.urdf.xacro')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot Description
    robot_description = Command(['xacro ', urdf_file])

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # Static TF: odom → base_footprint (placeholder until real odometry)
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    # RPLidar
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'scan_mode': 'DenseBoost',
        }],
        output='screen'
    )

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        robot_state_publisher_node,
        static_tf_odom,
        rplidar_node,
        slam_node,
        rviz_node,
    ])
```

### Step 2: Update localization.launch.py

**Same pattern as SLAM:**

1. Add robot_state_publisher
2. Change static TF from `odom → base_link` to `odom → base_footprint`
3. Remove `base_link → laser` static TF
4. Update AMCL base_frame parameter to `base_footprint`

**Key changes in localization.launch.py:**

```python
    # Robot description (same as SLAM)
    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Static TF (same as SLAM)
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    # Remove base_link → laser TF (now in URDF)
```

**Update config/amcl_params.yaml:**

```yaml
amcl:
  ros__parameters:
    base_frame_id: base_footprint  # Changed from base_link
    odom_frame_id: odom
    global_frame_id: map
    # ... rest of parameters
```

### Step 3: Update config/slam_params.yaml

**Change base_frame to base_footprint:**

```yaml
slam_toolbox:
  ros__parameters:
    base_frame: base_footprint  # Changed from base_link
    odom_frame: odom
    map_frame: map
    scan_topic: /scan
    # ... rest of parameters
```

### Step 4: Create LiDAR-Only Launch File

**New file:** `launch/lidar.launch.py`

```python
#!/usr/bin/env python3
"""
LiDAR-only launch file for testing RPLidar connection.
Includes robot description for proper TF tree.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_file = os.path.join(pkg_dir, 'urdf', 'wayfinder_robot.urdf.xacro')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = Command(['xacro ', urdf_file])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Static TF for testing (no odometry)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_footprint']
    )

    # RPLidar
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'scan_mode': 'DenseBoost',
        }],
        output='screen'
    )

    # RViz with LaserScan display
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        static_tf,
        rplidar,
        rviz,
    ])
```

### Step 5: Update Navigation Launch File (Future)

**When creating navigation.launch.py:**

```python
def generate_launch_description():
    # ... paths and configs

    robot_description = Command(['xacro ', urdf_file])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Include localization launch (which includes robot_state_publisher)
    # OR launch robot_state_publisher here and skip in localization

    # Nav2 nodes (controller, planner, etc.)
    # ...

    # Ensure nav2_params.yaml uses:
    # robot_base_frame: base_footprint
```

---

## Modified Launch Files

### Summary of Changes

| Launch File | Changes |
|-------------|---------|
| slam.launch.py | + robot_state_publisher<br>~ static TF: odom → base_footprint<br>- static TF: base_link → laser |
| localization.launch.py | + robot_state_publisher<br>~ static TF: odom → base_footprint<br>- static TF: base_link → laser |
| lidar.launch.py | NEW FILE: LiDAR testing with robot description |
| robot_state_publisher.launch.py | EXISTING: Standalone robot description viewer |

### Configuration File Updates

| Config File | Parameter | Old Value | New Value |
|-------------|-----------|-----------|-----------|
| slam_params.yaml | base_frame | base_link | base_footprint |
| amcl_params.yaml | base_frame_id | base_link | base_footprint |
| nav2_params.yaml (future) | robot_base_frame | base_link | base_footprint |

---

## Testing Integration

### Test 1: Validate URDF

```bash
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt
./scripts/validate_urdf.sh
```

**Expected output:**
```
✓ XACRO processed successfully
✓ URDF validation passed
✓ Validation complete!
```

### Test 2: Check Robot State Publisher Standalone

```bash
ros2 launch launch/robot_state_publisher.launch.py use_rviz:=true
```

**In RViz:**
- Add RobotModel display
- Set Fixed Frame: `base_link`
- Verify robot appears (blue base, black wheels, red LiDAR)
- Add TF display
- Verify frames: base_footprint → base_link → laser, wheels, caster

### Test 3: Check TF Tree

```bash
# In one terminal: Launch robot state publisher
ros2 launch launch/robot_state_publisher.launch.py

# In another terminal: Generate TF tree
ros2 run tf2_tools view_frames

# Open frames.pdf
evince frames.pdf
```

**Expected tree:**
```
base_footprint
 └─ base_link
     ├─ laser
     ├─ left_wheel
     ├─ right_wheel
     └─ caster_wheel
```

### Test 4: Test SLAM with URDF

```bash
# Update slam.launch.py with changes above, then:
ros2 launch launch/slam.launch.py use_sim_time:=false

# In RViz:
# - Add RobotModel display (should show robot)
# - Add LaserScan display (topic: /scan)
# - Add Map display (topic: /map)
# - Add TF display

# Check TF tree includes all frames
ros2 run tf2_tools view_frames
```

**Expected TF tree:**
```
map
 └─ odom
     └─ base_footprint
         └─ base_link
             ├─ laser
             ├─ left_wheel
             ├─ right_wheel
             └─ caster_wheel
```

### Test 5: Test Localization with URDF

```bash
# Update localization.launch.py, then:
ros2 launch launch/localization.launch.py map:=/path/to/saved_map.yaml

# Verify same TF tree structure
# Set initial pose in RViz
# Drive robot and check localization
```

### Test 6: Verify Transforms

```bash
# Check specific transforms
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo odom base_footprint

# Monitor all transforms
ros2 run tf2_ros tf2_monitor
```

**Expected output for base_link → laser:**
```
Translation: [0.000, 0.000, 0.125]
Rotation: [0.000, 0.000, 0.000, 1.000]
```

---

## Troubleshooting

### Issue: Robot doesn't appear in RViz

**Symptom:** RobotModel display is empty

**Debug:**
1. Check robot_state_publisher is running:
   ```bash
   ros2 node list | grep robot_state_publisher
   ```

2. Check robot_description topic:
   ```bash
   ros2 topic echo /robot_description --once | head -20
   ```

3. Check for errors in robot_state_publisher output

**Solution:**
- Verify URDF path is correct in launch file
- Ensure xacro command succeeded
- Check RViz Fixed Frame is set to existing frame

### Issue: TF tree is disconnected

**Symptom:** Frames exist but not connected

**Debug:**
```bash
ros2 run tf2_tools view_frames
# Check frames.pdf for multiple trees
```

**Common causes:**
- Missing static TF publisher for odom → base_footprint
- robot_state_publisher not running
- Typo in frame names

**Solution:**
- Verify all TF publishers are running
- Check frame names match exactly (case-sensitive)
- Ensure odom → base_footprint static TF is active

### Issue: "Frame [laser] does not exist"

**Symptom:** SLAM or localization can't find laser frame

**Debug:**
```bash
ros2 topic echo /tf_static
# Look for base_link → laser transform
```

**Solution:**
- Ensure robot_state_publisher is running
- Verify URDF defines laser frame
- Check scan_topic in SLAM/AMCL uses frame_id: laser

### Issue: Multiple publishers for same transform

**Warning:**
```
TF_REPEATED_DATA ignoring data with redundant timestamp
```

**Cause:** Both old static_transform_publisher and robot_state_publisher publishing base_link → laser

**Solution:**
- Remove manual static_transform_publisher for base_link → laser
- Only robot_state_publisher should publish this transform

### Issue: SLAM/AMCL fails with "base_link does not exist"

**Cause:** Config still uses base_link but we changed to base_footprint

**Solution:**
- Update slam_params.yaml: `base_frame: base_footprint`
- Update amcl_params.yaml: `base_frame_id: base_footprint`

---

## Rollback Plan

If integration causes issues:

### Quick Rollback

1. **Revert launch files** to previous version (remove robot_state_publisher)
2. **Keep manual static TF publishers** (odom → base_link, base_link → laser)
3. **Revert config files** (base_frame: base_link)

### Gradual Integration

1. **First:** Add robot_state_publisher to robot_state_publisher.launch.py only
   - Test standalone visualization
   - Don't modify SLAM/localization yet

2. **Second:** Update SLAM launch file only
   - Test mapping works with new TF tree
   - Keep localization unchanged

3. **Third:** Update localization launch file
   - Test localization works with new TF tree

4. **Finally:** Update navigation launch file (when created)

---

## Benefits of Integration

### Immediate Benefits

1. **Robot visualization** - See robot model in RViz
2. **Single source of truth** - URDF defines all transforms
3. **Easier maintenance** - Change sensor position once in URDF
4. **Scalability** - Adding sensors = editing URDF, not launch files
5. **Standard compliance** - Follows ROS best practices

### Future Benefits

1. **Odometry integration** - Can add wheel positions for encoder-based odometry
2. **Collision checking** - Nav2 can use collision geometry
3. **Simulation** - Can spawn in Gazebo directly
4. **Multi-sensor fusion** - Easy to add IMU, cameras, etc.
5. **Documentation** - URDF serves as robot documentation

---

## Next Steps

### Immediate (Priority 1)

1. ✅ URDF created
2. ✅ robot_state_publisher launch file created
3. ⬜ Update slam.launch.py with robot_state_publisher
4. ⬜ Update localization.launch.py with robot_state_publisher
5. ⬜ Test SLAM with URDF integration
6. ⬜ Test localization with URDF integration

### Short-term (Priority 2)

7. ⬜ Create nav2_params.yaml using base_footprint
8. ⬜ Create navigation.launch.py with robot_state_publisher
9. ⬜ Create RViz config with robot model
10. ⬜ Document integration in main README

### Medium-term (Priority 3)

11. ⬜ Implement odometry publisher (replace static TF)
12. ⬜ Update URDF with real robot measurements
13. ⬜ Test in Gazebo simulation
14. ⬜ Calibrate wheel separation for accurate rotation

---

## Conclusion

Integrating the URDF robot description with the WayfindR navigation stack provides a solid foundation for current and future development. The changes are straightforward and follow ROS best practices.

**Key points:**
- Replace manual static TF publishers with robot_state_publisher
- Change base frame from base_link to base_footprint in configs
- Test thoroughly after each change
- Benefits far outweigh integration effort

---

**Last Updated:** 2026-01-11
**Author:** WayfindR Development Team
**Status:** Ready for Implementation
**Estimated Time:** 1-2 hours for full integration
