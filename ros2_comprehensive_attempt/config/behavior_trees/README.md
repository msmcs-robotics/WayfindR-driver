# WayfindR Behavior Trees

This directory contains custom behavior tree XML files for the WayfindR autonomous navigation robot. These behavior trees extend Nav2's default navigation capabilities with mission-specific behaviors.

## Available Behavior Trees

### 1. multi_waypoint_patrol.xml
**Purpose:** Sequential navigation through multiple waypoints with recovery behaviors.

**Use Cases:**
- Security patrol routes
- Facility inspection tours
- Scheduled delivery routes

**Key Features:**
- Continuous replanning during transit
- Removes waypoints as they're reached
- Comprehensive recovery (clear costmaps, spin, wait, backup)
- Can be preempted by new goals

**Usage:**
```yaml
# In nav2_params.yaml
bt_navigator:
  ros__parameters:
    default_nav_through_poses_bt_xml: "$(find-pkg-share wayfindr_navigation)/config/behavior_trees/multi_waypoint_patrol.xml"
```

**Blackboard Variables:**
- `goals`: Vector of PoseStamped waypoints

### 2. patrol_with_battery_management.xml
**Purpose:** Battery-aware patrol that returns to dock when battery is low.

**Use Cases:**
- Long-duration autonomous missions
- Unmanned facility monitoring
- Extended area coverage

**Key Features:**
- Continuous battery monitoring
- Automatic return-to-dock on low battery (<15%)
- Charging station navigation with high retry count
- Waits until 80% charged before resuming
- Mission preemption for critical battery

**Usage:**
```yaml
bt_navigator:
  ros__parameters:
    default_nav_through_poses_bt_xml: "$(find-pkg-share wayfindr_navigation)/config/behavior_trees/patrol_with_battery_management.xml"
```

**Blackboard Variables:**
- `goals`: Patrol waypoint list
- `dock_pose`: Charging station location
- `battery_threshold`: Low battery percentage (default: 15.0)

**Requirements:**
- Battery state publisher on `/battery_status` (sensor_msgs/BatteryState)
- Nav2 `IsBatteryLow` condition node (built-in to Nav2)

### 3. exploration_behavior.xml
**Purpose:** Autonomous frontier-based exploration for mapping unknown areas.

**Use Cases:**
- Initial environment mapping
- Search and rescue
- Unknown area reconnaissance

**Key Features:**
- Frontier identification and selection
- Conservative navigation in unknown areas (max 0.15 m/s)
- Stores starting position for return home
- 360° scans at frontiers for complete coverage
- Battery safety monitoring
- Automatic return home when complete
- Map saving after exploration

**Usage:**
```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "$(find-pkg-share wayfindr_navigation)/config/behavior_trees/exploration_behavior.xml"
```

**Blackboard Variables:**
- `explore_start_pose`: Starting position (auto-set)
- `exploration_goal`: Current frontier target
- `exploration_complete`: Mission completion flag

**Requirements:**
- Frontier detection node (e.g., `explore_lite` package)
- SLAM for continuous mapping (`slam_toolbox`)
- Custom `GetNextExplorationGoal` node (see XML comments for implementation details)

### 4. dock_and_charge.xml
**Purpose:** Autonomous docking with charging station including precision alignment.

**Use Cases:**
- Automated battery management
- Integration with patrol/exploration behaviors
- Scheduled charging cycles

**Key Features:**
- Multi-stage docking: coarse navigation → dock detection → precision alignment → charging
- Sensor-based dock detection (AprilTag, IR, fiducials)
- Precision alignment for electrical connection
- Charging status verification
- Monitoring until target battery level reached
- Optional automatic undocking

**Usage:**
```yaml
bt_navigator:
  ros__parameters:
    # Use as standalone docking behavior
    default_nav_to_pose_bt_xml: "$(find-pkg-share wayfindr_navigation)/config/behavior_trees/dock_and_charge.xml"

# Or include as subtree in other behaviors (see patrol_with_battery_management.xml)
```

**Blackboard Variables:**
- `dock_pose`: Approximate dock location
- `precise_dock_pose`: Sensor-detected exact pose
- `dock_approach_pose`: Pre-docking alignment position
- `target_battery_level`: Charging target % (default: 80.0)
- `is_charging`: Charging status flag
- `charging_complete`: Completion flag

**Requirements:**
- Dock detection sensor and node (AprilTag camera, IR sensors)
- Battery state publisher with `power_supply_status` field
- Custom nodes (see XML for implementation details):
  - `CheckDockDetected`: Verify dock in view
  - `GetDockPose`: Get precise dock position from sensors
  - `ComputeApproachPose`: Calculate approach position
  - `CheckChargingStatus`: Verify electrical connection

## Configuration

### Loading Custom Behavior Trees

#### Method 1: Nav2 Parameters (Recommended)
Edit `config/nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    # For single-goal navigation
    default_nav_to_pose_bt_xml: "/absolute/path/to/custom_bt.xml"

    # For multi-waypoint navigation
    default_nav_through_poses_bt_xml: "/absolute/path/to/custom_bt.xml"

    # Leave empty to use Nav2 defaults
    # default_nav_to_pose_bt_xml: ""
```

#### Method 2: Runtime Parameter
```bash
ros2 param set /bt_navigator default_nav_to_pose_bt_xml "/path/to/bt.xml"
```

#### Method 3: Action Goal
Specify BT in NavigateToPose action goal (advanced usage).

### Adding Custom BT Plugins

If you create custom BT nodes, register them in `nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      # Default Nav2 plugins
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      # ... other defaults ...

      # Your custom plugins
      - nav2_get_next_exploration_goal_bt_node
      - nav2_check_dock_detected_condition_bt_node
      - nav2_get_dock_pose_action_bt_node
```

## Testing Behavior Trees

### 1. Syntax Validation
```bash
# Nav2 will validate XML on startup
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  -p default_nav_to_pose_bt_xml:=/path/to/your_bt.xml

# Check logs for XML parsing errors
```

### 2. Visualization with Groot

Install Groot2:
```bash
sudo apt install ros-humble-groot
```

Enable monitoring in `nav2_params.yaml`:
```yaml
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
```

Run Groot:
```bash
ros2 run groot Groot
# Connect to: localhost:1666
```

### 3. Testing Patrol Behavior
```bash
# Launch Nav2 with custom BT
ros2 launch wayfindr_navigation navigation.launch.py

# Send patrol waypoints
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "{
  poses: [
    {
      header: {frame_id: 'map'},
      pose: {position: {x: 1.0, y: 1.0, z: 0.0}}
    },
    {
      header: {frame_id: 'map'},
      pose: {position: {x: 2.0, y: 1.0, z: 0.0}}
    },
    {
      header: {frame_id: 'map'},
      pose: {position: {x: 2.0, y: 2.0, z: 0.0}}
    }
  ]
}"
```

### 4. Monitoring Battery Behavior
```bash
# Publish test battery state
ros2 topic pub /battery_status sensor_msgs/BatteryState "{
  voltage: 11.5,
  percentage: 15.0,
  power_supply_status: 2
}"

# Watch for dock navigation trigger
ros2 topic echo /plan
```

## Customization Tips

### Adjusting Recovery Behavior
Edit the `RecoveryNode` retry count:
```xml
<RecoveryNode number_of_retries="6" name="NavigateWithRecovery">
  <!-- More retries = more persistent, but takes longer to give up -->
</RecoveryNode>
```

### Changing Replanning Frequency
Edit `RateController` frequency:
```xml
<RateController hz="0.333">  <!-- Replans every 3 seconds -->
  <!-- Lower hz = less CPU, but less responsive to dynamic obstacles -->
</RateController>
```

### Adding Waypoint Pauses
Uncomment or add `Wait` node:
```xml
<Wait wait_duration="5.0"/>  <!-- Wait 5 seconds at each waypoint -->
```

### Modifying Battery Threshold
```xml
<IsBatteryLow battery_topic="/battery_status"
              min_battery="20.0"  <!-- Adjust threshold % -->
              is_voltage="false"/>
```

### Adjusting Exploration Speed
```xml
<FollowPath path="{exploration_path}"
           controller_id="FollowPath"
           max_vel_x="0.15"/>  <!-- Slow speed for safety -->
```

## Behavior Tree Design Patterns

### Pattern: Retry with Recovery
```xml
<RecoveryNode number_of_retries="3">
  <ActionThatMightFail/>
  <RecoverySequence/>
</RecoveryNode>
```

### Pattern: Conditional Execution
```xml
<ReactiveFallback>
  <Condition/>  <!-- If true, skip action -->
  <Action/>     <!-- Executed if condition false -->
</ReactiveFallback>
```

### Pattern: Continuous Replanning
```xml
<PipelineSequence>
  <RateController hz="1.0">
    <PlanPath/>
  </RateController>
  <FollowPath/>  <!-- Always uses latest path -->
</PipelineSequence>
```

### Pattern: Safety Monitor
```xml
<ReactiveFallback>
  <Inverter>
    <UnsafeCondition/>  <!-- If safe (not unsafe), continue -->
  </Inverter>
  <SafetyAction/>  <!-- Execute if unsafe -->
</ReactiveFallback>
```

## Troubleshooting

### BT Won't Load
**Symptom:** Error on startup about BT XML parsing

**Solutions:**
1. Check XML syntax (matching open/close tags)
2. Verify all node names are registered plugins
3. Check file path is absolute
4. Look for typos in node names

### Action Nodes Timeout
**Symptom:** BT keeps failing with timeout errors

**Solutions:**
1. Increase `default_server_timeout` in nav2_params.yaml
2. Check action servers are running: `ros2 action list`
3. Verify topics connected: `ros2 topic list`
4. Check network latency

### Recovery Loop
**Symptom:** BT stuck in recovery behaviors, never succeeds

**Solutions:**
1. Reduce `number_of_retries` to fail faster
2. Check if goal is actually reachable
3. Verify costmap configuration
4. Test goal manually with RViz "Nav2 Goal" tool

### Blackboard Variable Errors
**Symptom:** "Port not found" or variable type mismatch errors

**Solutions:**
1. Verify variable names match in all nodes: `{variable_name}`
2. Check port definitions in custom nodes
3. Ensure variable types are compatible
4. Initialize variables before use

### Custom Nodes Not Found
**Symptom:** "Plugin 'CustomNode' not found"

**Solutions:**
1. Add plugin library to `plugin_lib_names` in nav2_params.yaml
2. Verify plugin compiles and installs correctly
3. Check `BT_REGISTER_NODES` macro in source
4. Rebuild and source workspace

## Integration with WayfindR System

### Waypoint Manager Integration
The existing `waypoint_manager.py` can feed waypoints to behavior trees:

```python
# waypoint_manager.py publishes waypoints
waypoints = [pose1, pose2, pose3]
goal = NavigateThroughPoses.Goal()
goal.poses = waypoints
self.nav_through_poses_client.send_goal_async(goal)
```

### Battery Monitoring
Ensure your robot driver publishes battery state:

```python
battery_msg = BatteryState()
battery_msg.voltage = current_voltage
battery_msg.percentage = battery_percent
battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
self.battery_pub.publish(battery_msg)
```

### Dock Pose Configuration
Set dock location in launch file or parameter:

```yaml
# In nav2_params.yaml or separate config
wayfindr:
  dock_pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

## Further Reading

- **Comprehensive Guide:** `findings/nav2_behavior_trees_comprehensive_guide.md`
- **Nav2 BT Documentation:** https://docs.nav2.org/behavior_trees/
- **BT.CPP Documentation:** https://www.behaviortree.dev/
- **Nav2 Tuning:** `findings/nav2_research_findings_2026-01-11.md`

## Contributing

When creating new behavior trees:

1. **Document thoroughly** - Add detailed comments in XML
2. **Test extensively** - Validate in simulation before hardware
3. **Follow patterns** - Use established BT design patterns
4. **Name clearly** - Use descriptive names for trees and nodes
5. **Version control** - Track changes to BT files
6. **Share knowledge** - Update this README with new behaviors

---

**Last Updated:** 2026-01-11
**Maintainer:** WayfindR Team
**Version:** 1.0
