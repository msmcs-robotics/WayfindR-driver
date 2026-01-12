# Nav2 Behavior Trees Comprehensive Guide
**Date:** January 11, 2026
**Robot:** WayfindR Differential Drive with RP LIDAR C1M1
**ROS2 Distribution:** Humble
**BT.CPP Version:** 4.x

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [BT.CPP Basics and Nav2 Integration](#btcpp-basics-and-nav2-integration)
3. [Behavior Tree XML Structure](#behavior-tree-xml-structure)
4. [Nav2 Node Types](#nav2-node-types)
5. [Common BT Patterns](#common-bt-patterns)
6. [Default Nav2 Behavior Trees](#default-nav2-behavior-trees)
7. [Creating Custom BT Nodes](#creating-custom-bt-nodes)
8. [Custom Behaviors for WayfindR](#custom-behaviors-for-wayfindr)
9. [Integration with Waypoint System](#integration-with-waypoint-system)
10. [Testing Behavior Trees](#testing-behavior-trees)
11. [Best Practices](#best-practices)
12. [References](#references)

---

## Executive Summary

Behavior Trees (BTs) provide a modular, hierarchical approach to robot task execution in Nav2. Unlike traditional finite state machines, BTs offer better composability, reusability, and maintainability for complex autonomous navigation tasks.

**Key Benefits:**
- **Modularity**: Individual behaviors can be composed into complex sequences
- **Reactive**: Can respond to changing conditions during execution
- **Visual**: Can be designed and debugged with tools like Groot
- **Extensible**: Custom nodes can be added as plugins

**WayfindR Use Cases:**
- Multi-waypoint patrol routes
- Exploration with safety checks
- Battery monitoring and return-to-dock
- Dynamic obstacle recovery strategies

---

## BT.CPP Basics and Nav2 Integration

### What is BT.CPP?

BT.CPP (BehaviorTree.CPP) is a C++ library that implements behavior trees with a focus on:
- Performance and efficiency
- XML-based tree definition
- Plugin system for custom nodes
- Built-in visualization and debugging support

### Nav2's Use of BT.CPP

Nav2's `nav2_behavior_tree` module wraps BT.CPP to provide:
- ROS 2 action client integration via `BtActionNode`
- Navigation-specific nodes (planners, controllers, recovery behaviors)
- Plugin system for custom behaviors
- Integration with Nav2's lifecycle management

**Architecture Overview:**
```
┌─────────────────────────────────────────┐
│        BT Navigator Node                │
│  ┌───────────────────────────────────┐  │
│  │   BehaviorTreeEngine              │  │
│  │   (BT.CPP Core)                   │  │
│  │  ┌─────────────────────────────┐  │  │
│  │  │  Behavior Tree XML          │  │  │
│  │  │  - Control Nodes            │  │  │
│  │  │  - Action Nodes             │  │  │
│  │  │  - Condition Nodes          │  │  │
│  │  │  - Decorator Nodes          │  │  │
│  │  └─────────────────────────────┘  │  │
│  └───────────────────────────────────┘  │
└─────────────────────────────────────────┘
           ↓         ↓         ↓
    ┌──────────┐ ┌──────────┐ ┌──────────┐
    │ Planner  │ │Controller│ │ Recovery │
    │  Server  │ │  Server  │ │  Server  │
    └──────────┘ └──────────┘ └──────────┘
```

**Key Components:**

1. **bt_navigator**: Main node that loads and executes behavior trees
2. **nav2_behavior_tree**: Package with BT node implementations
3. **Behavior Tree XML**: Defines the tree structure and flow
4. **Plugin Libraries**: Custom BT nodes loaded dynamically

---

## Behavior Tree XML Structure

### Basic XML Format (BT.CPP 4.x)

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <!-- Tree structure goes here -->
    </BehaviorTree>
</root>
```

**Essential Elements:**

- `<root>`: Container with format version
- `BTCPP_format="4"`: Specifies BT.CPP version 4
- `main_tree_to_execute`: Entry point tree
- `<BehaviorTree ID="...">`: Named tree definition

### Node Representation

Each node is represented as an XML element:

```xml
<NodeType name="unique_instance_name" param1="value1" param2="value2">
    <!-- Child nodes for control/decorator nodes -->
</NodeType>
```

**Attributes:**
- `name`: Optional instance name for debugging
- Other attributes map to node input ports

### Subtrees

Multiple trees can be defined and included:

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <SubTree ID="RecoverySubtree"/>
    </BehaviorTree>

    <BehaviorTree ID="RecoverySubtree">
        <!-- Recovery logic -->
    </BehaviorTree>
</root>
```

### Blackboard Variables

Data sharing between nodes uses the blackboard:

```xml
<!-- Write to blackboard -->
<ComputePathToPose goal="{goal}" path="{path}"/>

<!-- Read from blackboard -->
<FollowPath path="{path}"/>
```

**Variable Syntax:**
- `{variable_name}`: Blackboard variable
- `"literal_value"`: Literal value
- Variables can be used for input/output ports

---

## Nav2 Node Types

### 1. Action Nodes

Perform actual work (planning, control, recovery).

**Common Action Nodes:**

| Node | Purpose | Key Parameters |
|------|---------|----------------|
| `ComputePathToPose` | Plan path to single goal | `goal`, `path`, `planner_id` |
| `ComputePathThroughPoses` | Plan through multiple waypoints | `goals`, `path` |
| `FollowPath` | Execute path with controller | `path`, `controller_id` |
| `Spin` | Rotate in place | `spin_dist` (radians) |
| `BackUp` | Drive backwards | `backup_dist`, `backup_speed` |
| `Wait` | Pause execution | `wait_duration` |
| `ClearEntireCostmap` | Reset costmap | `service_name` |
| `TruncatePath` | Shorten path | `distance` |

**Example:**
```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
```

### 2. Control Nodes

Manage execution flow of child nodes.

#### Sequence
Executes children left-to-right. Returns SUCCESS if all succeed, FAILURE if any fails.

```xml
<Sequence name="NavigateSequence">
    <ComputePathToPose goal="{goal}" path="{path}"/>
    <FollowPath path="{path}"/>
</Sequence>
```

#### ReactiveFallback
Ticks all children every cycle. Succeeds on first SUCCESS, continues on FAILURE.

```xml
<ReactiveFallback name="CheckGoalOrNavigate">
    <GoalReached goal="{goal}"/>  <!-- Check condition -->
    <NavigateToGoal goal="{goal}"/>  <!-- Execute if not reached -->
</ReactiveFallback>
```

#### PipelineSequence
Like Sequence but re-ticks running nodes each cycle. Essential for continuous behaviors.

```xml
<PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}"/>
    </RateController>
    <FollowPath path="{path}"/>
</PipelineSequence>
```

#### RecoveryNode
Links action with recovery. Retries action after recovery on failure.

```xml
<RecoveryNode number_of_retries="6" name="NavigateRecovery">
    <NavigateAction/>
    <RecoverySequence/>
</RecoveryNode>
```

#### RoundRobin
Tries children in sequence on each tick. Useful for cycling through recoveries.

```xml
<RoundRobin name="RecoveryActions">
    <ClearCostmaps/>
    <Spin spin_dist="1.57"/>
    <Wait wait_duration="5.0"/>
    <BackUp backup_dist="0.3" backup_speed="0.15"/>
</RoundRobin>
```

### 3. Decorator Nodes

Modify child behavior or add logic.

#### RateController
Limits child tick rate to specified frequency.

```xml
<RateController hz="1.0">
    <ComputePathToPose goal="{goal}" path="{path}"/>
</RateController>
```

#### DistanceController
Ticks child every X meters traveled.

```xml
<DistanceController distance="0.5">
    <UpdateMap/>
</DistanceController>
```

#### SpeedController
Adjusts robot speed based on conditions.

```xml
<SpeedController min_speed="0.1" max_speed="0.26">
    <FollowPath path="{path}"/>
</SpeedController>
```

#### GoalUpdater
Updates goal when new one received.

```xml
<GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
    <NavigateToGoal goal="{updated_goal}"/>
</GoalUpdater>
```

### 4. Condition Nodes

Check state and return SUCCESS/FAILURE.

**Common Conditions:**

| Node | Check | Returns SUCCESS when |
|------|-------|---------------------|
| `GoalReached` | Distance to goal | Within tolerance |
| `GoalUpdated` | Goal changed | New goal received |
| `IsBatteryLow` | Battery state | Below threshold |
| `IsStuck` | Robot stuck | No progress detected |
| `TimeExpired` | Timer | Duration elapsed |
| `TransformAvailable` | TF exists | Transform valid |

**Example:**
```xml
<ReactiveFallback>
    <IsBatteryLow battery_topic="/battery" threshold="20.0"/>
    <NavigateNormally/>
</ReactiveFallback>
```

---

## Common BT Patterns

### Pattern 1: Retry with Recovery

Execute action, retry with recovery on failure.

```xml
<RecoveryNode number_of_retries="6" name="NavigateWithRecovery">
    <Sequence>
        <ComputePathToPose goal="{goal}" path="{path}"/>
        <FollowPath path="{path}"/>
    </Sequence>
    <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
            <Sequence name="ClearingActions">
                <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
                <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Spin spin_dist="1.57"/>
            <Wait wait_duration="5.0"/>
            <BackUp backup_dist="0.3" backup_speed="0.15"/>
        </RoundRobin>
    </ReactiveFallback>
</RecoveryNode>
```

**Pattern Explanation:**
1. Try navigation sequence
2. On failure, enter recovery
3. Check if goal updated (abort recovery if so)
4. Cycle through recovery actions: clear → spin → wait → backup
5. Retry navigation after each recovery
6. Give up after 6 retries

### Pattern 2: Conditional Execution

"Do A unless B happens, then do C"

```xml
<ReactiveFallback name="ConditionalBehavior">
    <IsBatteryLow threshold="20.0"/>
    <Sequence name="NormalOperation">
        <NavigateToWaypoint/>
        <PerformTask/>
    </Sequence>
</ReactiveFallback>
```

**Pattern Explanation:**
1. Continuously check battery
2. If low, return SUCCESS (exits fallback)
3. Otherwise, execute normal operation
4. Pair with parent sequence to handle low battery action

### Pattern 3: Continuous Replanning

Replan periodically while following path.

```xml
<PipelineSequence name="NavigateWithReplanning">
    <RateController hz="0.333">
        <RecoveryNode number_of_retries="1">
            <ComputePathToPose goal="{goal}" path="{path}"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </RecoveryNode>
    </RateController>
    <RecoveryNode number_of_retries="1">
        <FollowPath path="{path}"/>
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
    </RecoveryNode>
</PipelineSequence>
```

**Pattern Explanation:**
1. PipelineSequence re-ticks all children
2. Planner runs at 0.333 Hz (every 3 seconds)
3. Controller continuously follows latest path
4. Each has single-retry recovery (costmap clearing)
5. Enables dynamic replanning around obstacles

### Pattern 4: Waypoint Progression

Process multiple waypoints sequentially.

```xml
<PipelineSequence name="ProcessWaypoints">
    <RateController hz="0.333">
        <ComputePathThroughPoses goals="{goals}" path="{path}">
            <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
        </ComputePathThroughPoses>
    </RateController>
    <FollowPath path="{path}"/>
</PipelineSequence>
```

**Pattern Explanation:**
1. RemovePassedGoals updates goals list as waypoints reached
2. ComputePathThroughPoses plans through remaining waypoints
3. FollowPath executes toward next waypoint
4. Cycle repeats until all waypoints cleared

### Pattern 5: Safety Monitor

Abort normal operation on safety condition.

```xml
<ReactiveFallback name="SafetyMonitor">
    <Inverter>
        <IsBatteryLow threshold="15.0"/>
    </Inverter>
    <ReturnToDock/>
</ReactiveFallback>
```

**Pattern Explanation:**
1. Inverter flips condition result
2. If battery NOT low, return SUCCESS (continue)
3. If battery low, proceed to return to dock
4. Use as top-level node wrapping main mission

---

## Default Nav2 Behavior Trees

### navigate_to_pose_w_replanning_and_recovery

**Purpose:** Navigate to single goal with replanning and comprehensive recovery.

**Structure:**
```
RecoveryNode (6 retries)
└─ PipelineSequence: NavigateWithReplanning
   ├─ RateController (1.0 Hz)
   │  └─ RecoveryNode (1 retry)
   │     ├─ ComputePathToPose
   │     └─ Recovery: WouldAPlannerRecoveryHelp → ClearEntireCostmap (global)
   └─ RecoveryNode (1 retry)
      ├─ FollowPath
      └─ Recovery: WouldAControllerRecoveryHelp → ClearEntireCostmap (local)
└─ ReactiveFallback: RecoveryFallback
   ├─ GoalUpdated (check for new goal)
   └─ RoundRobin: RecoveryActions
      ├─ Sequence: ClearingActions (local + global costmaps)
      ├─ Spin (1.57 rad / 90°)
      ├─ Wait (5.0 sec)
      └─ BackUp (0.30m at 0.15 m/s)
```

**Key Features:**
- **Two-tier recovery**: Contextual (costmap clearing) and system-level (spin/wait/backup)
- **Continuous replanning**: Planner runs at 1 Hz
- **Responsive**: Aborts recovery if new goal received
- **Robust**: Up to 6 full retry attempts

### navigate_through_poses_w_replanning_and_recovery

**Purpose:** Navigate through multiple waypoints sequentially.

**Structure:**
```
RecoveryNode (6 retries)
└─ PipelineSequence: NavigateWithReplanning
   ├─ RateController (0.333 Hz)
   │  └─ RecoveryNode (1 retry)
   │     ├─ ComputePathThroughPoses
   │     │  └─ RemovePassedGoals (radius: 0.7m)
   │     └─ Recovery: ClearEntireCostmap (global)
   └─ RecoveryNode (1 retry)
      ├─ FollowPath
      └─ Recovery: ClearEntireCostmap (local)
└─ ReactiveFallback: RecoveryFallback
   [Same as navigate_to_pose]
```

**Key Features:**
- **Waypoint management**: RemovePassedGoals culls reached waypoints
- **Multi-goal planning**: ComputePathThroughPoses handles goal vector
- **Slower replanning**: 0.333 Hz (every 3 sec) to reduce computation
- **Same recovery**: Identical system-level recovery as single-pose

**Use Cases:**
- Patrol routes
- Multi-stop delivery
- Area coverage patterns

---

## Creating Custom BT Nodes

### Overview

Custom BT nodes extend Nav2's behavior capabilities. Node types:
- **Action**: Perform work (inherit `BtActionNode` for ROS 2 actions)
- **Condition**: Check state (inherit `BT::ConditionNode`)
- **Decorator**: Modify child (inherit `BT::DecoratorNode`)
- **Control**: Custom flow (inherit `BT::ControlNode`)

### Step-by-Step: Custom Condition Node

**Example: Battery Monitoring Condition**

#### 1. Header File (include/nav2_behavior_tree/plugins/condition/is_battery_charging_node.hpp)

```cpp
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_CHARGING_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_CHARGING_NODE_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "sensor_msgs/msg/battery_state.hpp"

namespace nav2_behavior_tree
{

class IsBatteryChargingCondition : public BT::ConditionNode
{
public:
  IsBatteryChargingCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryChargingCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("battery_topic", "/battery_status", "Battery topic"),
    };
  }

private:
  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_msg_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_CHARGING_NODE_HPP_
```

#### 2. Source File (src/condition/is_battery_charging_node.cpp)

```cpp
#include "nav2_behavior_tree/plugins/condition/is_battery_charging_node.hpp"

namespace nav2_behavior_tree
{

IsBatteryChargingCondition::IsBatteryChargingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string battery_topic;
  getInput("battery_topic", battery_topic);

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1));
}

BT::NodeStatus IsBatteryChargingCondition::tick()
{
  if (!last_battery_msg_) {
    return BT::NodeStatus::FAILURE;
  }

  // Check if battery is charging (power supply status == 1)
  if (last_battery_msg_->power_supply_status ==
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(
  sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  last_battery_msg_ = msg;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBatteryChargingCondition>("IsBatteryCharging");
}
```

#### 3. CMakeLists.txt Registration

```cmake
add_library(nav2_is_battery_charging_condition_bt_node SHARED
  src/condition/is_battery_charging_node.cpp)

target_link_libraries(nav2_is_battery_charging_condition_bt_node
  ${library_name})

target_compile_definitions(nav2_is_battery_charging_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
```

#### 4. Plugin XML (nav2_behavior_tree_plugins.xml)

```xml
<library path="nav2_is_battery_charging_condition_bt_node">
  <class type="nav2_behavior_tree::IsBatteryChargingCondition"
         base_class_type="BT::ConditionNode">
    <description>Checks if battery is currently charging</description>
  </class>
</library>
```

#### 5. Usage in Behavior Tree XML

```xml
<ReactiveFallback>
    <IsBatteryCharging battery_topic="/battery_status"/>
    <ReturnToDock/>
</ReactiveFallback>
```

### Step-by-Step: Custom Action Node

**Example: Dock with Charging Station**

#### 1. Header File (include/nav2_behavior_tree/plugins/action/dock_robot_action.hpp)

```cpp
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_ROBOT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_ROBOT_ACTION_HPP_

#include <string>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/dock_robot.hpp"

namespace nav2_behavior_tree
{

class DockRobotAction : public BtActionNode<nav2_msgs::action::DockRobot>
{
public:
  DockRobotAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("dock_id", "Dock identifier"),
      BT::InputPort<std::string>("dock_type", "charger", "Type of dock"),
      BT::InputPort<bool>("use_dock_pose", true, "Navigate to dock pose"),
    });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_ROBOT_ACTION_HPP_
```

#### 2. Source File

```cpp
#include "nav2_behavior_tree/plugins/action/dock_robot_action.hpp"

namespace nav2_behavior_tree
{

DockRobotAction::DockRobotAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::DockRobot>(xml_tag_name, action_name, conf)
{
}

void DockRobotAction::on_tick()
{
  std::string dock_id, dock_type;
  bool use_dock_pose;

  getInput("dock_id", dock_id);
  getInput("dock_type", dock_type);
  getInput("use_dock_pose", use_dock_pose);

  goal_.dock_id = dock_id;
  goal_.dock_type = dock_type;
  goal_.use_dock_pose = use_dock_pose;
}

BT::NodeStatus DockRobotAction::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "Successfully docked!");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::DockRobotAction>(
        name, "dock_robot", config);
    };

  factory.registerBuilder<nav2_behavior_tree::DockRobotAction>("DockRobot", builder);
}
```

### Plugin Loading Configuration

Add to `nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      # ... other default plugins
      - nav2_is_battery_charging_condition_bt_node  # Custom condition
      - nav2_dock_robot_action_bt_node              # Custom action
```

---

## Custom Behaviors for WayfindR

### 1. Battery-Aware Navigation

**Scenario:** Monitor battery during navigation, return to dock if low.

**Features:**
- Continuous battery monitoring
- Preempt mission on low battery
- Navigate to charging station
- Resume mission after charging

**Implementation:** See `patrol_with_battery_management.xml`

### 2. Multi-Waypoint Patrol

**Scenario:** Patrol predefined route continuously.

**Features:**
- Loop through waypoint list
- Pause at each waypoint
- Handle recovery at any point
- Emergency stop capability

**Implementation:** See `multi_waypoint_patrol.xml`

### 3. Exploration Behavior

**Scenario:** Autonomous exploration of unknown areas.

**Features:**
- Frontier-based exploration
- Safety distance from obstacles
- Periodic mapping updates
- Return home when complete

**Implementation:** See `exploration_behavior.xml`

### 4. Dynamic Obstacle Response

**Scenario:** Enhanced recovery for dynamic environments.

**Features:**
- Detect moving obstacles
- Wait for obstacles to pass
- Adaptive recovery timing
- Path invalidation handling

**Implementation:** Integrated into patrol behaviors

---

## Integration with Waypoint System

### Waypoint Manager Integration

The existing `waypoint_manager.py` can feed waypoints to behavior trees via:

**1. Action Interface:**
```python
# waypoint_manager.py publishes waypoints
self.waypoint_pub = self.create_publisher(
    PoseArray, '/waypoint_manager/waypoints', 10)
```

**2. Behavior Tree Consumption:**
```xml
<PipelineSequence>
    <GetWaypointsFromTopic topic="/waypoint_manager/waypoints" waypoints="{goals}"/>
    <ComputePathThroughPoses goals="{goals}" path="{path}">
        <RemovePassedGoals input_goals="{goals}" output_goals="{goals}"/>
    </ComputePathThroughPoses>
    <FollowPath path="{path}"/>
</PipelineSequence>
```

### Custom Waypoint Action Node

For tighter integration, create `LoadWaypointsAction`:

```cpp
class LoadWaypointsAction : public BT::SyncActionNode
{
public:
  LoadWaypointsAction(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints")
    };
  }

  BT::NodeStatus tick() override
  {
    // Load waypoints from ROS parameter, file, or service
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    // ... load logic ...
    setOutput("waypoints", waypoints);
    return BT::NodeStatus::SUCCESS;
  }
};
```

### Waypoint Task Executor

Nav2's waypoint follower supports task executors:

```yaml
waypoint_follower:
  ros__parameters:
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      wait_duration: 5  # seconds at each waypoint
```

Custom task executors can:
- Take photos at waypoints
- Perform sensor scans
- Log position data
- Interact with environment

---

## Testing Behavior Trees

### 1. XML Validation

**Syntax Check:**
```bash
# BT.CPP includes XML schema validation
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  -p default_nav_to_pose_bt_xml:=/path/to/your_bt.xml
```

**Common XML Errors:**
- Missing closing tags
- Invalid node names (not registered)
- Incorrect port names
- Mismatched variable types

### 2. Groot Visualization

**Install Groot2:**
```bash
sudo apt install ros-humble-groot
```

**Monitor Live Execution:**
```yaml
# In nav2_params.yaml
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
```

```bash
# Run Groot
ros2 run groot Groot
# Connect to localhost:1666
```

**Features:**
- Real-time tree visualization
- Node status (SUCCESS/FAILURE/RUNNING)
- Blackboard variable inspection
- Execution path highlighting

### 3. Unit Testing Custom Nodes

**Example Test Structure:**
```cpp
#include <gtest/gtest.h>
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/condition/is_battery_charging_node.hpp"

TEST(BatteryChargingCondition, ReturnsSuccessWhenCharging)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsBatteryChargingCondition>("IsBatteryCharging");

  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree>
        <IsBatteryCharging battery_topic="/battery"/>
      </BehaviorTree>
    </root>
  )");

  // Publish charging message
  // ...

  auto status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}
```

### 4. Integration Testing

**Test Scenarios:**

1. **Happy Path:**
   - Navigate to goal successfully
   - Verify path execution
   - Check goal reached

2. **Recovery Scenarios:**
   - Block path mid-navigation
   - Verify costmap clearing
   - Verify spin/backup execution
   - Confirm recovery success

3. **Multi-Waypoint:**
   - Provide waypoint list
   - Verify sequential navigation
   - Check waypoint removal
   - Confirm all waypoints reached

4. **Battery Management:**
   - Simulate low battery mid-mission
   - Verify dock navigation triggered
   - Confirm mission preemption

**Test Tools:**
```bash
# Launch test environment
ros2 launch nav2_bringup tb3_simulation_launch.py

# Send test goals
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}}}}"

# Monitor topics
ros2 topic echo /plan
ros2 topic echo /cmd_vel
```

### 5. Debugging Techniques

**Enable Detailed Logging:**
```yaml
bt_navigator:
  ros__parameters:
    ros__log_level: DEBUG
```

**Common Issues:**

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Tree doesn't start | XML syntax error | Check logs for parse errors |
| Node always FAILURE | Missing ROS topic/action | Verify action servers running |
| Infinite loop | Missing success condition | Add condition node or retry limit |
| Blackboard error | Variable type mismatch | Check port definitions |
| Plugin not found | Library not loaded | Add to `plugin_lib_names` |

**Logging Best Practices:**
```cpp
// In custom node tick()
RCLCPP_DEBUG(node_->get_logger(), "Node ticking with goal: %s", goal_str.c_str());
RCLCPP_INFO(node_->get_logger(), "Action completed successfully");
RCLCPP_WARN(node_->get_logger(), "Retry attempt %d", retry_count);
RCLCPP_ERROR(node_->get_logger(), "Critical failure: %s", error.c_str());
```

---

## Best Practices

### 1. Tree Design Principles

**Keep Trees Simple:**
- Limit tree depth (3-4 levels max)
- Use subtrees for complex behaviors
- One tree per major task

**Modularity:**
- Reusable subtrees
- Parameterized nodes
- Clear input/output ports

**Readability:**
- Descriptive node names
- Consistent naming conventions
- Comments in XML

```xml
<!-- Good: Clear structure and naming -->
<Sequence name="InspectArea">
    <NavigateToInspectionPoint point="{inspection_point}"/>
    <PerformScan scan_type="detailed"/>
    <ReturnToBasePoint point="{base_point}"/>
</Sequence>

<!-- Bad: Unclear, nested too deep -->
<Sequence>
    <Sequence>
        <Sequence>
            <Action1/>
        </Sequence>
    </Sequence>
</Sequence>
```

### 2. Error Handling

**Always Use RecoveryNode:**
```xml
<RecoveryNode number_of_retries="3">
    <ActionThatMightFail/>
    <RecoverySequence/>
</RecoveryNode>
```

**Graceful Degradation:**
```xml
<ReactiveFallback name="PlanningWithFallback">
    <ComputeOptimalPath/>
    <ComputeSimplePath/>
    <NavigateSafely/>
</ReactiveFallback>
```

**Timeouts:**
```xml
<Timeout msec="30000">
    <LongRunningAction/>
</Timeout>
```

### 3. Performance Optimization

**Rate Limiting:**
- Don't replan every tick (use RateController)
- Typical planning: 0.5-1.0 Hz
- Typical control: 10-20 Hz

**Resource Management:**
- Unsubscribe when nodes halt
- Clear large data structures
- Avoid memory leaks in callbacks

**Blackboard Efficiency:**
- Use appropriate data types
- Clear unused variables
- Avoid copying large structures

### 4. Naming Conventions

**Nodes:**
- Actions: Verb + Object (ComputePath, FollowPath)
- Conditions: Is/Has + State (IsBatteryLow, HasGoal)
- Decorators: Descriptive (RateController, SpeedController)

**Variables:**
- Lowercase with underscores: `{current_goal}`
- Descriptive: `{global_path}` not `{path1}`
- Scoped: `{patrol_waypoints}` vs `{goals}`

**Tree IDs:**
- PascalCase: `NavigateToGoal`, `PatrolRoute`
- Descriptive of purpose
- Unique within XML file

### 5. Documentation

**XML Comments:**
```xml
<!--
  Multi-Waypoint Patrol Behavior
  - Loops through predefined waypoints
  - Monitors battery level
  - Returns to dock if battery low
  Author: WayfindR Team
  Date: 2026-01-11
-->
<root BTCPP_format="4" main_tree_to_execute="PatrolWithBattery">
  <BehaviorTree ID="PatrolWithBattery">
    <!-- Main patrol logic -->
  </BehaviorTree>
</root>
```

**Node Comments:**
```cpp
/**
 * @brief Custom condition to check if robot is stuck
 *
 * Monitors odometry and cmd_vel to detect lack of progress.
 * Returns SUCCESS if robot hasn't moved for stuck_timeout seconds.
 *
 * Input Ports:
 *   - stuck_timeout: Duration (sec) to consider robot stuck
 *   - distance_threshold: Min distance (m) to consider movement
 */
class IsStuckCondition : public BT::ConditionNode { ... };
```

### 6. Version Control

**Track Changes:**
- Keep BT XMLs in version control
- Document changes in commit messages
- Tag stable versions

**Testing:**
- Test BTs before committing
- Include test cases with BTs
- Document expected behavior

### 7. Security Considerations

**Input Validation:**
```cpp
BT::NodeStatus tick() override
{
  double speed;
  if (!getInput("speed", speed)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: speed");
    return BT::NodeStatus::FAILURE;
  }

  if (speed < 0 || speed > max_speed_) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid speed: %f", speed);
    return BT::NodeStatus::FAILURE;
  }

  // Proceed with valid input
}
```

**Resource Limits:**
- Set maximum retry counts
- Implement timeouts
- Monitor memory usage

**Fail-Safe Behaviors:**
- Emergency stop conditions
- Battery monitoring
- Communication loss handling

---

## References

### Official Documentation
- [Nav2 Behavior Trees Documentation](https://docs.nav2.org/behavior_trees/index.html)
- [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/)
- [Nav2 BT XML Nodes Reference](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html)
- [Writing New BT Plugin Tutorial](https://docs.nav2.org/plugin_tutorials/docs/writing_new_bt_plugin.html)
- [Detailed BT Walkthrough](https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html)

### Community Resources
- [Nav2 BT Navigator README](https://github.com/ros-planning/navigation2/blob/main/nav2_bt_navigator/README.md)
- [Nav2 Behavior Tree Package](https://github.com/ros-planning/navigation2/blob/main/nav2_behavior_tree/README.md)
- [Floris Jousselin's Nav2 BT Tutorial](https://robotcopper.github.io/ROS/behavior_tree.html)
- [Groot Visualization Tutorial](https://docs.nav2.org/tutorials/docs/using_groot.html)

### Related WayfindR Documentation
- `nav2_research_findings_2026-01-11.md` - Nav2 configuration research
- `INTEGRATION_GUIDE.md` - System integration guide
- `USAGE_GUIDE.md` - Operational procedures

---

## Appendix: Quick Reference

### Common Node Types

| Category | Node | Purpose |
|----------|------|---------|
| **Control** | Sequence | Execute children in order, fail on first failure |
| | ReactiveFallback | Try children until one succeeds, reactive |
| | PipelineSequence | Sequence that re-ticks running children |
| | RecoveryNode | Execute action with retry and recovery |
| | RoundRobin | Cycle through children on each tick |
| **Decorator** | RateController | Limit child tick rate |
| | Inverter | Flip child result |
| | ForceSuccess | Always return SUCCESS |
| | Timeout | Fail if child exceeds time |
| **Condition** | GoalReached | Check if at goal |
| | GoalUpdated | Check if goal changed |
| | IsBatteryLow | Check battery level |
| **Action** | ComputePathToPose | Plan to single goal |
| | ComputePathThroughPoses | Plan through waypoints |
| | FollowPath | Execute path |
| | Spin | Rotate in place |
| | BackUp | Drive backwards |
| | Wait | Pause execution |

### BT.CPP Status Values

| Status | Meaning | Use Case |
|--------|---------|----------|
| SUCCESS | Task completed | Goal reached, condition true |
| FAILURE | Task failed | Obstacle, condition false |
| RUNNING | Task in progress | Long-running action |

### Debugging Commands

```bash
# View active behavior tree
ros2 topic echo /behavior_tree_log

# Check BT navigator status
ros2 node info /bt_navigator

# Test custom BT XML
ros2 run nav2_bt_navigator bt_navigator \
  --ros-args -p default_nav_to_pose_bt_xml:=/path/to/bt.xml

# Monitor with Groot
ros2 run groot Groot
```

---

**Document Version:** 1.0
**Last Updated:** 2026-01-11
**Author:** Claude Code
**Status:** Complete - Ready for Implementation
