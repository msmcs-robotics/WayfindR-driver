# Custom Behavior Tree Node Examples for WayfindR

This document provides complete implementation examples for custom Nav2 behavior tree nodes specific to the WayfindR robot's requirements.

## Table of Contents

1. [Battery Monitoring Condition](#battery-monitoring-condition)
2. [Dock Detection Condition](#dock-detection-condition)
3. [Exploration Goal Action](#exploration-goal-action)
4. [Waypoint Loader Action](#waypoint-loader-action)
5. [Building and Integration](#building-and-integration)

---

## Battery Monitoring Condition

### Overview
Checks if battery is currently charging by examining the battery state message.

### Header File
**Location:** `include/wayfindr_bt_nodes/condition/is_battery_charging_condition.hpp`

```cpp
#ifndef WAYFINDR_BT_NODES__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_
#define WAYFINDR_BT_NODES__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "sensor_msgs/msg/battery_state.hpp"

namespace wayfindr_bt_nodes
{

/**
 * @brief BT condition node that checks if robot battery is charging
 *
 * Subscribes to a BatteryState topic and checks if the power supply
 * status indicates charging.
 *
 * Input Ports:
 *   battery_topic [string]: Topic publishing BatteryState (default: /battery_status)
 *   timeout_sec [double]: Max age of battery message to consider valid (default: 2.0)
 *
 * Returns:
 *   SUCCESS: Battery is currently charging
 *   FAILURE: Battery not charging or no recent message
 */
class IsBatteryChargingCondition : public BT::ConditionNode
{
public:
  IsBatteryChargingCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryChargingCondition() = delete;

  ~IsBatteryChargingCondition() override = default;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "battery_topic",
        "/battery_status",
        "Topic name for battery state messages"),
      BT::InputPort<double>(
        "timeout_sec",
        2.0,
        "Maximum age of battery message (seconds)")
    };
  }

private:
  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  sensor_msgs::msg::BatteryState::SharedPtr last_battery_msg_;
  rclcpp::Time last_msg_time_;
  std::mutex battery_mutex_;
};

}  // namespace wayfindr_bt_nodes

#endif  // WAYFINDR_BT_NODES__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_
```

### Source File
**Location:** `src/condition/is_battery_charging_condition.cpp`

```cpp
#include "wayfindr_bt_nodes/condition/is_battery_charging_condition.hpp"

namespace wayfindr_bt_nodes
{

IsBatteryChargingCondition::IsBatteryChargingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  last_msg_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string battery_topic;
  getInput("battery_topic", battery_topic);

  // Create callback group for separate executor
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1),
    sub_opt);

  RCLCPP_INFO(
    node_->get_logger(),
    "IsBatteryCharging condition initialized, topic: %s",
    battery_topic.c_str());
}

BT::NodeStatus IsBatteryChargingCondition::tick()
{
  // Spin callback executor to process messages
  callback_executor_.spin_some();

  std::lock_guard<std::mutex> lock(battery_mutex_);

  if (!last_battery_msg_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "No battery message received yet");
    return BT::NodeStatus::FAILURE;
  }

  // Check message age
  double timeout_sec;
  getInput("timeout_sec", timeout_sec);

  auto age = (node_->now() - last_msg_time_).seconds();
  if (age > timeout_sec) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Battery message is stale (%.1f sec old, timeout %.1f sec)",
      age, timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  // Check if charging
  bool is_charging =
    (last_battery_msg_->power_supply_status ==
     sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);

  if (is_charging) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Battery is charging (%.1f%%, %.2fV)",
      last_battery_msg_->percentage,
      last_battery_msg_->voltage);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Battery not charging (status: %d)",
    last_battery_msg_->power_supply_status);
  return BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(
  sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(battery_mutex_);
  last_battery_msg_ = msg;
  last_msg_time_ = node_->now();
}

}  // namespace wayfindr_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wayfindr_bt_nodes::IsBatteryChargingCondition>(
    "IsBatteryCharging");
}
```

### Usage in BT XML

```xml
<ReactiveFallback name="ChargingCheck">
  <IsBatteryCharging battery_topic="/battery_status" timeout_sec="2.0"/>
  <ReturnToDock/>
</ReactiveFallback>
```

---

## Dock Detection Condition

### Overview
Checks if charging dock is detected by vision or sensor system.

### Header File
**Location:** `include/wayfindr_bt_nodes/condition/check_dock_detected_condition.hpp`

```cpp
#ifndef WAYFINDR_BT_NODES__CONDITION__CHECK_DOCK_DETECTED_CONDITION_HPP_
#define WAYFINDR_BT_NODES__CONDITION__CHECK_DOCK_DETECTED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace wayfindr_bt_nodes
{

/**
 * @brief Condition node that checks if charging dock is detected
 *
 * Subscribes to dock detection topic (from AprilTag, fiducial, or
 * IR sensor system) and checks if dock is currently in view.
 *
 * Input Ports:
 *   dock_detection_topic [string]: Topic publishing detected dock pose
 *   detection_timeout [double]: Max time since last detection (default: 1.0 sec)
 *
 * Output Ports:
 *   detected_pose [geometry_msgs::PoseStamped]: Last detected dock pose
 *
 * Returns:
 *   SUCCESS: Dock detected recently
 *   FAILURE: Dock not detected or stale detection
 */
class CheckDockDetectedCondition : public BT::ConditionNode
{
public:
  CheckDockDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "dock_detection_topic",
        "/dock_detection/pose",
        "Topic publishing detected dock pose"),
      BT::InputPort<double>(
        "detection_timeout",
        1.0,
        "Maximum time since detection (seconds)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "detected_pose",
        "Last detected dock pose")
    };
  }

private:
  void dockDetectionCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_dock_pose_;
  rclcpp::Time last_detection_time_;
  std::mutex detection_mutex_;
};

}  // namespace wayfindr_bt_nodes

#endif  // WAYFINDR_BT_NODES__CONDITION__CHECK_DOCK_DETECTED_CONDITION_HPP_
```

### Source File

```cpp
#include "wayfindr_bt_nodes/condition/check_dock_detected_condition.hpp"

namespace wayfindr_bt_nodes
{

CheckDockDetectedCondition::CheckDockDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  last_detection_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string dock_topic;
  getInput("dock_detection_topic", dock_topic);

  dock_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    dock_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CheckDockDetectedCondition::dockDetectionCallback,
              this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "CheckDockDetected initialized, topic: %s",
    dock_topic.c_str());
}

BT::NodeStatus CheckDockDetectedCondition::tick()
{
  std::lock_guard<std::mutex> lock(detection_mutex_);

  if (!last_dock_pose_) {
    RCLCPP_DEBUG(node_->get_logger(), "No dock detection received");
    return BT::NodeStatus::FAILURE;
  }

  // Check detection age
  double timeout;
  getInput("detection_timeout", timeout);

  auto age = (node_->now() - last_detection_time_).seconds();

  if (age > timeout) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Dock detection stale (%.2f sec old)",
      age);
    return BT::NodeStatus::FAILURE;
  }

  // Output detected pose
  setOutput("detected_pose", *last_dock_pose_);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Dock detected at (%.2f, %.2f) in frame %s",
    last_dock_pose_->pose.position.x,
    last_dock_pose_->pose.position.y,
    last_dock_pose_->header.frame_id.c_str());

  return BT::NodeStatus::SUCCESS;
}

void CheckDockDetectedCondition::dockDetectionCallback(
  geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(detection_mutex_);
  last_dock_pose_ = msg;
  last_detection_time_ = node_->now();

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Dock detection updated: (%.2f, %.2f)",
    msg->pose.position.x,
    msg->pose.position.y);
}

}  // namespace wayfindr_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wayfindr_bt_nodes::CheckDockDetectedCondition>(
    "CheckDockDetected");
}
```

---

## Exploration Goal Action

### Overview
Gets the next exploration frontier from an exploration planner.

### Header File
**Location:** `include/wayfindr_bt_nodes/action/get_next_exploration_goal_action.hpp`

```cpp
#ifndef WAYFINDR_BT_NODES__ACTION__GET_NEXT_EXPLORATION_GOAL_ACTION_HPP_
#define WAYFINDR_BT_NODES__ACTION__GET_NEXT_EXPLORATION_GOAL_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/srv/get_plan.hpp"

namespace wayfindr_bt_nodes
{

/**
 * @brief Action node that gets next exploration frontier goal
 *
 * Calls a service provided by an exploration planner (like explore_lite)
 * to get the next best frontier to explore.
 *
 * Input Ports:
 *   service_name [string]: Exploration service name
 *   min_distance [double]: Minimum frontier distance (meters)
 *   max_distance [double]: Maximum frontier distance (meters)
 *   goal_frame [string]: Frame ID for goal (default: "map")
 *
 * Output Ports:
 *   goal [geometry_msgs::PoseStamped]: Next exploration goal
 *
 * Returns:
 *   SUCCESS: Goal obtained
 *   FAILURE: No frontiers available (exploration complete)
 */
class GetNextExplorationGoalAction : public BT::SyncActionNode
{
public:
  GetNextExplorationGoalAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "service_name",
        "/explore/get_frontier",
        "Service to get exploration frontiers"),
      BT::InputPort<double>(
        "min_distance",
        0.5,
        "Minimum frontier distance (m)"),
      BT::InputPort<double>(
        "max_distance",
        5.0,
        "Maximum frontier distance (m)"),
      BT::InputPort<std::string>(
        "goal_frame",
        "map",
        "Frame ID for goal pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "goal",
        "Next exploration goal")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr exploration_client_;
};

}  // namespace wayfindr_bt_nodes

#endif  // WAYFINDR_BT_NODES__ACTION__GET_NEXT_EXPLORATION_GOAL_ACTION_HPP_
```

### Source File

```cpp
#include "wayfindr_bt_nodes/action/get_next_exploration_goal_action.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace wayfindr_bt_nodes
{

GetNextExplorationGoalAction::GetNextExplorationGoalAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string service_name;
  getInput("service_name", service_name);

  exploration_client_ = node_->create_client<nav_msgs::srv::GetPlan>(
    service_name);

  RCLCPP_INFO(
    node_->get_logger(),
    "GetNextExplorationGoal initialized, service: %s",
    service_name.c_str());
}

BT::NodeStatus GetNextExplorationGoalAction::tick()
{
  // Wait for service
  if (!exploration_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Exploration service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Create request
  auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();

  // Get current robot pose
  geometry_msgs::msg::PoseStamped current_pose;
  // TODO: Get from TF or blackboard
  current_pose.header.frame_id = "map";
  current_pose.header.stamp = node_->now();

  request->start = current_pose;

  // Get parameters
  double min_dist, max_dist;
  std::string goal_frame;
  getInput("min_distance", min_dist);
  getInput("max_distance", max_dist);
  getInput("goal_frame", goal_frame);

  // Call service
  auto result_future = exploration_client_->async_send_request(request);

  // Wait for result
  if (rclcpp::spin_until_future_complete(
      node_, result_future, 10s) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed");
    return BT::NodeStatus::FAILURE;
  }

  auto response = result_future.get();

  // Check if frontier found
  if (response->plan.poses.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "No more frontiers - exploration complete");
    return BT::NodeStatus::FAILURE;
  }

  // Get goal from plan
  geometry_msgs::msg::PoseStamped goal = response->plan.poses.back();
  goal.header.frame_id = goal_frame;

  // Set output
  setOutput("goal", goal);

  RCLCPP_INFO(
    node_->get_logger(),
    "Got exploration goal: (%.2f, %.2f)",
    goal.pose.position.x,
    goal.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace wayfindr_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wayfindr_bt_nodes::GetNextExplorationGoalAction>(
    "GetNextExplorationGoal");
}
```

---

## Waypoint Loader Action

### Overview
Loads waypoints from a file or ROS parameter for patrol missions.

### Implementation

```cpp
#include <string>
#include <vector>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yaml-cpp/yaml.h"

namespace wayfindr_bt_nodes
{

class LoadWaypointsAction : public BT::SyncActionNode
{
public:
  LoadWaypointsAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("waypoint_file", "Path to waypoint YAML file"),
      BT::InputPort<std::string>("frame_id", "map", "Frame for waypoints"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "waypoints", "Loaded waypoint list")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string waypoint_file, frame_id;
    getInput("waypoint_file", waypoint_file);
    getInput("frame_id", frame_id);

    try {
      YAML::Node config = YAML::LoadFile(waypoint_file);
      std::vector<geometry_msgs::msg::PoseStamped> waypoints;

      for (const auto& wp : config["waypoints"]) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = node_->now();

        pose.pose.position.x = wp["x"].as<double>();
        pose.pose.position.y = wp["y"].as<double>();
        pose.pose.position.z = wp["z"].as<double>(0.0);

        pose.pose.orientation.x = wp["qx"].as<double>(0.0);
        pose.pose.orientation.y = wp["qy"].as<double>(0.0);
        pose.pose.orientation.z = wp["qz"].as<double>(0.0);
        pose.pose.orientation.w = wp["qw"].as<double>(1.0);

        waypoints.push_back(pose);
      }

      setOutput("waypoints", waypoints);

      RCLCPP_INFO(
        node_->get_logger(),
        "Loaded %zu waypoints from %s",
        waypoints.size(),
        waypoint_file.c_str());

      return BT::NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to load waypoints: %s",
        e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace wayfindr_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wayfindr_bt_nodes::LoadWaypointsAction>(
    "LoadWaypoints");
}
```

### Waypoint YAML Format

```yaml
# patrol_waypoints.yaml
waypoints:
  - x: 1.0
    y: 1.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0

  - x: 2.0
    y: 1.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.707
    qw: 0.707

  - x: 2.0
    y: 2.0
    z: 0.0
```

---

## Building and Integration

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(wayfindr_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Battery charging condition node
add_library(wayfindr_is_battery_charging_condition_bt_node SHARED
  src/condition/is_battery_charging_condition.cpp)
target_compile_definitions(wayfindr_is_battery_charging_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(wayfindr_is_battery_charging_condition_bt_node
  rclcpp behaviortree_cpp sensor_msgs)

# Dock detection condition node
add_library(wayfindr_check_dock_detected_condition_bt_node SHARED
  src/condition/check_dock_detected_condition.cpp)
target_compile_definitions(wayfindr_check_dock_detected_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(wayfindr_check_dock_detected_condition_bt_node
  rclcpp behaviortree_cpp geometry_msgs)

# Exploration goal action node
add_library(wayfindr_get_next_exploration_goal_action_bt_node SHARED
  src/action/get_next_exploration_goal_action.cpp)
target_compile_definitions(wayfindr_get_next_exploration_goal_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(wayfindr_get_next_exploration_goal_action_bt_node
  rclcpp behaviortree_cpp geometry_msgs nav_msgs)

# Waypoint loader action node
add_library(wayfindr_load_waypoints_action_bt_node SHARED
  src/action/load_waypoints_action.cpp)
target_compile_definitions(wayfindr_load_waypoints_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(wayfindr_load_waypoints_action_bt_node yaml-cpp)
ament_target_dependencies(wayfindr_load_waypoints_action_bt_node
  rclcpp behaviortree_cpp geometry_msgs)

# Install libraries
install(TARGETS
  wayfindr_is_battery_charging_condition_bt_node
  wayfindr_check_dock_detected_condition_bt_node
  wayfindr_get_next_exploration_goal_action_bt_node
  wayfindr_load_waypoints_action_bt_node
  LIBRARY DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>wayfindr_bt_nodes</name>
  <version>1.0.0</version>
  <description>Custom behavior tree nodes for WayfindR robot</description>
  <maintainer email="team@wayfindr.com">WayfindR Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>behaviortree_cpp</depend>
  <depend>nav2_behavior_tree</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>yaml-cpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Loading Custom Plugins

In `nav2_params.yaml`:

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      # Default Nav2 plugins
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      # ... other defaults ...

      # WayfindR custom plugins
      - wayfindr_is_battery_charging_condition_bt_node
      - wayfindr_check_dock_detected_condition_bt_node
      - wayfindr_get_next_exploration_goal_action_bt_node
      - wayfindr_load_waypoints_action_bt_node
```

### Building

```bash
# Navigate to workspace
cd ~/ros2_ws

# Build package
colcon build --packages-select wayfindr_bt_nodes

# Source workspace
source install/setup.bash
```

### Testing Individual Nodes

```cpp
// test/test_battery_charging_condition.cpp
#include <gtest/gtest.h>
#include "wayfindr_bt_nodes/condition/is_battery_charging_condition.hpp"

TEST(BatteryChargingTest, ReturnsSuccessWhenCharging)
{
  // Create test node
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");

  // Create BT factory and register node
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<wayfindr_bt_nodes::IsBatteryChargingCondition>(
    "IsBatteryCharging");

  // Create blackboard and set node
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  // Create tree
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree>
        <IsBatteryCharging battery_topic="/test_battery"/>
      </BehaviorTree>
    </root>
  )", blackboard);

  // Publish test battery message (charging)
  auto pub = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "/test_battery", 10);

  sensor_msgs::msg::BatteryState msg;
  msg.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  pub->publish(msg);

  // Spin to process message
  rclcpp::spin_some(node);

  // Tick tree
  auto status = tree.tickRoot();

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  rclcpp::shutdown();
}
```

---

## Summary

These custom nodes enable:
1. **Battery awareness** - Monitor charging status
2. **Dock detection** - Visual/sensor-based dock finding
3. **Exploration** - Frontier-based autonomous exploration
4. **Waypoint management** - Load and manage patrol routes

All nodes follow Nav2 BT best practices and integrate seamlessly with the existing WayfindR navigation stack.

---

**Last Updated:** 2026-01-11
**Version:** 1.0
