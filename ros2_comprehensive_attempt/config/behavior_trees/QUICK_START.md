# Behavior Trees Quick Start Guide

**For WayfindR Robot - Get Started in 5 Minutes**

---

## 1. Choose Your Mission (Pick One)

### Option A: Simple Patrol Route ⭐ RECOMMENDED FIRST
**Use:** `multi_waypoint_patrol.xml`
**Time to Deploy:** 10 minutes
**Requirements:** Just waypoints!

```bash
# 1. Edit nav2_params.yaml
default_nav_through_poses_bt_xml: "/absolute/path/to/multi_waypoint_patrol.xml"

# 2. Launch Nav2
ros2 launch wayfindr_navigation navigation.launch.py

# 3. Send waypoints (via waypoint_manager.py or action call)
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses ...
```

---

### Option B: Battery-Aware Patrol
**Use:** `patrol_with_battery_management.xml`
**Time to Deploy:** 30 minutes
**Requirements:** Battery state publisher + dock location

```yaml
# Additional config needed:
dock_pose: {x: 0.0, y: 0.0, z: 0.0}
battery_threshold: 15.0
```

---

### Option C: Autonomous Exploration
**Use:** `exploration_behavior.xml`
**Time to Deploy:** 2 hours
**Requirements:** explore_lite package + custom node

```bash
# Install exploration package
sudo apt install ros-humble-explore-lite

# Implement GetNextExplorationGoal node (see custom_node_examples.md)
```

---

### Option D: Docking System
**Use:** `dock_and_charge.xml`
**Time to Deploy:** 1-2 days
**Requirements:** AprilTag detection + 5 custom nodes

```bash
# Requires significant integration work
# See custom_node_examples.md for implementation details
```

---

## 2. Configuration (2 minutes)

### Edit nav2_params.yaml

```yaml
bt_navigator:
  ros__parameters:
    # For single goal navigation
    default_nav_to_pose_bt_xml: "/path/to/your_bt.xml"

    # For multi-waypoint patrol
    default_nav_through_poses_bt_xml: "/path/to/multi_waypoint_patrol.xml"
```

**Important:** Use absolute paths!

---

## 3. Launch (1 command)

```bash
ros2 launch wayfindr_navigation navigation.launch.py
```

**Check logs for:** "BT file loaded successfully"

---

## 4. Test (3 waypoints)

### Method 1: Python Script (waypoint_manager.py)
```python
waypoints = [
    create_pose(1.0, 1.0, 0.0),
    create_pose(2.0, 1.0, 0.0),
    create_pose(2.0, 2.0, 0.0)
]
send_waypoints(waypoints)
```

### Method 2: ROS 2 Action Command
```bash
ros2 action send_goal /navigate_through_poses \
  nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}]}"
```

### Method 3: RViz Nav2 Goal
1. Open RViz
2. Click "Nav2 Goal" button
3. Click on map
4. Repeat for each waypoint

---

## 5. Monitor (Pick One)

### Option A: Terminal Logs
```bash
ros2 topic echo /plan
ros2 topic echo /cmd_vel
```

### Option B: RViz Visualization
- Add `/plan` (Path) display
- Add `/local_plan` (Path) display
- Watch robot navigate!

### Option C: Groot (Advanced)
```bash
# In nav2_params.yaml:
enable_groot_monitoring: true
groot_zmq_publisher_port: 1666

# Launch Groot
ros2 run groot Groot
# Connect to localhost:1666
```

---

## Troubleshooting (1-minute checks)

### ❌ BT Won't Load
```bash
# Check file path (must be absolute)
ls -la /absolute/path/to/bt.xml

# Check XML syntax
cat your_bt.xml  # Look for missing </tags>
```

### ❌ Robot Doesn't Move
```bash
# Check if BT navigator is running
ros2 node list | grep bt_navigator

# Check if goal was received
ros2 topic echo /goal_pose
```

### ❌ Stuck in Recovery
```bash
# Check costmap
ros2 topic echo /local_costmap/costmap

# Check if goal is reachable in RViz
# Try sending goal in open space
```

---

## Quick Customization

### Change Retry Count
```xml
<!-- In your BT XML -->
<RecoveryNode number_of_retries="3">  <!-- Change from 6 to 3 -->
```

### Change Replanning Rate
```xml
<RateController hz="0.5">  <!-- Change from 0.333 to 0.5 (faster) -->
```

### Add Waypoint Pause
```xml
<!-- Add after FollowPath -->
<Wait wait_duration="5.0"/>  <!-- Wait 5 seconds at each waypoint -->
```

### Change Battery Threshold
```xml
<IsBatteryLow min_battery="20.0"/>  <!-- Change from 15.0 to 20.0 -->
```

---

## File Locations

### Behavior Tree XMLs
```
config/behavior_trees/
├── multi_waypoint_patrol.xml              ⭐ Start here
├── patrol_with_battery_management.xml
├── exploration_behavior.xml
└── dock_and_charge.xml
```

### Documentation
```
findings/
├── BEHAVIOR_TREES_SUMMARY.md              📖 Overview
├── nav2_behavior_trees_comprehensive_guide.md  📚 Deep dive
└── BEHAVIOR_TREES_COMPLETION_REPORT.md    ✅ Status

config/behavior_trees/
├── README.md                              📋 Usage guide
└── custom_node_examples.md                💻 C++ code
```

---

## Common Commands

### Launch with Custom BT
```bash
ros2 launch wayfindr_navigation navigation.launch.py \
  default_nav_through_poses_bt_xml:=/path/to/bt.xml
```

### Change BT at Runtime
```bash
ros2 param set /bt_navigator default_nav_to_pose_bt_xml "/path/to/bt.xml"
```

### Monitor BT Execution
```bash
ros2 topic echo /behavior_tree_log
```

### Check BT Navigator Status
```bash
ros2 node info /bt_navigator
```

---

## Success Checklist

- [ ] Chosen behavior tree
- [ ] Updated nav2_params.yaml with absolute path
- [ ] Launched Nav2 successfully
- [ ] Sent test waypoints
- [ ] Robot navigates to goals
- [ ] Recovery works when blocked
- [ ] Reviewed logs for errors

---

## Next Steps

### After First Success:
1. ✅ Try blocking robot's path → Watch recovery
2. ✅ Send new goal mid-navigation → Watch preemption
3. ✅ Test with 5+ waypoints
4. ✅ Set up Groot visualization
5. ✅ Customize parameters

### To Go Further:
1. 📖 Read `BEHAVIOR_TREES_SUMMARY.md`
2. 📚 Study `nav2_behavior_trees_comprehensive_guide.md`
3. 💻 Implement custom nodes (see `custom_node_examples.md`)
4. 🧪 Set up unit tests
5. 🎯 Create mission-specific behavior trees

---

## Getting Help

### Documentation Order:
1. **This file** - Quick start
2. **README.md** - Detailed usage
3. **BEHAVIOR_TREES_SUMMARY.md** - Project overview
4. **nav2_behavior_trees_comprehensive_guide.md** - Complete reference

### Online Resources:
- Nav2 Docs: https://docs.nav2.org/behavior_trees/
- BT.CPP Docs: https://www.behaviortree.dev/
- ROS Answers: https://answers.ros.org/
- Nav2 GitHub: https://github.com/ros-navigation/navigation2/

### Common Questions:

**Q: Which BT should I start with?**
A: `multi_waypoint_patrol.xml` - simplest and most practical.

**Q: Do I need to write C++ code?**
A: Not for basic patrol! Custom nodes only needed for advanced features.

**Q: Can I use these with my existing waypoint_manager.py?**
A: Yes! Just configure the BT and send goals as usual.

**Q: How do I visualize the behavior tree?**
A: Use Groot - see "Monitor" section above.

**Q: What if something doesn't work?**
A: Check "Troubleshooting" section, then consult README.md.

---

## Pro Tips

💡 **Always use absolute paths** in nav2_params.yaml

💡 **Test in simulation first** before deploying to hardware

💡 **Start with default settings** then tune incrementally

💡 **Use Groot** to visualize execution and debug

💡 **Check logs** - BT navigator provides detailed feedback

💡 **Read inline comments** in XML files - they're comprehensive

💡 **Save your customizations** in version control

---

**🚀 You're ready to go! Start with multi_waypoint_patrol.xml and have your robot navigating in 10 minutes.**

---

**Quick Start Version:** 1.0
**Last Updated:** 2026-01-11
**For More Details:** See README.md in this directory
