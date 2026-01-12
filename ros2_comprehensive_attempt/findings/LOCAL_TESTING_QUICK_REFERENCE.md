# Local Testing Quick Reference

**Quick access guide to testing checklist sections**
**Related:** [Full Checklist](LOCAL_TESTING_CHECKLIST.md)

---

## Test Categories Summary

| Category | Tests | Critical | Time Est. |
|----------|-------|----------|-----------|
| Prerequisites | 15 | YES | 15 min |
| Component Testing | 45 | YES | 60 min |
| Integration Testing | 12 | YES | 30 min |
| Performance Testing | 6 | RECOMMENDED | 20 min |
| Regression Testing | 4 | RECOMMENDED | 15 min |
| Documentation | 4 | OPTIONAL | 10 min |

**Total Time:** ~2.5 hours for complete validation

---

## Critical Path Testing (Minimum 30 Minutes)

Run these essential tests before any deployment:

### 1. Prerequisites (5 min)
```bash
# Check ROS2
source /opt/ros/humble/setup.bash
ros2 --version

# Check packages
ros2 pkg list | grep -E "(nav2_amcl|slam_toolbox)"

# Check Python
python3 -c "import rclpy, tf2_ros, yaml, numpy"
```

### 2. URDF Validation (2 min)
```bash
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt
./scripts/validate_urdf.sh
```

### 3. Generate Test Data (3 min)
```bash
cd scripts/testing
./generate_test_bag.sh validation_test 30 circular
```

### 4. SLAM Test (8 min)
```bash
./test_slam_with_bag.sh validation_test
python3 analyze_slam_quality.py slam_test_*/final_map.yaml
```

### 5. Localization Test (7 min)
```bash
./test_localization_with_bag.sh slam_test_*/final_map.yaml validation_test
```

### 6. Launch System Test (5 min)
```bash
cd ../..
timeout 20 ./scripts/start_navigation.sh \
  --map scripts/testing/slam_test_*/final_map.yaml \
  --no-rviz
```

**Result:** If all 6 pass, system is minimally validated.

---

## Full Validation Script

Use the automated quick validation:

```bash
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt
./scripts/quick_validate.sh
```

This runs 9 essential checks in ~5 minutes.

---

## Component-by-Component Testing

### URDF and Robot Description
```bash
# Validation
./scripts/validate_urdf.sh

# Interactive test
ros2 launch launch/robot_state_publisher.launch.py

# Check TF
ros2 run tf2_ros tf2_echo base_link laser
```

### SLAM Mapping
```bash
# Quick test with synthetic data
cd scripts/testing
./generate_test_bag.sh slam_test 30 circular
./test_slam_with_bag.sh slam_test

# Analyze quality
python3 analyze_slam_quality.py slam_test_*/final_map.yaml

# Interactive test
cd ../..
./scripts/start_mapping.sh
```

### AMCL Localization
```bash
# Automated test
cd scripts/testing
./test_localization_with_bag.sh slam_test_*/final_map.yaml slam_test

# Interactive test
cd ../..
./scripts/start_localization.sh --map scripts/testing/slam_test_*/final_map.yaml
```

### Nav2 Navigation
```bash
# Launch full stack
./scripts/start_navigation.sh --map scripts/testing/slam_test_*/final_map.yaml

# Check nodes
ros2 node list | grep -E "(controller|planner|bt_navigator)"

# Test goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

### cmd_vel Bridge
```bash
# Mock test
ros2 launch launch/cmd_vel_bridge.launch.py pi_api_url:=http://localhost:9999 &
sleep 5
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.1}}"
```

### Diagnostics
```bash
cd scripts/diagnostics

# Test individual tools
python3 system_diagnostics.py --help
python3 tf_tree_visualizer.py --help
python3 topic_checker.py --help

# Full suite
./run_full_diagnostics.sh /tmp/diag_output
```

### Behavior Trees
```bash
# Validate syntax
cd config/behavior_trees
for bt in *.xml; do
  xmllint --noout "$bt" && echo "✓ $bt"
done
```

---

## Integration Testing Commands

### TF Tree Check
```bash
# Launch system
./scripts/start_navigation.sh --map <map.yaml> --no-rviz &
sleep 20

# Generate TF tree
ros2 run tf2_tools view_frames

# View
evince frames_*.pdf
```

### Topic Connectivity
```bash
# Check topics exist
ros2 topic list | grep -E "(scan|odom|cmd_vel|map|amcl_pose)"

# Check data flow
ros2 topic hz /scan /odom /amcl_pose
```

### Parameter Verification
```bash
# SLAM parameters
ros2 param get /slam_toolbox resolution
ros2 param get /slam_toolbox max_laser_range

# AMCL parameters
ros2 param get /amcl min_particles
ros2 param get /amcl max_particles

# Nav2 parameters
ros2 param get /controller_server controller_frequency
```

---

## Performance Testing Commands

### Resource Monitoring
```bash
# During navigation, monitor resources
top -b -n 1 | grep -E "(slam_toolbox|amcl|planner|controller)"

# Memory check
free -h
ps aux --sort=-%mem | head -10

# Detailed profiling
cd scripts/diagnostics
python3 performance_profiler.py --output /tmp/perf_report.yaml
```

### Quality Metrics
```bash
# SLAM map quality
cd scripts/testing
python3 analyze_slam_quality.py <map.yaml>

# Localization quality
cd ../diagnostics
python3 localization_quality.py
```

---

## Regression Testing Commands

### Legacy Launch Files
```bash
# Test old SLAM launch
ros2 launch launch/slam.launch.py use_rviz:=false

# Test old localization launch
ros2 launch launch/localization.launch.py \
  map:=<map.yaml> \
  use_rviz:=false
```

### Backward Compatibility
```bash
# Check old scripts work
./scripts/start_mapping.sh --help
./scripts/start_localization.sh --help

# Verify configs parse
cd config
for cfg in *.yaml; do
  python3 -c "import yaml; yaml.safe_load(open('$cfg'))" && echo "✓ $cfg"
done
```

---

## Common Validation Failures

### Failure: ROS2 not found
**Fix:**
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Failure: Package missing
**Fix:**
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-slam-toolbox ros-humble-rplidar-ros
```

### Failure: Python import error
**Fix:**
```bash
pip3 install pyyaml numpy pillow rich psutil matplotlib
```

### Failure: Permission denied
**Fix:**
```bash
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt
chmod +x scripts/*.sh scripts/testing/*.sh scripts/diagnostics/*.sh
```

### Failure: URDF invalid
**Fix:**
```bash
sudo apt install liburdfdom-tools
check_urdf <(xacro urdf/wayfinder_robot.urdf.xacro)
# Review error messages and fix URDF
```

### Failure: TF tree incomplete
**Fix:**
```bash
# Check publishers
ros2 node list

# Verify transforms
ros2 topic echo /tf_static
ros2 run tf2_tools view_frames

# Check robot_state_publisher running
ros2 lifecycle get /robot_state_publisher
```

### Failure: Poor map quality
**Fix:**
- Increase test bag duration: `./generate_test_bag.sh test 60 circular`
- Use varied motion patterns
- Tune SLAM parameters in `config/slam_params.yaml`

### Failure: Localization not converging
**Fix:**
- Increase particles: Edit `config/amcl_params.yaml`
- Set better initial pose in RViz
- Check scan quality: `ros2 topic echo /scan`

---

## Quick Checklist Status

Use this for at-a-glance status:

```
Prerequisites:
□ ROS2 Humble installed
□ Nav2 packages present
□ SLAM Toolbox present
□ Python deps installed

Core Components:
□ URDF validated
□ Test bag generated
□ SLAM tested
□ AMCL tested
□ Nav2 tested
□ Bridge tested
□ Diagnostics tested
□ Behavior trees valid
□ Launch system works

Integration:
□ TF tree complete
□ Topics connected
□ Parameters loaded
□ Launches execute

Performance:
□ Map quality >60
□ Localization stable
□ Resources acceptable

Regression:
□ Old launches work
□ Backward compatible

Documentation:
□ READMEs accurate
□ Examples work
```

**All boxes checked?** → Ready for deployment!

---

## Testing by Time Available

### 5 Minutes: Smoke Test
```bash
./scripts/quick_validate.sh
```

### 15 Minutes: Essential Validation
1. Prerequisites check (3 min)
2. URDF validation (2 min)
3. Generate test bag (3 min)
4. SLAM test (5 min)
5. Check quality (2 min)

### 30 Minutes: Critical Path
Essential validation + localization test + launch system test

### 1 Hour: Component Testing
All components individually validated

### 2+ Hours: Full Validation
Complete checklist including performance and regression tests

---

## Files and Locations

| Item | Path |
|------|------|
| Full Checklist | `findings/LOCAL_TESTING_CHECKLIST.md` |
| Quick Validation Script | `scripts/quick_validate.sh` |
| Testing Scripts | `scripts/testing/` |
| Diagnostic Tools | `scripts/diagnostics/` |
| Configuration Files | `config/` |
| Launch Files | `launch/` |
| Test Data Output | `scripts/testing/slam_test_*/` |

---

## Getting Help

1. **Full checklist:** See `LOCAL_TESTING_CHECKLIST.md`
2. **Component guides:** See `findings/` directory
3. **Script help:** Run any script with `--help`
4. **Diagnostics validation:** See `scripts/diagnostics/VALIDATION_CHECKLIST.md`
5. **Testing guide:** See `scripts/testing/README.md`

---

**Version:** 1.0
**Last Updated:** 2026-01-11
**See Also:** [Full Testing Checklist](LOCAL_TESTING_CHECKLIST.md)
