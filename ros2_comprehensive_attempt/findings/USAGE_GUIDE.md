# Nav2 Configuration Usage Guide

## Quick Start

### Prerequisites
1. ROS2 Humble installed
2. Nav2 packages installed: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
3. Map file created (use SLAM to create a map first)
4. RP LIDAR C1M1 driver running and publishing to `/scan` topic
5. Robot description (URDF) published to `/robot_description` topic
6. Odometry published to `/odom` topic

### Launch Navigation

```bash
# Navigate to the package directory
cd /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/

# Launch navigation with your map
ros2 launch launch/navigation.launch.py map:=/path/to/your/map.yaml
```

### Launch Options

```bash
# With simulation time (for Gazebo)
ros2 launch launch/navigation.launch.py map:=/path/to/map.yaml use_sim_time:=true

# Without RViz (headless)
ros2 launch launch/navigation.launch.py map:=/path/to/map.yaml use_rviz:=false

# With custom parameters file
ros2 launch launch/navigation.launch.py map:=/path/to/map.yaml params_file:=/path/to/custom_params.yaml

# With custom RViz config
ros2 launch launch/navigation.launch.py map:=/path/to/map.yaml rviz_config:=/path/to/custom.rviz
```

## Files Created

### 1. config/nav2_params.yaml (16 KB)
Comprehensive Nav2 configuration including:
- **AMCL**: Localization parameters for differential drive
- **Controller Server**: DWB controller with optimized indoor navigation parameters
- **Planner Server**: Smac Planner 2D for cost-aware path planning
- **Behavior Server**: Recovery behaviors (spin, backup, wait, drive on heading)
- **Costmaps**: Local and global costmap configuration
- **Velocity Smoother**: Smooth velocity commands
- **Waypoint Follower**: Multi-goal navigation
- **Collision Monitor**: Emergency collision avoidance

### 2. config/rviz_nav2.rviz (13 KB)
RViz2 configuration with displays for:
- Robot model
- LaserScan (/scan)
- Map
- Global and local costmaps
- Global and local plans
- Particle cloud (AMCL)
- TF frames
- Goal pose
- Footprints
- Voxel grid

### 3. launch/navigation.launch.py (7.9 KB)
Launch file that starts:
- Map server
- AMCL localization
- Controller server
- Planner server
- Behavior server
- BT Navigator
- Waypoint follower
- Velocity smoother
- Lifecycle managers
- RViz2 (optional)

### 4. findings/nav2_research_findings_2026-01-11.md (23 KB)
Comprehensive research document covering:
- Controller selection (DWB vs RPP vs MPPI)
- Planner comparison (NavFn vs Smac)
- AMCL tuning for differential drive
- Costmap configuration for RP LIDAR C1M1
- Recovery behaviors
- Indoor navigation best practices
- Performance optimization
- Troubleshooting guide

## Setting Initial Pose in RViz

1. Click "2D Pose Estimate" button in RViz toolbar
2. Click on the map where the robot is located
3. Drag to set the robot's orientation
4. Watch particle cloud converge around robot

## Setting Navigation Goal

1. Click "2D Goal Pose" button in RViz toolbar
2. Click on the map where you want the robot to go
3. Drag to set desired final orientation
4. Robot will plan and execute path

## Monitoring Navigation

### Check Status
```bash
# View active nodes
ros2 node list

# Check controller status
ros2 topic echo /controller_server/transition_event

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check navigation result
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Visualize Topics
```bash
# View costmaps
ros2 run rqt_image_view rqt_image_view /local_costmap/costmap
ros2 run rqt_image_view rqt_image_view /global_costmap/costmap

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

## Tuning Parameters

### When Robot is Too Conservative
- Decrease `inflation_radius` in costmap config
- Decrease `cost_scaling_factor`
- Decrease `obstacle_max_range`

### When Robot Gets Too Close to Obstacles
- Increase `inflation_radius`
- Increase `cost_scaling_factor`
- Increase `obstacle_max_range`

### When Robot Moves Too Fast
- Decrease `max_vel_x`
- Decrease `max_vel_theta`
- Decrease acceleration limits

### When Robot is Too Slow
- Increase `max_vel_x`
- Increase `max_vel_theta`
- Increase `controller_frequency`

### When Localization is Poor
- Increase `max_particles`
- Increase `max_beams`
- Tune `alpha1-4` parameters
- Check map quality

### When Path Planning Fails
- Increase `max_planning_time`
- Decrease `tolerance`
- Set `allow_unknown: true`
- Check costmap inflation

## Common Issues

### Issue: "Timed out waiting for transform"
**Solution:** Check TF tree with `ros2 run tf2_tools view_frames`
- Ensure map→odom→base_link chain exists
- Check that AMCL is publishing map→odom transform
- Verify robot_state_publisher is running for odom→base_link

### Issue: Robot won't move
**Solution:**
1. Check velocity limits in nav2_params.yaml
2. Verify costmap isn't fully occupied (check RViz)
3. Ensure goal is not inside inflation radius
4. Check that cmd_vel topic has subscribers: `ros2 topic info /cmd_vel`

### Issue: Oscillating behavior
**Solution:**
1. Reduce PathAlign and GoalAlign weights in DWB config
2. Increase controller_frequency
3. Decrease inflation_radius

### Issue: Robot gets stuck frequently
**Solution:**
1. Verify recovery behaviors are enabled
2. Tune spin and backup parameters
3. Consider adding rotation shim controller
4. Check progress_checker parameters

## Performance Monitoring

```bash
# Check CPU usage
top -H -p $(pgrep -f "controller_server|planner_server|amcl")

# Check topic frequencies
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel

# Check topic bandwidth
ros2 topic bw /scan
ros2 topic bw /local_costmap/costmap

# Monitor transforms
ros2 run tf2_ros tf2_monitor
```

## Integration with Your Robot

### Required Topics (Your robot must publish)
- `/scan` (sensor_msgs/LaserScan) - from RP LIDAR C1M1
- `/odom` (nav_msgs/Odometry) - from wheel encoders
- `/robot_description` (std_msgs/String) - URDF description

### Required TF Frames (Your robot must provide)
- `odom` → `base_footprint` → `base_link` → `base_scan`

### Subscribed Topics (Nav2 will use)
- `/scan` - for costmap obstacle layer
- `/odom` - for controller and localization

### Published Topics (Nav2 will provide)
- `/cmd_vel` - velocity commands to robot
- `/plan` - global path
- `/local_plan` - local trajectory
- `/particle_cloud` - AMCL particles
- `/map` - loaded map
- TF: `map` → `odom` (from AMCL)

## Next Steps

1. **Create a map** using SLAM (e.g., slam_toolbox)
2. **Test localization** with AMCL on the map
3. **Tune parameters** based on robot behavior
4. **Test navigation** in controlled environment
5. **Iterate and improve** based on performance

## Additional Resources

- Nav2 Official Docs: https://docs.nav2.org/
- Research Findings: See `nav2_research_findings_2026-01-11.md`
- Nav2 Tutorials: https://navigation.ros.org/tutorials/index.html
- ROS2 Docs: https://docs.ros.org/

## Support

For issues or questions:
1. Check the research findings document for detailed explanations
2. Review Nav2 official documentation
3. Search Nav2 GitHub issues: https://github.com/ros-navigation/navigation2/issues
4. Post on ROS Discourse: https://discourse.ros.org/

---

**Created:** 2026-01-11
**Configuration Version:** 1.0
**ROS2 Distribution:** Humble
**Target Robot:** WayfindR Differential Drive with RP LIDAR C1M1
