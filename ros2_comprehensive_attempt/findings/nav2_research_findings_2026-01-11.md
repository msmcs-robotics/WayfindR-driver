# Nav2 Research Findings for Differential Drive Robot with RP LIDAR C1M1
**Date:** January 11, 2026
**Robot Type:** Differential Drive
**Sensor:** RP LIDAR C1M1
**Environment:** Indoor Navigation
**ROS2 Distribution:** Humble

---

## Executive Summary

This document contains comprehensive research findings on Nav2 best practices for differential drive robots equipped with LiDAR sensors, specifically targeting the RP LIDAR C1M1. The research covers controller selection, planner configuration, costmap tuning, AMCL localization parameters, and recovery behaviors optimized for indoor environments.

Key recommendations:
- Use **DWB Controller** for dynamic obstacle avoidance in indoor spaces
- Use **Smac Planner 2D** for cost-aware path planning
- Configure **AMCL** with differential motion model
- Implement comprehensive recovery behaviors (spin, backup, wait)
- Tune costmap layers specifically for RP LIDAR C1M1 characteristics

---

## 1. Controller Selection and Configuration

### 1.1 DWB Controller (Dynamic Window Approach)

**Why DWB for Differential Drive Indoor Navigation:**
- Default controller in Nav2, well-tested and mature
- Supports both omnidirectional and differential drive robots
- Provides dynamic obstacle avoidance while path following
- Highly configurable through critic plugins

**Key Configuration Parameters:**

```yaml
# Velocity Constraints for Differential Drive
max_vel_x: 0.26          # Maximum forward velocity (m/s)
max_vel_y: 0.0           # MUST be 0.0 for differential drive
max_vel_theta: 1.0       # Maximum rotational velocity (rad/s)

# Acceleration Limits
acc_lim_x: 2.5           # Forward acceleration limit
acc_lim_y: 0.0           # No lateral acceleration
acc_lim_theta: 3.2       # Rotational acceleration limit

# Trajectory Sampling
vx_samples: 20           # Forward velocity samples
vy_samples: 1            # Only 1 for differential drive
vtheta_samples: 40       # Rotational velocity samples
sim_time: 1.7            # Lookahead time for simulation
```

**Critic Weights for Indoor Navigation:**
- **PathAlign**: 32.0 - Keeps robot aligned with path
- **GoalAlign**: 24.0 - Ensures proper goal orientation
- **GoalDist**: 24.0 - Drives toward goal efficiently
- **PathDist**: 32.0 - Maintains proximity to planned path
- **RotateToGoal**: 32.0 - Smooth rotation at goal
- **BaseObstacle**: 0.02 - Obstacle avoidance (low weight for indoor)

**Controller Frequency:**
- Recommended: 20 Hz for responsive control
- Balances computational load and control responsiveness

### 1.2 Alternative Controllers

**Regulated Pure Pursuit (RPP):**
- Best for exact path following
- Ideal for differential drive robots
- Good for environments with few dynamic obstacles
- Less computational overhead than DWB

**MPPI Controller:**
- Model Predictive Path Integral controller
- More computationally intensive
- Better for complex dynamic environments
- May have issues with close maneuvering on differential drive

**Recommendation:** Use DWB for indoor navigation with moderate dynamic obstacles. Consider RPP if computational resources are limited and path accuracy is critical.

---

## 2. Planner Configuration

### 2.1 Planner Comparison: NavFn vs Smac Planner

**NavFn Planner:**
- Fast, classical holonomic planner
- Uses circular footprint approximation
- Best for circular differential drive robots
- Makes broad, sweeping curves
- Produces paths ~5% longer by design
- Most computationally efficient

**Smac Planner 2D:**
- Cost-aware A* implementation
- Supports differential drive explicitly
- Considers actual costmap costs in planning
- Better for tight spaces
- More accurate path generation
- Similar performance to NavFn with optimized heuristics

**Smac Planner Lattice:**
- Supports any vehicle type (differential, ackermann, omni)
- Ensures kinematically feasible paths
- More computational overhead
- Best for non-circular robots or complex kinematics

**Recommendation:** Use **Smac Planner 2D** for differential drive indoor navigation. It provides cost-awareness while maintaining good performance.

### 2.2 Smac Planner 2D Configuration

```yaml
GridBased:
  plugin: "nav2_smac_planner/SmacPlanner2D"
  tolerance: 0.125                # Goal tolerance (meters)
  downsample_costmap: false       # Keep full resolution
  allow_unknown: true             # Plan through unknown space
  max_iterations: 1000000         # Maximum search iterations
  max_planning_time: 5.0          # Timeout (seconds)
  motion_model_for_search: "MOORE"  # 8-connected grid
  cost_travel_multiplier: 2.0     # Penalty for high-cost areas
```

**Key Parameters Explained:**
- **tolerance**: Distance from goal considered success (0.125m typical)
- **allow_unknown**: Critical for exploration and partial maps
- **cost_travel_multiplier**: Higher values avoid high-cost areas more aggressively
- **motion_model_for_search**: MOORE (8-connected) vs VON_NEUMANN (4-connected)

---

## 3. AMCL Localization Parameters

### 3.1 Differential Drive Motion Model

**Critical Configuration:**
```yaml
robot_model_type: nav2_amcl::DifferentialMotionModel
```

This setting is **essential** for differential drive robots. Using the wrong motion model will result in poor localization.

### 3.2 Alpha Parameters (Process Noise)

Alpha parameters represent expected odometry noise:

```yaml
alpha1: 0.2  # Rotation noise from rotation
alpha2: 0.2  # Rotation noise from translation
alpha3: 0.2  # Translation noise from translation
alpha4: 0.2  # Translation noise from rotation
alpha5: 0.2  # Translation noise (strafe, omni only)
```

**Tuning Guidelines:**
- Start with 0.2 for all parameters (good baseline)
- Increase if robot has poor odometry (slipping wheels)
- Decrease if odometry is very accurate (high-quality encoders)
- Monitor particle spread during navigation

### 3.3 Particle Filter Settings

```yaml
min_particles: 500       # Minimum particle count
max_particles: 2000      # Maximum particle count
pf_err: 0.05            # Population error threshold
pf_z: 0.99              # Population density (99th percentile)
```

**Particle Count Trade-offs:**
- More particles = Better accuracy, slower computation
- Fewer particles = Faster, less robust to ambiguity
- 500-2000 is optimal for most indoor environments

### 3.4 Update Thresholds

```yaml
update_min_d: 0.25      # Minimum translation (meters)
update_min_a: 0.2       # Minimum rotation (radians)
```

**Why Update Thresholds Matter:**
- Prevents unnecessary updates when robot is stationary
- Reduces computational load
- Improves localization stability
- Values should match typical robot movement patterns

### 3.5 Laser Model Configuration

**For RP LIDAR C1M1:**
```yaml
laser_model_type: likelihood_field
laser_min_range: 0.15    # C1M1 minimum range
laser_max_range: 12.0    # C1M1 maximum range (12 meters)
max_beams: 60            # Reduce for performance
```

**Likelihood Field Parameters:**
```yaml
z_hit: 0.95      # Probability of expected hit
z_short: 0.1     # Probability of short reading
z_max: 0.05      # Probability of max range
z_rand: 0.05     # Probability of random reading
sigma_hit: 0.2   # Standard deviation for hits
```

**Tuning for RP LIDAR C1M1:**
- C1M1 has 12m range, 360° coverage, 0.15m minimum
- High accuracy sensor, use high z_hit (0.95)
- Low noise environment, keep z_rand low (0.05)
- Increase max_beams for better accuracy (trade-off: performance)

---

## 4. Costmap Configuration

### 4.1 Local vs Global Costmap

**Local Costmap:**
- Used by controller for obstacle avoidance
- Rolling window around robot
- Higher update frequency (5 Hz)
- Smaller size (3x3 meters typical)
- Uses odometry frame

**Global Costmap:**
- Used by planner for long-range planning
- Covers entire map
- Lower update frequency (1 Hz)
- Uses map frame
- Includes static layer from map

### 4.2 Costmap Layers

**Three Essential Layers:**

1. **Static Layer** (Global only)
   - Loads map from map server
   - Represents known obstacles

2. **Obstacle/Voxel Layer**
   - Processes sensor data (LiDAR)
   - Marks dynamic obstacles
   - Clears free space via raytracing

3. **Inflation Layer**
   - Creates cost gradient around obstacles
   - Prevents robot from getting too close
   - Critical for smooth navigation

### 4.3 RP LIDAR C1M1 Obstacle Layer Configuration

```yaml
observation_sources: scan

scan:
  topic: /scan
  data_type: "LaserScan"
  max_obstacle_height: 2.0      # Mark obstacles up to 2m
  min_obstacle_height: 0.0      # From ground level
  clearing: true                # Enable raytrace clearing
  marking: true                 # Enable obstacle marking
  obstacle_max_range: 2.5       # Mark obstacles within 2.5m
  obstacle_min_range: 0.0
  raytrace_max_range: 3.0       # Clear space up to 3m
  raytrace_min_range: 0.0
```

**Key Concepts:**
- **obstacle_max_range**: How far to trust obstacle detections
- **raytrace_max_range**: How far to clear free space
- raytrace_max_range should be ≥ obstacle_max_range
- Lower ranges = more conservative, higher = more optimistic

**Why Different Ranges:**
- RP LIDAR C1M1 can see up to 12m
- But for safety, only mark obstacles within 2.5m as definite
- Clear space up to 3m to avoid phantom obstacles
- Balances safety and navigation efficiency

### 4.4 Inflation Layer Tuning

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.55
```

**Parameters Explained:**
- **inflation_radius**: Distance to inflate obstacles (meters)
  - Should be > robot_radius for safety margin
  - Typical: robot_radius + 0.3m

- **cost_scaling_factor**: Rate of cost decay
  - Higher = steeper gradient (robot avoids obstacles more)
  - Lower = gentler gradient (robot can get closer)
  - Range: 1.0-10.0, typical indoor: 3.0-5.0

**Tuning Strategy:**
1. Start with inflation_radius = robot_radius + 0.3
2. If robot gets too close: increase radius or scaling factor
3. If robot is too conservative: decrease values
4. Monitor costmap visualization in RViz

### 4.5 Voxel Layer vs Obstacle Layer

**Obstacle Layer:**
- 2D representation
- Simpler, faster
- Good for flat environments

**Voxel Layer:**
- 3D representation
- Handles multi-level obstacles
- Better clearing behavior
- Recommended for complex indoor spaces

**Recommendation:** Use Voxel Layer for local costmap, Obstacle Layer for global costmap (performance optimization).

---

## 5. Recovery Behaviors

### 5.1 Behavior Server Configuration

Recovery behaviors help robot escape stuck situations:

**Standard Recovery Behaviors:**

1. **Spin** - Rotate in place
   ```yaml
   spin:
     plugin: "nav2_behaviors::Spin"
     max_rotational_vel: 1.0
     min_rotational_vel: 0.4
     rotational_acc_lim: 3.2
     simulate_ahead_time: 2.0
   ```

2. **Backup** - Drive in reverse
   ```yaml
   backup:
     plugin: "nav2_behaviors::BackUp"
     backup_speed: -0.15      # Negative = reverse
     backup_time: 2.0         # Duration
   ```

3. **Wait** - Pause for dynamic obstacles
   ```yaml
   wait:
     plugin: "nav2_behaviors::Wait"
   ```

4. **Drive on Heading** - Move in specific direction
   ```yaml
   drive_on_heading:
     plugin: "nav2_behaviors::DriveOnHeading"
     max_linear_vel: 0.26
     min_linear_vel: -0.15
     linear_acc_lim: 2.5
   ```

### 5.2 Rotation Shim Controller

For differential drive robots in tight spaces:

```yaml
rotation_shim:
  plugin: "nav2_rotation_shim_controller/RotationShimController"
  angular_dist_threshold: 0.785    # 45 degrees
  forward_sampling_distance: 0.5
  rotate_to_heading_angular_vel: 1.8
  max_angular_accel: 3.2
```

**How It Works:**
- Checks heading difference to new path
- If > threshold, rotates in place first
- Then passes control to primary controller
- Prevents "crabbing" motion in differential drive

**When to Use:**
- Tight indoor corridors
- Doorways and narrow passages
- When path following accuracy is critical

### 5.3 Recovery Behavior Tree

Nav2 uses behavior trees for recovery sequences:

**Typical Recovery Sequence:**
1. Clear costmap
2. Spin (360°)
3. Wait (for dynamic obstacles to clear)
4. Backup (if still stuck)
5. Spin again
6. Retry navigation

**Customization:**
- Edit behavior tree XML files
- Adjust recovery sequence order
- Add custom behaviors
- Configure failure thresholds

---

## 6. Differential Drive Specific Optimizations

### 6.1 Kinematic Constraints

**Critical Settings for Differential Drive:**

```yaml
# Controller
max_vel_y: 0.0           # MUST be zero (no lateral motion)
acc_lim_y: 0.0           # MUST be zero

# Goal Checker
xy_goal_tolerance: 0.15  # Position tolerance
yaw_goal_tolerance: 0.15 # Orientation tolerance
```

### 6.2 Goal Orientation Handling

For symmetric differential drive robots:

```yaml
symmetric_yaw_tolerance: true
```

**Benefit:** Allows robot to reach goals without unnecessary 180° rotations if robot can function in either direction.

**Use Case:** Robots with sensors on both ends, or where orientation doesn't matter for the task.

### 6.3 Footprint Configuration

**Circular Robot:**
```yaml
robot_radius: 0.22  # Single radius value
```

**Non-Circular Robot:**
```yaml
footprint: "[[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]"
```

**Why It Matters:**
- Accurate footprint enables planning into tighter spaces
- Circular approximation is conservative but safe
- Non-circular footprint requires more computation
- Use actual robot dimensions for best results

### 6.4 Odometry Covariance

**Recommended Starting Values:**
```yaml
# In robot driver configuration (not Nav2 params)
pose_covariance: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
twist_covariance: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
```

**Tuning Process:**
1. Record odometry while driving straight
2. Compare to ground truth (SLAM or external tracking)
3. Calculate actual variance
4. Update covariance matrices
5. Higher values = less trust in odometry

---

## 7. Indoor Navigation Best Practices

### 7.1 Environmental Considerations

**Indoor Challenges:**
- Dynamic obstacles (people, doors)
- Narrow corridors
- Glass/reflective surfaces
- Poor GPS/external localization

**Nav2 Solutions:**
- High-frequency costmap updates
- Conservative inflation parameters
- Robust recovery behaviors
- Adaptive localization (AMCL)

### 7.2 RP LIDAR C1M1 Specific Tuning

**Sensor Characteristics:**
- Range: 0.15m - 12m
- Frequency: 10 Hz typical
- Accuracy: ±20mm typical
- Coverage: 360°

**Optimization Tips:**
1. Set laser_min_range to 0.15m (sensor spec)
2. Use full 12m range for global costmap
3. Limit local costmap to 2.5-3m for safety
4. Increase max_beams if CPU allows (better accuracy)
5. Monitor scan quality in RViz

### 7.3 Performance Optimization

**Computational Trade-offs:**

| Component | Parameter | Fast | Accurate |
|-----------|-----------|------|----------|
| AMCL | max_particles | 500 | 2000 |
| AMCL | max_beams | 30 | 180 |
| Costmap | update_frequency | 1 Hz | 5 Hz |
| Costmap | resolution | 0.1 | 0.05 |
| Controller | controller_frequency | 10 Hz | 20 Hz |
| Planner | max_planning_time | 2s | 5s |

**Recommended for WayfindR:**
- Balanced configuration (middle ground)
- Start conservative, optimize if needed
- Monitor CPU usage with `top` or `htop`
- Profile with `ros2 topic hz` and `ros2 topic bw`

### 7.4 Testing and Validation

**Test Scenarios:**
1. **Straight line** - Verify path following accuracy
2. **90° turns** - Check rotation behavior
3. **Narrow corridor** - Test collision avoidance
4. **Dynamic obstacles** - Recovery behavior validation
5. **Goal orientations** - Ensure proper final pose
6. **Long distance** - Path planning efficiency

**Metrics to Track:**
- Success rate
- Time to goal
- Path length efficiency
- Recovery behavior frequency
- Localization accuracy (particle spread)
- CPU/memory usage

---

## 8. Common Issues and Solutions

### 8.1 Robot Won't Move

**Possible Causes:**
1. Incorrect velocity limits (max_vel_x = 0)
2. Costmap entirely occupied
3. Goal inside inflation radius
4. Transform issues (TF tree)

**Solutions:**
1. Verify controller parameters
2. Check costmap visualization in RViz
3. Adjust inflation parameters
4. Run `ros2 run tf2_tools view_frames.py`

### 8.2 Poor Localization

**Symptoms:**
- Particle cloud spread out
- Robot jumps around map
- Failed navigation attempts

**Solutions:**
1. Increase max_particles
2. Tune alpha parameters (increase for noisy odometry)
3. Verify laser_min/max_range matches sensor
4. Check map quality (clear features needed)
5. Ensure good initial pose estimate

### 8.3 Oscillating Behavior

**Causes:**
- DWB critics imbalanced
- Inflation radius too large
- Controller frequency too low

**Solutions:**
1. Reduce PathAlign/GoalAlign weights
2. Decrease inflation_radius
3. Increase controller_frequency
4. Add oscillation critic with higher weight

### 8.4 Robot Gets Stuck

**Recovery Strategy:**
1. Ensure all recovery behaviors enabled
2. Tune behavior parameters (spin speed, backup distance)
3. Consider rotation shim controller
4. Adjust progress checker thresholds
5. Increase failure_tolerance

---

## 9. Advanced Features

### 9.1 Waypoint Following

```yaml
waypoint_follower:
  loop_rate: 20
  stop_on_failure: false
  waypoint_task_executor_plugin: "wait_at_waypoint"
```

**Use Cases:**
- Multi-goal navigation
- Patrol routes
- Delivery missions

### 9.2 Velocity Smoother

```yaml
velocity_smoother:
  smoothing_frequency: 20.0
  max_velocity: [0.26, 0.0, 1.0]
  max_accel: [2.5, 0.0, 3.2]
```

**Benefits:**
- Smoother motion
- Reduced mechanical stress
- Better trajectory tracking
- Improved passenger comfort (if applicable)

### 9.3 Collision Monitor

```yaml
collision_monitor:
  base_frame_id: "base_link"
  polygons: ["FootprintApproach"]
  observation_sources: ["scan"]
```

**Purpose:**
- Emergency stop for imminent collisions
- Additional safety layer
- Complements costmap-based avoidance

### 9.4 Keepout Zones

**Configuration:**
- Define zones in map
- Robot avoids these areas
- Useful for restricted areas, hazards

**Implementation:**
- Use costmap filter plugins
- Define keepout filter mask
- Load via map server

---

## 10. Recommended Parameter Values Summary

### For WayfindR Differential Drive Robot with RP LIDAR C1M1

**Controller (DWB):**
- max_vel_x: 0.26 m/s
- max_vel_theta: 1.0 rad/s
- acc_lim_x: 2.5 m/s²
- acc_lim_theta: 3.2 rad/s²
- controller_frequency: 20 Hz

**Planner (Smac 2D):**
- tolerance: 0.125 m
- max_planning_time: 5.0 s
- cost_travel_multiplier: 2.0

**AMCL:**
- min_particles: 500
- max_particles: 2000
- alpha1-4: 0.2
- laser_max_range: 12.0 m
- max_beams: 60

**Local Costmap:**
- size: 3x3 m
- resolution: 0.05 m
- update_frequency: 5 Hz
- obstacle_max_range: 2.5 m
- inflation_radius: 0.55 m

**Global Costmap:**
- resolution: 0.05 m
- update_frequency: 1 Hz
- inflation_radius: 0.55 m

---

## 11. References and Resources

### Official Nav2 Documentation
- Nav2 Documentation: https://docs.nav2.org/
- Tuning Guide: https://docs.nav2.org/tuning/index.html
- DWB Controller: https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html
- Smac Planner: https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- AMCL: https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Costmap 2D: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- Behavior Server: https://docs.nav2.org/configuration/packages/configuring-behavior-server.html

### Community Resources
- ROS 2 Navigation Tuning Guide: https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/
- ThinkRobotics Nav2 Guide: https://thinkrobotics.com/blogs/learn/ros-2-navigation2-configuration
- GitHub Issues: https://github.com/ros-navigation/navigation2/issues

### Academic Resources
- Smac Planner Paper: "Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics" (2024)
- "From the Desks of ROS Maintainers: A Survey of Modern & Capable" (2023)

---

## 12. Configuration File Organization

The configuration files created from this research:

### /config/nav2_params.yaml
Comprehensive Nav2 parameters including:
- AMCL configuration
- Controller server (DWB)
- Planner server (Smac 2D)
- Behavior server
- Costmap configuration (local and global)
- Velocity smoother
- Waypoint follower
- Collision monitor
- Lifecycle managers

### /config/rviz_nav2.rviz
RViz visualization configuration including:
- Robot model display
- LaserScan visualization
- Map display
- Global and local costmaps
- Global and local plans
- Particle cloud (AMCL)
- TF frames
- Goal pose visualization
- Footprint displays

### /launch/navigation.launch.py
Launch file that starts:
- Map server
- AMCL localization
- Full Nav2 navigation stack
- RViz with proper configuration
- Lifecycle managers

---

## 13. Next Steps and Future Improvements

### Immediate Actions
1. Test configuration with actual hardware
2. Tune parameters based on real-world performance
3. Create test maps for validation
4. Benchmark performance metrics

### Tuning Process
1. Start with provided baseline parameters
2. Test in controlled environment
3. Adjust one parameter at a time
4. Document changes and results
5. Iterate until satisfactory performance

### Advanced Optimizations
1. Custom behavior tree for specific use case
2. Dynamic reconfigure for runtime tuning
3. Map annotation with semantic information
4. Multi-robot coordination (if applicable)
5. Integration with higher-level planning

### Monitoring and Maintenance
1. Log navigation metrics
2. Track failure cases
3. Update parameters seasonally (if environment changes)
4. Review Nav2 updates and improvements
5. Participate in Nav2 community discussions

---

## Conclusion

This research provides a comprehensive foundation for implementing Nav2 on a differential drive robot with RP LIDAR C1M1. The configuration files created are based on current best practices as of January 2026 and are optimized for indoor navigation scenarios.

Key takeaways:
- Nav2 is highly configurable and requires tuning for specific robots
- Differential drive robots need specific kinematic constraints (max_vel_y = 0)
- RP LIDAR C1M1 provides excellent range (12m) suitable for indoor navigation
- Balanced parameters trade-off between performance and accuracy
- Recovery behaviors are critical for robust autonomous navigation
- Continuous tuning and testing are essential for optimal performance

The provided configuration serves as an excellent starting point but should be refined through real-world testing and iterative improvements.

---

**Document Version:** 1.0
**Last Updated:** 2026-01-11
**Author:** Claude Code (Research and Configuration)
**Status:** Ready for implementation and testing
