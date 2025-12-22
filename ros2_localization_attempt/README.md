# ROS2 Localization Attempt

**Purpose:** Load an existing map and determine the robot's position on it using LiDAR data.

**Status:** ✅ Tested and working on remote system (192.168.0.7)

## What is Localization?

Localization answers the question: **"Where am I on this map?"**

Given:
- A pre-built map (from SLAM)
- Current sensor data (LiDAR scans)

The system estimates:
- Robot's position (x, y)
- Robot's orientation (theta/yaw)
- Confidence in the estimate

## Quick Start (On Remote System)

```bash
# SSH into the remote system or use the physical terminal
ssh devel@192.168.0.7

# Run the helper script (on the monitor)
~/start_localization.sh
```

This launches:
- RPLidar driver (Slamtec C1M1RP)
- Map server with `first_map.yaml`
- AMCL localization node
- Static TF transforms
- RViz visualization

### Set Initial Pose in RViz

1. Click **"2D Pose Estimate"** button in toolbar
2. Click on the map where you think the robot is
3. Drag to set orientation (green arrow)
4. Watch the particles converge!

### Monitor Localization Quality

```bash
# In another terminal
source /opt/ros/humble/setup.bash
python3 ~/Desktop/WayfindR-driver/ros2_localization_attempt/scripts/monitor_pose.py
```

## Localization Methods

### 1. AMCL (Adaptive Monte Carlo Localization) - **Used Here**
- **How it works:** Uses particle filter with many "guesses" of robot position
- **Pros:** Robust, handles ambiguity well, can recover from kidnapping
- **Cons:** Computationally heavier, requires good initial estimate
- **Package:** `ros-humble-nav2-amcl`

### 2. SLAM Toolbox Localization Mode
- **How it works:** Uses the same SLAM algorithm but in "localization only" mode
- **Pros:** Can use saved pose graph, smooth updates
- **Cons:** Less robust to large initial errors
- **Package:** `ros-humble-slam-toolbox`

## Files in This Directory

| File | Purpose |
|------|---------|
| `config/amcl_params.yaml` | AMCL configuration (tuned for Slamtec C1) |
| `config/slam_localization_params.yaml` | SLAM Toolbox localization config |
| `config/localization_view.rviz` | RViz config showing map, scan, particles |
| `launch/localization.launch.py` | AMCL-only launch (no LiDAR) |
| `launch/localization_with_lidar.launch.py` | Full launch with RPLidar + AMCL |
| `scripts/set_initial_pose.py` | Programmatically set robot pose |
| `scripts/monitor_pose.py` | Real-time pose monitoring |
| `scripts/start_localization.sh` | Helper script |

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LiDAR data |
| `/map` | nav_msgs/OccupancyGrid | Loaded map |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | Robot pose estimate |
| `/particlecloud` | nav2_msgs/ParticleCloud | Particle distribution |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | Set initial pose |

## Prerequisites

- A saved map (`.yaml` + `.pgm` files)
- LiDAR connected at `/dev/ttyUSB0`
- ROS2 Humble with Nav2 installed

## Troubleshooting

### "Transform odom to base_link not available"
The launch file includes static transforms. If running AMCL standalone:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

### Particles scattered everywhere
- Initial pose estimate is wrong - use "2D Pose Estimate" in RViz
- Map doesn't match current environment
- LiDAR not publishing - check `/scan` topic

### Robot "jumps" around
- AMCL confidence is low
- Try increasing number of particles in `amcl_params.yaml`
- Check LiDAR is publishing valid scans

### RViz won't start via SSH
RViz needs a display. Either:
1. Run on the physical monitor attached to the remote system
2. Use X11 forwarding: `ssh -X devel@192.168.0.7`

---

## Waypoint Navigation

The system includes waypoint management and A* pathfinding.

### Adding Waypoints

```bash
# Add office waypoints to the map
cd ~/Desktop/WayfindR-driver/ros2_localization_attempt/scripts
python3 add_office_waypoints.py
```

This creates `~/ros2_ws/maps/first_map_offices.yaml` with:
- **office1**: Left side of map (-1.58, -0.49)
- **office2**: Center of map (0.42, 0.51)
- **office3**: Right side of map (2.42, -0.49)

Plus routes:
- `office_tour`: office1 → office2 → office3
- `office_tour_reverse`: office3 → office2 → office1
- `office_patrol`: office1 → office2 → office3 → office2

### Pathfinding

```bash
source /opt/ros/humble/setup.bash
cd ~/Desktop/WayfindR-driver/ros2_localization_attempt/scripts

# Plan path to a specific office
python3 navigate_to_office.py office2 --dry-run

# Plan full route (shows all paths)
python3 navigate_to_office.py --route office_tour --dry-run

# List all waypoints and routes
python3 navigate_to_office.py --list
```

### Monitor Position Relative to Waypoints

```bash
# Terminal 1: Start localization
~/start_localization.sh

# Terminal 2: Monitor position with waypoint awareness
source /opt/ros/humble/setup.bash
python3 ~/Desktop/WayfindR-driver/ros2_localization_attempt/scripts/localization_with_waypoints.py
```

This shows:
- Current robot position from AMCL
- Distance and bearing to each waypoint
- Direction arrows (→ ↗ ↑ etc.)
- Path to nearest waypoint
- Localization quality indicator

---

**Created:** 2025-12-21
**Tested:** 2025-12-21 - LiDAR + AMCL + Map Server + Pathfinding working
