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

**Created:** 2025-12-21
**Tested:** 2025-12-21 - LiDAR + AMCL + Map Server working
