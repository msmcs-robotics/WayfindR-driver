# Quick Start Guide

Get up and running with the ROS2 Navigation System in 10 minutes.

## Prerequisites

- Ubuntu 22.04 LTS
- 2D LiDAR (Slamtec RPLidar recommended)
- USB connection to LiDAR

## Step 1: Install Dependencies

```bash
cd ros2_comprehensive_attempt
./scripts/install_dependencies.sh
```

This installs:
- ROS2 Humble Desktop
- SLAM Toolbox
- Navigation2
- RPLidar driver

## Step 2: Configure LiDAR

```bash
./scripts/setup_lidar.sh
```

Then **log out and back in** for group changes to take effect.

## Step 3: Verify LiDAR

```bash
source /opt/ros/humble/setup.bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0
```

In another terminal:
```bash
ros2 topic hz /scan
# Should show ~10 Hz
```

## Step 4: Create a Map

```bash
./scripts/start_mapping.sh
```

This opens RViz showing the map being built. Move the LiDAR around to explore the environment.

When done, save the map:
```bash
./scripts/save_map.sh my_office
```

## Step 5: Add Waypoints

```bash
python3 scripts/waypoint_manager.py \
    --map maps/my_office.yaml \
    --add-waypoint "entrance" 0.0 0.0 0 \
    --add-waypoint "desk1" 2.0 1.5 90 \
    --add-waypoint "kitchen" 4.0 0.0 180 \
    --output waypoints/my_office.yaml
```

## Step 6: Localize and Navigate

Start localization:
```bash
./scripts/start_localization.sh maps/my_office.yaml
```

In RViz:
1. Click "2D Pose Estimate"
2. Click where the robot is on the map
3. Drag to set orientation

Navigate to a waypoint:
```bash
python3 scripts/navigator.py \
    --map maps/my_office.yaml \
    --waypoints waypoints/my_office.yaml \
    --goto kitchen
```

## Quick Reference

| Task | Command |
|------|---------|
| Start mapping | `./scripts/start_mapping.sh` |
| Save map | `./scripts/save_map.sh <name>` |
| Start localization | `./scripts/start_localization.sh <map.yaml>` |
| List waypoints | `python3 scripts/navigator.py -w <waypoints.yaml> --list` |
| Plan path (dry) | `python3 scripts/navigator.py -w <waypoints.yaml> --goto <name> --dry-run` |

## Troubleshooting

**LiDAR not detected:**
```bash
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

**Map not appearing in RViz:**
- Set Fixed Frame to "map"
- Add Map display with topic "/map"

**Localization not converging:**
- Set initial pose in RViz
- Move LiDAR slightly to trigger updates

---

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for more help.
