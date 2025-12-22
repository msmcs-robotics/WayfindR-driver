# ROS2 Comprehensive Navigation System

**A drag-and-drop ROS2 Humble navigation system for Ubuntu 22.04 with Python 3.10**

## Overview

This package provides a complete, self-contained navigation system including:

- **SLAM Mapping**: Build 2D maps with SLAM Toolbox
- **Localization**: AMCL-based position estimation
- **Waypoint Management**: Add, save, and load named waypoints
- **Pathfinding**: A* algorithm for obstacle avoidance
- **Navigation**: Simple Commander API integration

## Requirements

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10
- 2D LiDAR (tested with Slamtec C1/RPLidar)

## Quick Install

```bash
# 1. Clone or copy to your system
cp -r ros2_comprehensive_attempt ~/ros2_nav

# 2. Install dependencies
~/ros2_nav/scripts/install_dependencies.sh

# 3. Configure LiDAR
~/ros2_nav/scripts/setup_lidar.sh

# 4. Start mapping OR localization
~/ros2_nav/scripts/start_mapping.sh
# OR
~/ros2_nav/scripts/start_localization.sh /path/to/map.yaml
```

## Directory Structure

```
ros2_comprehensive_attempt/
├── README.md                 # This file
├── config/                   # Configuration files
│   ├── slam_params.yaml      # SLAM Toolbox settings
│   ├── amcl_params.yaml      # Localization settings
│   ├── nav2_params.yaml      # Navigation settings
│   └── rviz_config.rviz      # RViz display settings
├── launch/                   # ROS2 launch files
│   ├── slam.launch.py        # SLAM mapping
│   ├── localization.launch.py # AMCL localization
│   ├── navigation.launch.py  # Full Nav2 stack
│   └── lidar.launch.py       # LiDAR only
├── scripts/                  # Python & bash scripts
│   ├── install_dependencies.sh
│   ├── setup_lidar.sh
│   ├── start_mapping.sh
│   ├── start_localization.sh
│   ├── save_map.sh
│   ├── waypoint_manager.py
│   ├── pathfinder.py
│   └── navigator.py
├── maps/                     # Saved maps
├── waypoints/                # Waypoint files
└── docs/                     # Documentation
    ├── QUICKSTART.md
    ├── CONFIGURATION.md
    ├── TROUBLESHOOTING.md
    └── API_REFERENCE.md
```

## Workflow

### Phase 1: Create a Map

```bash
# Start SLAM mapping with LiDAR
./scripts/start_mapping.sh

# Drive robot around (or carry LiDAR)
# Watch map build in RViz

# Save map when done
./scripts/save_map.sh my_office
```

### Phase 2: Add Waypoints

```bash
# Add waypoints to saved map
python3 scripts/waypoint_manager.py \
    --map maps/my_office.yaml \
    --add-waypoint "reception" 1.0 2.0 90 \
    --add-waypoint "conference" 5.0 3.0 0 \
    --output waypoints/my_office_waypoints.yaml
```

### Phase 3: Navigate

```bash
# Start localization with map
./scripts/start_localization.sh maps/my_office.yaml

# Navigate to waypoint
python3 scripts/navigator.py \
    --waypoints waypoints/my_office_waypoints.yaml \
    --goto reception
```

## Python API

### Pathfinding

```python
from scripts.pathfinder import PathFinder

pf = PathFinder('maps/my_office.yaml')
path = pf.find_path(start_x, start_y, goal_x, goal_y)
simplified = pf.simplify_path(path)
```

### Waypoint Management

```python
from scripts.waypoint_manager import WaypointManager, MapInfo

map_info = MapInfo.from_yaml('maps/my_office.yaml')
manager = WaypointManager(map_info)

manager.add_waypoint(Waypoint('kitchen', x=3.0, y=1.5, yaw_degrees=45))
manager.save_to_yaml('waypoints/my_office_waypoints.yaml')
```

### Navigation (requires ROS2)

```python
from scripts.navigator import Navigator

nav = Navigator('maps/my_office.yaml', 'waypoints/my_office_waypoints.yaml')
nav.go_to_waypoint('kitchen')
nav.execute_route(['reception', 'kitchen', 'conference'])
```

## Configuration

### LiDAR Settings

Edit `config/lidar_params.yaml`:
```yaml
serial_port: /dev/ttyUSB0    # or /dev/rplidar
serial_baudrate: 460800
scan_mode: DenseBoost
frame_id: laser
```

### SLAM Settings

Edit `config/slam_params.yaml`:
```yaml
resolution: 0.05             # 5cm per pixel
max_laser_range: 12.0        # Match your LiDAR
do_loop_closing: true
```

### AMCL Settings

Edit `config/amcl_params.yaml`:
```yaml
min_particles: 500
max_particles: 2000
laser_model_type: likelihood_field
```

## Tested Hardware

| LiDAR | Status |
|-------|--------|
| Slamtec C1/C1M1RP | Tested, working |
| Slamtec A1/A2/A3 | Should work |
| Velodyne VLP-16 | Needs config changes |

## Troubleshooting

See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

### Common Issues

1. **LiDAR not detected**: Check USB permissions, udev rules
2. **Map not building**: Verify TF tree, check /scan topic
3. **Localization drifts**: Provide initial pose, increase particles
4. **Path not found**: Lower obstacle threshold, check map quality

## License

MIT License - Use freely in your projects.

---

**Version:** 1.0.0
**ROS2:** Humble
**Python:** 3.10
**Ubuntu:** 22.04
