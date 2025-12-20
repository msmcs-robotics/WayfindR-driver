# Map Saving and Loading Guide

This guide explains how to save maps created with SLAM Toolbox and load them for localization and navigation.

## Map File Format

When you save a map, two files are created:

### 1. PGM Image File (`map_name.pgm`)
- **Format**: Portable Gray Map (image file)
- **Content**: Grid representation of the environment
- **Values**:
  - `0` (black) = Obstacle
  - `254` (white) = Free space
  - `205` (gray) = Unknown/unexplored

### 2. YAML Metadata File (`map_name.yaml`)
- **Format**: YAML configuration
- **Content**: Map parameters and metadata

Example `my_map.yaml`:
```yaml
image: my_map.pgm
resolution: 0.05      # meters per pixel
origin: [-10.0, -10.0, 0.0]  # [x, y, theta] offset
negate: 0             # 0 = occupied is dark
occupied_thresh: 0.65 # probability threshold for occupied
free_thresh: 0.196    # probability threshold for free
```

## Creating Maps with SLAM Toolbox

### Method 1: Online Mapping (Real-time)

This is the most common approach for robots driving around an environment.

#### Step 1: Launch SLAM Toolbox

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch SLAM Toolbox in online mode
ros2 launch slam_toolbox online_async_launch.py
```

#### Step 2: Drive the Robot

You need to move the robot around while SLAM builds the map. Options:

**Option A: Manual teleop with keyboard**
```bash
# In another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Option B: Your custom motor control**
```bash
# Use your existing motor control scripts
# Make sure they publish to /cmd_vel
```

**Option C: Joystick/RC controller**
```bash
ros2 launch teleop_twist_joy teleop.launch.py
```

#### Step 3: Monitor the Map

If you have RViz2 available (not in WSL):
```bash
rviz2
```

Then add displays:
- `/map` topic (nav_msgs/OccupancyGrid)
- `/scan` topic (sensor_msgs/LaserScan)
- `/robot` TF frames

#### Step 4: Save the Map

While SLAM is still running:

```bash
# Save map to ~/maps/ directory
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_indoor_map

# This creates:
# ~/maps/my_indoor_map.pgm
# ~/maps/my_indoor_map.yaml
```

**Important**: Make sure to save the map BEFORE shutting down SLAM Toolbox!

### Method 2: Offline Mapping (from bag file)

If you recorded sensor data to a bag file, you can create maps offline.

#### Step 1: Record data while driving

```bash
# Record LiDAR and odometry
ros2 bag record /scan /odom /tf /tf_static
```

#### Step 2: Process bag file with SLAM

```bash
# Launch SLAM in offline mode
ros2 launch slam_toolbox offline_launch.py

# Play back the bag file
ros2 bag play your_bagfile
```

#### Step 3: Save the map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/offline_map
```

## Loading Maps for Navigation

Once you have a saved map, you can use it for localization and navigation.

### Method 1: Using Nav2 Bringup (Full Stack)

```bash
# Launch localization with your map
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/maps/my_indoor_map.yaml

# In another terminal, launch navigation
ros2 launch nav2_bringup navigation_launch.py \
    map:=$HOME/maps/my_indoor_map.yaml
```

### Method 2: Using Map Server Standalone

```bash
# Just load the map server
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=$HOME/maps/my_indoor_map.yaml

# Activate the lifecycle node
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### Method 3: Using SLAM Toolbox for Localization

SLAM Toolbox can also be used for localization only (no mapping):

```bash
# Launch in localization mode with existing map
ros2 launch slam_toolbox localization_launch.py \
    map_file_name:=$HOME/maps/my_indoor_map.yaml
```

## Map Organization Best Practices

### Directory Structure

```
~/maps/
├── warehouse/
│   ├── warehouse.pgm
│   ├── warehouse.yaml
│   └── README.txt
├── office_floor1/
│   ├── office_floor1.pgm
│   ├── office_floor1.yaml
│   └── README.txt
└── test_area/
    ├── test_area.pgm
    ├── test_area.yaml
    └── README.txt
```

### Naming Convention

Use descriptive names:
- `warehouse_2025-12-20.yaml` (date-stamped)
- `office_floor1_v2.yaml` (versioned)
- `lab_full_map.yaml` (descriptive)

### Map Metadata File

Create a README.txt alongside each map:

```
Map: Warehouse Main Floor
Date Created: 2025-12-20
Robot: WayfindR Tank v1
LiDAR: Slamtec C1M1
Area: ~200 sq meters
Notes: Includes loading dock area, avoid pallets may move
```

## Map Quality Tips

### 1. Drive Slowly and Consistently
- Don't move too fast (< 0.5 m/s recommended)
- Smooth movements help SLAM converge

### 2. Cover the Entire Area
- Drive through all accessible spaces
- Close loops (return to starting point)
- Multiple passes improve accuracy

### 3. Overlap Scans
- Ensure LiDAR scans overlap between positions
- Don't skip areas or move too quickly

### 4. Good LiDAR Coverage
- Avoid driving in the dark (poor LiDAR data)
- Stay away from glass/mirrors (bad reflections)
- Avoid moving obstacles (people, carts)

### 5. Verify Map Quality

Before saving, check:
- Walls are continuous (not broken)
- Rooms are properly enclosed
- Hallways are clear
- No major artifacts or noise

## Editing Maps

You can manually edit maps if needed:

### Using GIMP (Image Editor)

1. Open the `.pgm` file in GIMP
2. Use paint tools to:
   - Fill in gaps in walls (black)
   - Clean up noise (white for free space)
   - Mark unknown areas (gray)
3. Save as `.pgm` (not PNG or JPG!)

### Using RViz2 (for waypoints - see next guide)

You can add waypoints and annotations in RViz2, then save them separately.

## Troubleshooting

### Map is distorted or warped
- **Cause**: Poor odometry or too-fast movement
- **Fix**: Drive slower, improve wheel encoders, add IMU

### Map has duplicate features
- **Cause**: Loop closure failure
- **Fix**: Drive more slowly, ensure good overlap, retrace paths

### Map is incomplete
- **Cause**: Didn't cover all areas
- **Fix**: Drive through missed areas, save again

### Cannot load map
- **Cause**: File path or YAML errors
- **Check**:
  ```bash
  ls -l ~/maps/my_map.*
  cat ~/maps/my_map.yaml
  ```

### Map server won't start
```bash
# Check if map files exist
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=$HOME/maps/my_map.yaml \
    --log-level debug
```

## Map Coordinate System

### Understanding Map Frames

- **map frame**: Fixed global frame, origin is where you started mapping
- **odom frame**: Odometry frame (drifts over time)
- **base_link**: Robot center
- **laser_frame**: LiDAR sensor location

### Transform Tree
```
map → odom → base_link → laser_frame
```

SLAM Toolbox estimates the `map → odom` transform to correct drift.

## Example Workflow

### Complete mapping session:

```bash
# Terminal 1: Start SLAM
source ~/.bashrc
ros2 launch slam_toolbox online_async_launch.py

# Terminal 2: Drive robot around
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Drive for 5-10 minutes, covering entire area

# Terminal 3: Save map when done
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_$(date +%Y%m%d)

# Output:
# Map saved to:
#   ~/maps/warehouse_20251220.pgm
#   ~/maps/warehouse_20251220.yaml
```

## Next Steps

After creating and saving maps:
1. Load map for localization testing
2. Add waypoints using RViz2 (see [04_waypoint_workflow.md](04_waypoint_workflow.md))
3. Test autonomous navigation to waypoints
4. Deploy to Raspberry Pi for production use

## Resources

- [Nav2 Map Server Documentation](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [PGM Image Format Specification](http://netpbm.sourceforge.net/doc/pgm.html)
