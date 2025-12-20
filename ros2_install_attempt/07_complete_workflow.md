# Complete ROS2 SLAM & Navigation Workflow

This guide provides a step-by-step workflow from installation to autonomous navigation.

## Table of Contents

1. [Installation](#installation)
2. [Hardware Setup](#hardware-setup)
3. [Creating Your First Map](#creating-your-first-map)
4. [Adding Waypoints](#adding-waypoints)
5. [Autonomous Navigation](#autonomous-navigation)
6. [Deployment to Raspberry Pi](#deployment-to-raspberry-pi)
7. [Troubleshooting](#troubleshooting)

---

## Installation

### On Ubuntu 22.04 (WSL, x86_64, or Raspberry Pi)

```bash
cd /home/devel/WayfindR-driver/ros2_install_attempt

# Step 1: Install ROS2 Humble base
sudo bash 01_install_ros2_humble.sh

# Step 2: Install SLAM and Navigation packages
sudo bash 02_install_slam_navigation.sh

# Step 3: Verify installation
bash 03_verify_installation.sh
```

**Important**: After installation, open a new terminal or run:
```bash
source ~/.bashrc
```

---

## Hardware Setup

### Required Hardware

1. **LiDAR Sensor**
   - RPLIDAR A1/A2/A3
   - Slamtec C1M1
   - YouYeeToo LiDAR
   - Or any sensor publishing `sensor_msgs/LaserScan`

2. **IMU (Recommended)**
   - MPU6050
   - BNO055
   - Or any sensor publishing `sensor_msgs/Imu`

3. **Robot Base**
   - Differential drive (2-wheel + caster)
   - Skid-steer (4-wheel tank tracks)
   - Must respond to `geometry_msgs/Twist` on `/cmd_vel`

4. **Computer**
   - Raspberry Pi 4 (4GB+ recommended)
   - Or any Ubuntu 22.04 system

### Hardware Connections

```
┌─────────────────┐
│  Raspberry Pi   │
└────────┬────────┘
         │
    ┌────┴────┬─────────┬──────────┐
    │         │         │          │
┌───▼───┐ ┌──▼──┐  ┌───▼────┐ ┌───▼────┐
│ LiDAR │ │ IMU │  │ Motors │ │  WiFi  │
│ /USB  │ │ I2C │  │  GPIO  │ │  SSH   │
└───────┘ └─────┘  └────────┘ └────────┘
```

### Topic Requirements

Your robot must publish/subscribe to these topics:

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/scan` | sensor_msgs/LaserScan | Publish | LiDAR data |
| `/imu` | sensor_msgs/Imu | Publish | IMU data (optional) |
| `/odom` | nav_msgs/Odometry | Publish | Wheel odometry |
| `/cmd_vel` | geometry_msgs/Twist | Subscribe | Motor commands |
| `/tf` | tf2_msgs/TFMessage | Publish | Transforms |

### Create Robot URDF

You need a basic robot description. Create `~/ros2_ws/src/my_robot/urdf/robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="wayfind_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.15"/>
      </geometry>
    </visual>
  </link>

  <!-- LiDAR link -->
  <link name="laser_frame"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- IMU link -->
  <link name="imu_frame"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_frame"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## Creating Your First Map

### Phase 1: Mapping with SLAM Toolbox

#### Terminal 1: Launch SLAM Toolbox

```bash
source ~/.bashrc

# Using default configuration
ros2 launch slam_toolbox online_async_launch.py

# Or with custom parameters
ros2 launch slam_toolbox online_async_launch.py \
    params_file:=$HOME/WayfindR-driver/ros2_install_attempt/config/slam_params.yaml
```

**What this does:**
- Subscribes to `/scan` from your LiDAR
- Subscribes to `/odom` from your robot
- Publishes `/map` (the map being built)
- Publishes `map -> odom` transform

#### Terminal 2: Launch Your LiDAR Driver

Example for RPLIDAR:
```bash
ros2 run rplidar_ros rplidar_composition
```

Example for Slamtec C1M1:
```bash
ros2 launch sllidar_ros2 sllidar_launch.py
```

**Verify LiDAR is working:**
```bash
ros2 topic echo /scan --once
```

#### Terminal 3: Launch Robot Base / Odometry

This publishes `/odom` and subscribes to `/cmd_vel`:

```bash
# Your custom motor control node
ros2 run my_robot motor_controller

# Or use existing diff_drive_controller
ros2 run diff_drive_controller diff_drive_controller
```

**Verify odometry is working:**
```bash
ros2 topic echo /odom --once
```

#### Terminal 4: Drive the Robot

Manual teleop with keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Mapping tips:**
- Drive slowly (< 0.5 m/s)
- Cover entire area systematically
- Return to starting point (loop closure)
- Avoid sudden movements
- Multiple passes improve quality

#### Terminal 5: Monitor Progress (Optional)

On a machine with GUI:
```bash
rviz2
```

Add displays:
- `/map` → Map
- `/scan` → LaserScan
- `TF` → TF frames

#### Save the Map

When mapping is complete:
```bash
# Create a descriptive name
MAP_NAME="warehouse_$(date +%Y%m%d_%H%M)"

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/$MAP_NAME

# Verify files created
ls -lh ~/maps/$MAP_NAME.*
```

You should see:
- `warehouse_20251220_1430.pgm` (image)
- `warehouse_20251220_1430.yaml` (metadata)

---

## Adding Waypoints

### Option A: Using RViz2 (GUI)

**On a computer with GUI (not WSL):**

#### Step 1: Load the map

```bash
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/maps/warehouse_20251220_1430.yaml
```

#### Step 2: Open RViz2

```bash
rviz2
```

Configure displays:
1. Add → Map → `/map`
2. Add → TF
3. Fixed Frame: `map`

#### Step 3: Click waypoints

1. Click **"Publish Point"** tool (or **"2D Nav Goal"**)
2. Click on the map at desired locations
3. Record coordinates from terminal

#### Step 4: Save waypoints to YAML

Create `~/waypoints/warehouse_waypoints.yaml`:

```yaml
waypoints:
  - name: "home_base"
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

  - name: "checkpoint_1"
    position: {x: 3.2, y: 1.5, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}

  - name: "inspection_zone"
    position: {x: 5.8, y: -1.2, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}

routes:
  patrol_route:
    - "home_base"
    - "checkpoint_1"
    - "inspection_zone"
    - "home_base"
```

### Option B: Programmatically (No GUI)

Use the Python script:

```bash
python3 ~/WayfindR-driver/ros2_install_attempt/06_navigation_python_examples.py

# Select option 5: Create waypoint file
```

Or manually create the YAML file with known coordinates.

---

## Autonomous Navigation

### Phase 2: Localization and Navigation

#### Terminal 1: Launch Localization

```bash
source ~/.bashrc

# Load your saved map for localization
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/maps/warehouse_20251220_1430.yaml \
    params_file:=$HOME/WayfindR-driver/ros2_install_attempt/config/nav2_params.yaml
```

#### Terminal 2: Launch Navigation Stack

```bash
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$HOME/WayfindR-driver/ros2_install_attempt/config/nav2_params.yaml
```

#### Terminal 3: Launch LiDAR (same as before)

```bash
ros2 launch sllidar_ros2 sllidar_launch.py
```

#### Terminal 4: Launch Robot Base (same as before)

```bash
ros2 run my_robot motor_controller
```

#### Terminal 5: Set Initial Pose

Tell the robot where it is on the map:

```bash
# If at origin
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'},
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                  orientation: {w: 1.0}}}}"
```

Or use RViz2's "2D Pose Estimate" tool.

#### Terminal 6: Navigate to Waypoint

**Option A: Command line**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'},
           pose: {position: {x: 3.2, y: 1.5, z: 0.0},
                  orientation: {w: 1.0}}}}"
```

**Option B: Python script**
```bash
cd ~/WayfindR-driver/ros2_install_attempt
python3 06_navigation_python_examples.py

# Select option 1: Navigate to single waypoint
```

**Option C: Patrol route**
```bash
python3 06_navigation_python_examples.py

# Select option 3: Continuous patrol route
```

### Monitor Navigation

```bash
# Watch robot position
ros2 topic echo /odom

# Watch navigation status
ros2 topic echo /navigate_to_pose/_action/feedback

# View cost maps (in RViz2)
# Add: /local_costmap/costmap
# Add: /global_costmap/costmap
```

---

## Deployment to Raspberry Pi

### Transfer Files to Pi

```bash
# Copy maps
scp ~/maps/* pi@raspberrypi.local:~/maps/

# Copy waypoints
scp ~/waypoints/* pi@raspberrypi.local:~/waypoints/

# Copy configuration
scp -r ~/WayfindR-driver/ros2_install_attempt/config pi@raspberrypi.local:~/robot_config/
```

### Install ROS2 on Raspberry Pi

```bash
ssh pi@raspberrypi.local

# Run installation scripts
cd ~/WayfindR-driver/ros2_install_attempt
sudo bash 01_install_ros2_humble.sh
sudo bash 02_install_slam_navigation.sh
```

### Create Systemd Service for Auto-Start

Create `/etc/systemd/system/robot-navigation.service`:

```ini
[Unit]
Description=Robot Navigation Service
After=network.target

[Service]
Type=simple
User=pi
Environment="ROS_DOMAIN_ID=0"
ExecStart=/home/pi/start_navigation.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Create `/home/pi/start_navigation.sh`:

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch navigation with saved map
ros2 launch nav2_bringup navigation_launch.py \
    map:=/home/pi/maps/warehouse_20251220_1430.yaml \
    params_file:=/home/pi/robot_config/nav2_params.yaml
```

Enable service:
```bash
chmod +x /home/pi/start_navigation.sh
sudo systemctl enable robot-navigation.service
sudo systemctl start robot-navigation.service
```

---

## Troubleshooting

### SLAM Issues

**Problem: Map is distorted**
- Cause: Poor odometry
- Fix: Calibrate wheel encoders, add IMU, drive slower

**Problem: Map has duplicates**
- Cause: Loop closure failure
- Fix: Drive slower, retrace paths, ensure good LiDAR coverage

**Problem: No map appearing**
- Check: `ros2 topic list | grep /map`
- Check: `ros2 topic echo /scan --once`
- Check: TF tree: `ros2 run tf2_tools view_frames`

### Navigation Issues

**Problem: Robot won't move**
- Check: `/cmd_vel` is being published: `ros2 topic echo /cmd_vel`
- Check: Motors respond to manual teleop
- Check: Navigation is active: `ros2 lifecycle get /controller_server`

**Problem: Robot gets stuck**
- Increase costmap inflation radius
- Reduce planner tolerance
- Check for obstacles blocking path

**Problem: Robot veers off course**
- Improve odometry calibration
- Add IMU for better orientation
- Tune controller parameters

### Sensor Issues

**Problem: No LiDAR data**
```bash
# Check device permissions
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# Check data
ros2 topic hz /scan
```

**Problem: IMU drift**
- Calibrate IMU before each run
- Use Madgwick or Complementary filter
- Check mounting orientation

### Transform Issues

**Problem: `map -> odom -> base_link` broken**
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

---

## Summary Checklist

### Mapping Phase
- [ ] ROS2 Humble installed
- [ ] SLAM packages installed
- [ ] LiDAR publishing `/scan`
- [ ] Robot publishing `/odom`
- [ ] Robot responding to `/cmd_vel`
- [ ] SLAM Toolbox running
- [ ] Map created and saved

### Waypoint Phase
- [ ] Map loaded in RViz2 (or programmatically)
- [ ] Waypoints marked and recorded
- [ ] Waypoints saved to YAML file
- [ ] Routes defined (optional)

### Navigation Phase
- [ ] Map loaded for localization
- [ ] Nav2 stack running
- [ ] Initial pose set
- [ ] Navigation to waypoints working
- [ ] Patrol routes tested

### Deployment Phase
- [ ] Files transferred to Pi
- [ ] ROS2 installed on Pi
- [ ] Services configured for auto-start
- [ ] Remote monitoring setup
- [ ] Production testing complete

---

## Next Steps

1. **Integrate with Fleet Management**
   - Connect to FastAPI dashboard
   - Send waypoint commands via HTTP API
   - Monitor multiple robots

2. **Add Advanced Features**
   - Frontier exploration (autonomous mapping)
   - Dynamic obstacle avoidance
   - Multi-floor navigation
   - Battery monitoring and auto-charge

3. **Optimize Performance**
   - Tune SLAM parameters for your environment
   - Calibrate odometry and IMU
   - Adjust navigation parameters
   - Profile CPU/memory usage

4. **Production Hardening**
   - Error handling and recovery
   - Logging and diagnostics
   - Remote updates
   - Security (authentication, encryption)

---

## Resources

- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox
- **Nav2**: https://navigation.ros.org/
- **Your Project Docs**: [../docs/](../docs/)

For more details, see individual guide files in this directory.
