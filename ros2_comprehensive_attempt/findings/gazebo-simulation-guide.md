# WayfindR Gazebo Simulation Guide

**Author:** WayfindR Development Team
**Date:** 2026-01-11
**ROS2 Version:** Humble
**Gazebo Version:** Fortress
**Ubuntu Version:** 22.04 LTS

---

## Table of Contents

1. [Introduction](#introduction)
2. [Why Use Gazebo Simulation?](#why-use-gazebo-simulation)
3. [Gazebo Classic vs Modern Gazebo](#gazebo-classic-vs-modern-gazebo)
4. [Installation](#installation)
5. [Understanding the Simulation Stack](#understanding-the-simulation-stack)
6. [Quick Start Guide](#quick-start-guide)
7. [World Creation Guide](#world-creation-guide)
8. [Integration with Existing System](#integration-with-existing-system)
9. [Testing Procedures](#testing-procedures)
10. [Gazebo vs Rosbag Testing](#gazebo-vs-rosbag-testing)
11. [Troubleshooting](#troubleshooting)
12. [Advanced Topics](#advanced-topics)
13. [References](#references)

---

## Introduction

This guide provides comprehensive instructions for setting up and using Gazebo Fortress simulation for the WayfindR robot. Simulation allows you to develop, test, and validate your navigation algorithms without requiring physical hardware, significantly accelerating development and reducing hardware wear.

### What You'll Learn

- How to install Gazebo Fortress with ROS2 Humble
- How to spawn the WayfindR robot in simulation
- How to create custom test environments
- How to run SLAM and navigation in simulation
- How to integrate simulation with your existing workflow
- When to use simulation vs rosbag testing

---

## Why Use Gazebo Simulation?

### Key Benefits

1. **Pre-Deployment Testing**
   - Test everything before deploying to Raspberry Pi
   - Catch bugs and issues early in development
   - Validate algorithms in controlled environments

2. **Hardware-Free Development**
   - Develop navigation algorithms without physical robot
   - No need to worry about battery life or hardware failures
   - Work from anywhere without access to hardware

3. **Reproducible Testing**
   - Exact same conditions every time
   - Deterministic behavior (with fixed random seeds)
   - Easy to share test scenarios with team members

4. **Rapid Iteration**
   - Faster than real-time simulation possible
   - Instant reset and retry
   - No setup time between tests

5. **Safety**
   - Test dangerous scenarios without risk
   - No physical damage from crashes or collisions
   - Safe to test extreme edge cases

6. **Cost Effective**
   - No wear and tear on hardware
   - No replacement parts needed
   - Simulate multiple robots without buying hardware

7. **Educational Value**
   - Visualize sensor data in real-time
   - Understand robot behavior better
   - Learn navigation concepts safely

### Use Cases for WayfindR

- **Algorithm Development:** Test SLAM, localization, and path planning
- **Parameter Tuning:** Optimize Nav2 parameters without hardware
- **Integration Testing:** Verify all components work together
- **Scenario Testing:** Create specific challenging environments
- **Regression Testing:** Automated tests in CI/CD pipelines
- **Demonstrations:** Show functionality without physical robot

---

## Gazebo Classic vs Modern Gazebo

### Understanding the Naming

The Gazebo ecosystem has undergone a major transition:

- **Gazebo Classic** (formerly just "Gazebo")
  - Numbered versions: Gazebo 7, 9, 11
  - Legacy architecture
  - Limited ROS2 support
  - **NOT recommended for new projects**

- **Modern Gazebo** (formerly "Ignition Gazebo")
  - Letter-based versions: Citadel, Fortress, Garden, Harmonic
  - Complete rewrite with modern architecture
  - First-class ROS2 support
  - **Recommended for all ROS2 projects**

### Why Gazebo Fortress for ROS2 Humble?

According to official ROS2 documentation and community consensus:

1. **Official Support:** Gazebo Fortress is the officially paired simulator for ROS2 Humble
2. **Binary Packages:** Available via `ros-humble-ros-gz` packages
3. **Stability:** Mature and well-tested on Ubuntu 22.04
4. **Long-term Support:** Aligned with ROS2 Humble lifecycle
5. **Community:** Largest user base for ROS2 Humble development

### Compatibility Matrix

| Ubuntu Version | ROS2 Distribution | Recommended Gazebo | Package Name |
|---------------|-------------------|-------------------|--------------|
| 22.04 LTS | Humble | Fortress | ros-humble-ros-gz |
| 22.04 LTS | Iron | Fortress/Garden | ros-iron-ros-gz |
| 24.04 LTS | Jazzy | Harmonic | ros-jazzy-ros-gz |

**For WayfindR:** Ubuntu 22.04 + ROS2 Humble + Gazebo Fortress

---

## Installation

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble (installed and sourced)
- Sufficient disk space (approximately 2GB for Gazebo)
- Working ROS2 workspace with WayfindR packages

### Step 1: Install Gazebo Fortress

Add the official Gazebo repository and install Gazebo Fortress:

```bash
# Add Gazebo repository GPG key
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add Gazebo repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package list
sudo apt update

# Install Gazebo Fortress
sudo apt install ignition-fortress -y
```

### Step 2: Install ROS-Gazebo Bridge

Install the integration packages that bridge ROS2 and Gazebo:

```bash
# Install ros_gz packages for ROS2 Humble
sudo apt install ros-humble-ros-gz -y
```

This installs several packages:
- `ros_gz_bridge` - Message translation between ROS2 and Gazebo
- `ros_gz_sim` - Launch utilities and spawning tools
- `ros_gz_image` - Image message bridging

### Step 3: Install Additional Dependencies

```bash
# Install xacro for URDF processing (if not already installed)
sudo apt install ros-humble-xacro -y

# Install Python dependencies
pip3 install xacro
```

### Step 4: Verify Installation

Test that Gazebo Fortress is installed correctly:

```bash
# Check Gazebo version
gz sim --version

# Expected output: Gazebo Sim, version 6.x.x
# (Version 6 = Fortress release)
```

Launch Gazebo GUI to verify:

```bash
# Launch empty world
gz sim

# You should see the Gazebo Fortress GUI window
# Close it with Ctrl+C when done
```

Test ROS-Gazebo bridge:

```bash
# Source your ROS2 workspace
source /opt/ros/humble/setup.bash

# Check if ros_gz packages are available
ros2 pkg list | grep ros_gz

# Expected output:
# ros_gz
# ros_gz_bridge
# ros_gz_image
# ros_gz_interfaces
# ros_gz_sim
```

### Step 5: Configure Environment (Optional)

Add to your `~/.bashrc` for convenience:

```bash
# Gazebo resource paths (if you create custom models/worlds)
export GZ_SIM_RESOURCE_PATH=$HOME/your_workspace/install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/worlds

# Source ROS2
source /opt/ros/humble/setup.bash
source $HOME/your_workspace/install/setup.bash
```

### Installation Verification Checklist

- [ ] Gazebo Fortress installed (`gz sim --version` works)
- [ ] Gazebo GUI launches successfully
- [ ] `ros-humble-ros-gz` packages installed
- [ ] ROS2 workspace sourced
- [ ] No error messages during installation

---

## Understanding the Simulation Stack

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 Application Layer                   │
│  (Nav2, SLAM Toolbox, Your Nodes, RViz)                     │
└────────────────────┬────────────────────────────────────────┘
                     │ ROS2 Topics (/cmd_vel, /scan, /odom)
                     │
┌────────────────────▼────────────────────────────────────────┐
│                   ros_gz_bridge                              │
│  (Message Translation: ROS2 ↔ Gazebo Transport)            │
└────────────────────┬────────────────────────────────────────┘
                     │ Gazebo Transport (Protocol Buffers)
                     │
┌────────────────────▼────────────────────────────────────────┐
│                  Gazebo Fortress Simulator                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Physics Engine (dynamics, collisions)              │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Sensor Simulation (LIDAR, cameras, IMU)            │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Robot Model (from URDF with Gazebo plugins)        │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  World Environment (SDF format)                      │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

#### 1. Gazebo Fortress (Simulator Core)

- **Physics Engine:** Simulates real-world physics (gravity, friction, collisions)
- **Rendering:** Visualizes the 3D world
- **Sensor Simulation:** Generates realistic sensor data
- **Plugin System:** Extends functionality (differential drive, sensors, etc.)

#### 2. ros_gz_bridge (Integration Layer)

- **Message Translation:** Converts between ROS2 messages and Gazebo messages
- **Topic Mapping:** Maps ROS2 topics to Gazebo topics
- **Bidirectional:** Supports both ROS2→Gazebo and Gazebo→ROS2

#### 3. URDF with Gazebo Plugins

The WayfindR URDF (`wayfinder_robot.urdf.xacro`) already includes Gazebo-specific tags:

```xml
<!-- Differential Drive Plugin -->
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <commandTopic>/cmd_vel</commandTopic>
    <odometryTopic>/odom</odometryTopic>
    <!-- ... more configuration ... -->
  </plugin>
</gazebo>

<!-- LIDAR Sensor Plugin -->
<gazebo reference="laser">
  <sensor type="ray" name="rplidar_sensor">
    <!-- ... sensor configuration ... -->
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <topicName>/scan</topicName>
    </plugin>
  </sensor>
</gazebo>
```

These plugins:
- Enable robot control via `/cmd_vel`
- Publish odometry to `/odom`
- Simulate LIDAR and publish to `/scan`
- Publish joint states

#### 4. World Files (SDF Format)

SDF (Simulation Description Format) files define the simulation environment:
- Ground plane
- Walls and obstacles
- Lighting
- Physics parameters
- Initial object positions

### Data Flow Example: Moving the Robot

1. **ROS2 Node** publishes `Twist` message to `/cmd_vel`
2. **ros_gz_bridge** translates to Gazebo `Twist` message
3. **Gazebo Plugin** (differential_drive_controller) receives command
4. **Physics Engine** calculates wheel forces and robot motion
5. **Gazebo Plugin** computes odometry from simulated motion
6. **ros_gz_bridge** translates odometry to ROS2 `Odometry` message
7. **ROS2 Nodes** receive odometry on `/odom` topic

### Simulated vs Real Hardware

| Aspect | Real Hardware | Gazebo Simulation |
|--------|--------------|-------------------|
| **Sensor Data** | Real sensor noise | Configurable noise model |
| **Odometry** | Wheel encoder drift | Perfect or realistic drift |
| **Physics** | Real-world unpredictability | Deterministic physics |
| **Time** | Real-time only | Can be slower/faster |
| **Repeatability** | Hard to reproduce | Perfectly repeatable |
| **Safety** | Physical risks | Completely safe |

---

## Quick Start Guide

### Launch Gazebo Simulation

The simplest way to start:

```bash
# Navigate to workspace
cd ~/your_workspace

# Source workspace
source install/setup.bash

# Launch Gazebo with WayfindR robot in empty world
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py
```

This will:
1. Start Gazebo Fortress (server and GUI)
2. Spawn the WayfindR robot at origin
3. Launch robot_state_publisher
4. Start ros_gz_bridge for sensor data
5. Open RViz with simulation configuration

### Test Basic Movement

In a new terminal:

```bash
# Source workspace
source install/setup.bash

# Send velocity command to move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10

# Stop the robot (Ctrl+C to stop publishing)
```

You should see:
- Robot moving in Gazebo
- LIDAR scanning visible in Gazebo
- Odometry updating in RViz

### Verify Sensor Data

Check that sensor topics are publishing:

```bash
# List active topics
ros2 topic list

# Expected topics:
# /scan          (LaserScan)
# /odom          (Odometry)
# /cmd_vel       (Twist)
# /tf            (TFMessage)
# /clock         (Clock)

# Echo LIDAR data
ros2 topic echo /scan --once

# Echo odometry
ros2 topic echo /odom --once
```

### Launch Options

#### Use Test Room World

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=test_room
```

#### Use Obstacle Course World

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=obstacle_course
```

#### Launch Without RViz

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_rviz:=false
```

#### Launch Headless (No Gazebo GUI)

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_gui:=false
```

#### Custom Spawn Position

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  x_pose:=2.0 \
  y_pose:=3.0 \
  yaw_pose:=1.57
```

---

## World Creation Guide

### Understanding SDF Format

SDF (Simulation Description Format) is XML-based and describes:
- Models (robots, objects, obstacles)
- Sensors and lights
- Physics properties
- Plugins

### Basic World Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="my_world">

    <!-- Physics configuration -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Required plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Your obstacles and models here -->

  </world>
</sdf>
```

### Creating a Simple Wall

```xml
<model name="wall_1">
  <static>true</static>
  <pose>0 5 1 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>10 0.2 2</size>  <!-- length width height -->
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>10 0.2 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.7 0.7 0.7 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Creating Obstacles

#### Box Obstacle

```xml
<model name="box_obstacle">
  <static>true</static>
  <pose>2 2 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

#### Cylinder Obstacle (Column)

```xml
<model name="column">
  <static>true</static>
  <pose>-3 3 0.75 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.3</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
        <diffuse>0.2 0.2 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### World Design Best Practices

1. **Start Simple**
   - Begin with empty room
   - Add obstacles incrementally
   - Test LIDAR visibility at each step

2. **LIDAR-Friendly Design**
   - Ensure obstacles are tall enough (LIDAR height is ~0.15m)
   - Avoid perfectly flat walls (add texture/features)
   - Include corners and edges for feature detection

3. **Navigation Testing**
   - Create known-good paths
   - Include narrow passages (test size limits)
   - Add dead ends (test recovery behaviors)
   - Include open areas (test goal seeking)

4. **Realistic Dimensions**
   - Match real-world scenarios
   - Consider robot size (0.2m diameter base)
   - Allow safe clearance (at least 2x robot width)

5. **Performance Considerations**
   - Limit number of objects (start with <20)
   - Use static models where possible
   - Avoid complex geometries if not needed

### Provided Worlds

#### test_room.sdf

Simple rectangular room for basic testing:
- 10m x 8m enclosed space
- Four walls, no obstacles
- Good for: Initial testing, basic movement, LIDAR verification

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py world:=test_room
```

#### obstacle_course.sdf

Complex environment with various obstacles:
- 15m x 12m room
- Multiple obstacles (boxes, cylinders)
- Narrow passages (0.8m gap)
- Good for: Navigation testing, path planning, obstacle avoidance

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py world:=obstacle_course
```

### Creating Your Own World

1. **Copy Template**
   ```bash
   cd ~/your_workspace/install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/worlds
   cp test_room.sdf my_custom_world.sdf
   ```

2. **Edit World**
   - Open `my_custom_world.sdf` in text editor
   - Modify world name
   - Add/remove obstacles
   - Adjust dimensions

3. **Test World**
   ```bash
   ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
     world:=/full/path/to/my_custom_world.sdf
   ```

4. **Iterate**
   - Launch simulation
   - Observe robot behavior
   - Adjust obstacles
   - Repeat

---

## Integration with Existing System

### Using Simulation with Bringup

The existing `bringup.launch.py` supports simulation mode:

```bash
# Launch SLAM in simulation mode
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=slam \
  use_sim_time:=true
```

**Important:** When using simulation:
1. **First terminal:** Launch Gazebo simulation
   ```bash
   ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py
   ```

2. **Second terminal:** Launch navigation stack with sim time
   ```bash
   ros2 launch ros2_comprehensive_attempt bringup.launch.py \
     mode:=slam \
     use_sim_time:=true
   ```

### Complete Workflow: SLAM Mapping

#### Step 1: Launch Gazebo with World

```bash
# Terminal 1: Start Gazebo with obstacle course
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=obstacle_course
```

#### Step 2: Launch SLAM

```bash
# Terminal 2: Start SLAM Toolbox
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=slam \
  use_sim_time:=true \
  use_rviz:=true
```

#### Step 3: Drive Robot to Build Map

Option A - Keyboard Teleop:
```bash
# Terminal 3: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Option B - Manual Commands:
```bash
# Terminal 3: Manual velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
```

#### Step 4: Save Map

```bash
# Terminal 4: Save map when complete
ros2 run nav2_map_server map_saver_cli -f ~/maps/sim_obstacle_course
```

### Complete Workflow: Navigation Testing

#### Step 1: Launch Gazebo

```bash
# Terminal 1
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=obstacle_course
```

#### Step 2: Launch Navigation Stack

```bash
# Terminal 2
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=navigation \
  map:=~/maps/sim_obstacle_course.yaml \
  use_sim_time:=true
```

#### Step 3: Set Initial Pose in RViz

- Click "2D Pose Estimate" button
- Click and drag on map to set robot position and orientation
- Robot should localize (AMCL particles converge)

#### Step 4: Send Navigation Goal

- Click "2D Nav Goal" button in RViz
- Click destination on map
- Robot should plan path and navigate autonomously

### Integration Checklist

When running simulation with navigation stack:

- [ ] `use_sim_time:=true` set for bringup.launch.py
- [ ] Gazebo launched before navigation stack
- [ ] All nodes show `use_sim_time: true` in parameters
- [ ] `/clock` topic publishing (from Gazebo)
- [ ] Sensor data flowing (`/scan`, `/odom`)
- [ ] TF tree complete (check with `ros2 run tf2_tools view_frames`)

---

## Testing Procedures

### Basic Functionality Tests

#### Test 1: Robot Spawning

**Goal:** Verify robot appears correctly in Gazebo

```bash
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py
```

**Success Criteria:**
- [ ] Robot model visible in Gazebo
- [ ] All links present (base, wheels, LIDAR)
- [ ] Robot at correct height (wheels touching ground)
- [ ] No warning/error messages

#### Test 2: Velocity Control

**Goal:** Verify cmd_vel commands work

```bash
# Terminal 1: Launch simulation
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py

# Terminal 2: Send forward command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10
```

**Success Criteria:**
- [ ] Robot moves forward in Gazebo
- [ ] Speed approximately 0.3 m/s
- [ ] Straight line motion (no drift)
- [ ] Smooth movement

#### Test 3: LIDAR Data

**Goal:** Verify LIDAR sensor simulation

```bash
# Terminal 1: Launch with test room
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=test_room

# Terminal 2: Check scan data
ros2 topic echo /scan
```

**Success Criteria:**
- [ ] `/scan` topic publishing at ~10 Hz
- [ ] 360 range measurements
- [ ] Ranges between 0.15m (min) and 12.0m (max)
- [ ] Detects walls at correct distances
- [ ] Visualized correctly in RViz

#### Test 4: Odometry

**Goal:** Verify odometry accuracy

```bash
# Terminal 1: Launch simulation
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py

# Terminal 2: Monitor odometry
ros2 topic echo /odom
```

Drive robot in square:
```bash
# Forward 1m
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
# (after 5 seconds, stop)

# Turn 90 degrees
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --rate 10
# (after ~3 seconds, stop)

# Repeat 4 times
```

**Success Criteria:**
- [ ] Odometry updates continuously
- [ ] Position changes match commanded motion
- [ ] Returns approximately to start position after square
- [ ] TF tree includes odom→base_footprint

### SLAM Testing

#### Test 5: SLAM Mapping

**Goal:** Create map of simulated environment

```bash
# Terminal 1: Launch Gazebo with obstacle course
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=obstacle_course

# Terminal 2: Launch SLAM
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=slam \
  use_sim_time:=true

# Terminal 3: Teleoperate robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Success Criteria:**
- [ ] Map appears in RViz
- [ ] Map matches world geometry
- [ ] No excessive drift
- [ ] Loop closure detection works
- [ ] Map quality good (clear walls, no artifacts)

#### Test 6: Save and Load Map

```bash
# Save map (after SLAM test)
ros2 run nav2_map_server map_saver_cli -f ~/maps/test_map

# Restart with saved map
# Terminal 1: Gazebo
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=obstacle_course

# Terminal 2: Navigation with saved map
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=navigation \
  map:=~/maps/test_map.yaml \
  use_sim_time:=true
```

**Success Criteria:**
- [ ] Map loads without errors
- [ ] Map displayed in RViz
- [ ] Map matches environment
- [ ] Robot can localize on map

### Navigation Testing

#### Test 7: Localization (AMCL)

**Goal:** Verify robot can localize on known map

```bash
# Launch navigation stack (from Test 6)
# In RViz:
# 1. Click "2D Pose Estimate"
# 2. Click robot's actual position
# 3. Drag to set orientation
```

**Success Criteria:**
- [ ] AMCL particles converge
- [ ] Robot pose matches true position
- [ ] Localization stable during motion
- [ ] Re-localization works after poor estimate

#### Test 8: Path Planning

**Goal:** Verify global path planner works

```bash
# Launch navigation stack
# In RViz:
# 1. Set initial pose (2D Pose Estimate)
# 2. Set goal (2D Nav Goal)
```

**Success Criteria:**
- [ ] Path appears in RViz (green line)
- [ ] Path avoids obstacles
- [ ] Path is reasonable (not excessively long)
- [ ] Path updates if goal changes

#### Test 9: Autonomous Navigation

**Goal:** Verify complete navigation stack

```bash
# Launch navigation stack
# Set multiple goals in different areas
```

**Success Criteria:**
- [ ] Robot reaches goals successfully
- [ ] Avoids obstacles dynamically
- [ ] Recovers from being stuck
- [ ] Handles narrow passages
- [ ] Smooth motion (no excessive oscillation)

### Performance Testing

#### Test 10: Real-time Performance

**Goal:** Verify simulation runs at real-time

```bash
# Terminal 1: Launch simulation
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py

# Terminal 2: Monitor real-time factor
gz topic -e -t /stats
```

**Success Criteria:**
- [ ] Real-time factor close to 1.0
- [ ] Update rate stable
- [ ] No excessive CPU usage (<50% on modern hardware)
- [ ] GUI responsive

### Regression Testing

Create automated test scripts for CI/CD:

```bash
#!/bin/bash
# test_simulation.sh

# Launch Gazebo headless
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_gui:=false \
  use_rviz:=false &
GAZEBO_PID=$!

# Wait for Gazebo to start
sleep 10

# Test velocity control
timeout 5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10

# Check odometry updated
ODOM_COUNT=$(timeout 2 ros2 topic echo /odom | wc -l)
if [ $ODOM_COUNT -gt 0 ]; then
  echo "✓ Odometry working"
else
  echo "✗ Odometry failed"
  exit 1
fi

# Check LIDAR
SCAN_COUNT=$(timeout 2 ros2 topic echo /scan | wc -l)
if [ $SCAN_COUNT -gt 0 ]; then
  echo "✓ LIDAR working"
else
  echo "✗ LIDAR failed"
  exit 1
fi

# Cleanup
kill $GAZEBO_PID

echo "All tests passed!"
```

---

## Gazebo vs Rosbag Testing

### When to Use Gazebo Simulation

**Use Gazebo When:**

1. **Algorithm Development**
   - Developing new navigation algorithms
   - Testing parameter changes
   - Experimenting with different sensor configurations

2. **Pre-Hardware Testing**
   - No physical robot available yet
   - Hardware under repair
   - Testing destructive scenarios

3. **Controlled Experiments**
   - Need exact repeatability
   - Testing specific scenarios
   - Validating theoretical predictions

4. **Training and Learning**
   - Teaching navigation concepts
   - Demonstrating functionality
   - Learning ROS2/Nav2 without hardware

5. **Rapid Iteration**
   - Need to test many variations quickly
   - Exploring parameter space
   - A/B testing different approaches

### When to Use Rosbag Testing

**Use Rosbags When:**

1. **Real-World Validation**
   - Testing with actual sensor data
   - Validating against real sensor noise
   - Verifying real-world performance

2. **Debugging Hardware Issues**
   - Investigating specific failure cases
   - Reproducing bugs from real runs
   - Analyzing anomalous behavior

3. **Performance Benchmarking**
   - Measuring actual processing times
   - Testing with real data rates
   - Validating resource usage

4. **Data-Driven Development**
   - Using recorded data for testing
   - Regression testing against known-good data
   - Comparing algorithm versions on same data

5. **Limited Simulation Fidelity**
   - Testing sensor fusion (multiple sensors)
   - Dealing with specific environmental conditions
   - Handling edge cases hard to simulate

### Comparison Table

| Aspect | Gazebo Simulation | Rosbag Testing |
|--------|-------------------|----------------|
| **Setup Time** | Moderate (world creation) | Low (just play bag) |
| **Repeatability** | Perfect | Perfect |
| **Realism** | Good (physics-based) | Excellent (real data) |
| **Flexibility** | High (change anything) | Low (fixed data) |
| **Interaction** | Full (can control robot) | Limited (replay only) |
| **Sensor Fidelity** | Simulated (noise models) | Real (actual sensors) |
| **Cost** | Free (computation) | Requires hardware initially |
| **Speed** | Adjustable (faster/slower) | Adjustable (faster/slower) |
| **Edge Cases** | Create any scenario | Limited to recorded data |
| **Learning** | Good for concepts | Good for real-world issues |

### Recommended Workflow

**Optimal Development Cycle:**

1. **Phase 1: Algorithm Development (Gazebo)**
   - Develop and test basic algorithms
   - Tune parameters in controlled environments
   - Verify functionality in simple scenarios

2. **Phase 2: Scenario Testing (Gazebo)**
   - Test in complex environments
   - Validate edge case handling
   - Ensure robustness

3. **Phase 3: Real-World Validation (Rosbag)**
   - Test with recorded real sensor data
   - Identify simulation-to-reality gaps
   - Refine based on real-world performance

4. **Phase 4: Hardware Testing**
   - Deploy to physical robot
   - Record new rosbags of problematic scenarios
   - Iterate between rosbag testing and Gazebo refinement

5. **Phase 5: Regression Testing (Both)**
   - Maintain Gazebo test suite for unit/integration tests
   - Maintain rosbag test suite for real-world validation
   - Run both in CI/CD pipeline

### Hybrid Approach

Combine both methods:

```bash
# Use simulation for algorithm development
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py

# Record simulated sensor data
ros2 bag record /scan /odom /tf /cmd_vel

# Later, test navigation stack with recorded sim data
ros2 bag play <sim_bag> &
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=navigation \
  map:=<map_file> \
  use_sim_time:=true
```

This allows:
- Consistent testing environment
- Easy sharing of test scenarios
- Automated regression tests
- Reproducible bug reports

---

## Troubleshooting

### Common Issues and Solutions

#### Issue 1: Gazebo Won't Start

**Symptoms:**
- `gz sim` command not found
- Gazebo crashes on startup
- Black screen or frozen window

**Solutions:**

```bash
# Check if Gazebo is installed
gz sim --version

# If not installed or wrong version:
sudo apt update
sudo apt install ignition-fortress --reinstall

# Check for conflicting Gazebo installations
dpkg -l | grep gazebo
dpkg -l | grep ignition

# Remove old Gazebo Classic if present
sudo apt remove gazebo* libgazebo*
sudo apt autoremove

# Clear Gazebo cache
rm -rf ~/.gz/
rm -rf ~/.gazebo/

# Try launching again
gz sim
```

#### Issue 2: Robot Doesn't Spawn

**Symptoms:**
- Gazebo launches but robot not visible
- "Spawn failed" error messages
- URDF parsing errors

**Solutions:**

```bash
# Check robot_description topic is published
ros2 topic list | grep robot_description
ros2 topic echo /robot_description --once

# Verify URDF is valid
cd ~/your_workspace
source install/setup.bash
xacro install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/urdf/wayfinder_robot.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf

# Check for ROS-Gazebo bridge
ros2 pkg list | grep ros_gz

# Verify spawn command works manually
ros2 run ros_gz_sim create \
  -name wayfinder \
  -file /tmp/test.urdf \
  -x 0 -y 0 -z 0.1
```

#### Issue 3: No Sensor Data

**Symptoms:**
- `/scan` or `/odom` topics not publishing
- Empty messages
- RViz shows no data

**Solutions:**

```bash
# Check if topics exist
ros2 topic list | grep -E '(scan|odom)'

# Check topic data rates
ros2 topic hz /scan
ros2 topic hz /odom

# Verify bridges are running
ros2 node list | grep bridge

# Check if use_sim_time is set correctly
ros2 param get /robot_state_publisher use_sim_time

# Restart bridges
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry &

# Check Gazebo topics directly
gz topic -l
gz topic -e -t /scan
```

#### Issue 4: Robot Falls Through Ground

**Symptoms:**
- Robot drops below floor
- Collision detection not working
- Physics behaving oddly

**Solutions:**

```xml
<!-- In URDF, ensure proper inertia and collision tags -->
<link name="base_link">
  <collision>
    <geometry>
      <cylinder radius="${base_radius}" length="${base_height}"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="${base_mass}"/>
    <!-- Proper inertia tensor -->
  </inertial>
</link>

<!-- Ensure wheels have collision geometry -->
<!-- Ensure gazebo friction parameters are set -->
<gazebo reference="left_wheel">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

```bash
# Check physics plugin is loaded
gz model -m wayfinder -i

# Adjust spawn height
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py z_pose:=0.2
```

#### Issue 5: use_sim_time Errors

**Symptoms:**
- TF timeout warnings
- "Could not transform" errors
- Navigation stack failures

**Solutions:**

```bash
# Ensure /clock topic is publishing
ros2 topic list | grep clock
ros2 topic echo /clock

# Check all nodes have use_sim_time=true
ros2 param list | grep use_sim_time

# Launch with explicit sim_time
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=slam \
  use_sim_time:=true

# Verify in ROS parameters
ros2 param get /slam_toolbox use_sim_time
```

#### Issue 6: Performance Issues

**Symptoms:**
- Slow simulation
- Low real-time factor
- Laggy GUI

**Solutions:**

```bash
# Launch without GUI for better performance
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_gui:=false

# Reduce physics update rate (in world SDF)
<physics>
  <max_step_size>0.005</max_step_size>  <!-- was 0.001 -->
  <real_time_factor>1.0</real_time_factor>
</physics>

# Disable shadows and reduce quality
# In Gazebo GUI: Window → Rendering → Disable Shadows

# Close RViz if not needed
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  use_rviz:=false

# Check system resources
top  # Look for gz process CPU/memory usage
```

#### Issue 7: World Not Loading

**Symptoms:**
- Empty world in Gazebo
- "Could not find world file" error
- World file parsing errors

**Solutions:**

```bash
# Check world file path
ls -l ~/your_workspace/install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/worlds/

# Use absolute path
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=/full/path/to/world.sdf

# Validate SDF syntax
gz sdf -k /path/to/world.sdf

# Set resource path
export GZ_SIM_RESOURCE_PATH=/path/to/worlds:$GZ_SIM_RESOURCE_PATH
```

### Debug Commands

```bash
# List all Gazebo topics
gz topic -l

# Echo Gazebo topic
gz topic -e -t /topic_name

# List models in simulation
gz model -l

# Get model info
gz model -m model_name -i

# List services
gz service -l

# Check Gazebo version
gz sim --version

# Verbose Gazebo output
gz sim -v 4  # verbosity level 4
```

### Getting Help

If issues persist:

1. **Check Logs**
   ```bash
   # ROS logs
   cat ~/.ros/log/latest/rosout.log

   # Gazebo logs
   cat ~/.gz/sim/log/latest/gazebo.log
   ```

2. **Community Resources**
   - [Gazebo Answers](https://answers.gazebosim.org/)
   - [ROS Answers](https://answers.ros.org/)
   - [Gazebo Forums](https://community.gazebosim.org/)

3. **Report Issues**
   - Provide Gazebo version: `gz sim --version`
   - Provide ROS version: `ros2 --version`
   - Include error messages and logs
   - Describe steps to reproduce

---

## Advanced Topics

### Parameter Tuning in Simulation

Use simulation to optimize Nav2 parameters:

```bash
# Terminal 1: Launch simulation
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=obstacle_course

# Terminal 2: Launch navigation
ros2 launch ros2_comprehensive_attempt bringup.launch.py \
  mode:=navigation \
  map:=<map_file> \
  use_sim_time:=true \
  nav2_params:=<custom_params.yaml>

# Terminal 3: Monitor performance metrics
ros2 topic echo /performance_metrics

# Iterate on parameters and test
```

### Realistic Sensor Simulation

Enhance LIDAR realism in URDF:

```xml
<gazebo reference="laser">
  <sensor type="ray" name="rplidar_sensor">
    <ray>
      <range>
        <min>0.15</min>
        <max>12.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>  <!-- Increase for more noise -->
      </noise>
    </ray>
    <!-- Add dropout simulation -->
    <dropout>
      <probability>0.01</probability>  <!-- 1% dropout rate -->
    </dropout>
  </sensor>
</gazebo>
```

### Multi-Robot Simulation

Spawn multiple robots:

```python
# In launch file
for i in range(3):
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', f'wayfinder_{i}',
            '-topic', '/robot_description',
            '-x', str(i * 2.0),
            '-y', '0',
            '-z', '0.1',
        ],
        namespace=f'robot_{i}'
    )
    nodes.append(spawn_robot)
```

### Recording Simulated Data

Create test datasets:

```bash
# Start simulation
ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
  world:=obstacle_course

# Record all relevant topics
ros2 bag record -o sim_test_run \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /cmd_vel \
  /clock

# Use recorded data for testing
ros2 bag play sim_test_run
```

### Automated Testing Scripts

Create regression tests:

```python
#!/usr/bin/env python3
"""Automated simulation test suite."""

import unittest
import subprocess
import time

class SimulationTests(unittest.TestCase):

    def test_spawn_robot(self):
        """Test robot spawns successfully."""
        # Launch simulation
        proc = subprocess.Popen([
            'ros2', 'launch',
            'ros2_comprehensive_attempt',
            'gazebo_sim.launch.py',
            'use_gui:=false'
        ])

        time.sleep(10)  # Wait for startup

        # Check robot_description topic
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True
        )

        self.assertIn('/robot_description', result.stdout)

        proc.terminate()

    # More tests...

if __name__ == '__main__':
    unittest.main()
```

### Custom Sensor Plugins

Add custom sensors (example: GPS):

```xml
<gazebo reference="base_link">
  <sensor name="gps_sensor" type="gps">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <gps>
      <position_sensing>
        <horizontal>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.0</stddev>
          </noise>
        </horizontal>
      </position_sensing>
    </gps>
    <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
      <ros>
        <namespace>/</namespace>
        <remapping>gps:=gps/fix</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## References

### Official Documentation

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Fortress Documentation](https://gazebosim.org/docs/fortress)
- [ROS2 + Gazebo Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

### Key GitHub Repositories

- [ros_gz](https://github.com/gazebosim/ros_gz) - ROS-Gazebo bridge
- [gz-sim](https://github.com/gazebosim/gz-sim) - Gazebo simulator
- [Navigation2](https://github.com/ros-planning/navigation2)

### Tutorials and Guides

- [Installing Gazebo with ROS](https://gazebosim.org/docs/latest/ros_installation/)
- [Using URDF in Gazebo](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-a-URDF-in-Gazebo.html)
- [Setting up Robot Simulation](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Automatic Addison - Mobile Robot Simulation](https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/)
- [Black Coffee Robotics - Migration Guide](https://www.blackcoffeerobotics.com/blog/migration-from-gazebo-classic-to-ignition-with-ros-2)

### Community Resources

- [ROS Discourse](https://discourse.ros.org/)
- [Gazebo Community](https://community.gazebosim.org/)
- [ROS Answers](https://answers.ros.org/)
- [Gazebo Answers](https://answers.gazebosim.org/)

### Research Papers

If using in academic work, consider citing:

- Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.

- Macenski, S., Martín, F., White, R., & Clavero, J. G. (2020). The Marathon 2: A Navigation System. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.

---

## Conclusion

Gazebo Fortress provides a powerful, realistic simulation environment for developing and testing the WayfindR robot. By following this guide, you can:

- Set up a complete simulation environment
- Test navigation algorithms safely and efficiently
- Iterate rapidly without hardware constraints
- Validate functionality before real-world deployment

Remember to use simulation as part of a comprehensive testing strategy that includes both simulated and real-world validation.

**Next Steps:**

1. Install Gazebo Fortress following the installation section
2. Run the Quick Start guide to verify your setup
3. Create a simple test scenario in simulation
4. Run SLAM mapping in the obstacle course world
5. Test autonomous navigation
6. Integrate with your existing development workflow

For questions or issues, refer to the Troubleshooting section or reach out to the community resources listed in References.

Happy simulating!

---

**Document Version:** 1.0
**Last Updated:** 2026-01-11
**Maintainer:** WayfindR Development Team
