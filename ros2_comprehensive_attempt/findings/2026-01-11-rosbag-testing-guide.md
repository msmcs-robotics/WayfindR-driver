# ROS2 Nav2 Testing with Rosbags and Simulation
**Date:** 2026-01-11
**Author:** WayfindR Development Team
**Purpose:** Comprehensive guide for testing Nav2 SLAM, localization, and navigation without physical hardware

## Table of Contents
1. [Overview](#overview)
2. [Testing Approaches](#testing-approaches)
3. [Rosbag2 Fundamentals](#rosbag2-fundamentals)
4. [Recording Real LiDAR Data](#recording-real-lidar-data)
5. [Creating Synthetic Scan Data](#creating-synthetic-scan-data)
6. [Using Simulation Time](#using-simulation-time)
7. [Testing SLAM with Rosbags](#testing-slam-with-rosbags)
8. [Testing Localization with Rosbags](#testing-localization-with-rosbags)
9. [Testing Navigation with Rosbags](#testing-navigation-with-rosbags)
10. [Best Practices](#best-practices)
11. [Example Test Scenarios](#example-test-scenarios)
12. [Troubleshooting](#troubleshooting)

---

## Overview

Testing ROS2 navigation systems without physical hardware is essential for:
- **Cost Efficiency**: Avoid expensive hardware during development
- **Safety**: Test potentially dangerous behaviors in simulation
- **Repeatability**: Run identical test scenarios consistently
- **CI/CD Integration**: Automated testing in continuous integration pipelines
- **Rapid Iteration**: Faster development cycles without hardware setup

### Key Testing Methods

1. **Gazebo Simulation**: Full physics simulation with TurtleBot3
2. **Rosbag Replay**: Recorded sensor data from real or simulated robots
3. **Synthetic Data Generation**: Programmatically created sensor messages
4. **Hybrid Approaches**: Combination of simulation and recorded data

---

## Testing Approaches

### 1. Simulation-Based Testing (Recommended for Beginners)

**Advantages:**
- Complete environment control
- No hardware required
- Physics-based sensor simulation
- Visual feedback through Gazebo

**Components:**
- Gazebo Classic or Gazebo Ignition
- TurtleBot3 simulation packages
- Nav2 stack
- RViz for visualization

**Quick Start:**
```bash
# Install TurtleBot3 simulation packages
sudo apt install ros-humble-turtlebot3-gazebo

# Launch TurtleBot3 in Gazebo world
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch Nav2 with SLAM
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

### 2. Rosbag-Based Testing (For Realistic Data)

**Advantages:**
- Uses real sensor characteristics
- Reproducible test conditions
- No need for simulation overhead
- Can capture edge cases from real environments

**Use Cases:**
- Regression testing
- Algorithm comparison
- Performance benchmarking
- Debugging specific scenarios

### 3. Synthetic Data Testing (For Controlled Scenarios)

**Advantages:**
- Precise control over sensor data
- Test edge cases and failure modes
- No dependency on recorded data
- Automated test generation

**Use Cases:**
- Unit testing
- Boundary condition testing
- Stress testing
- Continuous integration

---

## Rosbag2 Fundamentals

### What is Rosbag2?

Rosbag2 is ROS 2's data recording and playback system. It captures messages published on topics and stores them in a database format (default: SQLite3 or MCAP).

### Basic Commands

#### Recording Data
```bash
# Record specific topics
ros2 bag record /scan /odom /tf /tf_static

# Record all topics
ros2 bag record -a

# Record with custom name
ros2 bag record -o my_test_data /scan /odom

# Record with compression
ros2 bag record -a --compression-mode file --compression-format zstd
```

#### Playing Back Data
```bash
# Basic playback
ros2 bag play my_bag_file

# Playback with simulation time
ros2 bag play my_bag_file --clock

# Playback at half speed
ros2 bag play my_bag_file --rate 0.5

# Loop playback
ros2 bag play my_bag_file --loop
```

#### Inspecting Bags
```bash
# Show bag info
ros2 bag info my_bag_file

# Convert bag format
ros2 bag convert -i input_bag -o output_bag
```

### Rosbag Storage Formats

**SQLite3 (Default):**
- Good for general use
- Wide compatibility
- Moderate performance

**MCAP (Recommended for large datasets):**
- Better performance
- Efficient compression
- Industry standard for logging
```bash
ros2 bag record -s mcap /scan /odom
```

---

## Recording Real LiDAR Data

### Essential Topics for Nav2

#### Minimum Required Topics
```bash
/scan                    # sensor_msgs/LaserScan (2D LiDAR)
/odom                    # nav_msgs/Odometry
/tf                      # tf2_msgs/TFMessage
/tf_static               # tf2_msgs/TFMessage
```

#### Recommended Additional Topics
```bash
/cmd_vel                 # geometry_msgs/Twist (for playback verification)
/map                     # nav_msgs/OccupancyGrid (if mapping)
/amcl_pose               # geometry_msgs/PoseWithCovarianceStamped
/robot_description       # Robot URDF
/joint_states            # sensor_msgs/JointState
```

### Recording Script for Nav2

```bash
#!/bin/bash
# Record comprehensive Nav2 dataset

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="nav2_dataset_${TIMESTAMP}"

echo "Recording Nav2 dataset to: ${BAG_NAME}"

ros2 bag record \
  -o ${BAG_NAME} \
  --compression-mode file \
  --compression-format zstd \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /cmd_vel \
  /map \
  /amcl_pose \
  /robot_description \
  /joint_states \
  /imu

echo "Recording started. Press Ctrl+C to stop."
```

### Recording from Simulation

When recording from Gazebo or other simulators:

```bash
# 1. Start simulation with sim_time enabled
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 2. Start SLAM or navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

# 3. Record with sim_time
ros2 bag record --use-sim-time \
  -o simulation_dataset \
  /scan /odom /tf /tf_static /map

# 4. Drive the robot (teleop or autonomous)
ros2 run turtlebot3_teleop teleop_keyboard
```

### Data Quality Considerations

**LiDAR Recording Tips:**
1. **Coverage**: Record diverse environments (open, cluttered, corners)
2. **Motion**: Include various speeds and turning rates
3. **Duration**: 2-5 minutes per scenario minimum
4. **Dynamic Objects**: Capture moving obstacles if needed
5. **Edge Cases**: Record difficult scenarios (narrow passages, symmetry)

**Validation Checklist:**
- [ ] Verify all topics are present
- [ ] Check message timestamps are monotonic
- [ ] Ensure TF tree is complete
- [ ] Validate LiDAR data has valid ranges
- [ ] Confirm odometry is reasonable

---

## Creating Synthetic Scan Data

### Why Synthetic Data?

- **Deterministic Testing**: Exact repeatability
- **Edge Case Generation**: Test scenarios hard to capture
- **Automated CI/CD**: No manual data collection
- **Rapid Prototyping**: Test without hardware or simulation

### Approach 1: Python Fake LaserScan Publisher

Create a simple test publisher for basic functionality testing:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FakeLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_scan_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.angle = 0.0

    def publish_scan(self):
        scan = LaserScan()

        # Header
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        # Scan parameters (typical for RPLiDAR A1)
        scan.angle_min = -3.14159  # -180 degrees
        scan.angle_max = 3.14159   # +180 degrees
        scan.angle_increment = 0.0174533  # 1 degree
        scan.time_increment = 0.0001
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0

        # Generate synthetic ranges (simple corridor)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = []
        scan.intensities = []

        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment

            # Simulate walls at 2m on left/right, 3m front/back
            if abs(angle) < 0.5:  # Front
                distance = 3.0
            elif abs(angle - 3.14159) < 0.5:  # Back
                distance = 3.0
            elif abs(angle - 1.5708) < 0.5:  # Left
                distance = 2.0
            elif abs(angle + 1.5708) < 0.5:  # Right
                distance = 2.0
            else:
                distance = 10.0  # Open space

            scan.ranges.append(distance)
            scan.intensities.append(100.0)

        self.publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Approach 2: Advanced Synthetic Data with Obstacles

For more realistic testing scenarios:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import random

class SyntheticNavDataPublisher(Node):
    """
    Publishes synthetic laser scan and odometry data simulating
    a robot moving in an environment with obstacles.
    """

    def __init__(self):
        super().__init__('synthetic_nav_data_publisher')

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.2  # m/s
        self.angular_vel = 0.1  # rad/s

        # Environment obstacles (x, y, radius)
        self.obstacles = [
            (2.0, 0.0, 0.3),
            (-1.5, 1.5, 0.4),
            (1.0, -2.0, 0.5),
        ]

        # Timers
        self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.create_timer(0.05, self.publish_odom)  # 20 Hz

    def publish_scan(self):
        """Publish synthetic laser scan with obstacles"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        # Scan configuration
        scan.angle_min = -3.14159
        scan.angle_max = 3.14159
        scan.angle_increment = 0.0174533  # 1 degree
        scan.time_increment = 0.0001
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)

        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            global_angle = self.theta + angle

            # Default max range
            min_range = scan.range_max

            # Check collision with each obstacle
            for obs_x, obs_y, obs_radius in self.obstacles:
                # Vector from robot to obstacle
                dx = obs_x - self.x
                dy = obs_y - self.y

                # Project onto scan ray
                ray_x = math.cos(global_angle)
                ray_y = math.sin(global_angle)

                # Distance to obstacle center
                dist_to_center = math.sqrt(dx*dx + dy*dy)

                # Angle to obstacle
                angle_to_obs = math.atan2(dy, dx)
                angle_diff = abs(angle_to_obs - global_angle)

                # If ray points toward obstacle
                if angle_diff < 0.1:  # Within cone
                    # Approximate range (simplified)
                    range_to_obs = dist_to_center - obs_radius
                    if range_to_obs > 0:
                        min_range = min(min_range, range_to_obs)

            # Add some noise
            range_with_noise = min_range + random.gauss(0, 0.02)
            range_with_noise = max(scan.range_min, min(scan.range_max, range_with_noise))

            scan.ranges.append(range_with_noise)
            scan.intensities.append(100.0)

        self.scan_pub.publish(scan)

    def publish_odom(self):
        """Publish synthetic odometry"""
        dt = 0.05

        # Update robot pose
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt

        # Normalize theta
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SyntheticNavDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Approach 3: Using Gazebo for Synthetic Bags

Record from Gazebo simulation for high-quality synthetic data:

```bash
#!/bin/bash
# Generate synthetic rosbag from Gazebo simulation

# 1. Launch Gazebo world
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
GAZEBO_PID=$!

sleep 10  # Wait for Gazebo to start

# 2. Start recording
ros2 bag record --use-sim-time \
  -o synthetic_test_$(date +%Y%m%d_%H%M%S) \
  /scan /odom /tf /tf_static &
RECORD_PID=$!

sleep 2

# 3. Drive robot in pattern
ros2 run turtlebot3_gazebo turtlebot3_drive &
DRIVE_PID=$!

# 4. Record for 60 seconds
sleep 60

# 5. Cleanup
kill $DRIVE_PID
kill $RECORD_PID
kill $GAZEBO_PID

echo "Synthetic bag recording complete!"
```

---

## Using Simulation Time

### Why Simulation Time Matters

In real-time systems, timestamps are critical for:
- **Sensor Fusion**: Synchronizing data from multiple sensors
- **TF Transforms**: Time-based coordinate transformations
- **Motion Planning**: Predicting future states
- **Localization**: Correlating observations with maps

When replaying rosbags, you want the system to use the **recorded timestamps**, not wall clock time.

### How Simulation Time Works

**Normal Mode (Wall Clock):**
- Nodes use system time
- `now()` returns current computer time
- Real-time execution

**Simulation Time Mode:**
- Nodes subscribe to `/clock` topic
- `now()` returns published clock value
- Playback controls time progression

### Enabling Simulation Time

#### Method 1: Command Line Parameter
```bash
# For each node that needs sim_time
ros2 run my_package my_node --ros-args -p use_sim_time:=true
```

#### Method 2: Launch File Parameter
```python
# In launch file
parameters=[{'use_sim_time': True}]
```

#### Method 3: YAML Configuration
```yaml
# config.yaml
/**:
  ros__parameters:
    use_sim_time: true
```

### Complete Rosbag Replay with Sim Time

```bash
# Terminal 1: Play bag with clock publishing
ros2 bag play my_bag --clock

# Terminal 2: Launch Nav2 with sim_time
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true

# Terminal 3: Launch RViz with sim_time
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

### Playback Speed Control

```bash
# Half speed (good for debugging)
ros2 bag play my_bag --clock --rate 0.5

# Double speed (faster testing)
ros2 bag play my_bag --clock --rate 2.0

# Pause/resume playback (interactive)
ros2 service call /rosbag2_player/pause std_srvs/srv/Empty
ros2 service call /rosbag2_player/resume std_srvs/srv/Empty
```

### Verifying Sim Time Configuration

```bash
# Check if a node is using sim_time
ros2 param get /my_node use_sim_time

# Monitor clock topic
ros2 topic echo /clock

# Verify TF is using correct time
ros2 run tf2_ros tf2_echo base_link laser_frame
```

### Common Sim Time Issues

**Problem: TF lookups fail with "future" errors**
```
Solution: Ensure ALL nodes have use_sim_time:=true
```

**Problem: SLAM/AMCL doesn't update**
```
Solution: Verify --clock flag on ros2 bag play
```

**Problem: Inconsistent behavior**
```
Solution: Check message timestamps match /clock topic
```

---

## Testing SLAM with Rosbags

### SLAM Testing Overview

SLAM (Simultaneous Localization and Mapping) is critical for autonomous navigation. Testing SLAM with rosbags allows:
- Comparing different SLAM algorithms
- Tuning parameters for your environment
- Regression testing after code changes
- Benchmarking performance

### Setup for SLAM Testing

#### Option 1: slam_toolbox (Recommended)

```bash
# Install slam_toolbox
sudo apt install ros-humble-slam-toolbox

# Create test configuration
mkdir -p ~/nav2_testing/config
```

Create `slam_toolbox_test.yaml`:
```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan

    # Mode
    mode: mapping  # or localization

    # Performance
    map_update_interval: 1.0
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_publish_period: 0.02

    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # Scan Matcher
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0

    # Loop Closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
```

#### Testing Script for SLAM

```bash
#!/bin/bash
# test_slam_with_bag.sh

BAG_FILE=$1
if [ -z "$BAG_FILE" ]; then
    echo "Usage: $0 <bag_file>"
    exit 1
fi

# Create output directory
OUTPUT_DIR="slam_test_$(date +%Y%m%d_%H%M%S)"
mkdir -p $OUTPUT_DIR

echo "Testing SLAM with bag: $BAG_FILE"
echo "Output directory: $OUTPUT_DIR"

# Launch slam_toolbox in background
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  params_file:=slam_toolbox_test.yaml &
SLAM_PID=$!

sleep 5

# Start recording SLAM outputs
ros2 bag record -o $OUTPUT_DIR/slam_output \
  /map /slam_toolbox/graph_visualization \
  /slam_toolbox/scan_visualization &
RECORD_PID=$!

# Play the input bag
ros2 bag play $BAG_FILE --clock --rate 1.0

# Allow SLAM to finalize
sleep 5

# Save the map
ros2 run nav2_map_server map_saver_cli -f $OUTPUT_DIR/final_map

# Cleanup
kill $RECORD_PID
kill $SLAM_PID

echo "SLAM test complete! Results in: $OUTPUT_DIR"
echo "Map saved as: $OUTPUT_DIR/final_map.pgm"
```

### SLAM Quality Metrics

Create a Python script to evaluate SLAM performance:

```python
#!/usr/bin/env python3
"""
Evaluate SLAM output quality from rosbag testing
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml

class SLAMEvaluator(Node):
    def __init__(self):
        super().__init__('slam_evaluator')
        self.map_data = None

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        self.map_data = msg

    def evaluate_map_quality(self):
        if self.map_data is None:
            print("No map data received")
            return

        data = np.array(self.map_data.data)

        # Calculate metrics
        total_cells = len(data)
        unknown_cells = np.sum(data == -1)
        free_cells = np.sum(data == 0)
        occupied_cells = np.sum(data > 0)

        unknown_pct = (unknown_cells / total_cells) * 100
        free_pct = (free_cells / total_cells) * 100
        occupied_pct = (occupied_cells / total_cells) * 100

        metrics = {
            'total_cells': int(total_cells),
            'unknown_cells': int(unknown_cells),
            'free_cells': int(free_cells),
            'occupied_cells': int(occupied_cells),
            'unknown_percentage': float(unknown_pct),
            'free_percentage': float(free_pct),
            'occupied_percentage': float(occupied_pct),
            'resolution': float(self.map_data.info.resolution),
            'width': int(self.map_data.info.width),
            'height': int(self.map_data.info.height),
        }

        # Save metrics
        with open('slam_metrics.yaml', 'w') as f:
            yaml.dump(metrics, f)

        print("\n=== SLAM Quality Metrics ===")
        for key, value in metrics.items():
            print(f"{key}: {value}")

        # Quality assessment
        if unknown_pct < 20:
            print("\n✓ Good map coverage (< 20% unknown)")
        else:
            print(f"\n✗ Poor map coverage ({unknown_pct:.1f}% unknown)")

        if 5 < occupied_pct < 30:
            print("✓ Reasonable obstacle density")
        else:
            print(f"✗ Unusual obstacle density ({occupied_pct:.1f}%)")

def main(args=None):
    rclpy.init(args=args)
    node = SLAMEvaluator()

    # Spin for 5 seconds to receive map
    import time
    start = time.time()
    while time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.evaluate_map_quality()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Testing Localization with Rosbags

### AMCL Testing Setup

AMCL (Adaptive Monte Carlo Localization) requires a pre-existing map and localizes the robot within it.

#### Prerequisites
1. A map file (`.pgm` + `.yaml`)
2. Rosbag with `/scan` and `/odom` data
3. Initial pose estimate

#### AMCL Configuration for Testing

Create `amcl_test.yaml`:
```yaml
amcl:
  ros__parameters:
    use_sim_time: true

    # Overall filter parameters
    min_particles: 500
    max_particles: 2000
    kld_err: 0.05
    kld_z: 0.99
    update_min_d: 0.2
    update_min_a: 0.2
    resample_interval: 1
    transform_tolerance: 0.5
    recovery_alpha_slow: 0.0
    recovery_alpha_fast: 0.0

    # Laser model parameters
    laser_max_range: 12.0
    laser_min_range: 0.15
    laser_max_beams: 60
    laser_z_hit: 0.5
    laser_z_short: 0.05
    laser_z_max: 0.05
    laser_z_rand: 0.5
    laser_sigma_hit: 0.2
    laser_lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_model_type: "likelihood_field"

    # Odometry model parameters
    odom_model_type: "diff-corrected"
    odom_alpha1: 0.2
    odom_alpha2: 0.2
    odom_alpha3: 0.2
    odom_alpha4: 0.2
    odom_alpha5: 0.1

    # Robot model
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    set_initial_pose: false
    always_reset_initial_pose: false

    # Frames
    global_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    scan_topic: "scan"
```

#### Localization Testing Script

```bash
#!/bin/bash
# test_localization_with_bag.sh

if [ $# -lt 2 ]; then
    echo "Usage: $0 <map_file> <bag_file>"
    echo "Example: $0 my_map.yaml test_data.db3"
    exit 1
fi

MAP_FILE=$1
BAG_FILE=$2
OUTPUT_DIR="localization_test_$(date +%Y%m%d_%H%M%S)"

mkdir -p $OUTPUT_DIR

echo "Testing localization with:"
echo "  Map: $MAP_FILE"
echo "  Bag: $BAG_FILE"
echo "  Output: $OUTPUT_DIR"

# Launch map server
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=$MAP_FILE -p use_sim_time:=true &
MAP_SERVER_PID=$!

sleep 2

# Activate map server lifecycle
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# Launch AMCL
ros2 run nav2_amcl amcl \
  --ros-args --params-file amcl_test.yaml &
AMCL_PID=$!

sleep 3

# Set initial pose (adjust coordinates based on your map)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {frame_id: 'map'},
  pose: {
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.06854]
  }
}"

# Record AMCL output
ros2 bag record -o $OUTPUT_DIR/amcl_output \
  /amcl_pose /particlecloud /tf &
RECORD_PID=$!

# Play the bag
ros2 bag play $BAG_FILE --clock

# Cleanup
sleep 3
kill $RECORD_PID
kill $AMCL_PID
kill $MAP_SERVER_PID

echo "Localization test complete! Results in: $OUTPUT_DIR"
```

### Localization Quality Analysis

```python
#!/usr/bin/env python3
"""
Analyze AMCL localization performance from test data
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

class LocalizationAnalyzer(Node):
    def __init__(self):
        super().__init__('localization_analyzer')

        self.amcl_poses = []
        self.odom_poses = []
        self.timestamps = []

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def amcl_callback(self, msg):
        pose = msg.pose.pose.position
        covariance = msg.pose.covariance

        self.amcl_poses.append({
            'x': pose.x,
            'y': pose.y,
            'cov_x': covariance[0],
            'cov_y': covariance[7],
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        })

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        self.odom_poses.append({
            'x': pose.x,
            'y': pose.y,
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        })

    def analyze(self):
        if len(self.amcl_poses) == 0:
            print("No AMCL data received")
            return

        # Extract data
        x_cov = [p['cov_x'] for p in self.amcl_poses]
        y_cov = [p['cov_y'] for p in self.amcl_poses]

        # Statistics
        mean_x_cov = np.mean(x_cov)
        mean_y_cov = np.mean(y_cov)
        max_x_cov = np.max(x_cov)
        max_y_cov = np.max(y_cov)

        print("\n=== Localization Quality Metrics ===")
        print(f"Total AMCL updates: {len(self.amcl_poses)}")
        print(f"Mean X covariance: {mean_x_cov:.4f}")
        print(f"Mean Y covariance: {mean_y_cov:.4f}")
        print(f"Max X covariance: {max_x_cov:.4f}")
        print(f"Max Y covariance: {max_y_cov:.4f}")

        # Quality assessment
        if mean_x_cov < 0.1 and mean_y_cov < 0.1:
            print("\n✓ Good localization certainty")
        elif mean_x_cov < 0.5 and mean_y_cov < 0.5:
            print("\n~ Moderate localization certainty")
        else:
            print("\n✗ Poor localization certainty")

        # Plot covariance over time
        plt.figure(figsize=(10, 6))
        times = [p['time'] - self.amcl_poses[0]['time'] for p in self.amcl_poses]
        plt.plot(times, x_cov, label='X Covariance')
        plt.plot(times, y_cov, label='Y Covariance')
        plt.xlabel('Time (s)')
        plt.ylabel('Covariance')
        plt.title('AMCL Localization Uncertainty Over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig('localization_covariance.png')
        print("\nPlot saved as: localization_covariance.png")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationAnalyzer()

    # Spin for data collection (adjust time as needed)
    import time
    start = time.time()
    while time.time() - start < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.analyze()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Testing Navigation with Rosbags

### Full Nav2 Stack Testing

Testing the complete navigation stack with rosbags validates:
- Global planning (path generation)
- Local planning (obstacle avoidance)
- Recovery behaviors
- Controller performance

#### Complete Nav2 Test Configuration

```yaml
# nav2_test_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

costmap:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    update_frequency: 5.0
    publish_frequency: 2.0
    rolling_window: false
    width: 20
    height: 20
    resolution: 0.05
```

#### Navigation Testing Script

```bash
#!/bin/bash
# test_navigation_with_bag.sh

if [ $# -lt 2 ]; then
    echo "Usage: $0 <map_file> <bag_file> [goal_x] [goal_y]"
    exit 1
fi

MAP_FILE=$1
BAG_FILE=$2
GOAL_X=${3:-2.0}
GOAL_Y=${4:-2.0}

OUTPUT_DIR="nav_test_$(date +%Y%m%d_%H%M%S)"
mkdir -p $OUTPUT_DIR

echo "Testing Nav2 with:"
echo "  Map: $MAP_FILE"
echo "  Bag: $BAG_FILE"
echo "  Goal: ($GOAL_X, $GOAL_Y)"

# Launch full Nav2 stack
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=$MAP_FILE \
  params_file:=nav2_test_params.yaml &
NAV2_PID=$!

sleep 10

# Set initial pose
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'},
  pose: {pose: {position: {x: 0.0, y: 0.0}}}}"

sleep 2

# Record navigation outputs
ros2 bag record -o $OUTPUT_DIR/nav_output \
  /cmd_vel /plan /local_plan /global_costmap/costmap \
  /local_costmap/costmap &
RECORD_PID=$!

# Start bag playback in background
ros2 bag play $BAG_FILE --clock &
BAG_PID=$!

sleep 5

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'},
           pose: {position: {x: $GOAL_X, y: $GOAL_Y},
                  orientation: {w: 1.0}}}}"

# Wait for completion (or timeout)
sleep 60

# Cleanup
kill $BAG_PID
kill $RECORD_PID
kill $NAV2_PID

echo "Navigation test complete! Results in: $OUTPUT_DIR"
```

---

## Best Practices

### 1. Rosbag Recording Best Practices

**DO:**
- ✓ Record TF and TF static always
- ✓ Use compression for large datasets
- ✓ Name bags descriptively with timestamps
- ✓ Record robot_description for reproducibility
- ✓ Document recording conditions (environment, robot state)
- ✓ Validate bag immediately after recording

**DON'T:**
- ✗ Record camera images unless necessary (huge file sizes)
- ✗ Record diagnostics topics (usually not needed)
- ✗ Mix real-time and sim-time data
- ✗ Forget to record odometry

### 2. Testing Workflow Best Practices

**Recommended Testing Sequence:**
1. **Unit Tests**: Synthetic data, single components
2. **Integration Tests**: Rosbag replay, multiple components
3. **System Tests**: Full stack in simulation
4. **Field Tests**: Real hardware validation

**Version Control for Test Data:**
```bash
# Use Git LFS for large bags
git lfs track "*.db3"
git lfs track "*.mcap"

# Or store bags externally with metadata in repo
# test_data/
#   datasets.yaml  <- Metadata and download links
#   README.md      <- Documentation
```

### 3. Reproducibility Best Practices

**Always Document:**
- ROS 2 distribution version
- Nav2 package versions
- Configuration files used
- Hardware specifications (if from real robot)
- Environmental conditions
- Known issues or anomalies

**Create Test Reports:**
```yaml
# test_report.yaml
test_date: 2026-01-11
ros_distro: humble
nav2_version: 1.1.10
bag_file: test_corridor_20260111.db3
duration_sec: 120
environment: office_corridor
robot_model: waffle
lidar_model: RPLIDAR A1

results:
  slam_success: true
  map_quality: good
  localization_error_mean: 0.05
  navigation_success_rate: 0.95

notes: |
  Good performance in straight corridors.
  Some drift in open areas without features.
```

### 4. Continuous Integration Best Practices

**Automated Testing Pipeline:**
```yaml
# .github/workflows/nav2_test.yml
name: Nav2 Rosbag Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    container:
      image: ros:humble

    steps:
      - uses: actions/checkout@v2

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y ros-humble-nav2-bringup

      - name: Download test bags
        run: |
          wget https://example.com/test_bags/corridor.db3

      - name: Run SLAM test
        run: |
          ./scripts/test_slam_with_bag.sh corridor.db3

      - name: Validate map output
        run: |
          python3 scripts/validate_map.py slam_output/

      - name: Upload results
        uses: actions/upload-artifact@v2
        with:
          name: test-results
          path: slam_output/
```

### 5. Performance Testing Best Practices

**Benchmark Key Metrics:**
- Map building time
- Localization convergence time
- Path planning latency
- Controller update frequency
- CPU/Memory usage

**Example Benchmarking Script:**
```python
#!/usr/bin/env python3
import psutil
import time
import rclpy

class PerformanceMonitor:
    def __init__(self):
        self.process = psutil.Process()
        self.start_time = time.time()

    def get_metrics(self):
        return {
            'cpu_percent': self.process.cpu_percent(),
            'memory_mb': self.process.memory_info().rss / 1024 / 1024,
            'elapsed_time': time.time() - self.start_time
        }

# Use during testing to track performance
```

---

## Example Test Scenarios

### Scenario 1: Corridor Navigation

**Objective**: Test robot navigation in narrow corridor

**Setup:**
```bash
# Record or use synthetic corridor data
./scripts/generate_corridor_bag.sh

# Test SLAM
./scripts/test_slam_with_bag.sh corridor.db3

# Test localization
./scripts/test_localization_with_bag.sh corridor_map.yaml corridor.db3

# Test navigation end-to-end
./scripts/test_navigation_with_bag.sh corridor_map.yaml corridor.db3 5.0 0.0
```

**Success Criteria:**
- Map shows clear corridor boundaries
- Localization error < 10cm
- Robot reaches goal without collisions
- Path follows corridor center

### Scenario 2: Obstacle Avoidance

**Objective**: Test dynamic obstacle detection and avoidance

**Synthetic Data Generation:**
```python
# Create bag with moving obstacles
./scripts/generate_obstacle_course.py --num-obstacles 3 --duration 60
```

**Test Execution:**
```bash
# Run navigation with obstacles
./scripts/test_navigation_with_bag.sh test_map.yaml obstacles.db3 10.0 5.0
```

**Success Criteria:**
- Detects all obstacles in costmap
- Replans path when blocked
- No collisions (measured via proximity checks)
- Reaches goal within timeout

### Scenario 3: Loop Closure Testing

**Objective**: Validate SLAM loop closure in revisited areas

**Requirements:**
- Rosbag with robot returning to start position
- Recognizable landmarks

**Test:**
```bash
# Run SLAM with loop closure enabled
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true do_loop_closing:=true

ros2 bag play loop_test.db3 --clock
```

**Success Criteria:**
- Loop closure detected (check logs)
- Map consistency after loop closure
- Position error < 20cm after closure

### Scenario 4: Multi-Floor Testing

**Objective**: Test SLAM/localization with elevation changes

**Setup:**
```bash
# Record data with IMU for floor transitions
ros2 bag record /scan /odom /imu /tf /tf_static
```

**Validation:**
- 2D SLAM ignores elevation (expected)
- Odometry drift analysis
- Document limitations for future 3D SLAM

### Scenario 5: Kidnapped Robot Problem

**Objective**: Test AMCL recovery when robot is "teleported"

**Test Setup:**
```python
# Create bag with sudden pose jump
# This tests global localization recovery
```

**Success Criteria:**
- AMCL converges to correct pose
- Convergence time < 10 seconds
- Uncertainty (covariance) eventually decreases

---

## Troubleshooting

### Common Issues and Solutions

#### Issue: "Lookup would require extrapolation into the future"

**Cause**: Node is using wall clock while bag uses sim time

**Solution:**
```bash
# Ensure ALL nodes have use_sim_time:=true
ros2 param set /my_node use_sim_time true

# Verify
ros2 param get /my_node use_sim_time
```

#### Issue: SLAM doesn't build map during playback

**Cause**: Messages playing too fast or clock not published

**Solution:**
```bash
# Play slower
ros2 bag play my_bag --clock --rate 0.5

# Check clock is being published
ros2 topic hz /clock
```

#### Issue: TF tree incomplete

**Cause**: Missing tf_static in recording

**Solution:**
```bash
# Always record both
ros2 bag record /tf /tf_static /scan /odom

# Verify TF tree
ros2 run rqt_tf_tree rqt_tf_tree
```

#### Issue: "No map received"

**Cause**: Map server not activated (lifecycle node)

**Solution:**
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

# Verify
ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState
```

#### Issue: High CPU usage during playback

**Cause**: Too many topics or high-frequency data

**Solution:**
```bash
# Play specific topics only
ros2 bag play my_bag --topics /scan /odom /tf

# Reduce playback rate
ros2 bag play my_bag --rate 0.5
```

#### Issue: Bag file corrupted

**Cause**: Recording interrupted without proper shutdown

**Solution:**
```bash
# Check bag integrity
ros2 bag info my_bag

# Try to recover (if SQLite)
sqlite3 my_bag/my_bag.db3 "PRAGMA integrity_check;"
```

#### Issue: Time synchronization errors

**Cause**: Mixed message timestamps

**Solution:**
```bash
# Verify timestamps are consistent
ros2 topic echo /scan --field header.stamp

# Check clock source
ros2 topic echo /clock
```

### Debugging Tools

#### Visualize Bag Contents
```bash
# Install Foxglove Studio
# Open .mcap or .db3 files for visualization
# https://foxglove.dev/
```

#### Analyze TF Data
```bash
# View TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_link
```

#### Monitor Topic Rates
```bash
# Check message frequency
ros2 topic hz /scan

# Check message count in bag
ros2 bag info my_bag | grep "Message Count"
```

#### Profile Performance
```bash
# Install ros2-tracing
sudo apt install ros-humble-ros2trace

# Trace execution
ros2 trace

# Analyze with Trace Compass or other tools
```

---

## Summary

This guide covered comprehensive testing strategies for ROS2 Nav2 systems without requiring physical hardware:

**Key Takeaways:**

1. **Three Testing Approaches**: Simulation, rosbag replay, synthetic data
2. **Simulation Time is Critical**: Always use `--clock` and `use_sim_time:=true`
3. **Record Essential Topics**: /scan, /odom, /tf, /tf_static minimum
4. **Automate Testing**: Scripts for reproducible tests
5. **Validate Results**: Metrics and quality checks
6. **Document Everything**: Test reports and configuration

**Next Steps:**

1. Set up your testing environment
2. Record or generate test datasets
3. Run example test scenarios
4. Integrate into your development workflow
5. Add automated CI/CD testing

**Resources:**
- Nav2 Documentation: https://navigation.ros.org/
- rosbag2 GitHub: https://github.com/ros2/rosbag2
- TurtleBot3 Simulation: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

---

**Document Version:** 1.0
**Last Updated:** 2026-01-11
**Tested With:** ROS 2 Humble, Nav2 1.1.x
