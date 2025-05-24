# Indoor Mapping and Navigation System - Complete Setup Guide

## System Overview

This guide details the complete setup of a skid-steer robot with RPLIDAR A1M8 for autonomous indoor mapping and navigation using ROS Noetic on Raspberry Pi 4.

**System Components:**
- Robot Base: Custom skid-steer chassis with L298N motor drivers
- LIDAR: RPLIDAR A1M8 (360° 2D laser scanner)
- Compute: Raspberry Pi 4 (ARM64) with Ubuntu 20.04 LTS
- Desktop: Ubuntu PC for visualization and control
- Optional: Wheel encoders for improved odometry

---

## Initial Hardware Setup

### 1. Raspberry Pi 4 Setup
```bash
# Install Ubuntu 20.04 LTS ARM64 on Raspberry Pi 4
# Flash image to SD card using Raspberry Pi Imager

# Initial boot configuration
sudo apt update && sudo apt upgrade -y
sudo apt install openssh-server -y
sudo systemctl enable ssh

# Set hostname (replace 'robot' with your preferred name)
sudo hostnamectl set-hostname robot
```

### 2. Network Configuration with mDNS
```bash
# Install Avahi for automatic hostname discovery
sudo apt install avahi-daemon avahi-utils -y
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon

# Test mDNS resolution
avahi-browse -at
```

### 3. Hardware Connections
- Connect RPLIDAR A1M8 to USB port (typically `/dev/ttyUSB0`)
- Wire L298N motor drivers to GPIO pins on Raspberry Pi
- Connect wheel encoders if available (optional but recommended)
- Ensure adequate power supply for Pi and motors

---

## Software Installation

### 1. ROS Noetic Installation on Raspberry Pi
```bash
# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Noetic
sudo apt update
sudo apt install ros-noetic-desktop -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Add to bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Required ROS Packages
```bash
# SLAM packages
sudo apt install ros-noetic-gmapping -y
sudo apt install ros-noetic-cartographer-ros -y
sudo apt install ros-noetic-hector-slam -y

# Navigation packages
sudo apt install ros-noetic-navigation -y
sudo apt install ros-noetic-amcl -y
sudo apt install ros-noetic-move-base -y

# Utility packages
sudo apt install ros-noetic-teleop-twist-keyboard -y
sudo apt install ros-noetic-map-server -y
sudo apt install ros-noetic-tf2-tools -y
```

### 3. Install RPLIDAR ROS Driver
```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone RPLIDAR ROS driver
git clone https://github.com/Slamtec/rplidar_ros.git

# Build the workspace
cd ~/catkin_ws
catkin_make

# Add to bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. ROS Environment Configuration
Add to `~/.bashrc` on Raspberry Pi:
```bash
export ROS_MASTER_URI=http://robot.local:11311
export ROS_HOSTNAME=robot.local
export ROS_IP=$(hostname -I | awk '{print $1}')
```

### 5. Desktop PC Setup
Install ROS Noetic on Ubuntu desktop and add to `~/.bashrc`:
```bash
export ROS_MASTER_URI=http://robot.local:11311
export ROS_HOSTNAME=laptop.local
```

---

## Create Custom Motor Control Node

### 1. Create Motor Control Package
```bash
cd ~/catkin_ws/src
catkin_create_pkg skid_drive_control rospy geometry_msgs std_msgs

# Create motor control node
mkdir -p skid_drive_control/src
```

### 2. Motor Control Node (Python)
Create `~/catkin_ws/src/skid_drive_control/src/skid_drive_node.py`:
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class SkidDriveController:
    def __init__(self):
        rospy.init_node('skid_drive_controller')
        
        # Motor pins (adjust based on your wiring)
        self.left_motor_pins = {'in1': 18, 'in2': 19, 'ena': 12}
        self.right_motor_pins = {'in1': 20, 'in2': 21, 'ena': 13}
        
        self.setup_gpio()
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        rospy.loginfo("Skid drive controller initialized")
    
    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        for pin in self.left_motor_pins.values():
            GPIO.setup(pin, GPIO.OUT)
        for pin in self.right_motor_pins.values():
            GPIO.setup(pin, GPIO.OUT)
        
        # Create PWM instances
        self.left_pwm = GPIO.PWM(self.left_motor_pins['ena'], 1000)
        self.right_pwm = GPIO.PWM(self.right_motor_pins['ena'], 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
    
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Convert to differential drive
        left_speed = linear - angular
        right_speed = linear + angular
        
        # Normalize and convert to PWM (0-100)
        max_speed = 1.0
        left_pwm = max(min(abs(left_speed) / max_speed * 100, 100), 0)
        right_pwm = max(min(abs(right_speed) / max_speed * 100, 100), 0)
        
        # Set motor directions and speeds
        self.set_motor(self.left_motor_pins, left_speed >= 0, left_pwm)
        self.set_motor(self.right_motor_pins, right_speed >= 0, right_pwm)
    
    def set_motor(self, pins, forward, pwm_value):
        if forward:
            GPIO.output(pins['in1'], GPIO.HIGH)
            GPIO.output(pins['in2'], GPIO.LOW)
        else:
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.HIGH)
        
        if pins == self.left_motor_pins:
            self.left_pwm.ChangeDutyCycle(pwm_value)
        else:
            self.right_pwm.ChangeDutyCycle(pwm_value)
    
    def cleanup(self):
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        controller = SkidDriveController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cleanup()
```

Make executable:
```bash
chmod +x ~/catkin_ws/src/skid_drive_control/src/skid_drive_node.py
```

---

## Phase 1: Autonomous Mapping (SLAM)

### 1. Create SLAM Launch File
Create `~/catkin_ws/src/skid_drive_control/launch/slam_gmapping.launch`:
```xml
<launch>
  <!-- RPLIDAR Node -->
  <node pkg="rplidar_ros" type="rplidarNode" name="rplidar" output="screen">
    <param name="serial_port" value="/dev/ttyUSB0"/>
    <param name="frame_id" value="laser"/>
    <param name="serial_baudrate" value="115200"/>
  </node>

  <!-- Static transform from base_link to laser -->
  <node pkg="tf2_ros" type="static_transform_publisher" 
        name="laser_to_base" 
        args="0 0 0.1 0 0 0 base_link laser"/>

  <!-- Motor Control Node -->
  <node pkg="skid_drive_control" type="skid_drive_node.py" name="skid_drive_controller"/>

  <!-- GMapping SLAM -->
  <node pkg="gmapping" type="slam_gmapping" name="gmapping" output="screen">
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="odom"/>
    <param name="map_frame" value="map"/>
    <param name="scan_topic" value="/scan"/>
    
    <!-- SLAM parameters -->
    <param name="maxUrange" value="6.0"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    <param name="minimumScore" value="50"/>
    <param name="srr" value="0.1"/>
    <param name="srt" value="0.2"/>
    <param name="str" value="0.1"/>
    <param name="stt" value="0.2"/>
  </node>

  <!-- Teleop for manual driving -->
  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop"/>
</launch>
```

### 2. Start SLAM Mapping
On Raspberry Pi:
```bash
# Start roscore
roscore &

# Launch SLAM
roslaunch skid_drive_control slam_gmapping.launch
```

On Desktop PC:
```bash
# Launch RViz for visualization
rviz

# In RViz:
# 1. Set Fixed Frame to "map"
# 2. Add LaserScan display (topic: /scan)
# 3. Add Map display (topic: /map)
# 4. Add RobotModel display
# 5. Save configuration as mapping.rviz
```

### 3. Explore and Map Environment
```bash
# Drive robot manually using keyboard
# Use W/A/S/D keys to move around and build the map
# Monitor progress in RViz on desktop

# Alternative: Use joystick control
rosrun joy joy_node
rosrun teleop_twist_joy teleop_node
```

### 4. Save the Map
Once mapping is complete:
```bash
# Save the map (run from desktop or robot)
rosrun map_server map_saver -f ~/maps/mymap

# This creates:
# - mymap.pgm (occupancy grid image)
# - mymap.yaml (map metadata)
```

---

## Phase 2: Map Editing and Waypoint Setup

### 1. Load Saved Map
Create `~/catkin_ws/src/skid_drive_control/launch/map_edit.launch`:
```xml
<launch>
  <!-- Load the saved map -->
  <node pkg="map_server" type="map_server" name="map_server" 
        args="$(find skid_drive_control)/maps/mymap.yaml"/>

  <!-- RPLIDAR for current position -->
  <node pkg="rplidar_ros" type="rplidarNode" name="rplidar" output="screen">
    <param name="serial_port" value="/dev/ttyUSB0"/>
    <param name="frame_id" value="laser"/>
  </node>

  <!-- Static transforms -->
  <node pkg="tf2_ros" type="static_transform_publisher" 
        name="laser_to_base" 
        args="0 0 0.1 0 0 0 base_link laser"/>

  <!-- Motor control -->
  <node pkg="skid_drive_control" type="skid_drive_node.py" name="skid_drive_controller"/>

  <!-- AMCL for localization -->
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <param name="use_map_topic" value="true"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="global_frame_id" value="map"/>
  </node>

  <!-- Teleop -->
  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop"/>
</launch>
```

### 2. Start Map Editing Session
```bash
# On robot
roslaunch skid_drive_control map_edit.launch

# On desktop
rviz -d mapping.rviz
```

### 3. Record Waypoints
Create waypoint recording script `~/catkin_ws/src/skid_drive_control/src/record_waypoints.py`:
```python
#!/usr/bin/env python3
import rospy
import yaml
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_geometry_msgs import do_transform_pose
import tf_conversions

class WaypointRecorder:
    def __init__(self):
        rospy.init_node('waypoint_recorder')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.waypoints = {}
        
    def record_current_pose(self, name):
        try:
            # Get current transform from map to base_link
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            orientation = transform.transform.rotation
            euler = tf_conversions.transformations.euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            yaw = euler[2]
            
            self.waypoints[name] = [x, y, yaw]
            print(f"Recorded waypoint '{name}': [{x:.2f}, {y:.2f}, {yaw:.2f}]")
            
        except Exception as e:
            print(f"Error recording waypoint: {e}")
    
    def save_waypoints(self, filename):
        data = {'waypoints': self.waypoints}
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        print(f"Saved waypoints to {filename}")
    
    def interactive_recording(self):
        print("Waypoint Recorder Started")
        print("Commands:")
        print("  record <name> - Record current position as waypoint")
        print("  save <filename> - Save waypoints to file")
        print("  list - List recorded waypoints")
        print("  quit - Exit")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\n> ").strip().split()
                if not cmd:
                    continue
                    
                if cmd[0] == 'record' and len(cmd) > 1:
                    self.record_current_pose(cmd[1])
                elif cmd[0] == 'save' and len(cmd) > 1:
                    self.save_waypoints(cmd[1])
                elif cmd[0] == 'list':
                    for name, pose in self.waypoints.items():
                        print(f"  {name}: {pose}")
                elif cmd[0] == 'quit':
                    break
                else:
                    print("Invalid command")
                    
            except KeyboardInterrupt:
                break

if __name__ == '__main__':
    recorder = WaypointRecorder()
    recorder.interactive_recording()
```

### 4. Record Waypoints Process
```bash
# Drive robot to each desired location using teleop
# At each location, run:
python3 ~/catkin_ws/src/skid_drive_control/src/record_waypoints.py

# Example session:
# > record kitchen
# > record office
# > record charging_dock
# > save ~/catkin_ws/src/skid_drive_control/config/waypoints.yaml
# > quit
```

---

## Phase 3: Autonomous Navigation

### 1. Create Navigation Launch File
Create `~/catkin_ws/src/skid_drive_control/launch/navigation.launch`:
```xml
<launch>
  <!-- Load map -->
  <node pkg="map_server" type="map_server" name="map_server" 
        args="$(find skid_drive_control)/maps/mymap.yaml"/>

  <!-- Load waypoints as parameters -->
  <rosparam file="$(find skid_drive_control)/config/waypoints.yaml" command="load"/>

  <!-- RPLIDAR -->
  <node pkg="rplidar_ros" type="rplidarNode" name="rplidar" output="screen">
    <param name="serial_port" value="/dev/ttyUSB0"/>
    <param name="frame_id" value="laser"/>
  </node>

  <!-- Static transforms -->
  <node pkg="tf2_ros" type="static_transform_publisher" 
        name="laser_to_base" 
        args="0 0 0.1 0 0 0 base_link laser"/>

  <!-- Motor control -->
  <node pkg="skid_drive_control" type="skid_drive_node.py" name="skid_drive_controller"/>

  <!-- AMCL Localization -->
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <param name="use_map_topic" value="true"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="global_frame_id" value="map"/>
    
    <!-- Particle filter parameters -->
    <param name="min_particles" value="500"/>
    <param name="max_particles" value="2000"/>
    <param name="initial_pose_x" value="0.0"/>
    <param name="initial_pose_y" value="0.0"/>
    <param name="initial_pose_a" value="0.0"/>
  </node>

  <!-- Move Base Navigation -->
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS"/>
    
    <!-- Load configuration files -->
    <rosparam file="$(find skid_drive_control)/config/costmap_common_params.yaml" command="load" ns="global_costmap"/>
    <rosparam file="$(find skid_drive_control)/config/costmap_common_params.yaml" command="load" ns="local_costmap"/>
    <rosparam file="$(find skid_drive_control)/config/local_costmap_params.yaml" command="load"/>
    <rosparam file="$(find skid_drive_control)/config/global_costmap_params.yaml" command="load"/>
    <rosparam file="$(find skid_drive_control)/config/base_local_planner_params.yaml" command="load"/>
  </node>
</launch>
```

### 2. Create Configuration Files

**costmap_common_params.yaml:**
```yaml
robot_radius: 0.20
inflation_radius: 0.35

observation_sources: laser_scan_sensor

laser_scan_sensor:
  sensor_frame: laser
  data_type: LaserScan
  topic: scan
  marking: true
  clearing: true
```

**local_costmap_params.yaml:**
```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 4.0
  height: 4.0
  resolution: 0.05
```

**global_costmap_params.yaml:**
```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 0.5
  static_map: true
```

**base_local_planner_params.yaml:**
```yaml
DWAPlannerROS:
  max_vel_x: 0.5
  min_vel_x: 0.1
  max_vel_y: 0.0
  min_vel_y: 0.0
  max_rot_vel: 1.0
  min_rot_vel: 0.4
  acc_lim_x: 2.5
  acc_lim_y: 0.0
  acc_lim_theta: 3.2
```

### 3. Create Flask Web API
Create `~/catkin_ws/src/skid_drive_control/src/flask_navigation_server.py`:
```python
#!/usr/bin/env python3
import os
import threading
import rospy
import yaml
import tf_conversions
from geometry_msgs.msg import PoseStamped
from flask import Flask, request, jsonify
from actionlib_msgs.msg import GoalStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavigationServer:
    def __init__(self):
        # Initialize ROS in separate thread
        threading.Thread(target=self.init_ros, daemon=True).start()
        
        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Wait for ROS to initialize
        while not hasattr(self, 'move_base_client'):
            rospy.sleep(0.1)
    
    def init_ros(self):
        rospy.init_node('flask_navigation_server', disable_signals=True)
        
        # Load waypoints
        self.waypoints = rospy.get_param('/waypoints', {})
        rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints")
        
        # Move base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # Simple goal publisher (alternative to action client)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        rospy.loginfo("Navigation server ROS components initialized")
    
    def setup_routes(self):
        @self.app.route('/goto', methods=['POST'])
        def goto_waypoint():
            data = request.get_json()
            if not data or 'name' not in data:
                return jsonify({'status': 'error', 'message': 'Missing waypoint name'}), 400
            
            waypoint_name = data['name']
            if waypoint_name not in self.waypoints:
                return jsonify({
                    'status': 'error', 
                    'message': f'Unknown waypoint: {waypoint_name}',
                    'available_waypoints': list(self.waypoints.keys())
                }), 400
            
            try:
                x, y, yaw = self.waypoints[waypoint_name]
                
                # Create goal
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = 0.0
                
                # Convert yaw to quaternion
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
                goal.pose.orientation.x = q[0]
                goal.pose.orientation.y = q[1]
                goal.pose.orientation.z = q[2]
                goal.pose.orientation.w = q[3]
                
                # Send goal
                self.goal_pub.publish(goal)
                
                return jsonify({
                    'status': 'success',
                    'message': f'Navigating to {waypoint_name}',
                    'target_pose': [x, y, yaw]
                }), 200
                
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': f'Navigation failed: {str(e)}'
                }), 500
        
        @self.app.route('/waypoints', methods=['GET'])
        def list_waypoints():
            return jsonify({
                'status': 'success',
                'waypoints': self.waypoints
            }), 200
        
        @self.app.route('/stop', methods=['POST'])
        def stop_navigation():
            try:
                self.move_base_client.cancel_all_goals()
                return jsonify({
                    'status': 'success',
                    'message': 'Navigation cancelled'
                }), 200
            except Exception as e:
                return jsonify({
                    'status': 'error',
                    'message': f'Failed to stop: {str(e)}'
                }), 500
    
    def run(self):
        host = os.environ.get('ROS_IP', '0.0.0.0')
        self.app.run(host=host, port=5000, debug=False)

if __name__ == '__main__':
    server = NavigationServer()
    server.run()
```

### 4. Create Startup Service
Create `/etc/systemd/system/robot-navigation.service`:
```ini
[Unit]
Description=Robot Navigation Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/catkin_ws
Environment=ROS_MASTER_URI=http://robot.local:11311
Environment=ROS_HOSTNAME=robot.local
ExecStartPre=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/pi/catkin_ws/devel/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/pi/catkin_ws/devel/setup.bash && roslaunch skid_drive_control navigation.launch'
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl enable robot-navigation.service
sudo systemctl start robot-navigation.service
```

---

## Testing and Usage

### 1. Test Navigation System
```bash
# Start navigation (if not using systemd service)
roslaunch skid_drive_control navigation.launch

# In another terminal, start Flask server
python3 ~/catkin_ws/src/skid_drive_control/src/flask_navigation_server.py

# Test HTTP API
curl -X POST -H "Content-Type: application/json" \
     -d '{"name": "kitchen"}' \
     http://robot.local:5000/goto

# List available waypoints
curl http://robot.local:5000/waypoints

# Stop navigation
curl -X POST http://robot.local:5000/stop
```

### 2. Monitor with RViz
```bash
# On desktop PC
rviz

# Add displays:
# - Map (/map)
# - LaserScan (/scan)
# - Path (/move_base/TrajectoryPlannerROS/global_plan)
# - Goal (2D Nav Goal tool)
# - Pose (/amcl_pose)
```

### 3. Troubleshooting Commands
```bash
# Check ROS topics
rostopic list

# Monitor transforms
rosrun tf2_tools view_frames.py
evince frames.pdf

# Check navigation status
rostopic echo /move_base/status

# Monitor robot pose
rostopic echo /amcl_pose

# Test LIDAR
rostopic echo /scan

# Check motor commands
rostopic echo /cmd_vel
```

---

## Additional Enhancements

### 1. Add Obstacle Avoidance Parameters
Tune costmap parameters for better obstacle avoidance by adjusting inflation radius and observation sources.

### 2. Implement Battery Monitoring
Add battery level monitoring and automatic return to charging dock when low.

### 3. Add Camera Integration
Integrate a camera for visual odometry or object recognition to enhance navigation.

### 4. Implement Path Recording
Record and replay paths for repetitive tasks.

### 5. Add Web Dashboard
Create a web interface for easier control and monitoring of the robot.

This complete setup guide provides a fully functional indoor mapping and navigation system using ROS Noetic on Raspberry Pi 4 with RPLIDAR A1M8.