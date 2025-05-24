You: 

I want a comprehensive system to create a map of the inside of a building using RPlidar A1m8 and a raspberry pi 4. then I want to be able to add waypoints to that map in Rviz and save it for later. i then want to be able to use that map in a program where I can send commands via post request to a flask app on the raspberry pi to then have it navigate and pathfind to those waypoints. I want to have 3 stages, mapping, map editing, and navigating. mapping I want to have a robot that simply makes the map of the building, searching and moving and discovering all possible routes to move in the building and rooms and such so that I can let it go autonomously and it will make a comprehensive map streamed back to a desktop over wifi. i want to account for moving across multiple access points in commercial wifi, the wifi credentials will still work its just that the IP address might change or I don't know. further more when I am done running the mapping stage. i want to save the map in Rviz, then make an edited version of the map where I can move the robot around using that map then add waypoints corresponding to offices and rooms and save this map with custom waypoints for later. finally I want to have a phase where I turn on the robot, it figures out where its at in the map with the waypoints, then I can tell it to go to a waypoint and it will navigate to it. Ideallly I would use google cartographer as it seems easy to setup and make comprehensive maps. I want a comprehensive overview of this mapping, waypoint adding, navigation system that covers the ubuntu ROS on the rpi, the ROS on ubuntu/windows on a laptop/desktop, and the flask app hosted on the rpi to then receive and interpret commands. https://medium.com/robotics-weekends/2d-mapping-using-google-cartographer-and-rplidar-with-raspberry-pi-a94ce11e44c5 https://www.google.com/search?q=add+waypoints+to+map+in+rviz https://www.google.com/search?q=how+to+navigate+to+waypoints+on+an+rviz+map+then+send+to+robot https://github.com/Andrew-rw/gbot\_core

ChatGPT: 

Thanks for the detailed overview—this is a great project. To tailor the system overview and instructions accurately, could you clarify the following points:

1.  **Robot Base and Mobility**: What type of robot or base are you using for navigation (e.g., Turtlebot, custom differential drive, etc.)?
    
2.  **SLAM and Localization**: Are you committed to using Google Cartographer for both mapping and localization, or are you open to other packages like GMapping or AMCL for localization?
    
3.  **Navigation Stack**: Would you prefer to use the standard ROS Navigation Stack (move\_base, AMCL, etc.) for pathfinding, or are you looking for a lightweight custom solution?
    
4.  **WiFi IP Handling**: Would you prefer a dynamic discovery method (e.g., mDNS/Avahi or broadcasting over LAN) for dealing with IP address changes, or are you planning to use static IPs?
    
5.  **Flask App Functions**: Besides receiving navigation commands via POST, should the Flask app handle any other functions (e.g., status reporting, streaming location, receiving map updates)?
    

Once I have this information, I’ll put together a comprehensive system plan covering ROS setup, mapping, waypoint editing, navigation, and Flask integration across the Raspberry Pi and desktop.

You: 

Robot Base and Mobility: What type of robot or base are you using for navigation (e.g., Turtlebot, custom differential drive, etc.)? I am going to use acustomskid steer type of driving system where I take an L298motor driverand place one on the right side and one on the left side each motor driver will control two wheels so that I can get these skid steer type of mobility.If needed I can add wheel encoders to assist withthe mapping and navigation process. SLAM and Localization: Are you committed to using Google Cartographer for both mapping and localization, or are you open to other packages like GMapping or AMCL for localization? I am open to whatever would be the easiest possible setup to perform this mapping and localization process and then navigation in the map. I just want a system to put on my robotand use the rp Lidar a oneon top of my differential drive system to make a map inside of a buildingand then add waypoints to that map that correspond to office locationsand then have a navigation phase where I use that map with those setway points for the robot to navigate to offices. Navigation Stack: Would you prefer to use the standard ROS Navigation Stack (move\_base, AMCL, etc.) for pathfinding, or are you looking for a lightweight custom solution? WiFi IP Handling: Would you prefer a dynamic discovery method (e.g., mDNS/Avahi or broadcasting over LAN) for dealing with IP address changes, or are you planning to use static IPs? I may need to deal with IP address changes so I would like to compensate for these changes and be able to handle them in the system Flask App Functions: Besides receiving navigation commands via POST, should the Flask app handle any other functions (e.g., status reporting, streaming location, receiving map updates)? I will add functionality to the flask gap as needed and as I want to. Very much but all I need is for it to listen for Post request that it then handles as commands for movementand interacts with these skid steerL298 in motor drivers as needed.

ChatGPT: 

Great, I’ll compile a comprehensive system overview and implementation guide for a 3-phase building mapping and navigation system using a Raspberry Pi 4, RPLIDAR A1M8, and a custom skid steer robot with L298N motor drivers. The plan will include:

-   SLAM and localization (favoring simplicity and reliability)
    
-   ROS setup on both the Raspberry Pi (Ubuntu) and a desktop workstation
    
-   Map creation and live visualization over WiFi
    
-   Waypoint editing via RViz
    
-   Navigation using the ROS Navigation Stack
    
-   A Flask server for receiving waypoint navigation commands
    
-   Strategies for robust communication over commercial WiFi with dynamic IPs
    

I’ll update you once the system design and workflow are ready for review.

Research completed in 3m · 29 sources · 51 searches

Indoor Mapping and Navigation System With ROS and Flask on Raspberry Pi

# Indoor Mapping and Navigation System Overview

A skid-steer robot equipped with a 360° RPLIDAR A1M8 and a Raspberry Pi 4 can autonomously map and navigate an indoor area in three phases. In **Phase 1 (Mapping)**, the robot uses ROS-based 2D SLAM (Simultaneous Localization and Mapping) to explore and build an occupancy grid of the environment. In **Phase 2 (Map Editing)**, the completed map is loaded in RViz, the robot is manually moved (via teleop or simulation) to survey key points, and named waypoints (rooms, landmarks) are marked and saved. In **Phase 3 (Navigation)**, the robot boots with the saved map and uses ROS localization (e.g. AMCL) to determine its pose. A lightweight Flask web server on the Pi exposes HTTP endpoints (via POST) so that external clients can send waypoint names; the server looks up coordinates and issues ROS navigation goals (move\_base) to drive the robot autonomously to those points. Networking is configured so that the laptop and robot automatically discover each other (e.g. via Avahi/mDNS and ROS\_HOSTNAME), avoiding manual IP management.

This guide details each stage with hardware/software setup, package choices, and example configurations. It compares SLAM and localization packages, outlines ROS launch files and RViz usage, and shows how to integrate a Flask HTTP API with ROS topics and actions. Citations from ROS and robotics references are provided for key components and design choices.

![https://www.intermodalics.ai/blog/how-to-map-large-indoor-spaces-with-google-cartographer](blob:https://chatgpt.com/1231323a-9ea6-41d3-a2f5-e0de2423a6ef)

*Example 2D map (warehouse floor) generated by Google’s Cartographer SLAM package using LIDAR scans. High-resolution maps like this enable reliable navigation [intermodalics.ai](https://www.intermodalics.ai/blog/how-to-map-large-indoor-spaces-with-google-cartographer#:~:text=Cartographer%20is%20a%20real,of%20indoor%20and%20outdoor%20environments).*

## System Components

-   **Robot Base:** A custom skid-steer chassis with two L298N motor drivers. Each driver controls a pair of wheels (4 wheels total). The skid-steer design is a form of differential drive (no castor wheels) where turning in place causes wheel slip. Odometry can be estimated from wheel encoders if available, or via dead-reckoning; encoders are recommended for better accuracy but are optional.
    
-   **LIDAR Sensor:** RPLIDAR A1M8 (360° 2D laser scanner). This low-cost LIDAR provides distance measurements (up to ~8m for A1) in all directions, suitable for indoor SLAM and obstacle avoidance[hackster.io](https://www.hackster.io/shahizat/lidar-integration-with-ros-noetic-on-raspberry-pi-os-8ea140#:~:text=RPLIDAR%20is%20a%20low,the%20other%20applications%20such%20as)wiki.ros.org. The RPLIDAR node publishes `sensor_msgs/LaserScan` on the `/scan` topic in ROS.
    
-   **Compute:** Raspberry Pi 4 (ARM64) running Ubuntu 20.04 LTS and ROS Noetic (ROS1). ROS Noetic is the current ROS1 LTS, with binary packages available for Ubuntu Focal on ARM64wiki.ros.org. The Pi will host ROS nodes for the LIDAR, motor control, SLAM, and navigation. A desktop PC/laptop (Ubuntu or ROS-compatible OS) will be used for visualization (RViz), map saving/editing, and as a ROS node for teleoperation and monitoring.
    
-   **Motor Control:** Two L298N H-bridge modules (each drives 2 wheels). The Pi’s GPIO/PWM pins (e.g. via pigpio or RPi.GPIO) can directly control the L298N inputs to drive the motors. Alternatively, an Arduino (or other microcontroller) could be used as a ROS node to interface between the Pi and L298N. The robot subscribes to `geometry_msgs/Twist` (e.g. from teleop or move\_base) and translates linear/angular velocity into PWM signals for differential drive (skid-steer) motion.
    
-   **Wheel Encoders (Optional):** Quadrature encoders on wheels (2 or 4 wheels) can feed wheel odometry into ROS using `nav_msgs/Odometry`. This improves SLAM and localization accuracy. Without encoders, SLAM must rely solely on LIDAR scan matching (as in Hector SLAM or Cartographer without odom). With encoders, GMapping or Cartographer can fuse odometry for more stable mapping[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=HECTOR_SLAM)[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=GMAPPING).
    
-   **Networking:** The robot and desktop connect via WiFi. To avoid manual IP configuration, use mDNS/Avahi so that each host can be reached at a consistent hostname (e.g. `robot.local`). In ROS, set `ROS_HOSTNAME=robot.local` and `ROS_MASTER_URI=http://robot.local:11311` on each device, so nodes discover each other by name[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/102417/how-to-configure-ros-master-uri-ros-hostname#:~:text=In%20case%20your%20network%20does,day%21%20On%20the%20publishing%20computer). Install and enable `avahi-daemon` on the Pi and desktop (`sudo apt install avahi-daemon` and `sudo systemctl enable avahi-daemon`) so that hostnames are broadcast automatically (e.g. `robot.local`, `laptop.local`). Ensure the firewall allows mDNS (UDP port 5353). This way, whether the robot’s IP changes via DHCP or WiFi reconnects, the hostname remains resolvable[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/102417/how-to-configure-ros-master-uri-ros-hostname#:~:text=In%20case%20your%20network%20does,day%21%20On%20the%20publishing%20computer).
    

## Software Setup

1.  **Install ROS Noetic and Dependencies:** On the Pi (Ubuntu 20.04), follow the standard ROS Noetic install guide. In addition, install required ROS packages:
    
    -   SLAM: `ros-noetic-gmapping` (GMapping), or `ros-noetic-cartographer` (Cartographer), or others (Hector, slam\_toolbox).
        
    -   Navigation: `ros-noetic-navigation` (move\_base, AMCL, costmap\_2d, etc.) or Nav2 on ROS2 if preferred (but we assume ROS1).
        
    -   LIDAR driver: build or install `rplidar_ros` (Slamtec’s ROS node). E.g. clone and `catkin_make` or install from packages if available. The node topic is typically `/scan`.
        
    -   Motor control: a custom ROS node/package. For example, write a `skid_drive_node` that subscribes to `/cmd_vel` and publishes PWM signals to GPIO or communicates with an Arduino. The ROS-Industrial `ros_control` with `diff_drive_controller` could be used if integrating with `ros_control`. Ensure TF frames: `base_link`, `odom`, and static transform `base_link`→`laser` for the LIDAR pose (height and offset).
        
    -   Teleop (optional): `ros-noetic-teleop-twist-keyboard` for keyboard driving, or `joy` packages for gamepad.
        
2.  **LIDAR Node:** Launch the RPLIDAR driver. A typical launch file (`rplidar.launch`) includes parameters for the serial port (`/dev/ttyUSB0`) and frame (e.g. `frame_id: base_laser`). It publishes `sensor_msgs/LaserScan` on `/scan`. Test that RViz shows the 2D scan by adding a LaserScan display. The RPLIDAR node is maintained (MIT-licensed) by Slamtecwiki.ros.org.
    
3.  **SLAM Node:** Choose a 2D SLAM algorithm. Common options:
    
    | Package | Needs Odometry | LIDAR-only support | Pros | Cons |
    | --- | --- | --- | --- | --- |
    | **GMapping** | Yes[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=GMAPPING) | No | Easy to use, good for small indoor maps[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=GMAPPING). Wide community use. | Requires reliable odometry (encoders), can drift in large open spaces. |
    | **Hector SLAM** | No | Yes[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=HECTOR_SLAM) | Works without odom (scan-matching only). Good for small areas with low vibration. | Sensitive to fast motion/vibration; struggles on rough terrain or poor LIDAR signals[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=HECTOR_SLAM). |
    | **Cartographer** | Optional (better) | Yes | High-resolution maps; can incorporate odometry, IMU. Real-time loop closure and graph optimization[intermodalics.ai](https://www.intermodalics.ai/blog/how-to-map-large-indoor-spaces-with-google-cartographer#:~:text=Cartographer%20is%20a%20real,of%20indoor%20and%20outdoor%20environments). Handles large areas. | More complex setup and tuning; heavier CPU usage. Official support as `cartographer_ros`. |
    | **Karto** | Yes | Yes | Produces GMapping-like results, more robust to temporary odom failure[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=KARTO). Visual feedback (trajectory). | Less commonly used; still relies on good odom. |
    | **slam\_toolbox** | Optional (yes) | Yes | Supports online/offline mapping, lifetime mapping. Very flexible (persistent maps)[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=SLAM_TOOLBOX). | Newer; similar performance to GMapping in small areas. |
    
    *Table: Comparison of 2D ROS SLAM packages.* All above packages generate an occupancy grid (`nav_msgs/OccupancyGrid`) as `/map`.
    
    *Recommendation:* For a straightforward indoor map, **GMapping** is often simplest (especially if encoders are available). The user generated map in real-time while driving produces good results in home/office environments[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=GMAPPING). If you have no encoders, **Hector SLAM** or **Cartographer** with no-odom mode can still operate, though with potential drift. **Cartographer** yields very accurate, high-res maps (see image above) by fusing scans, wheel odometry, and optional IMU[intermodalics.ai](https://www.intermodalics.ai/blog/how-to-map-large-indoor-spaces-with-google-cartographer#:~:text=Cartographer%20is%20a%20real,of%20indoor%20and%20outdoor%20environments). Setup Cartographer with a suitable Lua config (set `tracking_frame= “base_link”`, `published_frame= “base_link”`, etc.).
    
4.  **Localization (Phase 3):** Use **AMCL (Adaptive Monte Carlo Localization)** for localizing on the known map. AMCL is the standard ROS 2D localization that uses a particle filter against a known mapwiki.ros.org. It requires the map and the `odom->base_link` transforms (odometry data) and the LIDAR scans as input. AMCL publishes an estimated pose on `/amcl_pose`. This allows `move_base` to know where the robot is.
    
5.  **Navigation Stack:** Install `ros-noetic-navigation` which provides the `move_base` node and related costmap/inflation/trajectory planners. Configure two costmaps (`global_costmap`, `local_costmap`) with inflation radius and obstacles (from `/scan`). Use a local planner like DWA. Define robot footprint parameters, max speeds. The `move_base` node takes goals (as `geometry_msgs/PoseStamped` or via action) in the map frame and outputs `cmd_vel` to drive the robot. During mapping (Phase 1), we may not need `move_base`, but in Phase 3 `move_base` is essential.
    
6.  **RViz Configuration:** Create a reusable RViz config (`.rviz` file) for mapping and editing. It should include:
    
    -   Fixed Frame: `map` (or `odom` during mapping if using SLAM).
        
    -   Displays: LaserScan (`/scan`), RobotModel (for visualization of `base_link`), and Map (`/map`) once published. Also TF tree display can help.
        
    -   For waypoint editing: add **2D Pose Estimate** and **2D Nav Goal** tools from the top toolbar. These publish to `/initialpose` and `/move_base_simple/goal`, respectively. They help set the initial location and goal arrows.
        
7.  **Networking and ROS Setup:** On the Pi (robot) and on the PC (desktop), add to `~/.bashrc` or launch scripts:
    
    ```bash
    export ROS_MASTER_URI=http://robot.local:11311
    export ROS_HOSTNAME=$(hostname).local
    ```
    
    Replace `robot.local` with the actual hostname of the Pi as registered by Avahi. After enabling Avahi, you can access the Pi as `raspberrypi.local` (default) or change `/etc/hostname`. On the desktop, set `ROS_MASTER_URI` to the same and `ROS_HOSTNAME=laptop.local`. This ensures ROS topics are advertised with `.local` names[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/102417/how-to-configure-ros-master-uri-ros-hostname#:~:text=In%20case%20your%20network%20does,day%21%20On%20the%20publishing%20computer). In Phase 1, you may run `roscore` on the Pi and have RViz on the desktop connect to it (set the above env vars).
    

## Phase 1: Autonomous Mapping (SLAM)

1.  **Launch SLAM:** With ROS master on the robot, start the SLAM node. For example, a launch file might include:
    
    ```xml
    <!-- slam_gmapping.launch -->
    <launch>
      <node pkg="rplidar_ros" type="rplidarNode" name="rplidar" output="screen">
        <param name="serial_port" value="/dev/ttyUSB0"/>
        <param name="frame_id" value="laser"/>
      </node>
      <node pkg="tf2_ros" type="static_transform_publisher" name="laser_to_base" args="0 0 0.1 0 0 0 base_link laser"/>
      <node pkg="gmapping" type="slam_gmapping" name="gmapping" output="screen">
        <!-- set parameters as needed -->
      </node>
    </launch>
    ```
    
    For Cartographer, use its `cartographer_ros` launch with your `.lua` config. Ensure wheel odometry (if any) is being published on `/odom` (e.g., via a `diff_drive_controller` or custom node) and that the odom frame is broadcast.
    
2.  **Robot Exploration:** The robot can be driven manually (e.g. with teleop) around the environment to capture LIDAR data. Optionally, implement an autonomous exploration behavior (ROS package `frontier_exploration` or a simple rule-based wanderer) to drive to unexplored frontiers. As the robot moves, the SLAM node incrementally builds a map. On the desktop, run RViz to visualize `/scan`, `/odom`, and `/map` in real time. Use RViz’s **2D Pose Estimate** tool to initialize the robot’s pose if needed (publishes to `/initialpose`) – especially important if using AMCL or if SLAM requires an initial guess.
    
3.  **Map Saving:** Once exploration is complete and the map looks correct in RViz, save the map to files. Use ROS’s `map_saver` (map\_server) utility:
    
    ```nginx
    rosrun map_server map_saver -f mymap
    ```
    
    This saves `mymap.pgm` (the occupancy image) and `mymap.yaml` (metadata). You can also do this from the desktop by calling the ROS service provided by `map_server`. The saved map (PGM+YAML) will be used in Phase 3 for navigation. Ensure to note the final map’s coordinate frame (origin) if offsetting is needed later.
    
4.  **Verify Map:** Load the saved map file in RViz or in `map_server` to ensure it aligns with reality. If the map has drifts or gaps, you may need to revisit mapping (adjust SLAM parameters, improve odometry). Otherwise, proceed.
    

## Phase 2: Map Editing and Waypoint Setup

1.  **Load Map in RViz:** Run:
    
    ```nginx
    rosrun map_server map_server mymap.yaml
    ```
    
    This starts a node that publishes the saved map on `/map`. Use RViz with Fixed Frame `map` to visualize the loaded map and robot position. If using AMCL, also start AMCL (with the map) to allow RViz to place the robot. Alternatively, you can manually publish a static transform or use `2D Pose Estimate` to set the robot’s initial pose on the map.
    
2.  **Move Robot to Waypoints:** With the robot localized on the map, navigate it to locations you want to mark (via teleop keyboard, joystick, or by entering 2D Nav Goals in RViz). After each move, ensure the robot stops precisely at the point to be labeled (e.g., center of “Room A”, office desk, etc.).
    
3.  **Record Waypoints:** At each desired location, record the robot’s current pose (x, y, yaw). You can do this by reading from the `/amcl_pose` topic (if localization is running) or from the `tf` transform between `map` and `base_link`. For example:
    
    ```bash
    rostopic echo /amcl_pose/pose/pose
    ```
    
    This prints position and orientation (quaternion). Convert quaternion to yaw angle. Alternatively, write a small script to query `tf` and save poses.
    
4.  **Save Waypoints to YAML:** Collect all named locations into a YAML file (e.g. `waypoints.yaml`). A simple format is:
    
    ```yaml
    waypoints:
      kitchen: [1.23, 4.56, 0.79]
      office: [5.67, 2.34, -1.57]
      dock: [0.0, 0.0, 0.0]
    ```
    
    Each entry is `[x, y, yaw]` in the map frame. Load this file as a ROS parameter or custom file in your navigation node. As suggested on ROS Answers, waypoints can be stored as parameters and read at runtime[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/97108/how-to-save-wavepoints#:~:text=1,in%20the%20order%20you%20decide).
    
5.  **RViz Visualization (Optional):** For convenience, you can visualize waypoints in RViz by publishing small markers (`visualization_msgs/Marker` or `PoseArray`) at each point with labels. This requires a custom ROS node but is not strictly necessary. Instead, ensure you have the YAML backed list of waypoints.
    

## Phase 3: Autonomous Navigation to Waypoints

1.  **Localization on Startup:** On robot boot (Phase 3), launch the ROS navigation stack. Example launch (`navigation.launch`):
    
    ```xml
    <launch>
      <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <param name="yaml_filename" value="$(find my_pkg)/maps/mymap.yaml"/>
        <!-- parameters: odom frame, map frame, etc. -->
      </node>
      <node pkg="move_base" type="move_base" name="move_base" output="screen"/>
      <!-- costmap and planners are loaded via move_base params -->
      <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_laser" args="..."/>
      <!-- motor controller node, etc. -->
    </launch>
    ```
    
    This runs AMCL to track the robot’s pose on the known map. The `move_base` node will then accept goals.
    
2.  **Flask Web API:** On the Pi, run a Flask app that interfaces with ROS. Using a threaded approach avoids blocking ROS and Flask. For example (Python pseudocode):
    
    ```python
    # flask_move_server.py
    import os
    import threading
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from flask import Flask, request
    
    # Initialize ROS node in a separate thread
    threading.Thread(target=lambda: rospy.init_node('flask_ros_node', disable_signals=True)).start()
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    app = Flask(__name__)
    
    # Load waypoints from YAML (as a ROS param or file)
    waypoints = rospy.get_param('/waypoints')  # loaded at startup via rosparam
    
    @app.route('/goto', methods=['POST'])
    def go_to_waypoint():
        data = request.get_json()
        name = data.get('name')
        if name not in waypoints:
            return {'status': 'error', 'message': 'Unknown waypoint'}, 400
        x, y, yaw = waypoints[name]
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        # convert yaw to quaternion
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        pub.publish(goal)
        return {'status': 'ok', 'target': name}, 200
    
    if __name__ == '__main__':
        # Ensure ROS_IP/ROS_HOSTNAME is set for Flask to bind correctly
        host = os.environ.get('ROS_IP', '0.0.0.0')
        app.run(host=host, port=5000)
    ```
    
    This example (inspired by robotics forums) initializes ROS in a separate thread and creates a Flask route `/goto` that accepts JSON `{"name": "office"}`. It looks up the waypoint, creates a `PoseStamped` goal, and publishes to `/move_base_simple/goal`. The robot then navigates there using move\_base[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/73961/easiest-way-to-implement-http-server-that-can-send-ros-messages#:~:text=def%20ros_callback)[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/97108/how-to-save-wavepoints#:~:text=1,in%20the%20order%20you%20decide).
    
    Alternatively, you could use the `actionlib` interface of `move_base` for feedback/cancel, but publishing to `/move_base_simple/goal` (a shortcut to send a goal) is simpler.
    
3.  **ROS–Flask Integration:** The above pattern (Ros + Flask threads) is a known solution. As noted on ROS Answers, running `rospy.init_node` in a separate thread allows Flask to coexist with ROS in one script[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/73961/easiest-way-to-implement-http-server-that-can-send-ros-messages#:~:text=def%20ros_callback). When Flask receives an HTTP POST, it executes the goal publish on the ROS side. No rosbridge (websocket) is needed in this approach[campus-rover.gitbook.io](https://campus-rover.gitbook.io/lab-notebook/fiiva/cr-package/web-application/flask-and-ros#:~:text=For%20example%2C%20when%20run%2Fserved%2C%20the,Flask%20app%20would).
    
4.  **Dynamic IP Handling:** Since the Pi’s IP may change, ensure `ROS_HOSTNAME` uses the mDNS name (e.g. `robot.local`). The Flask app binds to `host=os.environ['ROS_IP']` or `0.0.0.0` so that any network interface (including WiFi) is accessible. Clients on the same network can discover the Flask API at `http://robot.local:5000/goto` (where `robot.local` is the Pi’s hostname). Avahi allows DNS-like resolution of `.local` names.
    
5.  **Using the System:**
    
    -   In Phase 1, run the SLAM launch on the robot and drive it around to explore. Monitor with RViz on the PC. After mapping, save the map.
        
    -   In Phase 2, load the map, drive to save waypoint coordinates, and edit `waypoints.yaml` accordingly.
        
    -   In Phase 3, power on the robot; it will auto-localize with AMCL. From the PC or any client, send HTTP POST commands to the Flask server to instruct navigation. For example (using `curl` or a Python requests client):
        
        ```bash
        curl -X POST -H "Content-Type: application/json" -d '{"name": "kitchen"}' http://robot.local:5000/goto
        ```
        
        The robot will plan and move to the “kitchen” waypoint. Additional endpoints could be added (e.g. `/stop` to cancel goals, or `/teleop` to directly publish to cmd\_vel).
        
6.  **Example Launch Files and Configs:**
    
    -   **SLAM Launch (gmapping):** Shown above.
        
    -   **Navigation Launch:** Start `amcl`, `move_base`, and TF static publisher. Configure `move_base` via a YAML file (`costmap_common_params.yaml`, `local_costmap.yaml`, etc.).
        
    -   **RViz Config:** Save an RViz session with map, laser, odom, and goal markers.
        
    -   **Flask Node Launch:** A simple launch or systemd service can start the Python script above after `roscore` is running. Ensure the ROS environment is sourced.
        
7.  **Troubleshooting and Tuning:**
    
    -   Adjust SLAM parameters (e.g. laser range, scan matching thresholds, odom variance) to get a good map.
        
    -   Tune `amcl` (particle filter params) for stable localization.
        
    -   Ensure TF tree is correct (`map->odom->base_link->laser`).
        
    -   Test `move_base` by setting a goal in RViz before integrating Flask.
        
    -   Verify ROS networking: `rostopic list` from PC should list topics from the robot.
        

## Summary

By combining 2D LIDAR SLAM (e.g. GMapping or Cartographer) for mapping, RViz for manual waypoint definition, and the ROS navigation stack for localization and path planning, a Raspberry Pi–based robot can autonomously explore and navigate an indoor environment. A simple Flask-based HTTP interface enables remote control via named waypoints. Key considerations include reliable odometry (encoders), robust SLAM selection, and careful network configuration (using Avahi/mDNS and ROS\_HOSTNAME to avoid static IP issues[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/102417/how-to-configure-ros-master-uri-ros-hostname#:~:text=In%20case%20your%20network%20does,day%21%20On%20the%20publishing%20computer)). This three-stage approach provides a modular workflow: build the map, define goals, then run. The result is an affordable self-mapping robot platform suitable for research or hobby projects.

**Sources:** ROS documentation and community tutorials on SLAM and navigation[intermodalics.ai](https://www.intermodalics.ai/blog/how-to-map-large-indoor-spaces-with-google-cartographer#:~:text=Cartographer%20is%20a%20real,of%20indoor%20and%20outdoor%20environments)[adityakamath.github.io](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/#:~:text=GMAPPING)wiki.ros.org, plus examples of Flask-ROS integration and waypoint usage[campus-rover.gitbook.io](https://campus-rover.gitbook.io/lab-notebook/fiiva/cr-package/web-application/flask-and-ros#:~:text=For%20example%2C%20when%20run%2Fserved%2C%20the,Flask%20app%20would)[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/73961/easiest-way-to-implement-http-server-that-can-send-ros-messages#:~:text=def%20ros_callback)[robotics.stackexchange.com](https://robotics.stackexchange.com/questions/97108/how-to-save-wavepoints#:~:text=1,in%20the%20order%20you%20decide). These provide implementation details and best practices for each component.

![Favicon](https://www.google.com/s2/favicons?domain=https://robotics.stackexchange.com&sz=32)

![Favicon](https://www.google.com/s2/favicons?domain=https://adityakamath.github.io&sz=32)

![Favicon](https://www.google.com/s2/favicons?domain=http://wiki.ros.org&sz=32)

![Favicon](https://www.google.com/s2/favicons?domain=https://www.hackster.io&sz=32)

![Favicon](https://www.google.com/s2/favicons?domain=https://www.intermodalics.ai&sz=32)
