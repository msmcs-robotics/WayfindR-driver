https://github.com/AdroitAnandAI/SLAM-on-Raspberry-Pi

[RPLidar SDK](https://www.robotshop.com/products/rplidar-a1m8-360-degree-laser-scanner-development-kit)

[Adafruit RPLidar Python Test](https://docs.circuitpython.org/projects/rplidar/en/latest/examples.html)


[Using Google Cartographer with RPLidar](https://medium.com/robotics-weekends/2d-mapping-using-google-cartographer-and-rplidar-with-raspberry-pi-a94ce11e44c5)


[RPLidar A1M8 RPI Mount](https://www.thingiverse.com/thing:3970110)

---

**✅ To continue building an accurate map while moving:**
You need to track the motion of the LiDAR platform. There are two main approaches:

**🧭 Option 1: Use Odometry (Recommended for Wheeled Robots)**
If your LiDAR is mounted on a mobile robot (like a wheeled platform), you can use wheel encoders or motor feedback to compute:

Linear movement (Δx, Δy)

Angular rotation (Δθ)

This is called odometry and is essential for SLAM algorithms to update the robot’s pose (position + orientation) over time.

Update Needed in Code:

```python
position = [x, y, theta]  # Update each loop from odometry
```

**📦 Option 2: Use an IMU (Inertial Measurement Unit)**
For handheld or non-wheeled setups:

An IMU sensor like the MPU6050 or BNO055 can provide orientation (roll, pitch, yaw).

Combine accelerometer + gyroscope + magnetometer to estimate movement.

However, IMUs drift over time and are best used in combination with odometry or LiDAR corrections.

**🧠 Option 3: Use SLAM Alone (Scan Matching)**
More advanced SLAM algorithms like:

TinySLAM, GMapping, Cartographer, or Hector SLAM

These infer movement from LiDAR scan alignment (called scan matching).

⚠️ But this needs more complex math — typically using particle filters or graph-based optimization — and is less accurate without odometry or IMU.


Breadth-First Search


---


I want a comprehensive system to create a map of the inside of a building using slamtech C1M1 and a raspberry pi 4. then I want to be able to add waypoints to that map in Rviz and save it for later. i then want to be able to use that map in a program where I can send commands via post request to a flask app on the raspberry pi to then have it navigate and pathfind to those waypoints. I want to have 3 stages, mapping, map editing, and navigating.

mapping I want to have a robot that simply makes the map of the building, searching and moving and discovering all possible routes to move in the building and rooms and such so that I can let it go autonomously and it will make a comprehensive map streamed back to a desktop over wifi. i want to account for moving across multiple access points in commercial wifi, the wifi credentials will still work its just that the IP address might change or I don't know.

further more when I am done running the mapping stage. i want to save the map in Rviz, then make an edited version of the map where I can move the robot around using that map then add waypoints corresponding to offices and rooms and save this map with custom waypoints for later. 

finally I want to have a phase where I turn on the robot, it figures out where its at in the map with the waypoints, then I can tell it to go to a waypoint and it will navigate to it.

Ideallly I would use google cartographer as it seems easy to setup and make comprehensive maps.

I want a comprehensive overview of this mapping, waypoint adding, navigation system that covers the ubuntu ROS on the rpi, the ROS on ubuntu/windows on a laptop/desktop, and the flask app hosted on the rpi to then receive and interpret commands.

https://medium.com/robotics-weekends/2d-mapping-using-google-cartographer-and-rplidar-with-raspberry-pi-a94ce11e44c5


https://www.google.com/search?q=add+waypoints+to+map+in+rviz

https://www.google.com/search?q=how+to+navigate+to+waypoints+on+an+rviz+map+then+send+to+robot

https://github.com/Andrew-rw/gbot_core


Robot Base and Mobility: What type of robot or base are you using for navigation (e.g., Turtlebot, custom differential drive, etc.)?

I am going to use acustomskid steer type of driving system where I take an L298motor driverand place one on the right side and one on the left side each motor driver will control two wheels so that I can get these skid steer type of mobility.If needed I can add wheel encoders to assist withthe mapping and navigation process.

SLAM and Localization: Are you committed to using Google Cartographer for both mapping and localization, or are you open to other packages like GMapping or AMCL for localization?

I am open to whatever would be the easiest possible setup to perform this mapping and localization process and then navigation in the map. I just want a system to put on my robotand use the rp Lidar a oneon top of my differential drive system to make a map inside of a buildingand then add waypoints to that map that correspond to office locationsand then have a navigation phase where I use that map with those setway points for the robot to navigate to offices.

Navigation Stack: Would you prefer to use the standard ROS Navigation Stack (move_base, AMCL, etc.) for pathfinding, or are you looking for a lightweight custom solution?

WiFi IP Handling: Would you prefer a dynamic discovery method (e.g., mDNS/Avahi or broadcasting over LAN) for dealing with IP address changes, or are you planning to use static IPs?

I may need to deal with IP address changes so I would like to compensate for these changes and be able to handle them in the system

Flask App Functions: Besides receiving navigation commands via POST, should the Flask app handle any other functions (e.g., status reporting, streaming location, receiving map updates)?

I will add functionality to the flask gap as needed and as I want to. Very much but all I need is for it to listen for Post request that it then handles as commands for movementand interacts with these skid steerL298 in motor drivers as needed.