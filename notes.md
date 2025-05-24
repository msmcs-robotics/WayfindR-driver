https://github.com/AdroitAnandAI/SLAM-on-Raspberry-Pi

[RPLidar SDK](https://www.robotshop.com/products/rplidar-a1m8-360-degree-laser-scanner-development-kit)

[Adafruit RPLidar Python Test](https://docs.circuitpython.org/projects/rplidar/en/latest/examples.html)

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