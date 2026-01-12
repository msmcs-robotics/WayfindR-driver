#!/usr/bin/env python3
"""
Synthetic Navigation Data Publisher for Testing

Publishes realistic synthetic sensor data including:
- LaserScan messages with obstacles
- Odometry with simulated robot motion
- TF transforms

This allows testing Nav2 stack without hardware or full simulation.

Usage:
    python3 synthetic_nav_data_publisher.py

Parameters can be configured via ROS parameters.

Author: WayfindR Development Team
Date: 2026-01-11
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
import random


class SyntheticNavDataPublisher(Node):
    """
    Publishes synthetic navigation data simulating a robot moving
    through an environment with obstacles.
    """

    def __init__(self):
        super().__init__('synthetic_nav_data_publisher')

        # Declare parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_frame', 'laser_frame')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('odom_rate', 20.0)
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 0.1)
        self.declare_parameter('motion_pattern', 'circular')  # circular, straight, square

        # Get parameters
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        scan_rate = self.get_parameter('scan_rate').value
        odom_rate = self.get_parameter('odom_rate').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.motion_pattern = self.get_parameter('motion_pattern').value

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time_elapsed = 0.0

        # Environment: obstacles as (x, y, radius)
        self.obstacles = [
            (2.0, 0.0, 0.3),
            (-1.5, 1.5, 0.4),
            (1.0, -2.0, 0.5),
            (-2.0, -1.0, 0.3),
            (3.0, 2.0, 0.4),
        ]

        # Walls (xmin, xmax, ymin, ymax)
        self.walls = (-5.0, 5.0, -5.0, 5.0)

        # Timers
        self.create_timer(1.0 / scan_rate, self.publish_scan)
        self.create_timer(1.0 / odom_rate, self.publish_odom)

        self.get_logger().info('Synthetic navigation data publisher started')
        self.get_logger().info(f'Motion pattern: {self.motion_pattern}')
        self.get_logger().info(f'Number of obstacles: {len(self.obstacles)}')

    def publish_scan(self):
        """Publish synthetic laser scan with obstacles"""
        scan = LaserScan()

        # Header
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.scan_frame

        # Scan configuration (typical 2D LiDAR)
        scan.angle_min = -3.14159
        scan.angle_max = 3.14159
        scan.angle_increment = 0.0174533  # ~1 degree
        scan.time_increment = 0.0001
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)

        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            global_angle = self.theta + angle

            # Start with max range
            min_range = scan.range_max

            # Check walls
            wall_dist = self._distance_to_walls(self.x, self.y, global_angle)
            min_range = min(min_range, wall_dist)

            # Check obstacles
            for obs_x, obs_y, obs_radius in self.obstacles:
                obs_dist = self._distance_to_obstacle(
                    self.x, self.y, global_angle,
                    obs_x, obs_y, obs_radius
                )
                if obs_dist is not None:
                    min_range = min(min_range, obs_dist)

            # Add realistic noise
            noise = random.gauss(0, 0.02)
            range_with_noise = min_range + noise

            # Occasionally add invalid readings (simulate occlusion/reflection)
            if random.random() < 0.02:  # 2% invalid readings
                range_with_noise = float('inf')

            # Clamp to valid range
            range_with_noise = max(scan.range_min, min(scan.range_max, range_with_noise))

            scan.ranges.append(range_with_noise)
            scan.intensities.append(random.uniform(80, 120))  # Variable intensity

        self.scan_pub.publish(scan)

    def publish_odom(self):
        """Publish synthetic odometry"""
        dt = 0.05  # 20 Hz

        # Update velocity based on motion pattern
        if self.motion_pattern == 'circular':
            linear = self.linear_vel
            angular = self.angular_vel
        elif self.motion_pattern == 'straight':
            linear = self.linear_vel
            angular = 0.0
        elif self.motion_pattern == 'square':
            # Square pattern: straight for 2s, turn for 1s
            cycle_time = self.time_elapsed % 3.0
            if cycle_time < 2.0:
                linear = self.linear_vel
                angular = 0.0
            else:
                linear = 0.0
                angular = math.pi / 2.0  # 90 degree turn in 1s
        else:
            linear = self.linear_vel
            angular = self.angular_vel

        # Update robot pose
        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt
        self.theta += angular * dt
        self.time_elapsed += dt

        # Normalize theta
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

        # Add odometry drift (realistic error accumulation)
        drift_x = random.gauss(0, 0.001)
        drift_y = random.gauss(0, 0.001)
        drift_theta = random.gauss(0, 0.002)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position
        odom.pose.pose.position.x = self.x + drift_x
        odom.pose.pose.position.y = self.y + drift_y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        quat = self._yaw_to_quaternion(self.theta + drift_theta)
        odom.pose.pose.orientation = quat

        # Covariance (6x6 matrix, flattened)
        # Increase uncertainty over time
        position_var = 0.01 + self.time_elapsed * 0.001
        orientation_var = 0.01 + self.time_elapsed * 0.002

        odom.pose.covariance = [
            position_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, position_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, orientation_var
        ]

        # Velocity
        odom.twist.twist.linear.x = linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular

        # Velocity covariance
        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        self.odom_pub.publish(odom)

        # Publish TF transform
        self._publish_tf()

    def _publish_tf(self):
        """Publish odom -> base_footprint transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        quat = self._yaw_to_quaternion(self.theta)
        t.transform.rotation = quat

        self.tf_broadcaster.sendTransform(t)

        # Also publish base_footprint -> laser_frame (static offset)
        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = self.base_frame
        t_laser.child_frame_id = self.scan_frame

        # Laser offset from base (adjust for your robot)
        t_laser.transform.translation.x = 0.0
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.2

        t_laser.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t_laser)

    def _distance_to_walls(self, x, y, angle):
        """Calculate distance to room walls"""
        dx = math.cos(angle)
        dy = math.sin(angle)

        xmin, xmax, ymin, ymax = self.walls
        distances = []

        # Right wall
        if dx > 0.001:
            t = (xmax - x) / dx
            if t > 0:
                y_int = y + t * dy
                if ymin <= y_int <= ymax:
                    distances.append(t)

        # Left wall
        if dx < -0.001:
            t = (xmin - x) / dx
            if t > 0:
                y_int = y + t * dy
                if ymin <= y_int <= ymax:
                    distances.append(t)

        # Top wall
        if dy > 0.001:
            t = (ymax - y) / dy
            if t > 0:
                x_int = x + t * dx
                if xmin <= x_int <= xmax:
                    distances.append(t)

        # Bottom wall
        if dy < -0.001:
            t = (ymin - y) / dy
            if t > 0:
                x_int = x + t * dx
                if xmin <= x_int <= xmax:
                    distances.append(t)

        return min(distances) if distances else 12.0

    def _distance_to_obstacle(self, x, y, angle, obs_x, obs_y, obs_radius):
        """
        Calculate distance from (x,y) to obstacle in given direction.
        Returns None if ray doesn't hit obstacle.
        """
        # Vector from robot to obstacle
        dx_to_obs = obs_x - x
        dy_to_obs = obs_y - y

        # Ray direction
        ray_dx = math.cos(angle)
        ray_dy = math.sin(angle)

        # Project obstacle center onto ray
        # Parametric ray: p(t) = (x, y) + t * (ray_dx, ray_dy)
        # Find t where ray is closest to obstacle center
        t_closest = (dx_to_obs * ray_dx + dy_to_obs * ray_dy)

        if t_closest < 0:
            return None  # Obstacle is behind

        # Point on ray closest to obstacle
        closest_x = x + t_closest * ray_dx
        closest_y = y + t_closest * ray_dy

        # Distance from closest point to obstacle center
        dist_to_center = math.sqrt(
            (closest_x - obs_x)**2 + (closest_y - obs_y)**2
        )

        # Check if ray intersects obstacle
        if dist_to_center <= obs_radius:
            # Calculate intersection point
            offset = math.sqrt(obs_radius**2 - dist_to_center**2)
            t_intersect = t_closest - offset
            if t_intersect > 0:
                return t_intersect

        return None

    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticNavDataPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down synthetic nav data publisher')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
