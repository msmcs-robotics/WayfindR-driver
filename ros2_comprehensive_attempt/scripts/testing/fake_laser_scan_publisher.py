#!/usr/bin/env python3
"""
Fake LaserScan Publisher for Testing Nav2 without Hardware

This script publishes synthetic LaserScan messages to /scan topic,
simulating a 2D LiDAR sensor. Useful for testing SLAM, localization,
and navigation algorithms without physical hardware or full simulation.

Usage:
    ros2 run <package> fake_laser_scan_publisher.py

    Or directly:
    python3 fake_laser_scan_publisher.py

Author: WayfindR Development Team
Date: 2026-01-11
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random


class FakeLaserScanPublisher(Node):
    """
    Publishes synthetic LaserScan data simulating a simple environment.
    Default: rectangular room with walls.
    """

    def __init__(self):
        super().__init__('fake_laser_scan_publisher')

        # Declare parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('angle_min', -3.14159)  # radians
        self.declare_parameter('angle_max', 3.14159)   # radians
        self.declare_parameter('angle_increment', 0.0174533)  # ~1 degree
        self.declare_parameter('range_min', 0.15)      # meters
        self.declare_parameter('range_max', 12.0)      # meters
        self.declare_parameter('room_width', 4.0)      # meters
        self.declare_parameter('room_length', 6.0)     # meters
        self.declare_parameter('add_noise', True)
        self.declare_parameter('noise_stddev', 0.02)   # meters

        # Get parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.room_width = self.get_parameter('room_width').value
        self.room_length = self.get_parameter('room_length').value
        self.add_noise = self.get_parameter('add_noise').value
        self.noise_stddev = self.get_parameter('noise_stddev').value

        # Create publisher
        self.publisher = self.create_publisher(LaserScan, scan_topic, 10)

        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_scan)

        self.get_logger().info(f'Fake LaserScan publisher started on {scan_topic}')
        self.get_logger().info(f'Room dimensions: {self.room_width}m x {self.room_length}m')
        self.get_logger().info(f'Publishing at {publish_rate} Hz')

    def publish_scan(self):
        """Publish a synthetic LaserScan message"""
        scan = LaserScan()

        # Header
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        # Scan parameters
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0001
        scan.scan_time = 1.0 / 10.0  # Assumes 10 Hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Generate ranges and intensities
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        scan.ranges = []
        scan.intensities = []

        # Robot is assumed to be at center of room
        robot_x = 0.0
        robot_y = 0.0

        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment

            # Calculate distance to nearest wall
            distance = self._calculate_wall_distance(
                robot_x, robot_y, angle,
                self.room_width, self.room_length
            )

            # Add noise if enabled
            if self.add_noise:
                noise = random.gauss(0, self.noise_stddev)
                distance += noise

            # Clamp to valid range
            distance = max(self.range_min, min(self.range_max, distance))

            scan.ranges.append(distance)
            scan.intensities.append(100.0)  # Constant intensity

        self.publisher.publish(scan)

    def _calculate_wall_distance(self, x, y, angle, width, length):
        """
        Calculate distance from (x,y) to nearest wall in given angle direction.

        Assumes rectangular room centered at origin:
        - Width extends from -width/2 to +width/2 (left/right walls)
        - Length extends from -length/2 to +length/2 (front/back walls)
        """
        # Ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)

        distances = []

        # Check intersection with right wall (x = width/2)
        if dx > 0.001:
            t = (width / 2.0 - x) / dx
            if t > 0:
                y_intersect = y + t * dy
                if abs(y_intersect) <= length / 2.0:
                    distances.append(t)

        # Check intersection with left wall (x = -width/2)
        if dx < -0.001:
            t = (-width / 2.0 - x) / dx
            if t > 0:
                y_intersect = y + t * dy
                if abs(y_intersect) <= length / 2.0:
                    distances.append(t)

        # Check intersection with front wall (y = length/2)
        if dy > 0.001:
            t = (length / 2.0 - y) / dy
            if t > 0:
                x_intersect = x + t * dx
                if abs(x_intersect) <= width / 2.0:
                    distances.append(t)

        # Check intersection with back wall (y = -length/2)
        if dy < -0.001:
            t = (-length / 2.0 - y) / dy
            if t > 0:
                x_intersect = x + t * dx
                if abs(x_intersect) <= width / 2.0:
                    distances.append(t)

        # Return minimum distance (nearest wall)
        if distances:
            return min(distances)
        else:
            return self.range_max


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserScanPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down fake laser scan publisher')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
