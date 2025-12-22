#!/usr/bin/env python3
"""
Set Initial Pose for AMCL Localization

This script publishes an initial pose estimate to help AMCL
converge faster. Can be used instead of clicking in RViz.

Usage:
    # Set pose at origin (0, 0) facing forward
    ros2 run localization set_initial_pose.py

    # Set specific pose (x, y, yaw_degrees)
    ros2 run localization set_initial_pose.py 1.0 2.0 90

    # Or run directly
    python3 set_initial_pose.py 1.0 2.0 45
"""

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (radians) to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0, 0, 0, 0]
    q[0] = cr * cp * cy + sr * sp * sy  # w
    q[1] = sr * cp * cy - cr * sp * sy  # x
    q[2] = cr * sp * cy + sr * cp * sy  # y
    q[3] = cr * cp * sy - sr * sp * cy  # z
    return q


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.get_logger().info('Initial Pose Publisher initialized')

    def publish_pose(self, x, y, yaw_degrees):
        """Publish initial pose estimate."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Position
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Orientation (convert degrees to radians)
        yaw_rad = math.radians(float(yaw_degrees))
        q = euler_to_quaternion(0, 0, yaw_rad)
        msg.pose.pose.orientation.w = q[0]
        msg.pose.pose.orientation.x = q[1]
        msg.pose.pose.orientation.y = q[2]
        msg.pose.pose.orientation.z = q[3]

        # Covariance (6x6 matrix, row-major)
        # Position covariance (x, y, z)
        msg.pose.covariance[0] = 0.25   # x variance
        msg.pose.covariance[7] = 0.25   # y variance
        msg.pose.covariance[14] = 0.0   # z variance (not used in 2D)
        # Orientation covariance (roll, pitch, yaw)
        msg.pose.covariance[21] = 0.0   # roll variance
        msg.pose.covariance[28] = 0.0   # pitch variance
        msg.pose.covariance[35] = 0.068 # yaw variance (~15 degrees)

        # Publish multiple times to ensure delivery
        for _ in range(3):
            self.publisher.publish(msg)
            self.get_logger().info(
                f'Published initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw_degrees:.1f}°'
            )
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    # Parse command line arguments
    x = 0.0
    y = 0.0
    yaw = 0.0

    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])
    elif len(sys.argv) == 2 and sys.argv[1] in ['-h', '--help']:
        print(__doc__)
        return

    # Initialize ROS2
    rclpy.init()
    node = InitialPosePublisher()

    try:
        node.publish_pose(x, y, yaw)
        print(f'\nInitial pose set to: ({x}, {y}) with yaw {yaw}°')
        print('AMCL should now converge around this position.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
