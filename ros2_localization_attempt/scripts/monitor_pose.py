#!/usr/bin/env python3
"""
Monitor AMCL Pose Estimates

Subscribes to AMCL pose topic and displays real-time position updates.
Useful for debugging and understanding localization quality.

Usage:
    python3 monitor_pose.py

    # Or with ROS2
    ros2 run localization monitor_pose.py
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.pose_count = 0
        self.last_pose = None

        self.get_logger().info('Pose Monitor started')
        self.get_logger().info('Waiting for AMCL pose messages on /amcl_pose...')
        self.get_logger().info('(Use RViz "2D Pose Estimate" tool to set initial pose)')
        print('\n' + '=' * 60)

    def pose_callback(self, msg):
        self.pose_count += 1

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation as yaw
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q)
        yaw_deg = math.degrees(yaw)

        # Extract covariance (diagonal elements)
        cov = msg.pose.covariance
        cov_x = cov[0]    # variance in x
        cov_y = cov[7]    # variance in y
        cov_yaw = cov[35] # variance in yaw

        # Calculate uncertainty (standard deviation)
        std_x = math.sqrt(cov_x) if cov_x > 0 else 0
        std_y = math.sqrt(cov_y) if cov_y > 0 else 0
        std_yaw = math.degrees(math.sqrt(cov_yaw)) if cov_yaw > 0 else 0

        # Calculate distance from last pose (if available)
        distance = 0.0
        if self.last_pose:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            distance = math.sqrt(dx*dx + dy*dy)

        self.last_pose = (x, y, yaw)

        # Print formatted output
        print(f'\n[Pose #{self.pose_count}]')
        print(f'  Position:    x={x:7.3f} m, y={y:7.3f} m')
        print(f'  Orientation: yaw={yaw_deg:7.2f}°')
        print(f'  Uncertainty: ±{std_x:.3f} m (x), ±{std_y:.3f} m (y), ±{std_yaw:.1f}° (yaw)')

        if distance > 0.001:
            print(f'  Movement:    {distance:.3f} m from last pose')

        # Quality indicator
        total_uncertainty = std_x + std_y
        if total_uncertainty < 0.1:
            quality = '★★★ EXCELLENT'
        elif total_uncertainty < 0.3:
            quality = '★★☆ GOOD'
        elif total_uncertainty < 0.5:
            quality = '★☆☆ FAIR'
        else:
            quality = '☆☆☆ POOR - Set initial pose!'

        print(f'  Quality:     {quality}')
        print('-' * 60)


def main():
    print('\n' + '=' * 60)
    print('  AMCL Pose Monitor')
    print('=' * 60)
    print('Press Ctrl+C to exit')

    rclpy.init()
    node = PoseMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\nMonitor stopped.')
        print(f'Total poses received: {node.pose_count}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
