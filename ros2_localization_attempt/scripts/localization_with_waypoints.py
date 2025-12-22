#!/usr/bin/env python3
"""
Localization Monitor with Waypoint Awareness

This script subscribes to AMCL pose and shows:
1. Current robot position
2. Distance and bearing to each waypoint
3. Path to nearest waypoint
4. Localization quality

Usage:
    # Must have localization running first:
    # ~/start_localization.sh

    # Then run this monitor:
    source /opt/ros/humble/setup.bash
    python3 localization_with_waypoints.py
"""

import os
import sys
import yaml
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

# Import local pathfinder
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
from simple_pathfinder import PathFinder


def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle in degrees."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def bearing_to(from_x, from_y, from_yaw, to_x, to_y):
    """Calculate bearing from current position to target."""
    dx = to_x - from_x
    dy = to_y - from_y
    absolute_bearing = math.degrees(math.atan2(dy, dx))
    relative_bearing = absolute_bearing - from_yaw

    # Normalize to -180 to 180
    while relative_bearing > 180:
        relative_bearing -= 360
    while relative_bearing < -180:
        relative_bearing += 360

    return relative_bearing


def direction_arrow(bearing):
    """Get arrow character for bearing direction."""
    if -22.5 <= bearing < 22.5:
        return "→"  # Ahead
    elif 22.5 <= bearing < 67.5:
        return "↗"  # Front-left
    elif 67.5 <= bearing < 112.5:
        return "↑"  # Left
    elif 112.5 <= bearing < 157.5:
        return "↖"  # Back-left
    elif bearing >= 157.5 or bearing < -157.5:
        return "←"  # Behind
    elif -157.5 <= bearing < -112.5:
        return "↙"  # Back-right
    elif -112.5 <= bearing < -67.5:
        return "↓"  # Right
    else:  # -67.5 <= bearing < -22.5
        return "↘"  # Front-right


class LocalizationWaypointMonitor(Node):
    """Monitor localization and show waypoint distances."""

    def __init__(self, waypoints_yaml: str, map_yaml: str):
        super().__init__('localization_waypoint_monitor')

        # Load waypoints
        with open(waypoints_yaml, 'r') as f:
            wp_data = yaml.safe_load(f)
        self.waypoints = {wp['name']: wp for wp in wp_data['waypoints']}

        # Initialize pathfinder
        self.pathfinder = PathFinder(map_yaml)

        # Current pose
        self.current_pose = None
        self.pose_count = 0
        self.last_update = time.time()

        # Subscribe to AMCL pose
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Localization Waypoint Monitor started')
        self.get_logger().info(f'Monitoring {len(self.waypoints)} waypoints')
        self.print_header()

    def print_header(self):
        """Print header info."""
        print("\n" + "=" * 70)
        print("  LOCALIZATION WAYPOINT MONITOR")
        print("=" * 70)
        print("Waiting for AMCL pose on /amcl_pose...")
        print("(Use RViz '2D Pose Estimate' tool to set initial pose)")
        print("-" * 70)

    def pose_callback(self, msg):
        """Handle AMCL pose update."""
        self.pose_count += 1
        now = time.time()

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract yaw
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q)

        # Extract covariance for quality estimate
        cov = msg.pose.covariance
        std_x = math.sqrt(cov[0]) if cov[0] > 0 else 0
        std_y = math.sqrt(cov[7]) if cov[7] > 0 else 0
        uncertainty = std_x + std_y

        self.current_pose = (x, y, yaw)

        # Throttle output to once per second
        if now - self.last_update < 1.0:
            return
        self.last_update = now

        # Clear and print status
        print("\033[2J\033[H")  # Clear screen
        print("=" * 70)
        print("  LOCALIZATION WAYPOINT MONITOR")
        print("=" * 70)
        print(f"\nRobot Position: ({x:.3f}, {y:.3f}) @ {yaw:.1f}°")
        print(f"Uncertainty: ±{std_x:.3f}m (x), ±{std_y:.3f}m (y)")

        # Quality indicator
        if uncertainty < 0.1:
            quality = "★★★ EXCELLENT"
        elif uncertainty < 0.3:
            quality = "★★☆ GOOD"
        elif uncertainty < 0.5:
            quality = "★☆☆ FAIR"
        else:
            quality = "☆☆☆ POOR"
        print(f"Quality: {quality}")

        print("\n" + "-" * 70)
        print("WAYPOINTS:")
        print("-" * 70)
        print(f"{'Name':<15} {'Distance':>10} {'Bearing':>10} {'Direction':>10}")
        print("-" * 70)

        # Calculate distances to each waypoint
        distances = []
        for name, wp in self.waypoints.items():
            wp_x = wp['position']['x']
            wp_y = wp['position']['y']

            dx = wp_x - x
            dy = wp_y - y
            dist = math.sqrt(dx**2 + dy**2)
            bearing = bearing_to(x, y, yaw, wp_x, wp_y)
            arrow = direction_arrow(bearing)

            distances.append((name, dist, bearing, arrow))

        # Sort by distance
        distances.sort(key=lambda x: x[1])

        for name, dist, bearing, arrow in distances:
            bearing_str = f"{bearing:+.0f}°"
            print(f"{name:<15} {dist:>8.2f}m {bearing_str:>10} {arrow:>10}")

        # Show nearest waypoint path
        nearest = distances[0]
        print("\n" + "-" * 70)
        print(f"NEAREST WAYPOINT: {nearest[0]} ({nearest[1]:.2f}m away)")

        # Calculate path to nearest
        target_wp = self.waypoints[nearest[0]]
        path = self.pathfinder.find_path(
            x, y,
            target_wp['position']['x'], target_wp['position']['y']
        )

        if path:
            path = self.pathfinder.simplify_path(path, tolerance=0.15)
            path_length = self.pathfinder.get_path_length(path)
            print(f"Path length: {path_length:.2f}m ({len(path)} waypoints)")
            print("Path:")
            for i, (px, py) in enumerate(path[:5]):  # Show first 5 points
                label = " (current)" if i == 0 else ""
                if i == len(path) - 1:
                    label = f" ({nearest[0]})"
                print(f"  {i+1}. ({px:.2f}, {py:.2f}){label}")
            if len(path) > 5:
                print(f"  ... and {len(path) - 5} more points")

        print("\n" + "-" * 70)
        print(f"Updates: {self.pose_count} | Press Ctrl+C to exit")
        print("-" * 70)


def main():
    # Paths
    map_yaml = os.path.expanduser('~/ros2_ws/maps/first_map.yaml')
    waypoints_yaml = os.path.expanduser('~/ros2_ws/maps/first_map_offices.yaml')

    # Verify files exist
    if not os.path.exists(map_yaml):
        print(f"Error: Map file not found: {map_yaml}")
        sys.exit(1)

    if not os.path.exists(waypoints_yaml):
        print(f"Error: Waypoints file not found: {waypoints_yaml}")
        print("Run add_office_waypoints.py first.")
        sys.exit(1)

    # Initialize ROS2
    rclpy.init()

    try:
        node = LocalizationWaypointMonitor(waypoints_yaml, map_yaml)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nMonitor stopped.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
