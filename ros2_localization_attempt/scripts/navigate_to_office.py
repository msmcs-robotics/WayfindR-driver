#!/usr/bin/env python3
"""
Navigate to Office Waypoints

This script demonstrates navigation between office waypoints using:
1. AMCL for localization (determines current position)
2. A* pathfinding (plans route avoiding obstacles)
3. Nav2 for execution (actually moves the robot)

Usage:
    # Navigate to a specific office
    python3 navigate_to_office.py office1

    # Navigate through all offices
    python3 navigate_to_office.py --route office_tour

    # Show current position and distance to waypoints
    python3 navigate_to_office.py --status

    # Dry run (show plan without executing)
    python3 navigate_to_office.py office2 --dry-run
"""

import os
import sys
import yaml
import math
import argparse
from typing import Optional, Tuple, List, Dict

# Try to import ROS2
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
    from nav_msgs.msg import Path
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Note: ROS2 not available. Running in dry-run mode only.")

# Import local pathfinder
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
from simple_pathfinder import PathFinder


class OfficeNavigator:
    """Navigate between office waypoints with localization awareness."""

    def __init__(self, map_yaml: str, waypoints_yaml: str):
        self.map_yaml = map_yaml
        self.waypoints_yaml = waypoints_yaml

        # Load waypoints
        with open(waypoints_yaml, 'r') as f:
            wp_data = yaml.safe_load(f)

        self.waypoints = {wp['name']: wp for wp in wp_data['waypoints']}
        self.routes = wp_data.get('routes', {})

        # Initialize pathfinder
        self.pathfinder = PathFinder(map_yaml)

        # Current robot pose (from AMCL)
        self.current_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw

        print(f"Loaded {len(self.waypoints)} waypoints")
        print(f"Loaded {len(self.routes)} routes")

    def list_waypoints(self):
        """Print all waypoints and their distances from current position."""
        print("\n" + "=" * 60)
        print("Office Waypoints")
        print("=" * 60)

        for name, wp in self.waypoints.items():
            x = wp['position']['x']
            y = wp['position']['y']
            yaw = wp.get('yaw_degrees', 0)

            dist_str = ""
            if self.current_pose:
                dx = x - self.current_pose[0]
                dy = y - self.current_pose[1]
                dist = math.sqrt(dx**2 + dy**2)
                dist_str = f" [{dist:.2f}m away]"

            print(f"  {name}: ({x:.2f}, {y:.2f}) @ {yaw}°{dist_str}")

        if self.routes:
            print("\nRoutes:")
            for name, waypoint_names in self.routes.items():
                print(f"  {name}: {' -> '.join(waypoint_names)}")

    def get_waypoint(self, name: str) -> Optional[Dict]:
        """Get waypoint by name."""
        return self.waypoints.get(name)

    def plan_path(self, goal_name: str) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from current position to waypoint.

        Returns:
            List of (x, y) coordinates for path, or None if no path found
        """
        goal = self.get_waypoint(goal_name)
        if not goal:
            print(f"Unknown waypoint: {goal_name}")
            return None

        goal_x = goal['position']['x']
        goal_y = goal['position']['y']

        if self.current_pose:
            start_x, start_y = self.current_pose[0], self.current_pose[1]
        else:
            # Assume robot is at map center if no pose available
            print("Warning: No current pose. Using map center as start.")
            start_x = 0.0
            start_y = 0.0

        path = self.pathfinder.find_path(start_x, start_y, goal_x, goal_y)

        if path:
            path = self.pathfinder.simplify_path(path, tolerance=0.15)

        return path

    def plan_route(self, route_name: str) -> Optional[List[List[Tuple[float, float]]]]:
        """
        Plan paths for an entire route.

        Returns:
            List of paths (each path is list of coordinates)
        """
        if route_name not in self.routes:
            print(f"Unknown route: {route_name}")
            print(f"Available routes: {list(self.routes.keys())}")
            return None

        waypoint_names = self.routes[route_name]
        all_paths = []

        for i, wp_name in enumerate(waypoint_names):
            wp = self.get_waypoint(wp_name)
            if not wp:
                print(f"Warning: Waypoint {wp_name} not found, skipping")
                continue

            if i == 0:
                # First waypoint: plan from current position
                path = self.plan_path(wp_name)
            else:
                # Subsequent waypoints: plan from previous waypoint
                prev_wp = self.get_waypoint(waypoint_names[i-1])
                path = self.pathfinder.find_path(
                    prev_wp['position']['x'], prev_wp['position']['y'],
                    wp['position']['x'], wp['position']['y']
                )
                if path:
                    path = self.pathfinder.simplify_path(path)

            if path:
                all_paths.append(path)

        return all_paths

    def print_path(self, path: List[Tuple[float, float]], goal_name: str):
        """Print path details."""
        if not path:
            print(f"No path to {goal_name}")
            return

        length = self.pathfinder.get_path_length(path)

        print(f"\nPath to {goal_name}:")
        print(f"  Total length: {length:.2f} meters")
        print(f"  Waypoints: {len(path)}")
        print(f"  Points:")
        for i, (x, y) in enumerate(path):
            label = ""
            if i == 0:
                label = " (start)"
            elif i == len(path) - 1:
                label = f" (goal: {goal_name})"
            print(f"    {i+1}. ({x:.3f}, {y:.3f}){label}")


# ROS2 node class - only defined when ROS2 is available
if ROS2_AVAILABLE:
    class RobotPoseSubscriber(Node):
        """ROS2 node to subscribe to AMCL pose."""

        def __init__(self, navigator: OfficeNavigator):
            super().__init__('office_navigator')
            self.navigator = navigator

            self.subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.pose_callback,
                10
            )

            self.get_logger().info('Waiting for AMCL pose...')

        def pose_callback(self, msg):
            """Handle incoming pose from AMCL."""
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Extract yaw from quaternion
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = math.degrees(2 * math.atan2(qz, qw))

            self.navigator.current_pose = (x, y, yaw)
            self.get_logger().info(f'Pose: ({x:.2f}, {y:.2f}) @ {yaw:.1f}°')


def main():
    parser = argparse.ArgumentParser(
        description='Navigate to office waypoints',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 navigate_to_office.py office1
    python3 navigate_to_office.py --route office_tour
    python3 navigate_to_office.py --status
    python3 navigate_to_office.py office2 --dry-run
        """
    )

    parser.add_argument('waypoint', nargs='?', type=str,
                       help='Waypoint name to navigate to')
    parser.add_argument('--route', '-r', type=str,
                       help='Execute a named route')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all waypoints and routes')
    parser.add_argument('--status', '-s', action='store_true',
                       help='Show current pose and distances')
    parser.add_argument('--dry-run', '-d', action='store_true',
                       help='Plan path but do not execute')
    parser.add_argument('--map', '-m', type=str,
                       default=os.path.expanduser('~/ros2_ws/maps/first_map.yaml'),
                       help='Path to map YAML file')
    parser.add_argument('--waypoints', '-w', type=str,
                       default=os.path.expanduser('~/ros2_ws/maps/first_map_offices.yaml'),
                       help='Path to waypoints YAML file')

    args = parser.parse_args()

    # Check files exist
    if not os.path.exists(args.map):
        print(f"Map file not found: {args.map}")
        sys.exit(1)

    if not os.path.exists(args.waypoints):
        print(f"Waypoints file not found: {args.waypoints}")
        print("Run add_office_waypoints.py first to create waypoints.")
        sys.exit(1)

    # Create navigator
    nav = OfficeNavigator(args.map, args.waypoints)

    # Handle list command
    if args.list:
        nav.list_waypoints()
        return

    # Handle status command
    if args.status:
        if ROS2_AVAILABLE:
            rclpy.init()
            node = RobotPoseSubscriber(nav)
            # Spin for a few seconds to get pose
            import time
            start = time.time()
            while time.time() - start < 3.0 and nav.current_pose is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.shutdown()

        nav.list_waypoints()
        return

    # Handle route command
    if args.route:
        print(f"\nPlanning route: {args.route}")
        paths = nav.plan_route(args.route)

        if paths:
            total_length = sum(nav.pathfinder.get_path_length(p) for p in paths)
            print(f"\nTotal route length: {total_length:.2f} meters")
            print(f"Number of segments: {len(paths)}")

            for i, path in enumerate(paths):
                wp_name = nav.routes[args.route][i]
                nav.print_path(path, wp_name)

        if args.dry_run:
            print("\n[DRY RUN] Would execute this route")
        return

    # Handle single waypoint navigation
    if args.waypoint:
        path = nav.plan_path(args.waypoint)

        if path:
            nav.print_path(path, args.waypoint)

            if args.dry_run:
                print("\n[DRY RUN] Would navigate to this waypoint")
            elif ROS2_AVAILABLE:
                print("\nTo execute, run with full Nav2 stack")
        return

    # No command specified
    parser.print_help()


if __name__ == '__main__':
    main()
