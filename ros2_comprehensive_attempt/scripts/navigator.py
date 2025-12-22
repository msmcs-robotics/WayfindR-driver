#!/usr/bin/env python3
"""
ROS2 Navigator - Unified navigation with waypoints

This script provides:
1. Waypoint-based navigation
2. A* pathfinding for obstacle avoidance
3. Position monitoring relative to waypoints
4. Route execution

Usage:
    # List waypoints
    python3 navigator.py --waypoints waypoints/office.yaml --list

    # Navigate to waypoint
    python3 navigator.py --map maps/office.yaml --waypoints waypoints/office.yaml --goto reception

    # Execute route
    python3 navigator.py --map maps/office.yaml --waypoints waypoints/office.yaml --route office_tour

    # Monitor position
    python3 navigator.py --map maps/office.yaml --waypoints waypoints/office.yaml --monitor

    # Dry run (no ROS2 required)
    python3 navigator.py --waypoints waypoints/office.yaml --goto reception --dry-run
"""

import os
import sys
import yaml
import math
import argparse
from typing import Optional, List, Dict, Tuple

# Add script directory to path for local imports
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from pathfinder import PathFinder

# Try to import ROS2
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class WaypointNavigator:
    """Navigate between waypoints with localization awareness."""

    def __init__(self, map_yaml: str = None, waypoints_yaml: str = None):
        self.map_yaml = map_yaml
        self.waypoints_yaml = waypoints_yaml
        self.waypoints = {}
        self.routes = {}
        self.pathfinder = None
        self.current_pose = None  # (x, y, yaw)

        if waypoints_yaml and os.path.exists(waypoints_yaml):
            self._load_waypoints()

        if map_yaml and os.path.exists(map_yaml):
            self.pathfinder = PathFinder(map_yaml)

    def _load_waypoints(self):
        """Load waypoints from YAML file."""
        with open(self.waypoints_yaml, 'r') as f:
            data = yaml.safe_load(f)

        self.waypoints = {wp['name']: wp for wp in data.get('waypoints', [])}
        self.routes = data.get('routes', {})
        print(f"Loaded {len(self.waypoints)} waypoints, {len(self.routes)} routes")

    def list_waypoints(self):
        """Print all waypoints."""
        print("\n" + "=" * 60)
        print("WAYPOINTS")
        print("=" * 60)

        if not self.waypoints:
            print("No waypoints loaded.")
            return

        for name, wp in self.waypoints.items():
            x = wp['position']['x']
            y = wp['position']['y']
            yaw = wp.get('yaw_degrees', 0)
            desc = wp.get('description', '')

            distance_str = ""
            if self.current_pose:
                dx = x - self.current_pose[0]
                dy = y - self.current_pose[1]
                dist = math.sqrt(dx**2 + dy**2)
                distance_str = f" [{dist:.2f}m away]"

            print(f"  {name}: ({x:.2f}, {y:.2f}) @ {yaw}°{distance_str}")
            if desc:
                print(f"    {desc}")

        if self.routes:
            print("\nROUTES:")
            for name, wps in self.routes.items():
                print(f"  {name}: {' -> '.join(wps)}")

    def get_waypoint(self, name: str) -> Optional[Dict]:
        """Get waypoint by name."""
        return self.waypoints.get(name)

    def plan_path(self, goal_name: str) -> Optional[List[Tuple[float, float]]]:
        """Plan path from current position to waypoint."""
        if not self.pathfinder:
            print("No pathfinder available (no map loaded)")
            return None

        goal = self.get_waypoint(goal_name)
        if not goal:
            print(f"Waypoint not found: {goal_name}")
            return None

        # Use current pose or map center
        if self.current_pose:
            start_x, start_y = self.current_pose[0], self.current_pose[1]
        else:
            print("Warning: No current pose, using origin")
            start_x, start_y = 0.0, 0.0

        goal_x = goal['position']['x']
        goal_y = goal['position']['y']

        path = self.pathfinder.find_path(start_x, start_y, goal_x, goal_y)
        if path:
            path = self.pathfinder.simplify_path(path, tolerance=0.15)
        return path

    def print_path(self, path: List[Tuple[float, float]], goal_name: str):
        """Print path details."""
        if not path:
            print(f"No path to {goal_name}")
            return

        length = self.pathfinder.get_path_length(path) if self.pathfinder else 0
        print(f"\nPath to {goal_name}: {length:.2f}m, {len(path)} points")
        for i, (x, y) in enumerate(path):
            marker = "(start)" if i == 0 else f"({goal_name})" if i == len(path) - 1 else ""
            print(f"  {i+1}. ({x:.3f}, {y:.3f}) {marker}")


def quaternion_to_yaw(qz, qw):
    """Convert quaternion z/w to yaw in degrees."""
    return math.degrees(2 * math.atan2(qz, qw))


def create_pose_stamped(waypoint: Dict, navigator=None):
    """Create PoseStamped from waypoint."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    if navigator:
        pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(waypoint['position']['x'])
    pose.pose.position.y = float(waypoint['position']['y'])
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = float(waypoint['orientation']['x'])
    pose.pose.orientation.y = float(waypoint['orientation']['y'])
    pose.pose.orientation.z = float(waypoint['orientation']['z'])
    pose.pose.orientation.w = float(waypoint['orientation']['w'])

    return pose


def navigate_to_waypoint(nav: WaypointNavigator, ros_nav, waypoint_name: str) -> bool:
    """Navigate to a waypoint using Nav2."""
    wp = nav.get_waypoint(waypoint_name)
    if not wp:
        print(f"Waypoint not found: {waypoint_name}")
        return False

    print(f"\nNavigating to: {waypoint_name}")
    print(f"  Position: ({wp['position']['x']:.2f}, {wp['position']['y']:.2f})")

    goal_pose = create_pose_stamped(wp, ros_nav)
    ros_nav.goToPose(goal_pose)

    # Wait for completion
    while not ros_nav.isTaskComplete():
        feedback = ros_nav.getFeedback()
        if feedback and hasattr(feedback, 'distance_remaining'):
            print(f"  Distance remaining: {feedback.distance_remaining:.2f}m", end='\r')
        rclpy.spin_once(ros_nav, timeout_sec=0.1)

    result = ros_nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"\n[SUCCESS] Reached {waypoint_name}")
        return True
    else:
        print(f"\n[FAILED] Could not reach {waypoint_name}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='ROS2 Waypoint Navigator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--map', '-m', type=str,
                       help='Path to map YAML file')
    parser.add_argument('--waypoints', '-w', type=str, required=True,
                       help='Path to waypoints YAML file')
    parser.add_argument('--goto', '-g', type=str,
                       help='Navigate to waypoint by name')
    parser.add_argument('--route', '-r', type=str,
                       help='Execute a named route')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all waypoints')
    parser.add_argument('--monitor', action='store_true',
                       help='Monitor current position')
    parser.add_argument('--dry-run', '-d', action='store_true',
                       help='Plan path without executing')

    args = parser.parse_args()

    # Create navigator
    nav = WaypointNavigator(args.map, args.waypoints)

    # List waypoints
    if args.list:
        nav.list_waypoints()
        return

    # Dry run path planning
    if args.dry_run:
        if args.goto:
            path = nav.plan_path(args.goto)
            if path:
                nav.print_path(path, args.goto)
        elif args.route:
            if args.route not in nav.routes:
                print(f"Route not found: {args.route}")
                return
            for wp_name in nav.routes[args.route]:
                nav.current_pose = None
                wp = nav.get_waypoint(wp_name)
                if wp:
                    nav.current_pose = (wp['position']['x'], wp['position']['y'], 0)
                path = nav.plan_path(wp_name)
                if path:
                    nav.print_path(path, wp_name)
        return

    # ROS2 operations
    if not ROS2_AVAILABLE:
        print("ROS2 not available. Use --dry-run for path planning only.")
        return

    rclpy.init()

    try:
        ros_nav = BasicNavigator()
        print("Waiting for Nav2 to become active...")
        ros_nav.waitUntilNav2Active()
        print("Nav2 is active!")

        if args.goto:
            navigate_to_waypoint(nav, ros_nav, args.goto)

        elif args.route:
            if args.route not in nav.routes:
                print(f"Route not found: {args.route}")
                return

            for wp_name in nav.routes[args.route]:
                success = navigate_to_waypoint(nav, ros_nav, wp_name)
                if not success:
                    print(f"Route interrupted at {wp_name}")
                    break

    except KeyboardInterrupt:
        print("\nNavigation cancelled")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
