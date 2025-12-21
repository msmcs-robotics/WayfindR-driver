#!/usr/bin/env python3
"""
ROS2 Waypoint Navigation Script

This script loads waypoints from a YAML file and navigates to them
using Nav2's Simple Commander API.

Requirements:
    - ROS2 Humble
    - Nav2 (navigation2) installed
    - nav2_simple_commander package
    - A running Nav2 stack with localization

Usage:
    ros2 run lidar_mapping navigate_waypoints.py --waypoints /path/to/waypoints.yaml

    # Or run directly:
    python3 navigate_waypoints.py --waypoints waypoints.yaml --waypoint map_center
    python3 navigate_waypoints.py --waypoints waypoints.yaml --route patrol
    python3 navigate_waypoints.py --waypoints waypoints.yaml --all
"""

import argparse
import time
import yaml
import sys
import math

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 packages not available. Running in dry-run mode.")


def load_waypoints(filepath: str) -> dict:
    """Load waypoints from YAML file."""
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    return data


def create_pose_stamped(waypoint: dict, navigator=None) -> 'PoseStamped':
    """Convert waypoint dict to PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'

    if navigator:
        pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(waypoint['position']['x'])
    pose.pose.position.y = float(waypoint['position']['y'])
    pose.pose.position.z = float(waypoint['position'].get('z', 0.0))

    pose.pose.orientation.x = float(waypoint['orientation']['x'])
    pose.pose.orientation.y = float(waypoint['orientation']['y'])
    pose.pose.orientation.z = float(waypoint['orientation']['z'])
    pose.pose.orientation.w = float(waypoint['orientation']['w'])

    return pose


def print_waypoint_info(waypoint: dict) -> None:
    """Print waypoint information."""
    pos = waypoint['position']
    yaw = waypoint.get('yaw_degrees', 0)
    print(f"  Name: {waypoint['name']}")
    print(f"  Position: ({pos['x']:.3f}, {pos['y']:.3f})")
    print(f"  Orientation: {yaw:.1f} degrees")
    if waypoint.get('description'):
        print(f"  Description: {waypoint['description']}")


def navigate_to_waypoint(navigator, waypoint: dict) -> bool:
    """
    Navigate to a single waypoint.
    Returns True if successful, False otherwise.
    """
    print(f"\n{'='*50}")
    print(f"Navigating to: {waypoint['name']}")
    print_waypoint_info(waypoint)
    print(f"{'='*50}")

    goal_pose = create_pose_stamped(waypoint, navigator)
    navigator.goToPose(goal_pose)

    # Monitor navigation progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:  # Print every 5 iterations
            # Calculate distance remaining
            if hasattr(feedback, 'distance_remaining'):
                print(f"  Distance remaining: {feedback.distance_remaining:.2f} m")
            elif hasattr(feedback, 'distance_to_goal'):
                print(f"  Distance to goal: {feedback.distance_to_goal:.2f} m")

        time.sleep(0.1)

    # Check result
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print(f"[SUCCESS] Reached waypoint: {waypoint['name']}")
        return True
    elif result == TaskResult.CANCELED:
        print(f"[CANCELED] Navigation to {waypoint['name']} was canceled")
        return False
    elif result == TaskResult.FAILED:
        print(f"[FAILED] Could not reach waypoint: {waypoint['name']}")
        return False
    else:
        print(f"[UNKNOWN] Navigation result: {result}")
        return False


def navigate_route(navigator, waypoints: list, route: list, loop: bool = False) -> None:
    """Navigate through a sequence of waypoints."""
    # Create waypoint lookup by name
    wp_map = {wp['name']: wp for wp in waypoints}

    iteration = 0
    while True:
        iteration += 1
        print(f"\n{'#'*60}")
        print(f"Route iteration: {iteration}")
        print(f"{'#'*60}")

        for wp_name in route:
            if wp_name not in wp_map:
                print(f"Warning: Waypoint '{wp_name}' not found, skipping")
                continue

            waypoint = wp_map[wp_name]
            success = navigate_to_waypoint(navigator, waypoint)

            if not success:
                print(f"Failed to reach {wp_name}. Continuing to next waypoint...")

            # Brief pause at waypoint
            time.sleep(1.0)

        if not loop:
            break

        print("\nRoute complete. Starting next iteration...")
        time.sleep(2.0)


def dry_run(waypoints_data: dict, target_waypoint: str = None,
            target_route: str = None, navigate_all: bool = False) -> None:
    """
    Print what would be executed without actually running ROS2.
    Useful for testing when ROS2 is not available.
    """
    waypoints = waypoints_data.get('waypoints', [])
    routes = waypoints_data.get('routes', {})

    print("\n" + "="*60)
    print("DRY RUN MODE (ROS2 not running)")
    print("="*60)

    print(f"\nLoaded {len(waypoints)} waypoints:")
    for wp in waypoints:
        print(f"  - {wp['name']}: ({wp['position']['x']:.2f}, {wp['position']['y']:.2f})")

    if routes:
        print(f"\nDefined routes:")
        for name, route in routes.items():
            print(f"  - {name}: {' -> '.join(route)}")

    if target_waypoint:
        wp = next((w for w in waypoints if w['name'] == target_waypoint), None)
        if wp:
            print(f"\nWould navigate to waypoint: {target_waypoint}")
            print_waypoint_info(wp)
        else:
            print(f"\nWaypoint '{target_waypoint}' not found!")

    elif target_route:
        if target_route in routes:
            print(f"\nWould execute route: {target_route}")
            for wp_name in routes[target_route]:
                wp = next((w for w in waypoints if w['name'] == wp_name), None)
                if wp:
                    print(f"  -> {wp_name}: ({wp['position']['x']:.2f}, {wp['position']['y']:.2f})")
        else:
            print(f"\nRoute '{target_route}' not found!")

    elif navigate_all:
        print("\nWould navigate to all waypoints in order:")
        for wp in waypoints:
            print(f"  -> {wp['name']}")


def main():
    parser = argparse.ArgumentParser(
        description='Navigate to waypoints using Nav2',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--waypoints', '-w', type=str, required=True,
                        help='Path to waypoints YAML file')
    parser.add_argument('--waypoint', '-p', type=str,
                        help='Navigate to a specific waypoint by name')
    parser.add_argument('--route', '-r', type=str,
                        help='Execute a named route')
    parser.add_argument('--all', '-a', action='store_true',
                        help='Navigate to all waypoints in order')
    parser.add_argument('--loop', '-l', action='store_true',
                        help='Loop the route continuously')
    parser.add_argument('--dry-run', '-d', action='store_true',
                        help='Print plan without executing')
    parser.add_argument('--list', action='store_true',
                        help='List all waypoints and routes')

    args = parser.parse_args()

    # Load waypoints
    try:
        waypoints_data = load_waypoints(args.waypoints)
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        sys.exit(1)

    waypoints = waypoints_data.get('waypoints', [])
    routes = waypoints_data.get('routes', {})

    # Just list waypoints
    if args.list:
        print(f"\nWaypoints ({len(waypoints)}):")
        for wp in waypoints:
            print(f"  {wp['name']}: ({wp['position']['x']:.2f}, {wp['position']['y']:.2f}) "
                  f"@ {wp.get('yaw_degrees', 0):.0f} deg")
        if routes:
            print(f"\nRoutes ({len(routes)}):")
            for name, route in routes.items():
                print(f"  {name}: {' -> '.join(route)}")
        return

    # Dry run mode
    if args.dry_run or not ROS2_AVAILABLE:
        dry_run(waypoints_data, args.waypoint, args.route, args.all)
        return

    # Initialize ROS2
    rclpy.init()

    navigator = BasicNavigator()

    # Wait for Nav2 to become active
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active!")

    try:
        if args.waypoint:
            # Navigate to single waypoint
            wp = next((w for w in waypoints if w['name'] == args.waypoint), None)
            if wp:
                navigate_to_waypoint(navigator, wp)
            else:
                print(f"Waypoint '{args.waypoint}' not found!")
                sys.exit(1)

        elif args.route:
            # Execute a route
            if args.route in routes:
                navigate_route(navigator, waypoints, routes[args.route], args.loop)
            else:
                print(f"Route '{args.route}' not found!")
                print(f"Available routes: {list(routes.keys())}")
                sys.exit(1)

        elif args.all:
            # Navigate to all waypoints
            route = [wp['name'] for wp in waypoints]
            navigate_route(navigator, waypoints, route, args.loop)

        else:
            print("Please specify --waypoint, --route, or --all")
            parser.print_help()

    except KeyboardInterrupt:
        print("\nNavigation interrupted by user")
        navigator.cancelTask()

    finally:
        navigator.lifecycleShutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
