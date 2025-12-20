#!/usr/bin/env python3
"""
Navigation Python Examples
Demonstrates how to use Nav2 with saved maps and waypoints
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
import time
import math
import os


# =============================================================================
# WAYPOINT LOADING AND MANAGEMENT
# =============================================================================

class WaypointManager:
    """Load and manage waypoints from YAML files."""

    def __init__(self, filepath):
        self.filepath = filepath
        self.data = None
        self.load_waypoints()

    def load_waypoints(self):
        """Load waypoints from YAML file."""
        try:
            with open(os.path.expanduser(self.filepath), 'r') as f:
                self.data = yaml.safe_load(f)
            print(f"✓ Loaded {len(self.data.get('waypoints', []))} waypoints from {self.filepath}")
        except Exception as e:
            print(f"✗ Error loading waypoints: {e}")
            self.data = {'waypoints': []}

    def get_waypoint_by_name(self, name):
        """Get waypoint data by name."""
        for wp in self.data.get('waypoints', []):
            if wp.get('name') == name:
                return wp
        return None

    def get_all_waypoints(self):
        """Get all waypoints."""
        return self.data.get('waypoints', [])

    def get_route(self, route_name):
        """Get a defined route by name."""
        routes = self.data.get('routes', {})
        return routes.get(route_name, [])

    def create_pose_stamped(self, waypoint, frame_id='map'):
        """Convert waypoint dict to PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        # Use current time
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0

        pose.pose.position.x = float(waypoint['position']['x'])
        pose.pose.position.y = float(waypoint['position']['y'])
        pose.pose.position.z = float(waypoint['position']['z'])

        pose.pose.orientation.x = float(waypoint['orientation']['x'])
        pose.pose.orientation.y = float(waypoint['orientation']['y'])
        pose.pose.orientation.z = float(waypoint['orientation']['z'])
        pose.pose.orientation.w = float(waypoint['orientation']['w'])

        return pose


# =============================================================================
# BASIC NAVIGATION EXAMPLES
# =============================================================================

def example_navigate_to_single_waypoint():
    """Navigate to a single waypoint."""
    rclpy.init()

    navigator = BasicNavigator()
    waypoint_manager = WaypointManager('~/waypoints/my_waypoints.yaml')

    # Wait for Nav2 to be ready
    print("Waiting for Nav2 to be ready...")
    navigator.waitUntilNav2Active()
    print("✓ Nav2 is active")

    # Get waypoint
    waypoint = waypoint_manager.get_waypoint_by_name('checkpoint_1')
    if not waypoint:
        print("✗ Waypoint not found")
        return

    # Create goal pose
    goal_pose = waypoint_manager.create_pose_stamped(waypoint)

    # Send goal
    print(f"Navigating to: {waypoint.get('name')}")
    navigator.goToPose(goal_pose)

    # Wait for navigation to complete
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(f"  Time elapsed: {feedback.navigation_time.sec}s")
        time.sleep(1.0)

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"✓ Successfully reached {waypoint.get('name')}")
    elif result == TaskResult.CANCELED:
        print(f"✗ Navigation was canceled")
    elif result == TaskResult.FAILED:
        print(f"✗ Navigation failed")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


def example_navigate_multiple_waypoints():
    """Navigate through multiple waypoints in sequence."""
    rclpy.init()

    navigator = BasicNavigator()
    waypoint_manager = WaypointManager('~/waypoints/my_waypoints.yaml')

    navigator.waitUntilNav2Active()

    # Get all waypoints
    waypoints = waypoint_manager.get_all_waypoints()

    for wp in waypoints:
        print(f"\n→ Navigating to: {wp.get('name')}")
        goal_pose = waypoint_manager.create_pose_stamped(wp)
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            time.sleep(0.5)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"  ✓ Reached {wp.get('name')}")
        else:
            print(f"  ✗ Failed to reach {wp.get('name')}")
            break  # Stop if navigation fails

    navigator.lifecycleShutdown()
    rclpy.shutdown()


def example_patrol_route():
    """Continuously patrol a defined route."""
    rclpy.init()

    navigator = BasicNavigator()
    waypoint_manager = WaypointManager('~/waypoints/warehouse_waypoints.yaml')

    navigator.waitUntilNav2Active()

    # Get route
    route_names = waypoint_manager.get_route('patrol_route')
    if not route_names:
        print("✗ Route 'patrol_route' not found")
        return

    print(f"Starting patrol with {len(route_names)} waypoints")

    patrol_count = 0
    try:
        while True:
            patrol_count += 1
            print(f"\n=== Patrol Loop {patrol_count} ===")

            for wp_name in route_names:
                wp = waypoint_manager.get_waypoint_by_name(wp_name)
                if not wp:
                    print(f"✗ Waypoint '{wp_name}' not found")
                    continue

                print(f"→ {wp_name}")
                goal_pose = waypoint_manager.create_pose_stamped(wp)
                navigator.goToPose(goal_pose)

                while not navigator.isTaskComplete():
                    time.sleep(0.5)

                if navigator.getResult() != TaskResult.SUCCEEDED:
                    print(f"  ✗ Failed, skipping rest of patrol")
                    break

                # Optional: wait at waypoint
                time.sleep(2.0)

    except KeyboardInterrupt:
        print(f"\n✓ Patrol stopped after {patrol_count} loops")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


# =============================================================================
# ADVANCED NAVIGATION WITH MONITORING
# =============================================================================

class NavigationMonitor(Node):
    """Monitor robot navigation status."""

    def __init__(self):
        super().__init__('navigation_monitor')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # State
        self.current_position = None
        self.current_velocity = None
        self.min_obstacle_distance = float('inf')

    def odom_callback(self, msg):
        """Update current position and velocity."""
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        """Update minimum obstacle distance."""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.min_obstacle_distance = min(valid_ranges)
        else:
            self.min_obstacle_distance = float('inf')

    def get_status(self):
        """Get current navigation status."""
        return {
            'position': self.current_position,
            'velocity': self.current_velocity,
            'min_obstacle_distance': self.min_obstacle_distance
        }


def example_navigate_with_monitoring():
    """Navigate with real-time monitoring."""
    rclpy.init()

    navigator = BasicNavigator()
    monitor = NavigationMonitor()
    waypoint_manager = WaypointManager('~/waypoints/my_waypoints.yaml')

    navigator.waitUntilNav2Active()

    waypoint = waypoint_manager.get_waypoint_by_name('checkpoint_1')
    goal_pose = waypoint_manager.create_pose_stamped(waypoint)

    print(f"Navigating to: {waypoint.get('name')}")
    navigator.goToPose(goal_pose)

    # Monitor navigation progress
    while not navigator.isTaskComplete():
        # Spin monitor to update
        rclpy.spin_once(monitor, timeout_sec=0.1)

        status = monitor.get_status()
        if status['position']:
            print(f"  Position: x={status['position'].x:.2f}, y={status['position'].y:.2f}")
            print(f"  Velocity: {status['velocity']:.2f} m/s")
            print(f"  Min obstacle: {status['min_obstacle_distance']:.2f} m")

        # Check for obstacles
        if status['min_obstacle_distance'] < 0.5:
            print("  ⚠ WARNING: Obstacle detected nearby!")

        time.sleep(1.0)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✓ Navigation succeeded")
    else:
        print("✗ Navigation failed")

    monitor.destroy_node()
    navigator.lifecycleShutdown()
    rclpy.shutdown()


# =============================================================================
# EMERGENCY STOP AND RECOVERY
# =============================================================================

class EmergencyStopNavigator:
    """Navigation with emergency stop capability."""

    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        self.node = rclpy.create_node('emergency_nav')

        # Publisher for manual control
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.navigator.waitUntilNav2Active()

    def emergency_stop(self):
        """Immediately stop the robot."""
        print("🛑 EMERGENCY STOP")
        self.navigator.cancelTask()

        # Send zero velocity
        stop_msg = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)

    def navigate_with_safety(self, waypoint_manager, waypoint_name):
        """Navigate with emergency stop on demand."""
        waypoint = waypoint_manager.get_waypoint_by_name(waypoint_name)
        goal_pose = waypoint_manager.create_pose_stamped(waypoint)

        print(f"Navigating to: {waypoint_name} (press Ctrl+C for emergency stop)")
        self.navigator.goToPose(goal_pose)

        try:
            while not self.navigator.isTaskComplete():
                time.sleep(0.5)

            result = self.navigator.getResult()
            return result == TaskResult.SUCCEEDED

        except KeyboardInterrupt:
            self.emergency_stop()
            return False

    def shutdown(self):
        self.navigator.lifecycleShutdown()
        self.node.destroy_node()
        rclpy.shutdown()


# =============================================================================
# WAYPOINT CREATION UTILITIES
# =============================================================================

def yaw_to_quaternion(yaw_degrees):
    """Convert yaw angle in degrees to quaternion."""
    yaw_rad = math.radians(yaw_degrees)
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw_rad / 2),
        'w': math.cos(yaw_rad / 2)
    }


def create_waypoint_template(name, x, y, yaw_degrees=0.0):
    """Create a waypoint dictionary template."""
    return {
        'name': name,
        'position': {
            'x': x,
            'y': y,
            'z': 0.0
        },
        'orientation': yaw_to_quaternion(yaw_degrees),
        'tolerance': {
            'position': 0.3,
            'orientation': 0.3
        }
    }


def save_waypoints_to_file(waypoints, filepath):
    """Save waypoints to YAML file."""
    data = {'waypoints': waypoints}
    with open(os.path.expanduser(filepath), 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"✓ Saved {len(waypoints)} waypoints to {filepath}")


# =============================================================================
# EXAMPLE: CREATE WAYPOINTS PROGRAMMATICALLY
# =============================================================================

def example_create_waypoint_file():
    """Create a waypoint file programmatically."""
    waypoints = [
        create_waypoint_template('home_base', 0.0, 0.0, 0.0),
        create_waypoint_template('checkpoint_1', 3.0, 2.0, 90.0),
        create_waypoint_template('checkpoint_2', 5.0, 5.0, 180.0),
        create_waypoint_template('checkpoint_3', 2.0, 5.0, 270.0),
    ]

    # Add routes
    data = {
        'waypoints': waypoints,
        'routes': {
            'patrol_route': ['home_base', 'checkpoint_1', 'checkpoint_2',
                           'checkpoint_3', 'home_base']
        }
    }

    filepath = '~/waypoints/auto_generated_waypoints.yaml'
    with open(os.path.expanduser(filepath), 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f"✓ Created waypoint file at {filepath}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Run examples."""
    print("ROS2 Navigation Examples")
    print("========================")
    print()
    print("Available examples:")
    print("  1. Navigate to single waypoint")
    print("  2. Navigate through multiple waypoints")
    print("  3. Continuous patrol route")
    print("  4. Navigation with monitoring")
    print("  5. Create waypoint file programmatically")
    print()

    choice = input("Enter choice (1-5): ").strip()

    if choice == '1':
        example_navigate_to_single_waypoint()
    elif choice == '2':
        example_navigate_multiple_waypoints()
    elif choice == '3':
        example_patrol_route()
    elif choice == '4':
        example_navigate_with_monitoring()
    elif choice == '5':
        example_create_waypoint_file()
    else:
        print("Invalid choice")


if __name__ == '__main__':
    main()
