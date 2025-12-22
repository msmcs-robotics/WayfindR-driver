"""
Navigation Service

Provides waypoint navigation and path planning capabilities.
Integrates with the ROS2 comprehensive system for localization.
"""

import asyncio
import math
import yaml
import os
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass


@dataclass
class Waypoint:
    """Navigation waypoint."""
    name: str
    x: float
    y: float
    theta: float = 0.0  # radians
    tolerance: float = 0.3  # meters


class NavigationService:
    """
    Waypoint navigation service.

    Provides:
    - Waypoint loading/saving
    - Path planning
    - Navigation execution
    - Position tracking
    """

    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.waypoints: Dict[str, Waypoint] = {}
        self.routes: Dict[str, List[str]] = {}
        self.current_target: Optional[str] = None
        self._navigation_task: Optional[asyncio.Task] = None
        self._cancel_navigation = False

        # Current estimated position (would come from localization)
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_theta = 0.0

    def load_waypoints(self, filepath: str):
        """Load waypoints from YAML file."""
        if not os.path.exists(filepath):
            print(f"Waypoint file not found: {filepath}")
            return

        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        self.waypoints = {}
        for wp in data.get('waypoints', []):
            self.waypoints[wp['name']] = Waypoint(
                name=wp['name'],
                x=wp['position']['x'],
                y=wp['position']['y'],
                theta=wp.get('yaw_degrees', 0) * math.pi / 180,
                tolerance=wp.get('tolerance', {}).get('position', 0.3)
            )

        self.routes = data.get('routes', {})
        print(f"Loaded {len(self.waypoints)} waypoints, {len(self.routes)} routes")

    def list_waypoints(self) -> List[Dict]:
        """Get list of all waypoints."""
        return [
            {
                "name": wp.name,
                "x": wp.x,
                "y": wp.y,
                "theta_degrees": wp.theta * 180 / math.pi,
                "distance": self._distance_to(wp.x, wp.y)
            }
            for wp in self.waypoints.values()
        ]

    def list_routes(self) -> Dict[str, List[str]]:
        """Get all routes."""
        return self.routes

    def _distance_to(self, x: float, y: float) -> float:
        """Calculate distance from current position to point."""
        dx = x - self.position_x
        dy = y - self.position_y
        return math.sqrt(dx*dx + dy*dy)

    def _bearing_to(self, x: float, y: float) -> float:
        """Calculate bearing from current position to point (radians)."""
        dx = x - self.position_x
        dy = y - self.position_y
        return math.atan2(dy, dx)

    async def navigate_to_waypoint(self, waypoint_name: str) -> bool:
        """
        Navigate to a named waypoint.

        This is a simplified implementation that works without
        actual localization. For real use, integrate with ROS2
        AMCL localization.

        Returns:
            True if navigation successful, False otherwise
        """
        if waypoint_name not in self.waypoints:
            print(f"Unknown waypoint: {waypoint_name}")
            return False

        wp = self.waypoints[waypoint_name]
        self.current_target = waypoint_name
        self.robot.state.navigation.target_waypoint = waypoint_name

        print(f"Navigating to {waypoint_name} at ({wp.x:.2f}, {wp.y:.2f})")

        # Simple point-to-point navigation
        # In reality, you'd use path planning and localization feedback

        distance = self._distance_to(wp.x, wp.y)
        bearing = self._bearing_to(wp.x, wp.y)

        # First, rotate to face target
        angle_diff = bearing - self.position_theta
        # Normalize to -pi to pi
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) > 0.1:  # More than ~6 degrees
            print(f"Rotating {math.degrees(angle_diff):.1f} degrees")
            await self.robot.rotate_angle(math.degrees(angle_diff), speed=0.3)

        # Then drive forward
        if distance > wp.tolerance:
            # Estimate time based on distance (very rough)
            travel_time = distance / 0.3  # Assuming 0.3 m/s
            print(f"Driving {distance:.2f}m (est. {travel_time:.1f}s)")
            await self.robot.move_for_duration(0.5, 0.0, travel_time)

        # Update simulated position
        self.position_x = wp.x
        self.position_y = wp.y
        self.position_theta = wp.theta

        self.current_target = None
        self.robot.state.navigation.target_waypoint = None
        print(f"Arrived at {waypoint_name}")

        return True

    async def execute_route(self, route_name: str, loop: bool = False):
        """
        Execute a named route (sequence of waypoints).

        Args:
            route_name: Name of the route
            loop: Whether to loop continuously
        """
        if route_name not in self.routes:
            print(f"Unknown route: {route_name}")
            return

        waypoint_names = self.routes[route_name]
        self.robot.state.navigation.route_name = route_name
        self._cancel_navigation = False

        iteration = 0
        while True:
            iteration += 1
            print(f"Route {route_name} - iteration {iteration}")

            for i, wp_name in enumerate(waypoint_names):
                if self._cancel_navigation:
                    print("Route cancelled")
                    return

                self.robot.state.navigation.waypoints_remaining = len(waypoint_names) - i
                self.robot.state.navigation.route_progress = (i / len(waypoint_names)) * 100

                success = await self.navigate_to_waypoint(wp_name)
                if not success:
                    print(f"Failed to reach {wp_name}, stopping route")
                    return

                # Brief pause at waypoint
                await asyncio.sleep(0.5)

            if not loop:
                break

        self.robot.state.navigation.route_name = None
        self.robot.state.navigation.route_progress = 100

    def cancel_navigation(self):
        """Cancel any ongoing navigation."""
        self._cancel_navigation = True
        self.current_target = None

    def update_position(self, x: float, y: float, theta: float):
        """
        Update current position (from localization).

        In a real system, this would be called from ROS2
        AMCL pose callback.
        """
        self.position_x = x
        self.position_y = y
        self.position_theta = theta

        # Update robot state
        self.robot.state.position.x = x
        self.robot.state.position.y = y
        self.robot.state.position.theta = theta
