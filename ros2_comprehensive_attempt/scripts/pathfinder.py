#!/usr/bin/env python3
"""
Simple Pathfinder for Waypoint Navigation

This module provides A* pathfinding on a 2D occupancy grid map.
It takes a map image and finds paths between waypoints while avoiding obstacles.

The pathfinder:
1. Loads the map as a grid
2. Plans paths between waypoints using A*
3. Simplifies paths to key turning points
4. Outputs paths as sequences of (x, y) coordinates

Usage:
    from simple_pathfinder import PathFinder

    pf = PathFinder('/path/to/map.yaml')
    path = pf.find_path(start_x, start_y, goal_x, goal_y)
"""

import os
import yaml
import math
import heapq
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class GridCell:
    """Represents a cell in the grid."""
    x: int
    y: int
    g: float = float('inf')  # Cost from start
    h: float = 0.0           # Heuristic to goal
    parent: Optional['GridCell'] = None

    @property
    def f(self) -> float:
        """Total cost (g + h)."""
        return self.g + self.h

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


class PathFinder:
    """A* pathfinder for occupancy grid maps."""

    def __init__(self, map_yaml_path: str):
        """
        Initialize pathfinder with a ROS map.

        Args:
            map_yaml_path: Path to the map YAML file
        """
        self.map_yaml_path = map_yaml_path
        self.grid = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin = [0, 0, 0]

        self._load_map()

    def _load_map(self):
        """Load map from YAML and PGM files."""
        with open(self.map_yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)

        map_dir = os.path.dirname(self.map_yaml_path)
        pgm_path = os.path.join(map_dir, map_data['image'])

        self.resolution = map_data['resolution']
        self.origin = map_data['origin']

        # Load PGM
        self.grid, self.width, self.height = self._load_pgm(pgm_path)

        print(f"Loaded map: {self.width}x{self.height} pixels, "
              f"resolution={self.resolution}m/pixel")

    def _load_pgm(self, filepath: str) -> Tuple[List[List[int]], int, int]:
        """Load PGM image as grid."""
        with open(filepath, 'rb') as f:
            magic = f.readline().decode().strip()
            if magic != 'P5':
                raise ValueError(f"Not a binary PGM: {magic}")

            line = f.readline().decode().strip()
            while line.startswith('#'):
                line = f.readline().decode().strip()

            width, height = map(int, line.split())
            f.readline()  # maxval

            pixels = []
            for y in range(height):
                row = []
                for x in range(width):
                    row.append(ord(f.read(1)))
                pixels.append(row)

            return pixels, width, height

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        gx = int((wx - self.origin[0]) / self.resolution)
        gy = int(self.height - ((wy - self.origin[1]) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates."""
        wx = self.origin[0] + (gx * self.resolution)
        wy = self.origin[1] + ((self.height - gy) * self.resolution)
        return wx, wy

    def is_free(self, gx: int, gy: int, inflation: int = 2) -> bool:
        """
        Check if a grid cell is free (not occupied).

        Args:
            gx, gy: Grid coordinates
            inflation: Extra cells of margin around obstacles
        """
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return False

        # Check cell and neighbors within inflation radius
        for dy in range(-inflation, inflation + 1):
            for dx in range(-inflation, inflation + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    # In ROS maps: 254=free, 0=occupied, 205=unknown
                    # We treat unknown (205) and free (254) as traversable
                    # Only occupied (< 100) is blocked
                    if self.grid[ny][nx] < 100:
                        return False
        return True

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, gx: int, gy: int) -> List[Tuple[int, int, float]]:
        """
        Get valid neighboring cells.
        Returns list of (x, y, move_cost) tuples.
        """
        neighbors = []
        # 8-connected grid
        moves = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),  # Cardinal
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)  # Diagonal
        ]

        for dx, dy, cost in moves:
            nx, ny = gx + dx, gy + dy
            if self.is_free(nx, ny):
                neighbors.append((nx, ny, cost))

        return neighbors

    def find_path_grid(self, start: Tuple[int, int],
                       goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Find path between grid coordinates using A*.

        Returns:
            List of (gx, gy) coordinates, or None if no path found
        """
        if not self.is_free(start[0], start[1], inflation=1):
            print(f"Warning: Start position {start} is not in free space")
            return None

        if not self.is_free(goal[0], goal[1], inflation=1):
            print(f"Warning: Goal position {goal} is not in free space")
            return None

        # Initialize
        open_set = []
        closed_set = set()
        start_cell = GridCell(start[0], start[1], g=0)
        start_cell.h = self.heuristic(start, goal)
        heapq.heappush(open_set, start_cell)

        # For reconstructing path
        came_from = {}
        g_scores = {start: 0}

        while open_set:
            current = heapq.heappop(open_set)

            if (current.x, current.y) == goal:
                # Reconstruct path
                path = [(current.x, current.y)]
                pos = (current.x, current.y)
                while pos in came_from:
                    pos = came_from[pos]
                    path.append(pos)
                path.reverse()
                return path

            closed_set.add((current.x, current.y))

            for nx, ny, move_cost in self.get_neighbors(current.x, current.y):
                if (nx, ny) in closed_set:
                    continue

                tentative_g = current.g + move_cost

                if tentative_g < g_scores.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = (current.x, current.y)
                    g_scores[(nx, ny)] = tentative_g

                    neighbor = GridCell(nx, ny, g=tentative_g)
                    neighbor.h = self.heuristic((nx, ny), goal)
                    heapq.heappush(open_set, neighbor)

        return None  # No path found

    def find_path(self, start_x: float, start_y: float,
                  goal_x: float, goal_y: float) -> Optional[List[Tuple[float, float]]]:
        """
        Find path between world coordinates.

        Args:
            start_x, start_y: Start position in world coordinates (meters)
            goal_x, goal_y: Goal position in world coordinates (meters)

        Returns:
            List of (x, y) world coordinates, or None if no path found
        """
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)

        print(f"Planning path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")
        print(f"  Grid: {start_grid} -> {goal_grid}")

        grid_path = self.find_path_grid(start_grid, goal_grid)

        if grid_path is None:
            print("  No path found!")
            return None

        # Convert to world coordinates
        world_path = [self.grid_to_world(gx, gy) for gx, gy in grid_path]

        print(f"  Found path with {len(world_path)} points")
        return world_path

    def simplify_path(self, path: List[Tuple[float, float]],
                      tolerance: float = 0.1) -> List[Tuple[float, float]]:
        """
        Simplify path using Ramer-Douglas-Peucker algorithm.

        Args:
            path: List of (x, y) coordinates
            tolerance: Maximum deviation from line in meters

        Returns:
            Simplified path
        """
        if len(path) < 3:
            return path

        def perpendicular_distance(point, line_start, line_end):
            """Calculate perpendicular distance from point to line."""
            x, y = point
            x1, y1 = line_start
            x2, y2 = line_end

            dx = x2 - x1
            dy = y2 - y1

            if dx == 0 and dy == 0:
                return math.sqrt((x - x1)**2 + (y - y1)**2)

            t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx**2 + dy**2)))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy

            return math.sqrt((x - proj_x)**2 + (y - proj_y)**2)

        def rdp(points, epsilon):
            """Recursive Douglas-Peucker simplification."""
            if len(points) < 3:
                return points

            # Find point with maximum distance
            max_dist = 0
            max_idx = 0
            for i in range(1, len(points) - 1):
                d = perpendicular_distance(points[i], points[0], points[-1])
                if d > max_dist:
                    max_dist = d
                    max_idx = i

            if max_dist > epsilon:
                # Recursive simplification
                left = rdp(points[:max_idx + 1], epsilon)
                right = rdp(points[max_idx:], epsilon)
                return left[:-1] + right
            else:
                return [points[0], points[-1]]

        simplified = rdp(path, tolerance)
        print(f"  Simplified path: {len(path)} -> {len(simplified)} points")
        return simplified

    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total path length in meters."""
        if len(path) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            length += math.sqrt(dx**2 + dy**2)

        return length


def plan_waypoint_path(map_yaml: str, waypoints_yaml: str,
                       start_waypoint: str, end_waypoint: str) -> Optional[List[Tuple[float, float]]]:
    """
    Plan a path between two named waypoints.

    Args:
        map_yaml: Path to map YAML file
        waypoints_yaml: Path to waypoints YAML file
        start_waypoint: Name of start waypoint
        end_waypoint: Name of goal waypoint

    Returns:
        List of (x, y) coordinates for the path
    """
    # Load waypoints
    with open(waypoints_yaml, 'r') as f:
        wp_data = yaml.safe_load(f)

    waypoints = {wp['name']: wp for wp in wp_data['waypoints']}

    if start_waypoint not in waypoints:
        print(f"Start waypoint '{start_waypoint}' not found")
        return None

    if end_waypoint not in waypoints:
        print(f"End waypoint '{end_waypoint}' not found")
        return None

    start = waypoints[start_waypoint]
    end = waypoints[end_waypoint]

    # Create pathfinder and find path
    pf = PathFinder(map_yaml)
    path = pf.find_path(
        start['position']['x'], start['position']['y'],
        end['position']['x'], end['position']['y']
    )

    if path:
        simplified = pf.simplify_path(path, tolerance=0.15)
        length = pf.get_path_length(simplified)
        print(f"Path length: {length:.2f} meters")
        return simplified

    return None


# Command-line interface
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='A* Pathfinder for ROS maps')
    parser.add_argument('--map', '-m', type=str, required=True,
                       help='Path to map YAML file')
    parser.add_argument('--waypoints', '-w', type=str,
                       help='Path to waypoints YAML file')
    parser.add_argument('--from', dest='start', type=str,
                       help='Start waypoint name')
    parser.add_argument('--to', dest='goal', type=str,
                       help='Goal waypoint name')
    parser.add_argument('--start-xy', nargs=2, type=float, metavar=('X', 'Y'),
                       help='Start coordinates (x y)')
    parser.add_argument('--goal-xy', nargs=2, type=float, metavar=('X', 'Y'),
                       help='Goal coordinates (x y)')
    parser.add_argument('--output', '-o', type=str,
                       help='Output file for path (YAML)')

    args = parser.parse_args()

    pf = PathFinder(args.map)

    path = None

    if args.waypoints and args.start and args.goal:
        path = plan_waypoint_path(args.map, args.waypoints, args.start, args.goal)

    elif args.start_xy and args.goal_xy:
        path = pf.find_path(
            args.start_xy[0], args.start_xy[1],
            args.goal_xy[0], args.goal_xy[1]
        )
        if path:
            path = pf.simplify_path(path)

    else:
        print("Please specify either --waypoints/--from/--to or --start-xy/--goal-xy")
        parser.print_help()
        exit(1)

    if path:
        print(f"\nPath ({len(path)} points):")
        for i, (x, y) in enumerate(path):
            print(f"  {i+1}. ({x:.3f}, {y:.3f})")

        if args.output:
            output_data = {
                'path': [{'x': x, 'y': y} for x, y in path],
                'length': pf.get_path_length(path)
            }
            with open(args.output, 'w') as f:
                yaml.dump(output_data, f)
            print(f"\nSaved path to: {args.output}")
