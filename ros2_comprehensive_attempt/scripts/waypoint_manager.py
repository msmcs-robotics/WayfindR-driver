#!/usr/bin/env python3
"""
Waypoint Manager for ROS2 Maps

This script provides tools to:
- Calculate map dimensions and center
- Add waypoints to maps programmatically
- Save/load waypoint files in YAML format
- Visualize waypoints on map images

Usage:
    python3 waypoint_manager.py --map-yaml /path/to/map.yaml --add-center
    python3 waypoint_manager.py --map-yaml /path/to/map.yaml --add-waypoint "kitchen" 2.5 1.0 90
    python3 waypoint_manager.py --waypoints /path/to/waypoints.yaml --list
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import yaml


@dataclass
class Waypoint:
    """Represents a navigation waypoint."""
    name: str
    x: float
    y: float
    z: float = 0.0
    yaw_degrees: float = 0.0  # Orientation in degrees
    description: str = ""
    tolerance_position: float = 0.3  # meters
    tolerance_orientation: float = 0.2  # radians

    @property
    def quaternion(self) -> Dict[str, float]:
        """Convert yaw angle to quaternion."""
        yaw_rad = math.radians(self.yaw_degrees)
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw_rad / 2),
            'w': math.cos(yaw_rad / 2)
        }

    def to_dict(self) -> Dict:
        """Convert waypoint to dictionary for YAML export."""
        return {
            'name': self.name,
            'description': self.description,
            'position': {
                'x': round(self.x, 4),
                'y': round(self.y, 4),
                'z': round(self.z, 4)
            },
            'orientation': {
                'x': round(self.quaternion['x'], 6),
                'y': round(self.quaternion['y'], 6),
                'z': round(self.quaternion['z'], 6),
                'w': round(self.quaternion['w'], 6)
            },
            'yaw_degrees': self.yaw_degrees,
            'tolerance': {
                'position': self.tolerance_position,
                'orientation': self.tolerance_orientation
            }
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'Waypoint':
        """Create waypoint from dictionary."""
        # Calculate yaw from quaternion if not provided
        yaw_degrees = data.get('yaw_degrees', 0.0)
        if yaw_degrees == 0.0 and 'orientation' in data:
            qz = data['orientation'].get('z', 0.0)
            qw = data['orientation'].get('w', 1.0)
            yaw_degrees = math.degrees(2 * math.atan2(qz, qw))

        return cls(
            name=data.get('name', 'unnamed'),
            x=data['position']['x'],
            y=data['position']['y'],
            z=data['position'].get('z', 0.0),
            yaw_degrees=yaw_degrees,
            description=data.get('description', ''),
            tolerance_position=data.get('tolerance', {}).get('position', 0.3),
            tolerance_orientation=data.get('tolerance', {}).get('orientation', 0.2)
        )


@dataclass
class MapInfo:
    """Information about a ROS2 map."""
    yaml_path: str
    image_path: str
    resolution: float  # meters per pixel
    origin_x: float    # world coordinates of bottom-left
    origin_y: float
    origin_z: float
    width_pixels: int = 0
    height_pixels: int = 0
    negate: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.25

    @property
    def width_meters(self) -> float:
        """Map width in meters."""
        return self.width_pixels * self.resolution

    @property
    def height_meters(self) -> float:
        """Map height in meters."""
        return self.height_pixels * self.resolution

    @property
    def center_world(self) -> Tuple[float, float]:
        """Center of map in world coordinates."""
        center_x = self.origin_x + (self.width_meters / 2)
        center_y = self.origin_y + (self.height_meters / 2)
        return (center_x, center_y)

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates."""
        # Note: Image y-axis is inverted (0 is top)
        world_x = self.origin_x + (px * self.resolution)
        world_y = self.origin_y + ((self.height_pixels - py) * self.resolution)
        return (world_x, world_y)

    def world_to_pixel(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to pixel coordinates."""
        px = int((wx - self.origin_x) / self.resolution)
        py = int(self.height_pixels - ((wy - self.origin_y) / self.resolution))
        return (px, py)

    @classmethod
    def from_yaml(cls, yaml_path: str) -> 'MapInfo':
        """Load map info from YAML file."""
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        image_path = os.path.join(yaml_dir, data['image'])

        origin = data.get('origin', [0, 0, 0])

        map_info = cls(
            yaml_path=yaml_path,
            image_path=image_path,
            resolution=data['resolution'],
            origin_x=origin[0],
            origin_y=origin[1],
            origin_z=origin[2] if len(origin) > 2 else 0.0,
            negate=data.get('negate', 0),
            occupied_thresh=data.get('occupied_thresh', 0.65),
            free_thresh=data.get('free_thresh', 0.25)
        )

        # Try to get image dimensions
        map_info._load_image_dimensions()

        return map_info

    def _load_image_dimensions(self):
        """Load image dimensions from PGM file."""
        try:
            with open(self.image_path, 'rb') as f:
                # Read PGM header
                magic = f.readline().decode().strip()
                if magic not in ('P5', 'P2'):
                    print(f"Warning: Unknown PGM format: {magic}")
                    return

                # Skip comments
                line = f.readline().decode().strip()
                while line.startswith('#'):
                    line = f.readline().decode().strip()

                # Parse dimensions
                parts = line.split()
                self.width_pixels = int(parts[0])
                self.height_pixels = int(parts[1])
        except Exception as e:
            print(f"Warning: Could not read image dimensions: {e}")


class WaypointManager:
    """Manages waypoints for a map."""

    def __init__(self, map_info: Optional[MapInfo] = None):
        self.map_info = map_info
        self.waypoints: List[Waypoint] = []
        self.routes: Dict[str, List[str]] = {}
        self.metadata: Dict = {}

    def add_waypoint(self, waypoint: Waypoint) -> None:
        """Add a waypoint to the list."""
        # Check for duplicate names
        existing_names = [wp.name for wp in self.waypoints]
        if waypoint.name in existing_names:
            print(f"Warning: Replacing existing waypoint '{waypoint.name}'")
            self.waypoints = [wp for wp in self.waypoints if wp.name != waypoint.name]

        self.waypoints.append(waypoint)
        print(f"Added waypoint '{waypoint.name}' at ({waypoint.x:.3f}, {waypoint.y:.3f})")

    def add_center_waypoint(self, name: str = "map_center",
                            yaw_degrees: float = 0.0) -> Waypoint:
        """Add a waypoint at the center of the map."""
        if not self.map_info:
            raise ValueError("No map info loaded. Cannot calculate center.")

        center_x, center_y = self.map_info.center_world

        waypoint = Waypoint(
            name=name,
            x=center_x,
            y=center_y,
            yaw_degrees=yaw_degrees,
            description=f"Center of map (auto-generated)"
        )

        self.add_waypoint(waypoint)
        return waypoint

    def add_corner_waypoints(self) -> List[Waypoint]:
        """Add waypoints at all four corners of the map."""
        if not self.map_info:
            raise ValueError("No map info loaded.")

        corners = []
        mi = self.map_info

        # Bottom-left (origin)
        corners.append(Waypoint(
            name="corner_bottom_left",
            x=mi.origin_x + 0.5,  # Offset slightly into map
            y=mi.origin_y + 0.5,
            yaw_degrees=45,
            description="Bottom-left corner"
        ))

        # Bottom-right
        corners.append(Waypoint(
            name="corner_bottom_right",
            x=mi.origin_x + mi.width_meters - 0.5,
            y=mi.origin_y + 0.5,
            yaw_degrees=135,
            description="Bottom-right corner"
        ))

        # Top-right
        corners.append(Waypoint(
            name="corner_top_right",
            x=mi.origin_x + mi.width_meters - 0.5,
            y=mi.origin_y + mi.height_meters - 0.5,
            yaw_degrees=225,
            description="Top-right corner"
        ))

        # Top-left
        corners.append(Waypoint(
            name="corner_top_left",
            x=mi.origin_x + 0.5,
            y=mi.origin_y + mi.height_meters - 0.5,
            yaw_degrees=315,
            description="Top-left corner"
        ))

        for wp in corners:
            self.add_waypoint(wp)

        return corners

    def add_route(self, name: str, waypoint_names: List[str]) -> None:
        """Add a named route (sequence of waypoints)."""
        self.routes[name] = waypoint_names
        print(f"Added route '{name}' with {len(waypoint_names)} waypoints")

    def get_waypoint(self, name: str) -> Optional[Waypoint]:
        """Get waypoint by name."""
        for wp in self.waypoints:
            if wp.name == name:
                return wp
        return None

    def list_waypoints(self) -> None:
        """Print all waypoints."""
        if not self.waypoints:
            print("No waypoints defined.")
            return

        print(f"\n{'='*60}")
        print(f"Waypoints ({len(self.waypoints)} total)")
        print(f"{'='*60}")

        for i, wp in enumerate(self.waypoints, 1):
            print(f"\n{i}. {wp.name}")
            print(f"   Position: ({wp.x:.3f}, {wp.y:.3f}, {wp.z:.3f})")
            print(f"   Orientation: {wp.yaw_degrees:.1f} degrees")
            print(f"   Quaternion: z={wp.quaternion['z']:.4f}, w={wp.quaternion['w']:.4f}")
            if wp.description:
                print(f"   Description: {wp.description}")

    def save_to_yaml(self, filepath: str) -> None:
        """Save waypoints to YAML file."""
        data = {
            'metadata': {
                'created_by': 'waypoint_manager.py',
                'map_yaml': self.map_info.yaml_path if self.map_info else None,
                'frame_id': 'map'
            },
            'waypoints': [wp.to_dict() for wp in self.waypoints]
        }

        if self.routes:
            data['routes'] = self.routes

        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)

        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        print(f"\nSaved {len(self.waypoints)} waypoints to: {filepath}")

    def load_from_yaml(self, filepath: str) -> None:
        """Load waypoints from YAML file."""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        self.metadata = data.get('metadata', {})
        self.routes = data.get('routes', {})

        self.waypoints = []
        for wp_data in data.get('waypoints', []):
            self.waypoints.append(Waypoint.from_dict(wp_data))

        print(f"Loaded {len(self.waypoints)} waypoints from: {filepath}")

    def print_map_info(self) -> None:
        """Print map information."""
        if not self.map_info:
            print("No map loaded.")
            return

        mi = self.map_info
        print(f"\n{'='*60}")
        print("Map Information")
        print(f"{'='*60}")
        print(f"YAML file: {mi.yaml_path}")
        print(f"Image file: {mi.image_path}")
        print(f"Resolution: {mi.resolution} m/pixel ({mi.resolution*100:.1f} cm/pixel)")
        print(f"Dimensions: {mi.width_pixels} x {mi.height_pixels} pixels")
        print(f"            {mi.width_meters:.2f} x {mi.height_meters:.2f} meters")
        print(f"Origin: ({mi.origin_x:.3f}, {mi.origin_y:.3f}, {mi.origin_z:.3f})")
        print(f"Center: ({mi.center_world[0]:.3f}, {mi.center_world[1]:.3f})")
        print(f"Thresholds: free={mi.free_thresh}, occupied={mi.occupied_thresh}")


def main():
    parser = argparse.ArgumentParser(
        description='Waypoint Manager for ROS2 Maps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Show map info and add center waypoint
  python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-center

  # Add a custom waypoint
  python3 waypoint_manager.py --map-yaml maps/first_map.yaml \\
      --add-waypoint "kitchen" 2.5 1.0 90

  # Add corner waypoints
  python3 waypoint_manager.py --map-yaml maps/first_map.yaml --add-corners

  # Load and list existing waypoints
  python3 waypoint_manager.py --waypoints waypoints.yaml --list
        """
    )

    parser.add_argument('--map-yaml', '-m', type=str,
                        help='Path to map YAML file')
    parser.add_argument('--waypoints', '-w', type=str,
                        help='Path to waypoints YAML file to load')
    parser.add_argument('--output', '-o', type=str,
                        help='Output path for waypoints YAML file')

    parser.add_argument('--add-center', action='store_true',
                        help='Add a waypoint at the center of the map')
    parser.add_argument('--add-corners', action='store_true',
                        help='Add waypoints at all four corners')
    parser.add_argument('--add-waypoint', nargs=4, metavar=('NAME', 'X', 'Y', 'YAW'),
                        help='Add a custom waypoint: name x y yaw_degrees')

    parser.add_argument('--list', '-l', action='store_true',
                        help='List all waypoints')
    parser.add_argument('--info', action='store_true',
                        help='Show map information')

    args = parser.parse_args()

    # Load map if specified
    map_info = None
    if args.map_yaml:
        try:
            map_info = MapInfo.from_yaml(args.map_yaml)
        except Exception as e:
            print(f"Error loading map: {e}")
            sys.exit(1)

    # Create waypoint manager
    manager = WaypointManager(map_info)

    # Load existing waypoints if specified
    if args.waypoints:
        try:
            manager.load_from_yaml(args.waypoints)
        except Exception as e:
            print(f"Error loading waypoints: {e}")
            sys.exit(1)

    # Show map info
    if args.info or args.map_yaml:
        manager.print_map_info()

    # Add center waypoint
    if args.add_center:
        if not map_info:
            print("Error: --map-yaml required to add center waypoint")
            sys.exit(1)
        manager.add_center_waypoint()

    # Add corner waypoints
    if args.add_corners:
        if not map_info:
            print("Error: --map-yaml required to add corner waypoints")
            sys.exit(1)
        manager.add_corner_waypoints()

    # Add custom waypoint
    if args.add_waypoint:
        name, x, y, yaw = args.add_waypoint
        waypoint = Waypoint(
            name=name,
            x=float(x),
            y=float(y),
            yaw_degrees=float(yaw)
        )
        manager.add_waypoint(waypoint)

    # List waypoints
    if args.list or manager.waypoints:
        manager.list_waypoints()

    # Save waypoints
    if args.output and manager.waypoints:
        manager.save_to_yaml(args.output)
    elif manager.waypoints and not args.output:
        # Default output path
        if args.map_yaml:
            base = os.path.splitext(args.map_yaml)[0]
            default_output = f"{base}_waypoints.yaml"
        else:
            default_output = "waypoints.yaml"
        manager.save_to_yaml(default_output)


if __name__ == '__main__':
    main()
