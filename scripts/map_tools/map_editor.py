#!/usr/bin/env python3
"""
Map Editor - Edit ROS2 occupancy grid maps and waypoints

This tool allows you to:
- Load existing maps (.pgm + .yaml)
- Add, remove, and edit waypoints
- Save waypoint files in YAML format
- Visualize maps with waypoints overlaid
- Export waypoints for ROS2 navigation

Usage:
    python3 map_editor.py --map-yaml /path/to/map.yaml
    python3 map_editor.py --map-yaml map.yaml --waypoints waypoints.yaml
    python3 map_editor.py --map-yaml map.yaml --visualize
"""

import argparse
import math
import os
import sys
from typing import List, Tuple, Dict, Optional

import yaml
import numpy as np
from PIL import Image, ImageDraw, ImageFont


class MapEditor:
    """Editor for ROS2 occupancy grid maps."""

    def __init__(self, map_yaml_path: str):
        """
        Initialize map editor.

        Args:
            map_yaml_path: Path to map YAML file
        """
        self.map_yaml_path = map_yaml_path
        self.map_data = None
        self.waypoints = []
        self.routes = {}
        self.metadata = {}

        # Load map
        self._load_map()

    def _load_map(self):
        """Load map YAML and PGM image."""
        # Load YAML metadata
        with open(self.map_yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)

        self.resolution = yaml_data['resolution']
        self.origin = yaml_data.get('origin', [0, 0, 0])
        self.negate = yaml_data.get('negate', 0)
        self.occupied_thresh = yaml_data.get('occupied_thresh', 0.65)
        self.free_thresh = yaml_data.get('free_thresh', 0.25)
        self.mode = yaml_data.get('mode', 'trinary')

        # Get image path (relative to YAML file)
        yaml_dir = os.path.dirname(os.path.abspath(self.map_yaml_path))
        image_file = yaml_data['image']
        self.image_path = os.path.join(yaml_dir, image_file)

        # Load PGM image
        print(f"Loading map image: {self.image_path}")
        self.map_image = Image.open(self.image_path)
        self.map_data = np.array(self.map_image)
        self.height, self.width = self.map_data.shape

        print(f"Map loaded: {self.width}x{self.height} pixels")
        print(f"Resolution: {self.resolution} m/pixel ({self.resolution*100:.1f} cm/pixel)")
        print(f"Size: {self.width*self.resolution:.2f}m x {self.height*self.resolution:.2f}m")
        print(f"Origin: {self.origin}")

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to pixel coordinates.

        Args:
            x: World X coordinate (meters)
            y: World Y coordinate (meters)

        Returns:
            (px, py): Pixel coordinates
        """
        px = int((x - self.origin[0]) / self.resolution)
        py = int(self.height - ((y - self.origin[1]) / self.resolution))
        return px, py

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """
        Convert pixel coordinates to world coordinates.

        Args:
            px: Pixel X coordinate
            py: Pixel Y coordinate

        Returns:
            (x, y): World coordinates in meters
        """
        x = self.origin[0] + (px * self.resolution)
        y = self.origin[1] + ((self.height - py) * self.resolution)
        return x, y

    def add_waypoint(self, name: str, x: float, y: float, yaw_degrees: float = 0.0,
                     description: str = "", tolerance_pos: float = 0.3,
                     tolerance_orient: float = 0.2) -> Dict:
        """
        Add a waypoint.

        Args:
            name: Waypoint name
            x: X coordinate (meters, world frame)
            y: Y coordinate (meters, world frame)
            yaw_degrees: Orientation in degrees
            description: Optional description
            tolerance_pos: Position tolerance (meters)
            tolerance_orient: Orientation tolerance (radians)

        Returns:
            Waypoint dictionary
        """
        # Check for duplicate
        for i, wp in enumerate(self.waypoints):
            if wp['name'] == name:
                print(f"Warning: Replacing existing waypoint '{name}'")
                self.waypoints.pop(i)
                break

        # Convert yaw to quaternion
        yaw_rad = math.radians(yaw_degrees)
        qz = math.sin(yaw_rad / 2)
        qw = math.cos(yaw_rad / 2)

        waypoint = {
            'name': name,
            'description': description,
            'position': {
                'x': round(x, 4),
                'y': round(y, 4),
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': round(qz, 6),
                'w': round(qw, 6)
            },
            'yaw_degrees': yaw_degrees,
            'tolerance': {
                'position': tolerance_pos,
                'orientation': tolerance_orient
            }
        }

        self.waypoints.append(waypoint)
        print(f"Added waypoint '{name}' at ({x:.3f}, {y:.3f}), yaw={yaw_degrees:.1f}°")

        return waypoint

    def remove_waypoint(self, name: str) -> bool:
        """
        Remove a waypoint by name.

        Args:
            name: Waypoint name

        Returns:
            True if removed, False if not found
        """
        for i, wp in enumerate(self.waypoints):
            if wp['name'] == name:
                self.waypoints.pop(i)
                print(f"Removed waypoint '{name}'")
                return True

        print(f"Waypoint '{name}' not found")
        return False

    def edit_waypoint(self, name: str, **kwargs) -> bool:
        """
        Edit an existing waypoint.

        Args:
            name: Waypoint name
            **kwargs: Fields to update (x, y, yaw_degrees, description, etc.)

        Returns:
            True if edited, False if not found
        """
        for wp in self.waypoints:
            if wp['name'] == name:
                # Update position
                if 'x' in kwargs:
                    wp['position']['x'] = round(kwargs['x'], 4)
                if 'y' in kwargs:
                    wp['position']['y'] = round(kwargs['y'], 4)
                if 'z' in kwargs:
                    wp['position']['z'] = round(kwargs['z'], 4)

                # Update orientation
                if 'yaw_degrees' in kwargs:
                    yaw_degrees = kwargs['yaw_degrees']
                    wp['yaw_degrees'] = yaw_degrees
                    yaw_rad = math.radians(yaw_degrees)
                    wp['orientation']['z'] = round(math.sin(yaw_rad / 2), 6)
                    wp['orientation']['w'] = round(math.cos(yaw_rad / 2), 6)

                # Update description
                if 'description' in kwargs:
                    wp['description'] = kwargs['description']

                # Update tolerances
                if 'tolerance_pos' in kwargs:
                    wp['tolerance']['position'] = kwargs['tolerance_pos']
                if 'tolerance_orient' in kwargs:
                    wp['tolerance']['orientation'] = kwargs['tolerance_orient']

                print(f"Updated waypoint '{name}'")
                return True

        print(f"Waypoint '{name}' not found")
        return False

    def add_route(self, route_name: str, waypoint_names: List[str]) -> None:
        """
        Add a named route (sequence of waypoints).

        Args:
            route_name: Name of the route
            waypoint_names: List of waypoint names in sequence
        """
        self.routes[route_name] = waypoint_names
        print(f"Added route '{route_name}' with {len(waypoint_names)} waypoints")

    def list_waypoints(self) -> None:
        """Print all waypoints."""
        if not self.waypoints:
            print("No waypoints defined.")
            return

        print(f"\n{'='*70}")
        print(f"Waypoints ({len(self.waypoints)} total)")
        print(f"{'='*70}")

        for i, wp in enumerate(self.waypoints, 1):
            print(f"\n{i}. {wp['name']}")
            pos = wp['position']
            print(f"   Position: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f}) meters")
            print(f"   Orientation: {wp['yaw_degrees']:.1f}°")
            orient = wp['orientation']
            print(f"   Quaternion: (z={orient['z']:.4f}, w={orient['w']:.4f})")
            if wp['description']:
                print(f"   Description: {wp['description']}")

            # Show pixel coordinates too
            px, py = self.world_to_pixel(pos['x'], pos['y'])
            print(f"   Pixel coords: ({px}, {py})")

    def load_waypoints(self, waypoint_yaml_path: str) -> None:
        """
        Load waypoints from YAML file.

        Args:
            waypoint_yaml_path: Path to waypoints YAML file
        """
        with open(waypoint_yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.metadata = data.get('metadata', {})
        self.routes = data.get('routes', {})
        self.waypoints = data.get('waypoints', [])

        print(f"Loaded {len(self.waypoints)} waypoints from {waypoint_yaml_path}")

    def save_waypoints(self, output_path: str) -> None:
        """
        Save waypoints to YAML file.

        Args:
            output_path: Path for output YAML file
        """
        data = {
            'metadata': {
                'created_by': 'map_editor.py',
                'map_yaml': self.map_yaml_path,
                'frame_id': 'map',
                'map_resolution': self.resolution,
                'map_size': {
                    'width_pixels': self.width,
                    'height_pixels': self.height,
                    'width_meters': round(self.width * self.resolution, 3),
                    'height_meters': round(self.height * self.resolution, 3)
                }
            },
            'waypoints': self.waypoints
        }

        if self.routes:
            data['routes'] = self.routes

        # Ensure directory exists
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        print(f"\nSaved {len(self.waypoints)} waypoints to {output_path}")

    def visualize(self, output_path: Optional[str] = None, show: bool = True) -> None:
        """
        Visualize map with waypoints.

        Args:
            output_path: Optional path to save visualization
            show: Whether to display the image
        """
        # Convert to RGB for visualization
        if self.map_image.mode == 'L':
            vis_image = self.map_image.convert('RGB')
        else:
            vis_image = self.map_image.copy()

        draw = ImageDraw.Draw(vis_image)

        # Try to load a font
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
            small_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
        except:
            font = ImageFont.load_default()
            small_font = font

        # Draw waypoints
        for i, wp in enumerate(self.waypoints, 1):
            pos = wp['position']
            px, py = self.world_to_pixel(pos['x'], pos['y'])

            # Skip if out of bounds
            if not (0 <= px < self.width and 0 <= py < self.height):
                print(f"Warning: Waypoint '{wp['name']}' is out of map bounds")
                continue

            # Draw waypoint marker (circle)
            radius = 8
            draw.ellipse([px-radius, py-radius, px+radius, py+radius],
                        fill='red', outline='darkred', width=2)

            # Draw orientation arrow
            yaw_rad = math.radians(wp['yaw_degrees'])
            arrow_len = 20
            end_x = px + arrow_len * math.cos(yaw_rad)
            end_y = py - arrow_len * math.sin(yaw_rad)  # Negative because y is inverted
            draw.line([px, py, end_x, end_y], fill='blue', width=3)

            # Draw arrowhead
            arrow_angle = 25  # degrees
            head_len = 8
            left_angle = yaw_rad + math.radians(180 - arrow_angle)
            right_angle = yaw_rad + math.radians(180 + arrow_angle)
            left_x = end_x + head_len * math.cos(left_angle)
            left_y = end_y - head_len * math.sin(left_angle)
            right_x = end_x + head_len * math.cos(right_angle)
            right_y = end_y - head_len * math.sin(right_angle)
            draw.line([end_x, end_y, left_x, left_y], fill='blue', width=3)
            draw.line([end_x, end_y, right_x, right_y], fill='blue', width=3)

            # Draw label
            label = f"{i}. {wp['name']}"
            label_x = px + radius + 5
            label_y = py - radius - 5

            # Draw text background
            bbox = draw.textbbox((label_x, label_y), label, font=small_font)
            draw.rectangle(bbox, fill='white', outline='black')
            draw.text((label_x, label_y), label, fill='black', font=small_font)

        # Save if requested
        if output_path:
            vis_image.save(output_path)
            print(f"Saved visualization to {output_path}")

        # Show if requested
        if show:
            vis_image.show()

    def get_map_center(self) -> Tuple[float, float]:
        """
        Get the center of the map in world coordinates.

        Returns:
            (x, y): Center coordinates in meters
        """
        center_px = self.width // 2
        center_py = self.height // 2
        return self.pixel_to_world(center_px, center_py)

    def get_map_bounds(self) -> Dict[str, float]:
        """
        Get map bounds in world coordinates.

        Returns:
            Dictionary with min_x, max_x, min_y, max_y
        """
        min_x, max_y = self.pixel_to_world(0, 0)
        max_x, min_y = self.pixel_to_world(self.width - 1, self.height - 1)

        return {
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y
        }


def main():
    parser = argparse.ArgumentParser(
        description='Map Editor - Edit ROS2 maps and waypoints',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Load a map and show info
  python3 map_editor.py --map-yaml maps/first_map.yaml

  # Load map and waypoints
  python3 map_editor.py --map-yaml maps/first_map.yaml --waypoints waypoints.yaml

  # Add a waypoint
  python3 map_editor.py --map-yaml maps/first_map.yaml \\
      --add "kitchen" 2.5 1.0 90 --output waypoints.yaml

  # Visualize map with waypoints
  python3 map_editor.py --map-yaml maps/first_map.yaml \\
      --waypoints waypoints.yaml --visualize

  # Remove a waypoint
  python3 map_editor.py --waypoints waypoints.yaml \\
      --remove "old_waypoint" --output waypoints.yaml
        """
    )

    parser.add_argument('--map-yaml', '-m', type=str, required=True,
                        help='Path to map YAML file')
    parser.add_argument('--waypoints', '-w', type=str,
                        help='Path to waypoints YAML file to load')
    parser.add_argument('--output', '-o', type=str,
                        help='Output path for waypoints YAML file')

    parser.add_argument('--add', nargs='+', metavar='ARGS',
                        help='Add waypoint: name x y yaw [description]')
    parser.add_argument('--remove', type=str, metavar='NAME',
                        help='Remove waypoint by name')
    parser.add_argument('--edit', nargs='+', metavar='ARGS',
                        help='Edit waypoint: name [x=X] [y=Y] [yaw=YAW] [desc="..."]')

    parser.add_argument('--list', '-l', action='store_true',
                        help='List all waypoints')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Visualize map with waypoints')
    parser.add_argument('--viz-output', type=str,
                        help='Save visualization to image file')

    args = parser.parse_args()

    # Load map
    try:
        editor = MapEditor(args.map_yaml)
    except Exception as e:
        print(f"Error loading map: {e}")
        sys.exit(1)

    # Load waypoints if specified
    if args.waypoints:
        try:
            editor.load_waypoints(args.waypoints)
        except Exception as e:
            print(f"Error loading waypoints: {e}")
            sys.exit(1)

    # Add waypoint
    if args.add:
        if len(args.add) < 4:
            print("Error: --add requires at least 4 arguments: name x y yaw")
            sys.exit(1)
        name = args.add[0]
        x = float(args.add[1])
        y = float(args.add[2])
        yaw = float(args.add[3])
        description = ' '.join(args.add[4:]) if len(args.add) > 4 else ""
        editor.add_waypoint(name, x, y, yaw, description)

    # Remove waypoint
    if args.remove:
        editor.remove_waypoint(args.remove)

    # Edit waypoint
    if args.edit:
        if len(args.edit) < 2:
            print("Error: --edit requires at least 2 arguments: name field=value ...")
            sys.exit(1)
        name = args.edit[0]
        kwargs = {}
        for arg in args.edit[1:]:
            if '=' in arg:
                key, value = arg.split('=', 1)
                if key in ['x', 'y', 'z', 'yaw_degrees', 'tolerance_pos', 'tolerance_orient']:
                    kwargs[key] = float(value)
                elif key in ['desc', 'description']:
                    kwargs['description'] = value.strip('"').strip("'")
        editor.edit_waypoint(name, **kwargs)

    # List waypoints
    if args.list or (not args.visualize and not args.output):
        editor.list_waypoints()

    # Visualize
    if args.visualize:
        editor.visualize(output_path=args.viz_output, show=True)

    # Save waypoints
    if args.output:
        editor.save_waypoints(args.output)
    elif args.add or args.remove or args.edit:
        # Auto-save if modifications were made
        if args.waypoints:
            editor.save_waypoints(args.waypoints)
        else:
            # Generate default output path
            base = os.path.splitext(args.map_yaml)[0]
            default_output = f"{base}_waypoints.yaml"
            editor.save_waypoints(default_output)


if __name__ == '__main__':
    main()
