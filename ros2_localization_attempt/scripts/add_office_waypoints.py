#!/usr/bin/env python3
"""
Add Office Waypoints to Map

This script adds three office waypoints (office1, office2, office3) to the
existing map. Waypoints are placed in free space based on map analysis.

Usage:
    python3 add_office_waypoints.py

This will:
1. Load the existing first_map.yaml
2. Analyze the map to find free space
3. Add office1, office2, office3 waypoints
4. Save to first_map_with_offices.yaml (waypoints file)
"""

import os
import sys
import yaml
import math

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_pgm(filepath):
    """Load PGM image and return as 2D list of values (0-255)."""
    with open(filepath, 'rb') as f:
        # Read header
        magic = f.readline().decode().strip()
        if magic != 'P5':
            raise ValueError(f"Not a binary PGM file: {magic}")

        # Skip comments
        line = f.readline().decode().strip()
        while line.startswith('#'):
            line = f.readline().decode().strip()

        # Get dimensions
        width, height = map(int, line.split())
        maxval = int(f.readline().decode().strip())

        # Read pixel data
        pixels = []
        for y in range(height):
            row = []
            for x in range(width):
                row.append(ord(f.read(1)))
            pixels.append(row)

        return pixels, width, height


def is_free_space(pixels, px, py, radius=3):
    """
    Check if a pixel and its neighbors are free space.
    In ROS maps: 254 = free, 0 = occupied, 205 = unknown
    """
    height = len(pixels)
    width = len(pixels[0])

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx, ny = px + dx, py + dy
            if 0 <= nx < width and 0 <= ny < height:
                # Free space is typically 254 (white)
                if pixels[ny][nx] < 250:  # Not clearly free
                    return False
    return True


def pixel_to_world(px, py, origin, resolution, height_pixels):
    """Convert pixel coordinates to world coordinates."""
    world_x = origin[0] + (px * resolution)
    world_y = origin[1] + ((height_pixels - py) * resolution)
    return world_x, world_y


def world_to_pixel(wx, wy, origin, resolution, height_pixels):
    """Convert world coordinates to pixel coordinates."""
    px = int((wx - origin[0]) / resolution)
    py = int(height_pixels - ((wy - origin[1]) / resolution))
    return px, py


def find_free_positions(pixels, origin, resolution, count=3):
    """
    Find 'count' well-distributed free positions in the map.
    Returns list of (world_x, world_y) tuples.
    """
    height = len(pixels)
    width = len(pixels[0])

    # Find all free pixels (with some margin from obstacles)
    free_pixels = []
    margin = 10  # Pixels from edge

    for y in range(margin, height - margin):
        for x in range(margin, width - margin):
            if is_free_space(pixels, x, y, radius=5):
                free_pixels.append((x, y))

    if len(free_pixels) < count:
        print(f"Warning: Only found {len(free_pixels)} free positions")
        return []

    # Divide map into regions and pick one point from each
    positions = []

    # Calculate map center in pixels
    center_px = width // 2
    center_py = height // 2

    # Define three regions: left, center-right, right
    # These correspond to office1, office2, office3
    regions = [
        (margin, width // 3),           # Left third (office1)
        (width // 3, 2 * width // 3),   # Middle third (office2)
        (2 * width // 3, width - margin) # Right third (office3)
    ]

    for region_start, region_end in regions:
        # Find free pixels in this region
        region_pixels = [(x, y) for x, y in free_pixels
                        if region_start <= x < region_end]

        if region_pixels:
            # Pick the one closest to vertical center
            best = min(region_pixels, key=lambda p: abs(p[1] - center_py))
            px, py = best
            wx, wy = pixel_to_world(px, py, origin, resolution, height)
            positions.append((wx, wy))

    return positions


def create_waypoint(name, x, y, yaw_degrees=0, description=""):
    """Create a waypoint dictionary."""
    yaw_rad = math.radians(yaw_degrees)
    return {
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
            'z': round(math.sin(yaw_rad / 2), 6),
            'w': round(math.cos(yaw_rad / 2), 6)
        },
        'yaw_degrees': yaw_degrees,
        'tolerance': {
            'position': 0.3,
            'orientation': 0.2
        }
    }


def main():
    # Paths - adjust for your system
    map_yaml = os.path.expanduser("~/ros2_ws/maps/first_map.yaml")

    # Check if running on remote or local
    if not os.path.exists(map_yaml):
        # Try local path
        map_yaml = "/home/devel/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml"

    if not os.path.exists(map_yaml):
        print(f"Error: Map file not found at {map_yaml}")
        print("Please run this on the system with the map files.")
        sys.exit(1)

    # Load map metadata
    with open(map_yaml, 'r') as f:
        map_data = yaml.safe_load(f)

    map_dir = os.path.dirname(map_yaml)
    pgm_path = os.path.join(map_dir, map_data['image'])
    resolution = map_data['resolution']
    origin = map_data['origin']

    print(f"Map: {map_yaml}")
    print(f"Image: {pgm_path}")
    print(f"Resolution: {resolution} m/pixel")
    print(f"Origin: {origin}")

    # Load map image
    pixels, width, height = load_pgm(pgm_path)
    print(f"Dimensions: {width} x {height} pixels")
    print(f"           ({width * resolution:.2f} x {height * resolution:.2f} meters)")

    # Find free positions for waypoints
    print("\nFinding free space for waypoints...")
    positions = find_free_positions(pixels, origin, resolution, count=3)

    if len(positions) < 3:
        print("Could not find enough free space. Using default positions.")
        # Fallback to map-relative positions
        map_center_x = origin[0] + (width * resolution) / 2
        map_center_y = origin[1] + (height * resolution) / 2

        positions = [
            (map_center_x - 2.0, map_center_y),      # Left of center
            (map_center_x, map_center_y + 1.0),      # Above center
            (map_center_x + 2.0, map_center_y),      # Right of center
        ]

    # Create waypoints
    waypoints = []
    office_names = ['office1', 'office2', 'office3']
    office_descriptions = [
        'First office - left side of map',
        'Second office - center of map',
        'Third office - right side of map'
    ]
    # Face towards center for each office
    yaw_angles = [0, 0, 180]  # office1 faces right, office2 faces right, office3 faces left

    print("\nCreating waypoints:")
    for i, (x, y) in enumerate(positions):
        wp = create_waypoint(
            name=office_names[i],
            x=x,
            y=y,
            yaw_degrees=yaw_angles[i],
            description=office_descriptions[i]
        )
        waypoints.append(wp)
        print(f"  {office_names[i]}: ({x:.3f}, {y:.3f}) @ {yaw_angles[i]}°")

    # Create output data
    output_data = {
        'metadata': {
            'created_by': 'add_office_waypoints.py',
            'map_yaml': 'first_map.yaml',
            'frame_id': 'map',
            'description': 'Office waypoints for navigation testing'
        },
        'waypoints': waypoints,
        'routes': {
            'office_tour': ['office1', 'office2', 'office3'],
            'office_tour_reverse': ['office3', 'office2', 'office1'],
            'office_patrol': ['office1', 'office2', 'office3', 'office2']
        }
    }

    # Save waypoints file
    output_path = os.path.join(map_dir, 'first_map_offices.yaml')
    with open(output_path, 'w') as f:
        yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)

    print(f"\nSaved waypoints to: {output_path}")
    print(f"\nRoutes defined:")
    for route_name, route_wps in output_data['routes'].items():
        print(f"  {route_name}: {' -> '.join(route_wps)}")

    print("\nTo use these waypoints:")
    print(f"  1. Start localization: ~/start_localization.sh")
    print(f"  2. Set initial pose in RViz")
    print(f"  3. Run navigation: python3 navigate_to_office.py office1")


if __name__ == '__main__':
    main()
