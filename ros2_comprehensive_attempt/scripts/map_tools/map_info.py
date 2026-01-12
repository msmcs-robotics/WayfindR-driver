#!/usr/bin/env python3
"""
ROS2 Map Information Display Tool

Displays detailed information about ROS2 occupancy grid maps.
Shows metadata, dimensions, statistics, and coordinate system information.

Usage:
    python3 map_info.py <map_yaml_file>
    python3 map_info.py /path/to/map.yaml
    python3 map_info.py --detailed map.yaml
"""

import sys
import os
import yaml
import numpy as np
from pathlib import Path
from PIL import Image
import argparse


class MapInfo:
    def __init__(self, yaml_path, detailed=False):
        self.yaml_path = Path(yaml_path)
        self.yaml_dir = self.yaml_path.parent
        self.detailed = detailed
        self.map_data = None
        self.image = None
        self.image_array = None

    def load_map(self):
        """Load map YAML and image files."""
        try:
            with open(self.yaml_path, 'r') as f:
                self.map_data = yaml.safe_load(f)
        except Exception as e:
            print(f"Error: Could not load YAML file: {e}")
            return False

        # Load image
        image_filename = self.map_data.get('image', '')
        if os.path.isabs(image_filename):
            image_path = Path(image_filename)
        else:
            image_path = self.yaml_dir / image_filename

        try:
            self.image = Image.open(image_path)
            self.image_array = np.array(self.image)
        except Exception as e:
            print(f"Error: Could not load image file: {e}")
            return False

        return True

    def display_info(self):
        """Display all map information."""
        print(f"\n{'='*70}")
        print(f"ROS2 Map Information")
        print(f"{'='*70}\n")

        self.display_file_info()
        self.display_map_metadata()
        self.display_dimensions()
        self.display_coordinate_system()
        self.display_cell_statistics()

        if self.detailed:
            self.display_detailed_statistics()
            self.display_histogram()

        print(f"{'='*70}\n")

    def display_file_info(self):
        """Display file information."""
        print("FILE INFORMATION")
        print("-" * 70)
        print(f"  YAML file: {self.yaml_path.name}")
        print(f"  Directory: {self.yaml_path.parent}")
        print(f"  Image file: {self.map_data.get('image', 'N/A')}")

        # File sizes
        yaml_size = self.yaml_path.stat().st_size
        image_path = self.yaml_dir / self.map_data.get('image', '')
        if image_path.exists():
            image_size = image_path.stat().st_size
            print(f"  YAML size: {yaml_size:,} bytes")
            print(f"  Image size: {image_size:,} bytes ({image_size/1024:.1f} KB)")
        print()

    def display_map_metadata(self):
        """Display map metadata from YAML."""
        print("MAP METADATA")
        print("-" * 70)
        print(f"  Resolution: {self.map_data.get('resolution', 'N/A')} m/pixel")
        print(f"  Origin: {self.map_data.get('origin', 'N/A')}")
        print(f"  Mode: {self.map_data.get('mode', 'trinary (default)')}")
        print(f"  Negate: {self.map_data.get('negate', 0)}")
        print(f"  Occupied threshold: {self.map_data.get('occupied_thresh', 'N/A')}")
        print(f"  Free threshold: {self.map_data.get('free_thresh', 'N/A')}")
        print()

    def display_dimensions(self):
        """Display map dimensions and real-world size."""
        print("DIMENSIONS")
        print("-" * 70)

        width, height = self.image.size
        resolution = self.map_data.get('resolution', 0)

        print(f"  Image dimensions: {width} x {height} pixels")
        print(f"  Total pixels: {width * height:,}")

        if resolution:
            width_m = width * resolution
            height_m = height * resolution
            area_m2 = width_m * height_m

            print(f"  Real-world width: {width_m:.2f} m ({width_m * 3.28084:.2f} ft)")
            print(f"  Real-world height: {height_m:.2f} m ({height_m * 3.28084:.2f} ft)")
            print(f"  Real-world area: {area_m2:.2f} m² ({area_m2 * 10.7639:.2f} ft²)")

            # Aspect ratio
            aspect = max(width, height) / min(width, height)
            print(f"  Aspect ratio: {aspect:.2f}:1")
        print()

    def display_coordinate_system(self):
        """Display coordinate system information."""
        print("COORDINATE SYSTEM")
        print("-" * 70)

        origin = self.map_data.get('origin', [0, 0, 0])
        resolution = self.map_data.get('resolution', 0)
        width, height = self.image.size

        print(f"  Origin (lower-left): ({origin[0]:.3f}, {origin[1]:.3f}) m")
        print(f"  Origin orientation: {origin[2]:.3f} rad ({origin[2] * 57.2958:.1f}°)")

        if resolution:
            # Calculate corner coordinates
            max_x = origin[0] + width * resolution
            max_y = origin[1] + height * resolution

            print(f"  Lower-left corner: ({origin[0]:.3f}, {origin[1]:.3f}) m")
            print(f"  Lower-right corner: ({max_x:.3f}, {origin[1]:.3f}) m")
            print(f"  Upper-left corner: ({origin[0]:.3f}, {max_y:.3f}) m")
            print(f"  Upper-right corner: ({max_x:.3f}, {max_y:.3f}) m")
            print(f"  Center: ({(origin[0] + max_x)/2:.3f}, {(origin[1] + max_y)/2:.3f}) m")
        print()

    def display_cell_statistics(self):
        """Display occupancy cell statistics."""
        print("CELL STATISTICS")
        print("-" * 70)

        # Analyze pixels
        pixels = self.image_array.flatten()
        total = len(pixels)

        # Thresholds
        occupied_thresh = self.map_data.get('occupied_thresh', 0.65)
        free_thresh = self.map_data.get('free_thresh', 0.25)
        negate = self.map_data.get('negate', 0)

        # Convert pixel values to probabilities (0-255 -> 0-1)
        # In standard PGM: 255=free, 0=occupied, 205=unknown
        if negate == 0:
            # Standard: white is free, black is occupied
            free_cells = np.sum(pixels > 254)
            occupied_cells = np.sum(pixels < 1)
            unknown_cells = total - free_cells - occupied_cells
        else:
            # Inverted: black is free, white is occupied
            free_cells = np.sum(pixels < 1)
            occupied_cells = np.sum(pixels > 254)
            unknown_cells = total - free_cells - occupied_cells

        print(f"  Total cells: {total:,}")
        print(f"  Free cells: {free_cells:,} ({100*free_cells/total:.2f}%)")
        print(f"  Occupied cells: {occupied_cells:,} ({100*occupied_cells/total:.2f}%)")
        print(f"  Unknown cells: {unknown_cells:,} ({100*unknown_cells/total:.2f}%)")

        # Calculate physical areas
        resolution = self.map_data.get('resolution', 0)
        if resolution:
            cell_area = resolution * resolution
            free_area = free_cells * cell_area
            occupied_area = occupied_cells * cell_area
            unknown_area = unknown_cells * cell_area

            print(f"\n  Physical areas (resolution = {resolution} m/pixel):")
            print(f"    Free area: {free_area:.2f} m² ({free_area * 10.7639:.2f} ft²)")
            print(f"    Occupied area: {occupied_area:.2f} m² ({occupied_area * 10.7639:.2f} ft²)")
            print(f"    Unknown area: {unknown_area:.2f} m² ({unknown_area * 10.7639:.2f} ft²)")

        # Pixel value statistics
        print(f"\n  Pixel value statistics:")
        print(f"    Min: {np.min(pixels)}")
        print(f"    Max: {np.max(pixels)}")
        print(f"    Mean: {np.mean(pixels):.2f}")
        print(f"    Median: {np.median(pixels):.2f}")
        print(f"    Std dev: {np.std(pixels):.2f}")
        print(f"    Unique values: {len(np.unique(pixels))}")
        print()

    def display_detailed_statistics(self):
        """Display detailed statistics (when --detailed flag is used)."""
        print("DETAILED STATISTICS")
        print("-" * 70)

        pixels = self.image_array.flatten()

        # Percentiles
        percentiles = [0, 10, 25, 50, 75, 90, 100]
        print("  Pixel value percentiles:")
        for p in percentiles:
            val = np.percentile(pixels, p)
            print(f"    {p:3d}%: {val:6.1f}")

        # Find contiguous regions (simple connected components)
        print("\n  Map complexity:")
        # Count transitions (simple measure of complexity)
        h_transitions = np.sum(np.abs(np.diff(self.image_array, axis=1)) > 0)
        v_transitions = np.sum(np.abs(np.diff(self.image_array, axis=0)) > 0)
        total_transitions = h_transitions + v_transitions
        max_transitions = 2 * self.image_array.size
        complexity = 100 * total_transitions / max_transitions
        print(f"    Edge density: {complexity:.2f}%")
        print(f"    Horizontal transitions: {h_transitions:,}")
        print(f"    Vertical transitions: {v_transitions:,}")
        print()

    def display_histogram(self):
        """Display pixel value histogram."""
        print("PIXEL VALUE HISTOGRAM")
        print("-" * 70)

        pixels = self.image_array.flatten()
        bins = [0, 50, 100, 150, 200, 250, 256]

        print("  Value range    Count         Percentage  Bar")
        print("  " + "-" * 60)

        for i in range(len(bins) - 1):
            count = np.sum((pixels >= bins[i]) & (pixels < bins[i+1]))
            percent = 100 * count / len(pixels)
            bar_length = int(percent / 2)  # Scale to 50 chars max
            bar = '█' * bar_length

            print(f"  {bins[i]:3d}-{bins[i+1]:3d}      {count:8,}      {percent:6.2f}%    {bar}")
        print()


def main():
    parser = argparse.ArgumentParser(
        description='Display information about ROS2 occupancy grid map',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 map_info.py map.yaml
  python3 map_info.py --detailed /path/to/office_map.yaml
  python3 map_info.py -d ../maps/first_map.yaml
        """
    )
    parser.add_argument('yaml_file', help='Path to map YAML file')
    parser.add_argument('-d', '--detailed', action='store_true',
                        help='Show detailed statistics and histogram')

    args = parser.parse_args()

    map_info = MapInfo(args.yaml_file, detailed=args.detailed)

    if not map_info.load_map():
        sys.exit(1)

    map_info.display_info()


if __name__ == '__main__':
    main()
