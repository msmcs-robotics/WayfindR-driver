#!/usr/bin/env python3
"""
ROS2 Map Converter Tool

Convert, resize, crop, and modify ROS2 occupancy grid maps.
Supports resolution changes, threshold adjustments, and map cropping.

Usage:
    python3 map_converter.py input.yaml --output output.yaml [options]
    python3 map_converter.py map.yaml --resize 0.1
    python3 map_converter.py map.yaml --crop 100 100 500 500
    python3 map_converter.py map.yaml --thresholds 0.2 0.7
"""

import sys
import os
import yaml
import numpy as np
from pathlib import Path
from PIL import Image
import argparse


class MapConverter:
    def __init__(self, input_yaml):
        self.input_yaml = Path(input_yaml)
        self.input_dir = self.input_yaml.parent
        self.map_data = None
        self.image = None
        self.image_array = None

    def load_map(self):
        """Load input map."""
        try:
            with open(self.input_yaml, 'r') as f:
                self.map_data = yaml.safe_load(f)
        except Exception as e:
            print(f"Error: Could not load YAML file: {e}")
            return False

        # Load image
        image_filename = self.map_data.get('image', '')
        if os.path.isabs(image_filename):
            image_path = Path(image_filename)
        else:
            image_path = self.input_dir / image_filename

        try:
            self.image = Image.open(image_path)
            self.image_array = np.array(self.image)
            print(f"Loaded map: {self.input_yaml.name}")
            print(f"  Size: {self.image.size[0]} x {self.image.size[1]} pixels")
            print(f"  Resolution: {self.map_data.get('resolution', 'N/A')} m/pixel")
        except Exception as e:
            print(f"Error: Could not load image file: {e}")
            return False

        return True

    def resize_map(self, new_resolution):
        """Resize map by changing resolution."""
        old_resolution = self.map_data.get('resolution', 0)
        if old_resolution == 0:
            print("Error: No resolution in original map")
            return False

        print(f"\nResizing map:")
        print(f"  Old resolution: {old_resolution} m/pixel")
        print(f"  New resolution: {new_resolution} m/pixel")

        # Calculate new dimensions
        scale_factor = old_resolution / new_resolution
        old_width, old_height = self.image.size
        new_width = int(old_width * scale_factor)
        new_height = int(old_height * scale_factor)

        print(f"  Old size: {old_width} x {old_height} pixels")
        print(f"  New size: {new_width} x {new_height} pixels")
        print(f"  Scale factor: {scale_factor:.3f}")

        # Resize image
        # Use NEAREST for occupancy grids to preserve discrete values
        self.image = self.image.resize((new_width, new_height), Image.Resampling.NEAREST)
        self.image_array = np.array(self.image)

        # Update metadata
        self.map_data['resolution'] = new_resolution

        print("  Resize complete")
        return True

    def crop_map(self, x1, y1, x2, y2):
        """Crop map to specified pixel coordinates."""
        width, height = self.image.size

        # Validate coordinates
        if x1 < 0 or y1 < 0 or x2 > width or y2 > height or x1 >= x2 or y1 >= y2:
            print(f"Error: Invalid crop coordinates")
            print(f"  Map size: {width} x {height}")
            print(f"  Requested: ({x1}, {y1}) to ({x2}, {y2})")
            return False

        print(f"\nCropping map:")
        print(f"  Original size: {width} x {height} pixels")
        print(f"  Crop region: ({x1}, {y1}) to ({x2}, {y2})")
        print(f"  New size: {x2-x1} x {y2-y1} pixels")

        # Crop image
        self.image = self.image.crop((x1, y1, x2, y2))
        self.image_array = np.array(self.image)

        # Update origin
        # In map frame: origin is at lower-left
        # In image frame: origin is at upper-left
        # So y-axis is inverted
        resolution = self.map_data.get('resolution', 0)
        if resolution > 0:
            old_origin = self.map_data.get('origin', [0, 0, 0])
            # Image y increases downward, map y increases upward
            new_origin_x = old_origin[0] + x1 * resolution
            new_origin_y = old_origin[1] + (height - y2) * resolution
            self.map_data['origin'] = [new_origin_x, new_origin_y, old_origin[2]]

            print(f"  Old origin: [{old_origin[0]:.3f}, {old_origin[1]:.3f}, {old_origin[2]:.3f}]")
            print(f"  New origin: [{new_origin_x:.3f}, {new_origin_y:.3f}, {old_origin[2]:.3f}]")

        print("  Crop complete")
        return True

    def adjust_thresholds(self, free_thresh, occupied_thresh):
        """Adjust occupancy thresholds."""
        # Validate thresholds
        if not (0 <= free_thresh <= 1) or not (0 <= occupied_thresh <= 1):
            print(f"Error: Thresholds must be in [0, 1]")
            return False

        if free_thresh >= occupied_thresh:
            print(f"Error: free_thresh must be < occupied_thresh")
            return False

        old_free = self.map_data.get('free_thresh', 0.25)
        old_occupied = self.map_data.get('occupied_thresh', 0.65)

        print(f"\nAdjusting thresholds:")
        print(f"  Old: free={old_free}, occupied={old_occupied}")
        print(f"  New: free={free_thresh}, occupied={occupied_thresh}")

        self.map_data['free_thresh'] = free_thresh
        self.map_data['occupied_thresh'] = occupied_thresh

        print("  Thresholds updated")
        return True

    def rotate_map(self, angle):
        """Rotate map by specified angle (90, 180, 270 degrees)."""
        if angle not in [90, 180, 270]:
            print("Error: Angle must be 90, 180, or 270 degrees")
            return False

        print(f"\nRotating map by {angle} degrees:")

        # Rotate image
        if angle == 90:
            self.image = self.image.rotate(90, expand=True)
        elif angle == 180:
            self.image = self.image.rotate(180, expand=True)
        elif angle == 270:
            self.image = self.image.rotate(270, expand=True)

        self.image_array = np.array(self.image)

        # Update origin and orientation
        # This is simplified - proper rotation of origin requires
        # knowing the map bounds and rotating around map center
        origin = self.map_data.get('origin', [0, 0, 0])
        new_theta = origin[2] + np.radians(angle)
        self.map_data['origin'][2] = new_theta

        print(f"  New orientation: {new_theta:.3f} rad ({np.degrees(new_theta):.1f}°)")
        print("  Rotation complete")
        print("  Note: Origin position may need manual adjustment")
        return True

    def flip_map(self, direction):
        """Flip map horizontally or vertically."""
        if direction not in ['horizontal', 'vertical']:
            print("Error: Direction must be 'horizontal' or 'vertical'")
            return False

        print(f"\nFlipping map {direction}:")

        if direction == 'horizontal':
            self.image = self.image.transpose(Image.FLIP_LEFT_RIGHT)
        else:
            self.image = self.image.transpose(Image.FLIP_TOP_BOTTOM)

        self.image_array = np.array(self.image)
        print("  Flip complete")
        print("  Note: Origin may need manual adjustment")
        return True

    def invert_colors(self):
        """Invert pixel colors."""
        print("\nInverting colors:")
        self.image_array = 255 - self.image_array
        self.image = Image.fromarray(self.image_array)

        # Toggle negate flag
        negate = self.map_data.get('negate', 0)
        self.map_data['negate'] = 1 - negate
        print(f"  Negate flag: {negate} -> {1-negate}")
        print("  Colors inverted")
        return True

    def save_map(self, output_yaml, output_image=None):
        """Save modified map to new files."""
        output_yaml = Path(output_yaml)
        output_dir = output_yaml.parent

        # Create output directory if needed
        output_dir.mkdir(parents=True, exist_ok=True)

        # Determine output image filename
        if output_image is None:
            output_image = output_yaml.stem + '.pgm'

        output_image_path = output_dir / output_image

        print(f"\nSaving map:")
        print(f"  YAML: {output_yaml}")
        print(f"  Image: {output_image_path}")

        try:
            # Save image
            self.image.save(output_image_path)

            # Update YAML with new image filename
            self.map_data['image'] = output_image

            # Save YAML
            with open(output_yaml, 'w') as f:
                yaml.dump(self.map_data, f, default_flow_style=False, sort_keys=False)

            print("  Save complete")
            return True

        except Exception as e:
            print(f"Error: Could not save map: {e}")
            return False

    def print_summary(self):
        """Print summary of current map state."""
        print(f"\n{'='*70}")
        print("CURRENT MAP STATE")
        print(f"{'='*70}")
        print(f"  Dimensions: {self.image.size[0]} x {self.image.size[1]} pixels")
        print(f"  Resolution: {self.map_data.get('resolution', 'N/A')} m/pixel")
        print(f"  Origin: {self.map_data.get('origin', 'N/A')}")
        print(f"  Thresholds: free={self.map_data.get('free_thresh', 'N/A')}, "
              f"occupied={self.map_data.get('occupied_thresh', 'N/A')}")
        print(f"  Negate: {self.map_data.get('negate', 0)}")
        print(f"{'='*70}\n")


def main():
    parser = argparse.ArgumentParser(
        description='Convert and modify ROS2 occupancy grid maps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Change resolution
  python3 map_converter.py input.yaml --output output.yaml --resize 0.1

  # Crop map
  python3 map_converter.py input.yaml --output output.yaml --crop 100 100 500 500

  # Adjust thresholds
  python3 map_converter.py input.yaml --output output.yaml --thresholds 0.2 0.7

  # Multiple operations
  python3 map_converter.py input.yaml --output output.yaml --resize 0.05 --crop 0 0 1000 1000

  # Rotate map
  python3 map_converter.py input.yaml --output output.yaml --rotate 90

  # Invert colors
  python3 map_converter.py input.yaml --output output.yaml --invert
        """
    )
    parser.add_argument('input_yaml', help='Input map YAML file')
    parser.add_argument('--output', '-o', required=True, help='Output map YAML file')
    parser.add_argument('--resize', type=float, metavar='RESOLUTION',
                        help='New resolution in m/pixel')
    parser.add_argument('--crop', nargs=4, type=int, metavar=('X1', 'Y1', 'X2', 'Y2'),
                        help='Crop to pixel coordinates (x1, y1, x2, y2)')
    parser.add_argument('--thresholds', nargs=2, type=float, metavar=('FREE', 'OCCUPIED'),
                        help='Set new free and occupied thresholds')
    parser.add_argument('--rotate', type=int, choices=[90, 180, 270],
                        help='Rotate map (90, 180, or 270 degrees)')
    parser.add_argument('--flip', choices=['horizontal', 'vertical'],
                        help='Flip map horizontally or vertically')
    parser.add_argument('--invert', action='store_true',
                        help='Invert pixel colors (toggle negate flag)')

    args = parser.parse_args()

    # Check that at least one operation is specified
    if not any([args.resize, args.crop, args.thresholds, args.rotate, args.flip, args.invert]):
        print("Error: No operations specified")
        print("Use --help to see available operations")
        sys.exit(1)

    converter = MapConverter(args.input_yaml)

    if not converter.load_map():
        sys.exit(1)

    # Perform operations in order
    success = True

    if args.resize:
        success = success and converter.resize_map(args.resize)

    if args.crop:
        success = success and converter.crop_map(*args.crop)

    if args.rotate:
        success = success and converter.rotate_map(args.rotate)

    if args.flip:
        success = success and converter.flip_map(args.flip)

    if args.thresholds:
        success = success and converter.adjust_thresholds(args.thresholds[0], args.thresholds[1])

    if args.invert:
        success = success and converter.invert_colors()

    if success:
        converter.print_summary()
        success = converter.save_map(args.output)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
