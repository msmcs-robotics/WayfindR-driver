#!/usr/bin/env python3
"""
ROS2 Map Validation Tool

Validates ROS2 occupancy grid maps (.pgm and .yaml files) for correctness.
Checks file format, resolution, origin, thresholds, and image dimensions.
Reports issues and suggests fixes.

Usage:
    python3 validate_map.py <map_yaml_file>
    python3 validate_map.py /path/to/map.yaml
"""

import sys
import os
import yaml
from pathlib import Path
from PIL import Image
import argparse


class MapValidator:
    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path)
        self.yaml_dir = self.yaml_path.parent
        self.errors = []
        self.warnings = []
        self.info = []
        self.map_data = None
        self.image = None

    def validate(self):
        """Run all validation checks."""
        print(f"\n{'='*70}")
        print(f"ROS2 Map Validation Tool")
        print(f"{'='*70}\n")
        print(f"Validating: {self.yaml_path}\n")

        # Check if YAML file exists
        if not self.yaml_path.exists():
            self.errors.append(f"YAML file does not exist: {self.yaml_path}")
            self.print_results()
            return False

        # Validate YAML file
        if not self.validate_yaml_file():
            self.print_results()
            return False

        # Validate PGM file
        if not self.validate_pgm_file():
            self.print_results()
            return False

        # Validate map parameters
        self.validate_resolution()
        self.validate_origin()
        self.validate_thresholds()
        self.validate_mode()
        self.validate_negate()

        # Validate image properties
        self.validate_image_dimensions()
        self.validate_image_format()

        # Print results
        self.print_results()

        return len(self.errors) == 0

    def validate_yaml_file(self):
        """Validate YAML file format and content."""
        try:
            with open(self.yaml_path, 'r') as f:
                self.map_data = yaml.safe_load(f)
            self.info.append(f"YAML file loaded successfully")
        except yaml.YAMLError as e:
            self.errors.append(f"Invalid YAML format: {e}")
            return False
        except Exception as e:
            self.errors.append(f"Error reading YAML file: {e}")
            return False

        # Check required fields
        required_fields = ['image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh']
        for field in required_fields:
            if field not in self.map_data:
                self.errors.append(f"Missing required field: '{field}'")

        return len(self.errors) == 0

    def validate_pgm_file(self):
        """Validate PGM file exists and can be loaded."""
        if 'image' not in self.map_data:
            return False

        image_filename = self.map_data['image']

        # Handle absolute and relative paths
        if os.path.isabs(image_filename):
            image_path = Path(image_filename)
        else:
            image_path = self.yaml_dir / image_filename

        if not image_path.exists():
            self.errors.append(f"Image file does not exist: {image_path}")
            self.errors.append(f"  Suggestion: Check that the 'image' field in YAML points to the correct file")
            return False

        try:
            self.image = Image.open(image_path)
            self.info.append(f"Image file loaded successfully: {image_path.name}")
            self.info.append(f"  Image size: {self.image.size[0]} x {self.image.size[1]} pixels")
            self.info.append(f"  Image mode: {self.image.mode}")
        except Exception as e:
            self.errors.append(f"Error loading image file: {e}")
            return False

        return True

    def validate_resolution(self):
        """Validate map resolution."""
        if 'resolution' not in self.map_data:
            return

        resolution = self.map_data['resolution']

        # Check type
        if not isinstance(resolution, (int, float)):
            self.errors.append(f"Resolution must be a number, got: {type(resolution).__name__}")
            return

        # Check value
        if resolution <= 0:
            self.errors.append(f"Resolution must be positive, got: {resolution}")
            return

        if resolution < 0.001:
            self.warnings.append(f"Resolution very small ({resolution}m/pixel) - map may be extremely large")
        elif resolution > 1.0:
            self.warnings.append(f"Resolution very large ({resolution}m/pixel) - map may be too coarse")
        else:
            self.info.append(f"Resolution: {resolution} m/pixel (OK)")

        # Calculate real-world dimensions if image is loaded
        if self.image:
            width_m = self.image.size[0] * resolution
            height_m = self.image.size[1] * resolution
            self.info.append(f"  Real-world size: {width_m:.2f}m x {height_m:.2f}m")

    def validate_origin(self):
        """Validate map origin."""
        if 'origin' not in self.map_data:
            return

        origin = self.map_data['origin']

        # Check type
        if not isinstance(origin, list):
            self.errors.append(f"Origin must be a list [x, y, theta], got: {type(origin).__name__}")
            return

        # Check length
        if len(origin) != 3:
            self.errors.append(f"Origin must have 3 values [x, y, theta], got {len(origin)} values")
            return

        # Check values are numbers
        for i, val in enumerate(origin):
            if not isinstance(val, (int, float)):
                self.errors.append(f"Origin[{i}] must be a number, got: {type(val).__name__}")
                return

        self.info.append(f"Origin: [{origin[0]}, {origin[1]}, {origin[2]}] (OK)")
        self.info.append(f"  Position: ({origin[0]:.2f}m, {origin[1]:.2f}m)")
        self.info.append(f"  Orientation: {origin[2]:.2f} rad ({origin[2] * 57.2958:.1f} deg)")

    def validate_thresholds(self):
        """Validate occupancy thresholds."""
        if 'occupied_thresh' not in self.map_data or 'free_thresh' not in self.map_data:
            return

        occupied_thresh = self.map_data['occupied_thresh']
        free_thresh = self.map_data['free_thresh']

        # Check types
        if not isinstance(occupied_thresh, (int, float)):
            self.errors.append(f"occupied_thresh must be a number, got: {type(occupied_thresh).__name__}")
            return
        if not isinstance(free_thresh, (int, float)):
            self.errors.append(f"free_thresh must be a number, got: {type(free_thresh).__name__}")
            return

        # Check ranges
        if not (0 <= occupied_thresh <= 1):
            self.errors.append(f"occupied_thresh must be in [0, 1], got: {occupied_thresh}")
        if not (0 <= free_thresh <= 1):
            self.errors.append(f"free_thresh must be in [0, 1], got: {free_thresh}")

        # Check relationship
        if free_thresh >= occupied_thresh:
            self.errors.append(f"free_thresh ({free_thresh}) must be < occupied_thresh ({occupied_thresh})")
            self.errors.append(f"  Suggestion: Typical values are free_thresh=0.25, occupied_thresh=0.65")
        else:
            self.info.append(f"Thresholds: free={free_thresh}, occupied={occupied_thresh} (OK)")

        # Check for common values
        if occupied_thresh == 0.65 and free_thresh == 0.25:
            self.info.append(f"  Using standard ROS2 threshold values")
        else:
            self.warnings.append(f"Non-standard threshold values (standard: free=0.25, occupied=0.65)")

    def validate_mode(self):
        """Validate map mode."""
        if 'mode' in self.map_data:
            mode = self.map_data['mode']
            valid_modes = ['trinary', 'scale', 'raw']

            if mode not in valid_modes:
                self.warnings.append(f"Unknown mode '{mode}', valid modes: {valid_modes}")
            else:
                self.info.append(f"Mode: {mode} (OK)")
        else:
            self.warnings.append("Mode not specified (will use default 'trinary')")

    def validate_negate(self):
        """Validate negate parameter."""
        if 'negate' in self.map_data:
            negate = self.map_data['negate']

            if not isinstance(negate, int) or negate not in [0, 1]:
                self.warnings.append(f"Negate should be 0 or 1, got: {negate}")
            else:
                self.info.append(f"Negate: {negate} (OK)")
                if negate == 1:
                    self.info.append(f"  Note: White pixels are occupied, black pixels are free")
                else:
                    self.info.append(f"  Note: Black pixels are occupied, white pixels are free")
        else:
            self.warnings.append("Negate not specified (will use default 0)")

    def validate_image_dimensions(self):
        """Validate image dimensions are reasonable."""
        if not self.image:
            return

        width, height = self.image.size

        if width > 10000 or height > 10000:
            self.warnings.append(f"Very large image ({width}x{height}) - may cause performance issues")
        elif width < 100 or height < 100:
            self.warnings.append(f"Very small image ({width}x{height}) - may lack detail")
        else:
            self.info.append(f"Image dimensions: {width}x{height} (OK)")

        # Check aspect ratio
        aspect_ratio = max(width, height) / min(width, height)
        if aspect_ratio > 10:
            self.warnings.append(f"Extreme aspect ratio ({aspect_ratio:.1f}:1) - map is very elongated")

    def validate_image_format(self):
        """Validate image format and color depth."""
        if not self.image:
            return

        # Check image mode
        if self.image.mode == 'L':
            self.info.append(f"Image format: Grayscale 8-bit (OK)")
        elif self.image.mode == '1':
            self.info.append(f"Image format: Binary (OK)")
        elif self.image.mode == 'RGB':
            self.warnings.append(f"Image is RGB - should be grayscale for occupancy grids")
        else:
            self.warnings.append(f"Unexpected image mode: {self.image.mode}")

        # Analyze pixel values
        pixels = list(self.image.getdata())
        unique_values = set(pixels)

        if len(unique_values) == 2:
            self.info.append(f"Image is binary (2 unique values)")
        elif len(unique_values) <= 10:
            self.info.append(f"Image has {len(unique_values)} unique values (quantized)")
        else:
            self.info.append(f"Image has {len(unique_values)} unique pixel values")

        # Count approximate cell types
        free_count = sum(1 for p in pixels if p > 250)
        occupied_count = sum(1 for p in pixels if p < 5)
        unknown_count = len(pixels) - free_count - occupied_count

        total = len(pixels)
        self.info.append(f"Approximate cell distribution:")
        self.info.append(f"  Free: {free_count:,} ({100*free_count/total:.1f}%)")
        self.info.append(f"  Occupied: {occupied_count:,} ({100*occupied_count/total:.1f}%)")
        self.info.append(f"  Unknown: {unknown_count:,} ({100*unknown_count/total:.1f}%)")

    def print_results(self):
        """Print validation results."""
        print(f"\n{'-'*70}")
        print("VALIDATION RESULTS")
        print(f"{'-'*70}\n")

        if self.errors:
            print(f"ERRORS ({len(self.errors)}):")
            for error in self.errors:
                print(f"  [ERROR] {error}")
            print()

        if self.warnings:
            print(f"WARNINGS ({len(self.warnings)}):")
            for warning in self.warnings:
                print(f"  [WARN]  {warning}")
            print()

        if self.info:
            print(f"INFORMATION ({len(self.info)}):")
            for info in self.info:
                print(f"  [INFO]  {info}")
            print()

        print(f"{'-'*70}")
        if self.errors:
            print("RESULT: VALIDATION FAILED")
            print(f"{len(self.errors)} error(s) must be fixed before using this map.")
        elif self.warnings:
            print("RESULT: VALIDATION PASSED WITH WARNINGS")
            print(f"Map is usable but has {len(self.warnings)} warning(s).")
        else:
            print("RESULT: VALIDATION PASSED")
            print("Map appears to be correctly formatted.")
        print(f"{'-'*70}\n")


def main():
    parser = argparse.ArgumentParser(
        description='Validate ROS2 occupancy grid map files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 validate_map.py map.yaml
  python3 validate_map.py /path/to/maps/office_map.yaml
  python3 validate_map.py ../maps/first_map.yaml
        """
    )
    parser.add_argument('yaml_file', help='Path to map YAML file')

    args = parser.parse_args()

    validator = MapValidator(args.yaml_file)
    success = validator.validate()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
