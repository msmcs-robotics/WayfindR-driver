#!/usr/bin/env python3
"""
ROS2 Map Comparison Tool

Compare two ROS2 occupancy grid maps to assess SLAM quality,
detect changes, or analyze differences between maps.

Usage:
    python3 compare_maps.py map1.yaml map2.yaml
    python3 compare_maps.py --output diff.png map1.yaml map2.yaml
    python3 compare_maps.py --overlay overlay.png map1.yaml map2.yaml
"""

import sys
import os
import yaml
import numpy as np
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import argparse


class MapComparer:
    def __init__(self, yaml1, yaml2):
        self.yaml1 = Path(yaml1)
        self.yaml2 = Path(yaml2)
        self.map1_data = None
        self.map2_data = None
        self.image1 = None
        self.image2 = None
        self.array1 = None
        self.array2 = None

    def load_maps(self):
        """Load both maps."""
        print("Loading maps...")

        # Load map 1
        try:
            with open(self.yaml1, 'r') as f:
                self.map1_data = yaml.safe_load(f)

            image1_path = self.yaml1.parent / self.map1_data['image']
            self.image1 = Image.open(image1_path)
            self.array1 = np.array(self.image1)
            print(f"  Map 1: {self.yaml1.name} ({self.image1.size[0]}x{self.image1.size[1]})")
        except Exception as e:
            print(f"Error loading map 1: {e}")
            return False

        # Load map 2
        try:
            with open(self.yaml2, 'r') as f:
                self.map2_data = yaml.safe_load(f)

            image2_path = self.yaml2.parent / self.map2_data['image']
            self.image2 = Image.open(image2_path)
            self.array2 = np.array(self.image2)
            print(f"  Map 2: {self.yaml2.name} ({self.image2.size[0]}x{self.image2.size[1]})")
        except Exception as e:
            print(f"Error loading map 2: {e}")
            return False

        return True

    def compare_metadata(self):
        """Compare map metadata."""
        print(f"\n{'='*70}")
        print("METADATA COMPARISON")
        print(f"{'='*70}\n")

        # Resolution
        res1 = self.map1_data.get('resolution', None)
        res2 = self.map2_data.get('resolution', None)
        print(f"Resolution:")
        print(f"  Map 1: {res1}")
        print(f"  Map 2: {res2}")
        if res1 != res2:
            print(f"  WARNING: Resolutions differ!")
        else:
            print(f"  Status: Match")

        # Dimensions
        size1 = self.image1.size
        size2 = self.image2.size
        print(f"\nDimensions:")
        print(f"  Map 1: {size1[0]} x {size1[1]} pixels")
        print(f"  Map 2: {size2[0]} x {size2[1]} pixels")
        if size1 != size2:
            print(f"  WARNING: Dimensions differ!")
        else:
            print(f"  Status: Match")

        # Origin
        origin1 = self.map1_data.get('origin', None)
        origin2 = self.map2_data.get('origin', None)
        print(f"\nOrigin:")
        print(f"  Map 1: {origin1}")
        print(f"  Map 2: {origin2}")
        if origin1 != origin2:
            print(f"  WARNING: Origins differ!")
        else:
            print(f"  Status: Match")

        # Thresholds
        print(f"\nThresholds:")
        print(f"  Map 1: free={self.map1_data.get('free_thresh')}, "
              f"occupied={self.map1_data.get('occupied_thresh')}")
        print(f"  Map 2: free={self.map2_data.get('free_thresh')}, "
              f"occupied={self.map2_data.get('occupied_thresh')}")

        # Real-world size
        if res1 and res1 == res2:
            width_m = size1[0] * res1
            height_m = size1[1] * res1
            print(f"\nReal-world size: {width_m:.2f}m x {height_m:.2f}m")

        print()

    def compare_pixels(self):
        """Compare pixel values statistically."""
        print(f"{'='*70}")
        print("PIXEL COMPARISON")
        print(f"{'='*70}\n")

        # Check if sizes match
        if self.array1.shape != self.array2.shape:
            print("ERROR: Map dimensions don't match - cannot compare pixels")
            print("  Consider resizing maps to same dimensions first")
            return False

        # Calculate differences
        diff = self.array1.astype(np.int16) - self.array2.astype(np.int16)
        abs_diff = np.abs(diff)

        # Statistics
        total_pixels = diff.size
        identical_pixels = np.sum(diff == 0)
        changed_pixels = total_pixels - identical_pixels

        print(f"Pixel Statistics:")
        print(f"  Total pixels: {total_pixels:,}")
        print(f"  Identical pixels: {identical_pixels:,} ({100*identical_pixels/total_pixels:.2f}%)")
        print(f"  Changed pixels: {changed_pixels:,} ({100*changed_pixels/total_pixels:.2f}%)")

        # Difference statistics
        print(f"\nDifference Statistics:")
        print(f"  Mean absolute difference: {np.mean(abs_diff):.2f}")
        print(f"  Max absolute difference: {np.max(abs_diff)}")
        print(f"  RMS difference: {np.sqrt(np.mean(diff**2)):.2f}")

        # Categorize changes
        small_changes = np.sum((abs_diff > 0) & (abs_diff <= 50))
        medium_changes = np.sum((abs_diff > 50) & (abs_diff <= 150))
        large_changes = np.sum(abs_diff > 150)

        print(f"\nChange Magnitude:")
        print(f"  Small (1-50): {small_changes:,} ({100*small_changes/total_pixels:.2f}%)")
        print(f"  Medium (51-150): {medium_changes:,} ({100*medium_changes/total_pixels:.2f}%)")
        print(f"  Large (151-255): {large_changes:,} ({100*large_changes/total_pixels:.2f}%)")

        # Occupancy changes
        print(f"\nOccupancy Changes:")

        # Free (255) to Occupied (0)
        free_to_occ = np.sum((self.array1 > 250) & (self.array2 < 5))
        print(f"  Free -> Occupied: {free_to_occ:,} ({100*free_to_occ/total_pixels:.2f}%)")

        # Occupied (0) to Free (255)
        occ_to_free = np.sum((self.array1 < 5) & (self.array2 > 250))
        print(f"  Occupied -> Free: {occ_to_free:,} ({100*occ_to_free/total_pixels:.2f}%)")

        # Unknown (205) changes
        unknown1 = np.sum((self.array1 > 5) & (self.array1 < 250))
        unknown2 = np.sum((self.array2 > 5) & (self.array2 < 250))
        print(f"  Unknown in Map 1: {unknown1:,} ({100*unknown1/total_pixels:.2f}%)")
        print(f"  Unknown in Map 2: {unknown2:,} ({100*unknown2/total_pixels:.2f}%)")

        # Similarity metrics
        print(f"\nSimilarity Metrics:")

        # Structural Similarity (simplified)
        similarity = 100 * (1 - np.mean(abs_diff) / 255)
        print(f"  Overall similarity: {similarity:.2f}%")

        if similarity > 95:
            print(f"  Assessment: Maps are nearly identical")
        elif similarity > 80:
            print(f"  Assessment: Maps are very similar")
        elif similarity > 60:
            print(f"  Assessment: Maps are moderately similar")
        else:
            print(f"  Assessment: Maps are significantly different")

        print()
        return True

    def generate_difference_map(self, output_path):
        """Generate difference map visualization."""
        print(f"Generating difference map: {output_path}")

        if self.array1.shape != self.array2.shape:
            print("  Error: Cannot generate difference map - dimensions don't match")
            return False

        # Calculate absolute difference
        diff = np.abs(self.array1.astype(np.int16) - self.array2.astype(np.int16))

        # Create RGB difference image
        # Red: pixels in map1 but not map2 (became occupied)
        # Green: pixels in map2 but not map1 (became free)
        # Blue: areas that stayed the same
        # Yellow: unknown changes

        height, width = diff.shape
        diff_rgb = np.zeros((height, width, 3), dtype=np.uint8)

        # Pixels that are the same -> gray
        same_mask = diff == 0
        diff_rgb[same_mask] = [200, 200, 200]

        # Free to Occupied -> Red
        free_to_occ = (self.array1 > 250) & (self.array2 < 5)
        diff_rgb[free_to_occ] = [255, 0, 0]

        # Occupied to Free -> Green
        occ_to_free = (self.array1 < 5) & (self.array2 > 250)
        diff_rgb[occ_to_free] = [0, 255, 0]

        # Other changes -> Yellow
        other_changes = (diff > 0) & ~free_to_occ & ~occ_to_free
        intensity = np.clip(diff[other_changes] * 2, 0, 255).astype(np.uint8)
        diff_rgb[other_changes, 0] = intensity  # Red
        diff_rgb[other_changes, 1] = intensity  # Green
        diff_rgb[other_changes, 2] = 0          # Blue

        # Create image and add legend
        diff_image = Image.fromarray(diff_rgb)

        # Add text legend
        draw = ImageDraw.Draw(diff_image)
        legend_y = 10
        legend_items = [
            ("Gray: No change", (200, 200, 200)),
            ("Red: Free -> Occupied", (255, 0, 0)),
            ("Green: Occupied -> Free", (0, 255, 0)),
            ("Yellow: Other changes", (255, 255, 0))
        ]

        for text, color in legend_items:
            draw.rectangle([10, legend_y, 30, legend_y + 15], fill=color)
            draw.text((35, legend_y), text, fill=(0, 0, 0))
            legend_y += 20

        diff_image.save(output_path)
        print(f"  Difference map saved")
        return True

    def generate_overlay(self, output_path):
        """Generate overlay visualization of both maps."""
        print(f"Generating overlay: {output_path}")

        if self.array1.shape != self.array2.shape:
            print("  Error: Cannot generate overlay - dimensions don't match")
            return False

        # Create RGB overlay
        # Map 1 in red channel
        # Map 2 in green channel
        # Overlap appears yellow

        height, width = self.array1.shape
        overlay = np.zeros((height, width, 3), dtype=np.uint8)

        overlay[:, :, 0] = 255 - self.array1  # Red: map 1 occupied areas
        overlay[:, :, 1] = 255 - self.array2  # Green: map 2 occupied areas
        overlay[:, :, 2] = 0                  # Blue: unused

        overlay_image = Image.fromarray(overlay)

        # Add legend
        draw = ImageDraw.Draw(overlay_image)
        legend_items = [
            ("Red: Map 1 only", (255, 0, 0)),
            ("Green: Map 2 only", (0, 255, 0)),
            ("Yellow: Both maps", (255, 255, 0)),
            ("Black: Free space", (0, 0, 0))
        ]

        legend_y = 10
        for text, color in legend_items:
            draw.rectangle([10, legend_y, 30, legend_y + 15], fill=color)
            draw.text((35, legend_y), text, fill=(255, 255, 255))
            legend_y += 20

        overlay_image.save(output_path)
        print(f"  Overlay saved")
        return True

    def generate_side_by_side(self, output_path):
        """Generate side-by-side comparison."""
        print(f"Generating side-by-side comparison: {output_path}")

        # Resize if dimensions don't match
        if self.image1.size != self.image2.size:
            print("  Resizing maps to match...")
            # Use the larger dimensions
            max_width = max(self.image1.size[0], self.image2.size[0])
            max_height = max(self.image1.size[1], self.image2.size[1])

            img1 = self.image1.resize((max_width, max_height), Image.Resampling.NEAREST)
            img2 = self.image2.resize((max_width, max_height), Image.Resampling.NEAREST)
        else:
            img1 = self.image1
            img2 = self.image2

        # Convert to RGB
        if img1.mode != 'RGB':
            img1 = img1.convert('RGB')
        if img2.mode != 'RGB':
            img2 = img2.convert('RGB')

        # Create side-by-side image
        width, height = img1.size
        combined = Image.new('RGB', (width * 2 + 10, height + 40))

        # Paste images
        combined.paste(img1, (0, 40))
        combined.paste(img2, (width + 10, 40))

        # Add labels
        draw = ImageDraw.Draw(combined)
        draw.text((10, 10), f"Map 1: {self.yaml1.name}", fill=(255, 255, 255))
        draw.text((width + 20, 10), f"Map 2: {self.yaml2.name}", fill=(255, 255, 255))

        combined.save(output_path)
        print(f"  Side-by-side comparison saved")
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Compare two ROS2 occupancy grid maps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic comparison
  python3 compare_maps.py map1.yaml map2.yaml

  # Generate difference visualization
  python3 compare_maps.py map1.yaml map2.yaml --diff difference.png

  # Generate overlay
  python3 compare_maps.py map1.yaml map2.yaml --overlay overlay.png

  # Generate all visualizations
  python3 compare_maps.py map1.yaml map2.yaml --diff diff.png --overlay over.png --side-by-side sbs.png

Color coding in difference map:
  - Gray: No change
  - Red: Became occupied (was free in map1, occupied in map2)
  - Green: Became free (was occupied in map1, free in map2)
  - Yellow: Other changes

Color coding in overlay:
  - Red: Only in map 1
  - Green: Only in map 2
  - Yellow: In both maps
  - Black: Free space in both
        """
    )
    parser.add_argument('map1_yaml', help='First map YAML file')
    parser.add_argument('map2_yaml', help='Second map YAML file')
    parser.add_argument('--diff', metavar='FILE',
                        help='Generate difference map visualization')
    parser.add_argument('--overlay', metavar='FILE',
                        help='Generate overlay visualization')
    parser.add_argument('--side-by-side', metavar='FILE',
                        help='Generate side-by-side comparison')

    args = parser.parse_args()

    comparer = MapComparer(args.map1_yaml, args.map2_yaml)

    if not comparer.load_maps():
        sys.exit(1)

    # Compare metadata
    comparer.compare_metadata()

    # Compare pixels
    comparer.compare_pixels()

    # Generate visualizations
    if args.diff:
        comparer.generate_difference_map(args.diff)

    if args.overlay:
        comparer.generate_overlay(args.overlay)

    if args.side_by_side:
        comparer.generate_side_by_side(args.side_by_side)

    print("\nComparison complete!\n")


if __name__ == '__main__':
    main()
