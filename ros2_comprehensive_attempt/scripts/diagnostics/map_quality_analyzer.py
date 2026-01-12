#!/usr/bin/env python3
"""
Map Quality Analyzer

Analyzes occupancy grid maps for quality and issues:
- Map coverage (occupied vs free space)
- Isolated regions
- Narrow passages
- Dead ends
- Map connectivity

Usage:
    python3 map_quality_analyzer.py --map maps/office.yaml
    python3 map_quality_analyzer.py --map maps/office.yaml --visualize
"""

import yaml
import numpy as np
import argparse
from pathlib import Path
from PIL import Image
from collections import deque
from typing import Tuple, List, Set


class MapQualityAnalyzer:
    """Analyze occupancy grid map quality."""

    def __init__(self, map_yaml_path: str):
        self.map_yaml_path = map_yaml_path
        self.map_data = None
        self.resolution = 0.0
        self.origin = [0.0, 0.0, 0.0]
        self.width = 0
        self.height = 0

        self._load_map()

    def _load_map(self):
        """Load map from YAML file."""
        with open(self.map_yaml_path, 'r') as f:
            map_config = yaml.safe_load(f)

        # Get map image path
        map_dir = Path(self.map_yaml_path).parent
        image_path = map_dir / map_config['image']

        # Load image
        img = Image.open(image_path)
        img_array = np.array(img)

        # Convert to occupancy values
        # 255 (white) = free (0)
        # 0 (black) = occupied (100)
        # gray = unknown (-1)

        self.map_data = np.zeros_like(img_array, dtype=np.int8)

        # Map image values to occupancy values
        free_thresh = int(map_config.get('free_thresh', 0.196) * 255)
        occupied_thresh = int(map_config.get('occupied_thresh', 0.65) * 255)

        self.map_data[img_array >= free_thresh] = 0       # Free
        self.map_data[img_array <= occupied_thresh] = 100  # Occupied
        self.map_data[(img_array > occupied_thresh) &
                     (img_array < free_thresh)] = -1       # Unknown

        self.resolution = map_config['resolution']
        self.origin = map_config['origin']
        self.height, self.width = self.map_data.shape

        print(f"Loaded map: {self.width}x{self.height} @ {self.resolution}m/pixel")

    def calculate_coverage(self) -> dict:
        """Calculate map coverage statistics."""
        total_cells = self.width * self.height
        free_cells = np.sum(self.map_data == 0)
        occupied_cells = np.sum(self.map_data == 100)
        unknown_cells = np.sum(self.map_data == -1)

        return {
            'total': total_cells,
            'free': free_cells,
            'occupied': occupied_cells,
            'unknown': unknown_cells,
            'free_percent': (free_cells / total_cells) * 100,
            'occupied_percent': (occupied_cells / total_cells) * 100,
            'unknown_percent': (unknown_cells / total_cells) * 100,
        }

    def find_connected_regions(self) -> List[Set[Tuple[int, int]]]:
        """Find connected free space regions using flood fill."""
        visited = np.zeros_like(self.map_data, dtype=bool)
        regions = []

        def flood_fill(start_x: int, start_y: int) -> Set[Tuple[int, int]]:
            """Flood fill from start position."""
            region = set()
            queue = deque([(start_x, start_y)])
            visited[start_y, start_x] = True

            while queue:
                x, y = queue.popleft()
                region.add((x, y))

                # Check 4-connected neighbors
                for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    nx, ny = x + dx, y + dy

                    if (0 <= nx < self.width and 0 <= ny < self.height and
                        not visited[ny, nx] and self.map_data[ny, nx] == 0):
                        visited[ny, nx] = True
                        queue.append((nx, ny))

            return region

        # Find all connected regions
        for y in range(self.height):
            for x in range(self.width):
                if self.map_data[y, x] == 0 and not visited[y, x]:
                    region = flood_fill(x, y)
                    if len(region) > 10:  # Ignore tiny regions
                        regions.append(region)

        return regions

    def find_narrow_passages(self, threshold: float = 0.5) -> List[Tuple[int, int]]:
        """Find narrow passages (bottlenecks)."""
        threshold_cells = int(threshold / self.resolution)
        narrow_passages = []

        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                if self.map_data[y, x] == 0:  # Free cell
                    # Check horizontal clearance
                    left_dist = 0
                    for dx in range(1, threshold_cells + 1):
                        if x - dx < 0 or self.map_data[y, x - dx] != 0:
                            break
                        left_dist += 1

                    right_dist = 0
                    for dx in range(1, threshold_cells + 1):
                        if x + dx >= self.width or self.map_data[y, x + dx] != 0:
                            break
                        right_dist += 1

                    # Check vertical clearance
                    up_dist = 0
                    for dy in range(1, threshold_cells + 1):
                        if y - dy < 0 or self.map_data[y - dy, x] != 0:
                            break
                        up_dist += 1

                    down_dist = 0
                    for dy in range(1, threshold_cells + 1):
                        if y + dy >= self.height or self.map_data[y + dy, x] != 0:
                            break
                        down_dist += 1

                    # If clearance is small in any direction
                    if (left_dist + right_dist < threshold_cells or
                        up_dist + down_dist < threshold_cells):
                        narrow_passages.append((x, y))

        return narrow_passages

    def find_dead_ends(self) -> List[Tuple[int, int]]:
        """Find dead ends (cells with only one exit)."""
        dead_ends = []

        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                if self.map_data[y, x] == 0:  # Free cell
                    # Count free neighbors
                    free_neighbors = 0
                    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                        nx, ny = x + dx, y + dy
                        if self.map_data[ny, nx] == 0:
                            free_neighbors += 1

                    # Dead end if only one free neighbor
                    if free_neighbors == 1:
                        dead_ends.append((x, y))

        return dead_ends

    def analyze(self) -> dict:
        """Run complete analysis."""
        print("\n" + "=" * 70)
        print("MAP QUALITY ANALYSIS")
        print("=" * 70)

        # Coverage
        print("\n1. MAP COVERAGE:")
        coverage = self.calculate_coverage()
        print(f"   Total cells: {coverage['total']:,}")
        print(f"   Free space: {coverage['free']:,} ({coverage['free_percent']:.1f}%)")
        print(f"   Occupied: {coverage['occupied']:,} ({coverage['occupied_percent']:.1f}%)")
        print(f"   Unknown: {coverage['unknown']:,} ({coverage['unknown_percent']:.1f}%)")

        # Connected regions
        print("\n2. CONNECTIVITY:")
        regions = self.find_connected_regions()
        print(f"   Connected regions: {len(regions)}")

        if len(regions) > 1:
            print(f"   ⚠ WARNING: Map has {len(regions)} disconnected regions!")
            for i, region in enumerate(sorted(regions, key=len, reverse=True)):
                area_m2 = len(region) * (self.resolution ** 2)
                print(f"      Region {i+1}: {len(region):,} cells ({area_m2:.2f} m²)")
        else:
            print(f"   ✓ Map is fully connected")

        # Narrow passages
        print("\n3. NARROW PASSAGES:")
        passages = self.find_narrow_passages(threshold=0.6)
        print(f"   Found {len(passages)} narrow passages")

        if len(passages) > 100:
            print(f"   ⚠ WARNING: Many narrow passages may cause navigation issues")
        elif len(passages) > 0:
            print(f"   ℹ Some narrow passages exist (robot clearance: check robot radius)")
        else:
            print(f"   ✓ No significant narrow passages")

        # Dead ends
        print("\n4. DEAD ENDS:")
        dead_ends = self.find_dead_ends()
        print(f"   Found {len(dead_ends)} dead end cells")

        if len(dead_ends) > 0:
            print(f"   ℹ Dead ends detected (may be normal for alcoves/corners)")

        # Overall assessment
        print("\n" + "=" * 70)
        print("OVERALL ASSESSMENT:")

        issues = []
        if coverage['unknown_percent'] > 20:
            issues.append("High unknown area percentage")
        if len(regions) > 1:
            issues.append("Disconnected regions")
        if len(passages) > 100:
            issues.append("Many narrow passages")

        if not issues:
            print("✓ Map quality is GOOD")
        else:
            print("⚠ Issues detected:")
            for issue in issues:
                print(f"  - {issue}")

        print("=" * 70)

        return {
            'coverage': coverage,
            'regions': len(regions),
            'narrow_passages': len(passages),
            'dead_ends': len(dead_ends),
            'issues': issues
        }

    def visualize(self, output_path: str = None):
        """Create visualization of analysis results."""
        try:
            import matplotlib.pyplot as plt
            from matplotlib.colors import ListedColormap
        except ImportError:
            print("Error: matplotlib required for visualization")
            print("Install with: pip3 install matplotlib")
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 12))

        # 1. Original map
        ax = axes[0, 0]
        cmap = ListedColormap(['white', 'gray', 'black'])
        normalized = np.copy(self.map_data)
        normalized[normalized == -1] = 50  # Unknown -> gray
        ax.imshow(normalized, cmap=cmap, origin='lower')
        ax.set_title('Original Map')
        ax.set_xlabel(f'Resolution: {self.resolution}m/pixel')

        # 2. Connected regions
        ax = axes[0, 1]
        regions = self.find_connected_regions()
        region_map = np.zeros((self.height, self.width), dtype=int)
        for i, region in enumerate(regions):
            for x, y in region:
                region_map[y, x] = i + 1
        ax.imshow(region_map, cmap='tab20', origin='lower')
        ax.set_title(f'Connected Regions ({len(regions)})')

        # 3. Narrow passages
        ax = axes[1, 0]
        passage_map = np.copy(normalized)
        passages = self.find_narrow_passages()
        for x, y in passages:
            passage_map[y, x] = 200  # Highlight
        ax.imshow(passage_map, cmap='hot', origin='lower')
        ax.set_title(f'Narrow Passages ({len(passages)})')

        # 4. Dead ends
        ax = axes[1, 1]
        dead_end_map = np.copy(normalized)
        dead_ends = self.find_dead_ends()
        for x, y in dead_ends:
            dead_end_map[y, x] = 200  # Highlight
        ax.imshow(dead_end_map, cmap='hot', origin='lower')
        ax.set_title(f'Dead Ends ({len(dead_ends)})')

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150)
            print(f"\nVisualization saved to: {output_path}")
        else:
            plt.show()


def main():
    parser = argparse.ArgumentParser(description='Map Quality Analyzer')
    parser.add_argument('--map', '-m', type=str, required=True,
                       help='Path to map YAML file')
    parser.add_argument('--visualize', '-v', action='store_true',
                       help='Create visualization')
    parser.add_argument('--output', '-o', type=str,
                       help='Output path for visualization')

    args = parser.parse_args()

    try:
        analyzer = MapQualityAnalyzer(args.map)
        results = analyzer.analyze()

        if args.visualize:
            analyzer.visualize(args.output)

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        exit(1)


if __name__ == '__main__':
    main()
