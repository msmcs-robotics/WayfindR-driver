#!/usr/bin/env python3
"""
Map Viewer - Simple visualization tool for ROS2 occupancy grid maps

This tool provides:
- Load and display .pgm maps
- Show map metadata from .yaml
- Display grid coordinates and scale
- Help plan waypoint locations
- Export coordinate information

Usage:
    python3 map_viewer.py --map-yaml /path/to/map.yaml
    python3 map_viewer.py --map-yaml map.yaml --grid
    python3 map_viewer.py --map-yaml map.yaml --coords 100 200
"""

import argparse
import os
import sys
from typing import Tuple, Optional

import yaml
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec


class MapViewer:
    """Viewer for ROS2 occupancy grid maps."""

    def __init__(self, map_yaml_path: str):
        """
        Initialize map viewer.

        Args:
            map_yaml_path: Path to map YAML file
        """
        self.map_yaml_path = map_yaml_path
        self._load_map()

    def _load_map(self):
        """Load map YAML and image."""
        # Load YAML
        with open(self.map_yaml_path, 'r') as f:
            self.yaml_data = yaml.safe_load(f)

        self.resolution = self.yaml_data['resolution']
        self.origin = self.yaml_data.get('origin', [0, 0, 0])
        self.negate = self.yaml_data.get('negate', 0)
        self.occupied_thresh = self.yaml_data.get('occupied_thresh', 0.65)
        self.free_thresh = self.yaml_data.get('free_thresh', 0.25)
        self.mode = self.yaml_data.get('mode', 'trinary')

        # Load image
        yaml_dir = os.path.dirname(os.path.abspath(self.map_yaml_path))
        image_file = self.yaml_data['image']
        self.image_path = os.path.join(yaml_dir, image_file)

        self.map_image = Image.open(self.image_path)
        self.map_data = np.array(self.map_image)
        self.height, self.width = self.map_data.shape

        print(f"Map loaded successfully:")
        print(f"  Image: {self.image_path}")
        print(f"  Size: {self.width} x {self.height} pixels")
        print(f"  Resolution: {self.resolution} m/pixel ({self.resolution*100:.1f} cm/pixel)")
        print(f"  Physical size: {self.width*self.resolution:.2f}m x {self.height*self.resolution:.2f}m")
        print(f"  Origin: ({self.origin[0]:.3f}, {self.origin[1]:.3f}, {self.origin[2]:.3f})")

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to pixel coordinates."""
        px = int((x - self.origin[0]) / self.resolution)
        py = int(self.height - ((y - self.origin[1]) / self.resolution))
        return px, py

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates."""
        x = self.origin[0] + (px * self.resolution)
        y = self.origin[1] + ((self.height - py) * self.resolution)
        return x, y

    def get_map_bounds(self) -> dict:
        """Get map bounds in world coordinates."""
        min_x, max_y = self.pixel_to_world(0, 0)
        max_x, min_y = self.pixel_to_world(self.width - 1, self.height - 1)

        return {
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y,
            'center_x': (min_x + max_x) / 2,
            'center_y': (min_y + max_y) / 2
        }

    def print_info(self):
        """Print detailed map information."""
        print(f"\n{'='*70}")
        print("MAP INFORMATION")
        print(f"{'='*70}")
        print(f"YAML File: {self.map_yaml_path}")
        print(f"Image File: {self.image_path}")
        print(f"\nDimensions:")
        print(f"  Pixels: {self.width} x {self.height}")
        print(f"  Meters: {self.width*self.resolution:.3f} x {self.height*self.resolution:.3f}")
        print(f"\nResolution: {self.resolution} m/pixel ({self.resolution*100:.2f} cm/pixel)")
        print(f"\nOrigin (world coordinates):")
        print(f"  X: {self.origin[0]:.3f} m")
        print(f"  Y: {self.origin[1]:.3f} m")
        print(f"  Z: {self.origin[2]:.3f} m")

        bounds = self.get_map_bounds()
        print(f"\nMap Bounds (world coordinates):")
        print(f"  X: [{bounds['min_x']:.3f}, {bounds['max_x']:.3f}]")
        print(f"  Y: [{bounds['min_y']:.3f}, {bounds['max_y']:.3f}]")
        print(f"  Center: ({bounds['center_x']:.3f}, {bounds['center_y']:.3f})")

        print(f"\nOccupancy Parameters:")
        print(f"  Mode: {self.mode}")
        print(f"  Occupied threshold: {self.occupied_thresh}")
        print(f"  Free threshold: {self.free_thresh}")
        print(f"  Negate: {self.negate}")

        # Analyze map content
        print(f"\nMap Content Analysis:")
        total_pixels = self.width * self.height
        if self.map_data.dtype == np.uint8:
            # Analyze occupancy
            occupied = np.sum(self.map_data < 128)  # Dark pixels
            free = np.sum(self.map_data > 200)  # Light pixels
            unknown = total_pixels - occupied - free

            print(f"  Occupied: {occupied} pixels ({100*occupied/total_pixels:.1f}%)")
            print(f"  Free: {free} pixels ({100*free/total_pixels:.1f}%)")
            print(f"  Unknown: {unknown} pixels ({100*unknown/total_pixels:.1f}%)")

        print(f"{'='*70}\n")

    def show_coordinates(self, px: int, py: int):
        """Show coordinates for a pixel location."""
        if not (0 <= px < self.width and 0 <= py < self.height):
            print(f"Error: Pixel coordinates ({px}, {py}) out of bounds")
            return

        wx, wy = self.pixel_to_world(px, py)
        value = self.map_data[py, px]

        print(f"\nCoordinate Information:")
        print(f"  Pixel: ({px}, {py})")
        print(f"  World: ({wx:.3f}, {wy:.3f}) meters")
        print(f"  Pixel value: {value}")

        if value < 128:
            occupancy = "Occupied"
        elif value > 200:
            occupancy = "Free"
        else:
            occupancy = "Unknown"
        print(f"  Occupancy: {occupancy}")

    def visualize(self, show_grid: bool = False, grid_spacing: int = 50,
                  show_ruler: bool = True, interactive: bool = True,
                  output_path: Optional[str] = None):
        """
        Visualize the map.

        Args:
            show_grid: Show pixel grid overlay
            grid_spacing: Grid spacing in pixels
            show_ruler: Show scale ruler
            interactive: Enable interactive mode (click for coordinates)
            output_path: Optional path to save image
        """
        fig = plt.figure(figsize=(14, 10))
        gs = GridSpec(2, 2, figure=fig, height_ratios=[4, 1], width_ratios=[3, 1])

        # Main map display
        ax_map = fig.add_subplot(gs[0, :])
        ax_map.imshow(self.map_data, cmap='gray', origin='upper')
        ax_map.set_title(f"Map: {os.path.basename(self.map_yaml_path)}", fontsize=14, weight='bold')
        ax_map.set_xlabel(f"X (pixels) | Resolution: {self.resolution}m/px")
        ax_map.set_ylabel("Y (pixels)")

        # Add grid if requested
        if show_grid:
            for x in range(0, self.width, grid_spacing):
                ax_map.axvline(x, color='cyan', alpha=0.3, linewidth=0.5)
            for y in range(0, self.height, grid_spacing):
                ax_map.axhline(y, color='cyan', alpha=0.3, linewidth=0.5)

        # Add center marker
        center_px = self.width // 2
        center_py = self.height // 2
        ax_map.plot(center_px, center_py, 'r+', markersize=20, markeredgewidth=2)
        center_x, center_y = self.pixel_to_world(center_px, center_py)
        ax_map.text(center_px + 10, center_py - 10,
                   f'Center\n({center_x:.2f}, {center_y:.2f})m',
                   color='red', fontsize=10, weight='bold',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        # Add scale ruler
        if show_ruler:
            ruler_length_m = 1.0  # 1 meter
            ruler_length_px = ruler_length_m / self.resolution
            ruler_x = self.width - ruler_length_px - 20
            ruler_y = self.height - 20

            ax_map.plot([ruler_x, ruler_x + ruler_length_px], [ruler_y, ruler_y],
                       'r-', linewidth=3)
            ax_map.plot([ruler_x, ruler_x], [ruler_y - 5, ruler_y + 5], 'r-', linewidth=2)
            ax_map.plot([ruler_x + ruler_length_px, ruler_x + ruler_length_px],
                       [ruler_y - 5, ruler_y + 5], 'r-', linewidth=2)
            ax_map.text(ruler_x + ruler_length_px/2, ruler_y - 15,
                       f'{ruler_length_m}m', color='red', fontsize=12,
                       weight='bold', ha='center',
                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        # Info panel
        ax_info = fig.add_subplot(gs[1, 0])
        ax_info.axis('off')

        info_text = f"""
Map Information:
  Size: {self.width} × {self.height} pixels
  Physical: {self.width*self.resolution:.2f}m × {self.height*self.resolution:.2f}m
  Resolution: {self.resolution}m/px ({self.resolution*100:.1f}cm/px)
  Origin: ({self.origin[0]:.2f}, {self.origin[1]:.2f}) m
"""
        bounds = self.get_map_bounds()
        info_text += f"""
  Bounds: X[{bounds['min_x']:.2f}, {bounds['max_x']:.2f}]
          Y[{bounds['min_y']:.2f}, {bounds['max_y']:.2f}]

Thresholds:
  Free: {self.free_thresh}
  Occupied: {self.occupied_thresh}
"""
        if interactive:
            info_text += "\nClick on map to see coordinates"

        ax_info.text(0.05, 0.95, info_text, transform=ax_info.transAxes,
                    fontsize=10, verticalalignment='top', family='monospace',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Legend
        ax_legend = fig.add_subplot(gs[1, 1])
        ax_legend.axis('off')

        legend_text = """
Legend:
  White: Free space
  Black: Occupied
  Gray: Unknown

  + Red cross: Center
  Red line: Scale
"""
        if show_grid:
            legend_text += "  Cyan: Grid\n"

        ax_legend.text(0.1, 0.95, legend_text, transform=ax_legend.transAxes,
                      fontsize=10, verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

        # Interactive click handler
        if interactive:
            def on_click(event):
                if event.inaxes == ax_map and event.xdata and event.ydata:
                    px = int(event.xdata)
                    py = int(event.ydata)
                    wx, wy = self.pixel_to_world(px, py)

                    # Update title with coordinates
                    ax_map.set_title(
                        f"Map: {os.path.basename(self.map_yaml_path)} | "
                        f"Click: pixel({px}, {py}) = world({wx:.3f}, {wy:.3f})m",
                        fontsize=14, weight='bold')
                    fig.canvas.draw()

                    # Print to console
                    print(f"Clicked: pixel({px}, {py}) = world({wx:.3f}, {wy:.3f})m")

            fig.canvas.mpl_connect('button_press_event', on_click)

        plt.tight_layout()

        # Save if requested
        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"Saved visualization to: {output_path}")

        plt.show()

    def export_grid_points(self, spacing_meters: float = 0.5, output_path: str = "grid_points.yaml"):
        """
        Export a grid of points for waypoint planning.

        Args:
            spacing_meters: Spacing between grid points in meters
            output_path: Output YAML file path
        """
        bounds = self.get_map_bounds()

        # Generate grid
        x_points = np.arange(bounds['min_x'], bounds['max_x'], spacing_meters)
        y_points = np.arange(bounds['min_y'], bounds['max_y'], spacing_meters)

        grid_points = []
        for x in x_points:
            for y in y_points:
                px, py = self.world_to_pixel(x, y)
                if 0 <= px < self.width and 0 <= py < self.height:
                    # Check if free space
                    if self.map_data[py, px] > 200:  # Free space
                        grid_points.append({
                            'world': {'x': round(x, 3), 'y': round(y, 3)},
                            'pixel': {'x': px, 'y': py}
                        })

        # Save to YAML
        data = {
            'metadata': {
                'map_yaml': self.map_yaml_path,
                'spacing_meters': spacing_meters,
                'total_points': len(grid_points)
            },
            'grid_points': grid_points
        }

        with open(output_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        print(f"Exported {len(grid_points)} grid points to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Map Viewer - Visualize and inspect ROS2 maps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # View map with info
  python3 map_viewer.py --map-yaml maps/first_map.yaml

  # View with grid overlay
  python3 map_viewer.py --map-yaml maps/first_map.yaml --grid

  # Check coordinates at pixel location
  python3 map_viewer.py --map-yaml maps/first_map.yaml --coords 100 200

  # Export grid points
  python3 map_viewer.py --map-yaml maps/first_map.yaml --export-grid 0.5
        """
    )

    parser.add_argument('--map-yaml', '-m', type=str, required=True,
                        help='Path to map YAML file')

    parser.add_argument('--info', '-i', action='store_true',
                        help='Print detailed map information')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Visualize the map')
    parser.add_argument('--grid', '-g', action='store_true',
                        help='Show grid overlay on map')
    parser.add_argument('--grid-spacing', type=int, default=50,
                        help='Grid spacing in pixels (default: 50)')
    parser.add_argument('--no-ruler', action='store_true',
                        help='Hide scale ruler')
    parser.add_argument('--coords', '-c', nargs=2, type=int, metavar=('PX', 'PY'),
                        help='Show coordinates for pixel location')
    parser.add_argument('--export-grid', type=float, metavar='SPACING',
                        help='Export grid points with spacing in meters')
    parser.add_argument('--output', '-o', type=str,
                        help='Save visualization to file')

    args = parser.parse_args()

    # Load map
    try:
        viewer = MapViewer(args.map_yaml)
    except Exception as e:
        print(f"Error loading map: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    # Print info
    if args.info:
        viewer.print_info()

    # Show coordinates
    if args.coords:
        px, py = args.coords
        viewer.show_coordinates(px, py)

    # Export grid
    if args.export_grid:
        output = args.output if args.output else "grid_points.yaml"
        viewer.export_grid_points(spacing_meters=args.export_grid, output_path=output)

    # Visualize (default action if no other action specified)
    if args.visualize or not (args.info or args.coords or args.export_grid):
        viewer.visualize(
            show_grid=args.grid,
            grid_spacing=args.grid_spacing,
            show_ruler=not args.no_ruler,
            interactive=True,
            output_path=args.output if args.output and not args.export_grid else None
        )


if __name__ == '__main__':
    main()
