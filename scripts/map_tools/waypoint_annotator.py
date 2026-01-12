#!/usr/bin/env python3
"""
Interactive Waypoint Annotator for ROS2 Maps

This tool provides an interactive GUI to:
- Load and display ROS2 maps
- Click to add waypoints
- Name and configure waypoints
- Save waypoint definitions to YAML

Usage:
    python3 waypoint_annotator.py --map-yaml /path/to/map.yaml
    python3 waypoint_annotator.py --map-yaml map.yaml --waypoints existing.yaml
"""

import argparse
import math
import os
import sys

import yaml
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, simpledialog
except ImportError:
    print("Error: tkinter not available. Install with: sudo apt-get install python3-tk")
    sys.exit(1)


class WaypointAnnotator:
    """Interactive waypoint annotation tool."""

    def __init__(self, map_yaml_path: str, waypoint_yaml_path: Optional[str] = None):
        """
        Initialize the annotator.

        Args:
            map_yaml_path: Path to map YAML file
            waypoint_yaml_path: Optional path to existing waypoints
        """
        self.map_yaml_path = map_yaml_path
        self.waypoint_yaml_path = waypoint_yaml_path

        # Load map
        self._load_map()

        # Waypoints
        self.waypoints = []
        if waypoint_yaml_path:
            self._load_waypoints()

        # Annotation state
        self.current_waypoint_pos = None
        self.annotation_mode = 'position'  # 'position' or 'orientation'
        self.temp_waypoint_name = None

        # Create GUI
        self._create_gui()

    def _load_map(self):
        """Load map YAML and image."""
        with open(self.map_yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)

        self.resolution = yaml_data['resolution']
        self.origin = yaml_data.get('origin', [0, 0, 0])
        self.negate = yaml_data.get('negate', 0)

        # Load image
        yaml_dir = os.path.dirname(os.path.abspath(self.map_yaml_path))
        image_file = yaml_data['image']
        image_path = os.path.join(yaml_dir, image_file)

        self.map_image = Image.open(image_path)
        self.map_data = np.array(self.map_image)
        self.height, self.width = self.map_data.shape

        print(f"Loaded map: {self.width}x{self.height} @ {self.resolution}m/px")

    def _load_waypoints(self):
        """Load existing waypoints."""
        try:
            with open(self.waypoint_yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            self.waypoints = data.get('waypoints', [])
            print(f"Loaded {len(self.waypoints)} existing waypoints")
        except Exception as e:
            print(f"Warning: Could not load waypoints: {e}")

    def world_to_pixel(self, x: float, y: float) -> tuple:
        """Convert world to pixel coordinates."""
        px = int((x - self.origin[0]) / self.resolution)
        py = int(self.height - ((y - self.origin[1]) / self.resolution))
        return px, py

    def pixel_to_world(self, px: float, py: float) -> tuple:
        """Convert pixel to world coordinates."""
        x = self.origin[0] + (px * self.resolution)
        y = self.origin[1] + ((self.height - py) * self.resolution)
        return x, y

    def _create_gui(self):
        """Create the GUI."""
        self.root = tk.Tk()
        self.root.title(f"Waypoint Annotator - {os.path.basename(self.map_yaml_path)}")

        # Create main frames
        left_frame = ttk.Frame(self.root)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_frame = ttk.Frame(self.root, width=300)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH)

        # Map display
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.imshow(self.map_data, cmap='gray', origin='upper')
        self.ax.set_title("Click to add waypoint position, then click again for orientation")
        self.ax.set_xlabel(f"X (pixels) | Resolution: {self.resolution}m/px")
        self.ax.set_ylabel("Y (pixels)")

        # Add to tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=left_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Connect click event
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)

        # Control panel
        ttk.Label(right_frame, text="Controls", font=('Arial', 14, 'bold')).pack(pady=10)

        # Instructions
        instructions = """
1. Click on map to set waypoint position
2. Click again to set orientation
3. Enter waypoint name
4. Repeat for more waypoints
5. Save when done
        """
        ttk.Label(right_frame, text=instructions, justify=tk.LEFT).pack(pady=10)

        # Buttons
        ttk.Button(right_frame, text="Save Waypoints", command=self._save_waypoints).pack(pady=5)
        ttk.Button(right_frame, text="Clear All", command=self._clear_all).pack(pady=5)
        ttk.Button(right_frame, text="Undo Last", command=self._undo_last).pack(pady=5)
        ttk.Button(right_frame, text="Refresh View", command=self._refresh_view).pack(pady=5)

        # Waypoint list
        ttk.Label(right_frame, text="Waypoints:", font=('Arial', 12, 'bold')).pack(pady=(20, 5))

        # Listbox with scrollbar
        list_frame = ttk.Frame(right_frame)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.waypoint_listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set)
        self.waypoint_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.waypoint_listbox.yview)

        # Bind double-click to delete
        self.waypoint_listbox.bind('<Double-Button-1>', self._on_listbox_double_click)

        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready - Click on map to add waypoint")
        ttk.Label(right_frame, textvariable=self.status_var, relief=tk.SUNKEN).pack(
            side=tk.BOTTOM, fill=tk.X)

        # Initial draw
        self._refresh_view()

    def _on_click(self, event):
        """Handle click on map."""
        if event.inaxes != self.ax:
            return

        px, py = event.xdata, event.ydata

        if self.annotation_mode == 'position':
            # First click - set position
            self.current_waypoint_pos = (px, py)
            self.annotation_mode = 'orientation'
            self.status_var.set(f"Position set at ({px:.1f}, {py:.1f}) - Click for orientation")

            # Draw temporary marker
            self.ax.plot(px, py, 'ro', markersize=10)
            self.canvas.draw()

        elif self.annotation_mode == 'orientation':
            # Second click - calculate orientation and add waypoint
            pos_x, pos_y = self.current_waypoint_pos

            # Calculate angle from position to orientation point
            dx = px - pos_x
            dy = py - pos_y
            yaw_rad = math.atan2(-dy, dx)  # Negative dy because y is inverted
            yaw_degrees = math.degrees(yaw_rad)

            # Prompt for name
            name = simpledialog.askstring("Waypoint Name",
                                         "Enter waypoint name:",
                                         parent=self.root)

            if name:
                # Convert to world coordinates
                world_x, world_y = self.pixel_to_world(pos_x, pos_y)

                # Add waypoint
                self._add_waypoint(name, world_x, world_y, yaw_degrees)

            # Reset for next waypoint
            self.current_waypoint_pos = None
            self.annotation_mode = 'position'
            self.status_var.set("Ready - Click on map to add waypoint")
            self._refresh_view()

    def _add_waypoint(self, name: str, x: float, y: float, yaw_degrees: float):
        """Add a waypoint."""
        # Check for duplicate
        for i, wp in enumerate(self.waypoints):
            if wp['name'] == name:
                if messagebox.askyesno("Duplicate Name",
                                      f"Waypoint '{name}' already exists. Replace?"):
                    self.waypoints.pop(i)
                else:
                    return

        # Convert to quaternion
        yaw_rad = math.radians(yaw_degrees)
        qz = math.sin(yaw_rad / 2)
        qw = math.cos(yaw_rad / 2)

        waypoint = {
            'name': name,
            'description': '',
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
            'yaw_degrees': round(yaw_degrees, 2),
            'tolerance': {
                'position': 0.3,
                'orientation': 0.2
            }
        }

        self.waypoints.append(waypoint)
        print(f"Added waypoint '{name}' at ({x:.3f}, {y:.3f}), yaw={yaw_degrees:.1f}°")

        # Update listbox
        self._update_listbox()

    def _refresh_view(self):
        """Refresh the map view with waypoints."""
        self.ax.clear()
        self.ax.imshow(self.map_data, cmap='gray', origin='upper')
        self.ax.set_title(f"Waypoint Annotator - {len(self.waypoints)} waypoints")
        self.ax.set_xlabel(f"X (pixels) | Resolution: {self.resolution}m/px")
        self.ax.set_ylabel("Y (pixels)")

        # Draw all waypoints
        for i, wp in enumerate(self.waypoints, 1):
            pos = wp['position']
            px, py = self.world_to_pixel(pos['x'], pos['y'])

            # Skip if out of bounds
            if not (0 <= px < self.width and 0 <= py < self.height):
                continue

            # Draw position marker
            self.ax.plot(px, py, 'ro', markersize=8)

            # Draw orientation arrow
            yaw_rad = math.radians(wp['yaw_degrees'])
            arrow_len = 30
            dx = arrow_len * math.cos(yaw_rad)
            dy = -arrow_len * math.sin(yaw_rad)  # Negative for image coordinates
            self.ax.arrow(px, py, dx, dy, head_width=10, head_length=10,
                         fc='blue', ec='blue', linewidth=2)

            # Draw label
            self.ax.text(px + 15, py - 15, f"{i}. {wp['name']}",
                        color='black', fontsize=9, weight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

        self.canvas.draw()

    def _update_listbox(self):
        """Update the waypoint listbox."""
        self.waypoint_listbox.delete(0, tk.END)
        for i, wp in enumerate(self.waypoints, 1):
            pos = wp['position']
            label = f"{i}. {wp['name']} ({pos['x']:.2f}, {pos['y']:.2f}) {wp['yaw_degrees']:.0f}°"
            self.waypoint_listbox.insert(tk.END, label)

    def _on_listbox_double_click(self, event):
        """Handle double-click on listbox item to delete."""
        selection = self.waypoint_listbox.curselection()
        if selection:
            index = selection[0]
            wp = self.waypoints[index]
            if messagebox.askyesno("Delete Waypoint",
                                  f"Delete waypoint '{wp['name']}'?"):
                self.waypoints.pop(index)
                self._update_listbox()
                self._refresh_view()

    def _clear_all(self):
        """Clear all waypoints."""
        if messagebox.askyesno("Clear All", "Delete all waypoints?"):
            self.waypoints.clear()
            self._update_listbox()
            self._refresh_view()
            self.status_var.set("All waypoints cleared")

    def _undo_last(self):
        """Remove the last waypoint."""
        if self.waypoints:
            removed = self.waypoints.pop()
            self._update_listbox()
            self._refresh_view()
            self.status_var.set(f"Removed '{removed['name']}'")
        else:
            messagebox.showinfo("Undo", "No waypoints to undo")

    def _save_waypoints(self):
        """Save waypoints to YAML."""
        if not self.waypoints:
            messagebox.showwarning("Save", "No waypoints to save")
            return

        # Determine output path
        if self.waypoint_yaml_path:
            output_path = self.waypoint_yaml_path
        else:
            base = os.path.splitext(self.map_yaml_path)[0]
            output_path = f"{base}_waypoints.yaml"

        # Prepare data
        data = {
            'metadata': {
                'created_by': 'waypoint_annotator.py',
                'map_yaml': self.map_yaml_path,
                'frame_id': 'map'
            },
            'waypoints': self.waypoints
        }

        # Save
        try:
            with open(output_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            messagebox.showinfo("Saved", f"Saved {len(self.waypoints)} waypoints to:\n{output_path}")
            self.status_var.set(f"Saved to {os.path.basename(output_path)}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save: {e}")

    def run(self):
        """Run the GUI."""
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(
        description='Interactive Waypoint Annotator for ROS2 Maps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Usage:
  1. Click on the map to set waypoint position (red dot appears)
  2. Click again to set orientation (blue arrow direction)
  3. Enter a name for the waypoint
  4. Repeat for additional waypoints
  5. Double-click waypoint in list to delete
  6. Click "Save Waypoints" when done

Examples:
  python3 waypoint_annotator.py --map-yaml maps/first_map.yaml
  python3 waypoint_annotator.py --map-yaml map.yaml --waypoints existing.yaml
        """
    )

    parser.add_argument('--map-yaml', '-m', type=str, required=True,
                        help='Path to map YAML file')
    parser.add_argument('--waypoints', '-w', type=str,
                        help='Path to existing waypoints YAML file')

    args = parser.parse_args()

    # Check if map exists
    if not os.path.exists(args.map_yaml):
        print(f"Error: Map file not found: {args.map_yaml}")
        sys.exit(1)

    # Create and run annotator
    try:
        annotator = WaypointAnnotator(args.map_yaml, args.waypoints)
        annotator.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
