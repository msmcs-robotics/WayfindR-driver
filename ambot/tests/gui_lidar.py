#!/usr/bin/env python3
"""
LiDAR GUI Diagnostic with Live Polar Map

Displays live LiDAR scan data in a polar plot.
Supports LD19 LiDAR (default) and raw data visualization mode.

Usage:
    python3 gui_lidar.py                      # Auto-detect /dev/ttyUSB0 with LD19
    python3 gui_lidar.py --port /dev/ttyUSB1  # Specific port
    python3 gui_lidar.py --raw --baud 460800  # Raw data mode for other LiDARs
    python3 gui_lidar.py --headless           # No GUI, save scan images
    python3 gui_lidar.py --headless --scans 3 # Headless with scan limit

Dependencies:
    pip3 install pyserial numpy matplotlib

Controls (GUI mode):
    q - Quit
    r - Reset/clear display
    p - Pause/resume
    s - Save current scan image
"""

import sys
import time
import math
import argparse
from datetime import datetime
from pathlib import Path
from collections import deque
import threading

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)

# Add parent for pathfinder import
sys.path.insert(0, str(SCRIPT_DIR.parent))


class LidarDataCollector:
    """Collects raw LiDAR data from serial port."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=460800, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None
        self._running = False
        self._thread = None
        self._data_buffer = deque(maxlen=5000)  # Raw bytes
        self._scan_points = deque(maxlen=1000)  # Parsed (angle, distance) tuples
        self._lock = threading.Lock()

    def connect(self):
        """Connect to serial port."""
        import serial
        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout
            )
            self._serial.reset_input_buffer()
            print(f"Connected to {self.port} @ {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        print("Disconnected")

    def start_collection(self):
        """Start data collection thread."""
        self._running = True
        self._thread = threading.Thread(target=self._collection_loop, daemon=True)
        self._thread.start()

    def _collection_loop(self):
        """Background data collection."""
        while self._running and self._serial and self._serial.is_open:
            try:
                data = self._serial.read(1024)
                if data:
                    with self._lock:
                        self._data_buffer.extend(data)
                        # Try to parse scan points from raw data
                        self._parse_raw_data(data)
            except Exception as e:
                if self._running:
                    print(f"Read error: {e}")
                break

    def _parse_raw_data(self, data):
        """
        Try to parse scan points from raw data.
        This is protocol-agnostic - it looks for patterns that might be scan data.
        """
        # Simple heuristic: look for 5-byte packets that might be scan data
        # RPLidar format: [quality<<2|S][angle_low][angle_high][distance_low][distance_high]

        buf = bytes(self._data_buffer)
        i = 0
        while i < len(buf) - 5:
            # Look for sync pattern or just try to parse
            b0, b1, b2, b3, b4 = buf[i:i+5]

            # RPLidar-style: check start bit flags
            if (b0 & 0x01) and (b1 & 0x01):  # Start flags
                quality = b0 >> 2
                angle = ((b1 >> 1) | (b2 << 7)) / 64.0
                distance = (b3 | (b4 << 8)) / 4.0

                if 0 <= angle <= 360 and 0 <= distance <= 50000 and quality > 0:
                    self._scan_points.append((angle, distance, quality))
                    i += 5
                    continue

            # Try different parsing if above didn't work
            # Some LiDARs output angle/distance directly
            angle_raw = b0 | (b1 << 8)
            distance_raw = b2 | (b3 << 8)

            angle = angle_raw / 100.0  # Some use 0.01 degree resolution
            distance = distance_raw

            if 0 <= angle <= 360 and 50 <= distance <= 12000:
                self._scan_points.append((angle, distance, 100))

            i += 1

    def get_scan_points(self):
        """Get current scan points as list of (angle, distance) tuples."""
        with self._lock:
            return list(self._scan_points)

    def get_raw_data_rate(self):
        """Get bytes per second."""
        with self._lock:
            return len(self._data_buffer)

    def clear_data(self):
        """Clear all collected data."""
        with self._lock:
            self._data_buffer.clear()
            self._scan_points.clear()


def detect_display():
    """Check if display is available."""
    import os
    display = os.environ.get("DISPLAY")
    if display:
        print(f"Display detected: {display}")
        return True
    print("No display detected (DISPLAY env var not set)")
    return False


def find_nearest_furthest(scan_points, min_distance=50):
    """Find the nearest and furthest points from scan data.

    Args:
        scan_points: List of (angle, distance, quality) tuples
        min_distance: Minimum distance to consider (filters noise)

    Returns:
        (nearest_point, furthest_point) as (angle, distance, quality) or None
    """
    if not scan_points:
        return None, None

    # Filter valid points (above noise threshold)
    valid_points = [p for p in scan_points if p[1] > min_distance]
    if not valid_points:
        return None, None

    nearest = min(valid_points, key=lambda p: p[1])
    furthest = max(valid_points, key=lambda p: p[1])

    return nearest, furthest


def create_polar_plot(scan_points, max_distance=6000, title="LiDAR Scan"):
    """Create a polar plot from scan points with nearest/furthest highlighting."""
    import numpy as np
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='polar')

    nearest_point = None
    furthest_point = None

    if scan_points:
        angles = [math.radians(p[0]) for p in scan_points]
        distances = [p[1] for p in scan_points]
        qualities = [p[2] for p in scan_points]

        # Color by quality (default points)
        colors = ['green' if q > 50 else 'yellow' if q > 20 else 'red' for q in qualities]
        ax.scatter(angles, distances, c=colors, s=2, alpha=0.7)

        # Find and highlight nearest and furthest points
        nearest_point, furthest_point = find_nearest_furthest(scan_points)

        if nearest_point:
            nearest_angle = math.radians(nearest_point[0])
            nearest_dist = nearest_point[1]
            # Red marker for nearest (danger/closest)
            ax.scatter([nearest_angle], [nearest_dist], c='red', s=200, marker='o',
                       edgecolors='white', linewidths=2, zorder=10, label=f'Nearest: {nearest_dist:.0f}mm @ {nearest_point[0]:.0f}°')
            # Add connecting line from center
            ax.plot([nearest_angle, nearest_angle], [0, nearest_dist], 'r--', linewidth=1, alpha=0.7)

        if furthest_point:
            furthest_angle = math.radians(furthest_point[0])
            furthest_dist = furthest_point[1]
            # Blue marker for furthest (clear/open space)
            ax.scatter([furthest_angle], [furthest_dist], c='blue', s=200, marker='s',
                       edgecolors='white', linewidths=2, zorder=10, label=f'Furthest: {furthest_dist:.0f}mm @ {furthest_point[0]:.0f}°')
            # Add connecting line from center
            ax.plot([furthest_angle, furthest_angle], [0, furthest_dist], 'b--', linewidth=1, alpha=0.7)

        # Add legend
        ax.legend(loc='upper right', fontsize=8)

    ax.set_rmax(max_distance)
    ax.set_title(title)
    ax.grid(True)

    return fig, ax


def run_lidar_gui(port="/dev/ttyUSB0", baudrate=230400, headless=False, raw_mode=False, max_scans=0):
    """Run the LiDAR GUI."""
    import numpy as np

    # Check for matplotlib
    try:
        import matplotlib
        if headless:
            matplotlib.use('Agg')  # Non-interactive backend for headless
        else:
            matplotlib.use('TkAgg')  # or 'Qt5Agg'
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError as e:
        print(f"ERROR: matplotlib not installed: {e}")
        print("Install with: pip3 install matplotlib")
        return 1

    print(f"Connecting to LiDAR at {port}...")

    # Use LD19 driver for non-raw mode, raw collector otherwise
    if raw_mode:
        collector = LidarDataCollector(port=port, baudrate=baudrate)
        if not collector.connect():
            return 1
        collector.start_collection()
        use_ld19 = False
    else:
        # Try LD19 driver first
        try:
            from pathfinder.lidar_ld19 import LD19Lidar
            lidar = LD19Lidar(port=port)
            if not lidar.connect():
                print("LD19 driver failed, falling back to raw mode")
                collector = LidarDataCollector(port=port, baudrate=baudrate)
                if not collector.connect():
                    return 1
                collector.start_collection()
                use_ld19 = False
            else:
                use_ld19 = True
                print("Using LD19 driver")
        except ImportError:
            print("LD19 driver not available, using raw mode")
            collector = LidarDataCollector(port=port, baudrate=baudrate)
            if not collector.connect():
                return 1
            collector.start_collection()
            use_ld19 = False

    print("Collecting data...")
    if max_scans > 0:
        print(f"Will stop after {max_scans} scans")
    else:
        print("Press Ctrl+C to stop")

    if headless:
        # Headless mode - periodic snapshots
        try:
            scan_count = 0

            if use_ld19:
                # Use LD19 iterator
                for scan in lidar.iter_scans(min_points=100):
                    scan_count += 1
                    points = [(p.angle, p.distance, p.quality) for p in scan]

                    print(f"Scan {scan_count}: {len(points)} points")

                    if points:
                        fig, ax = create_polar_plot(
                            points,
                            title=f"LiDAR Scan #{scan_count} ({len(points)} points)"
                        )
                        filename = RESULTS_DIR / f"lidar_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                        fig.savefig(filename, dpi=100)
                        plt.close(fig)
                        print(f"Saved: {filename}")

                    if max_scans > 0 and scan_count >= max_scans:
                        print(f"Reached {max_scans} scans, stopping")
                        break
            else:
                # Use raw collector
                while True:
                    time.sleep(2.0)  # Wait for data
                    points = collector.get_scan_points()
                    scan_count += 1

                    print(f"Scan {scan_count}: {len(points)} points")

                    if points:
                        fig, ax = create_polar_plot(
                            points,
                            title=f"LiDAR Scan #{scan_count} ({len(points)} points)"
                        )
                        filename = RESULTS_DIR / f"lidar_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                        fig.savefig(filename, dpi=100)
                        plt.close(fig)
                        print(f"Saved: {filename}")

                    collector.clear_data()

                    if max_scans > 0 and scan_count >= max_scans:
                        print(f"Reached {max_scans} scans, stopping")
                        break

        except KeyboardInterrupt:
            print("\nStopped by user")

    else:
        # Interactive GUI mode
        # For LD19 mode, we need a scan buffer since LD19 uses an iterator
        # instead of the raw collector's get_scan_points()/clear_data() API
        ld19_scan_data = {"points": [], "lock": threading.Lock(), "running": True}

        if use_ld19:
            # Start a background thread to collect LD19 scans for the GUI
            def ld19_collect():
                for scan in lidar.iter_scans(min_points=50):
                    if not ld19_scan_data["running"]:
                        break
                    points = [(p.angle, p.distance, p.quality) for p in scan]
                    with ld19_scan_data["lock"]:
                        ld19_scan_data["points"] = points

            ld19_thread = threading.Thread(target=ld19_collect, daemon=True)
            ld19_thread.start()

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='polar')
        scatter = ax.scatter([], [], s=2, c='green', alpha=0.7)

        # Create markers for nearest and furthest points
        nearest_marker = ax.scatter([], [], s=200, c='red', marker='o',
                                    edgecolors='white', linewidths=2, zorder=10)
        furthest_marker = ax.scatter([], [], s=200, c='blue', marker='s',
                                     edgecolors='white', linewidths=2, zorder=10)
        # Lines from center to nearest/furthest
        nearest_line, = ax.plot([], [], 'r--', linewidth=1, alpha=0.7)
        furthest_line, = ax.plot([], [], 'b--', linewidth=1, alpha=0.7)

        ax.set_rmax(6000)
        ax.set_title("LiDAR Live View")
        ax.grid(True)

        # Stats text
        stats_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                             verticalalignment='top', fontsize=9,
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Edge info text (right side)
        edge_text = ax.text(0.98, 0.98, '', transform=ax.transAxes,
                            verticalalignment='top', horizontalalignment='right', fontsize=9,
                            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

        paused = [False]  # Use list to allow modification in nested function
        start_time = [time.time()]

        def get_gui_points():
            """Get scan points from either LD19 or raw collector."""
            if use_ld19:
                with ld19_scan_data["lock"]:
                    return list(ld19_scan_data["points"])
            else:
                return collector.get_scan_points()

        def update(frame):
            if paused[0]:
                return scatter, stats_text, nearest_marker, furthest_marker, edge_text

            points = get_gui_points()

            nearest_info = "N/A"
            furthest_info = "N/A"

            if points:
                angles = np.array([math.radians(p[0]) for p in points])
                distances = np.array([p[1] for p in points])

                scatter.set_offsets(np.column_stack([angles, distances]))

                # Colors by quality
                qualities = [p[2] for p in points]
                colors = ['green' if q > 50 else 'yellow' if q > 20 else 'red' for q in qualities]
                scatter.set_color(colors)

                # Find and highlight nearest/furthest
                nearest, furthest = find_nearest_furthest(points)

                if nearest:
                    nearest_ang = math.radians(nearest[0])
                    nearest_marker.set_offsets([[nearest_ang, nearest[1]]])
                    nearest_line.set_data([nearest_ang, nearest_ang], [0, nearest[1]])
                    nearest_info = f"{nearest[1]:.0f}mm @ {nearest[0]:.0f}°"
                else:
                    nearest_marker.set_offsets([])
                    nearest_line.set_data([], [])

                if furthest:
                    furthest_ang = math.radians(furthest[0])
                    furthest_marker.set_offsets([[furthest_ang, furthest[1]]])
                    furthest_line.set_data([furthest_ang, furthest_ang], [0, furthest[1]])
                    furthest_info = f"{furthest[1]:.0f}mm @ {furthest[0]:.0f}°"
                else:
                    furthest_marker.set_offsets([])
                    furthest_line.set_data([], [])

            # Update stats
            elapsed = time.time() - start_time[0]
            mode_str = "LD19" if use_ld19 else "Raw"
            stats_text.set_text(
                f'Points: {len(points)} ({mode_str})\n'
                f'Elapsed: {elapsed:.1f}s\n'
                f'[P]ause | [R]eset | [S]ave | [Q]uit'
            )

            # Update edge info
            edge_text.set_text(
                f'EDGES\n'
                f'Nearest (red): {nearest_info}\n'
                f'Furthest (blue): {furthest_info}'
            )

            return scatter, stats_text, nearest_marker, furthest_marker, edge_text

        def on_key(event):
            if event.key == 'q':
                plt.close()
            elif event.key == 'p':
                paused[0] = not paused[0]
                print(f"{'Paused' if paused[0] else 'Resumed'}")
            elif event.key == 'r':
                if use_ld19:
                    with ld19_scan_data["lock"]:
                        ld19_scan_data["points"] = []
                else:
                    collector.clear_data()
                start_time[0] = time.time()
                print("Reset")
            elif event.key == 's':
                filename = RESULTS_DIR / f"lidar_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                fig.savefig(filename, dpi=150)
                print(f"Saved: {filename}")

        fig.canvas.mpl_connect('key_press_event', on_key)

        try:
            anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
            plt.show()
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            if use_ld19:
                ld19_scan_data["running"] = False

    # Cleanup
    if use_ld19:
        lidar.disconnect()
    else:
        collector.disconnect()

    print("Done!")
    return 0


def main():
    parser = argparse.ArgumentParser(description="LiDAR GUI Diagnostic with Live Polar Map")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=230400, help="Baud rate (default: 230400 for LD19)")
    parser.add_argument("--headless", action="store_true", help="Run without GUI (save scans only)")
    parser.add_argument("--raw", action="store_true", help="Raw data mode (bypass LD19 driver)")
    parser.add_argument("--scans", "-n", type=int, default=0, help="Max scans in headless mode (0=unlimited)")

    args = parser.parse_args()

    print("=" * 50)
    print("AMBOT LiDAR Diagnostic")
    print("=" * 50)
    print()

    # Check for pyserial
    try:
        import serial
        print(f"PySerial version: {serial.__version__}")
    except ImportError:
        print("ERROR: PySerial not installed")
        print("Install with: pip3 install pyserial")
        return 1

    # Check display
    if not args.headless:
        if not detect_display():
            print("Falling back to headless mode")
            args.headless = True

    print()
    print(f"Port: {args.port}")
    print(f"Baud rate: {args.baud}")
    print(f"Mode: {'headless' if args.headless else 'GUI'} {'(raw)' if args.raw else '(LD19 driver)'}")
    if args.scans > 0:
        print(f"Max scans: {args.scans}")
    print()

    return run_lidar_gui(
        port=args.port,
        baudrate=args.baud,
        headless=args.headless,
        raw_mode=args.raw,
        max_scans=args.scans
    )


if __name__ == "__main__":
    sys.exit(main())
