#!/usr/bin/env python3
"""
LiDAR Navigation GUI — Front orientation calibration + movement intention.

Shows live LiDAR polar plot with:
  - Configurable "front" marker (red arrow at angle 0 = physical front of robot)
  - Max clearance direction arrow (where robot would go)
  - Nearest obstacle marker
  - Front calibration mode: place object in front, press 'c' to calibrate

The front_offset_deg is saved to tests/results/lidar_calibration.json and can
be loaded automatically in future runs.

Usage:
    python3 tests/gui_lidar_nav.py                      # Live GUI
    python3 tests/gui_lidar_nav.py --headless -n 3       # Save 3 snapshots
    python3 tests/gui_lidar_nav.py --front-offset 90     # Override front angle

Controls:
    c - Calibrate front (place object directly in front first!)
    f - Toggle front marker
    p - Pause/resume
    s - Save screenshot
    r - Reset view
    q - Quit
"""

import sys
import os
import time
import math
import json
import argparse
import threading
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
CALIBRATION_FILE = RESULTS_DIR / "lidar_calibration.json"

sys.path.insert(0, str(SCRIPT_DIR.parent))


def load_calibration():
    """Load front offset from calibration file."""
    if CALIBRATION_FILE.exists():
        try:
            data = json.loads(CALIBRATION_FILE.read_text())
            offset = data.get("front_offset_deg", 0.0)
            print(f"Loaded calibration: front_offset_deg = {offset}")
            return offset
        except Exception as e:
            print(f"Warning: Could not load calibration: {e}")
    return 0.0


def save_calibration(front_offset_deg):
    """Save front offset to calibration file."""
    data = {
        "front_offset_deg": front_offset_deg,
        "calibrated_at": datetime.now().isoformat(),
        "note": "Angle offset so that LiDAR 0deg maps to physical front of robot",
    }
    CALIBRATION_FILE.write_text(json.dumps(data, indent=2))
    print(f"Calibration saved: front_offset_deg = {front_offset_deg}")


def find_nearest_cluster(scan_points, front_offset_deg, window_deg=60):
    """
    Find the nearest cluster of points within a window around the current front.

    For calibration: place an object directly in front, then call this to find
    the angle where that object appears in the LiDAR data.

    Returns the average angle (raw LiDAR degrees) of the nearest cluster.
    """
    if not scan_points:
        return None

    # Find the single nearest point
    min_dist = float('inf')
    nearest_angle = None
    for p in scan_points:
        if p.distance < min_dist and p.distance > 50:  # filter noise
            min_dist = p.distance
            nearest_angle = p.angle

    if nearest_angle is None:
        return None

    # Average the cluster around that point (±15 degrees, within 1.5x nearest dist)
    cluster_angles = []
    threshold = min_dist * 1.5
    for p in scan_points:
        if p.distance > 50 and p.distance < threshold:
            # Angular distance
            diff = (p.angle - nearest_angle + 180) % 360 - 180
            if abs(diff) < 15:
                cluster_angles.append(p.angle)

    if not cluster_angles:
        return nearest_angle

    # Circular mean
    sin_sum = sum(math.sin(math.radians(a)) for a in cluster_angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in cluster_angles)
    mean_angle = math.degrees(math.atan2(sin_sum, cos_sum)) % 360

    return mean_angle


def apply_offset(raw_angle, front_offset_deg):
    """Convert raw LiDAR angle to robot-relative angle (0 = front)."""
    return (raw_angle - front_offset_deg) % 360


def find_max_clearance(scan_points, front_offset_deg, num_bins=36):
    """
    Find the direction with maximum clearance using binned scan data.
    Returns (robot_relative_angle_deg, clearance_mm).
    """
    if not scan_points:
        return None, None

    bin_size = 360.0 / num_bins
    bins = [[] for _ in range(num_bins)]

    for p in scan_points:
        if p.distance < 50:
            continue
        robot_angle = apply_offset(p.angle, front_offset_deg)
        bin_idx = int(robot_angle / bin_size) % num_bins
        bins[bin_idx].append(p.distance)

    # Average distance per bin
    best_angle = None
    best_clearance = 0
    for i, b in enumerate(bins):
        if b:
            avg = sum(b) / len(b)
            if avg > best_clearance:
                best_clearance = avg
                best_angle = (i + 0.5) * bin_size
    return best_angle, best_clearance


def find_nearest_obstacle(scan_points, front_offset_deg):
    """Find nearest obstacle, return (robot_relative_angle, distance_mm)."""
    if not scan_points:
        return None, None

    min_dist = float('inf')
    nearest_angle = None
    for p in scan_points:
        if 50 < p.distance < min_dist:
            min_dist = p.distance
            nearest_angle = apply_offset(p.angle, front_offset_deg)
    return nearest_angle, min_dist


def run_lidar_nav(port="/dev/ttyUSB0", headless=False, max_scans=0, front_offset=None):
    """Run the LiDAR navigation GUI."""
    import numpy as np

    try:
        import matplotlib
        if headless:
            matplotlib.use('Agg')
        else:
            matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError as e:
        print(f"ERROR: matplotlib not installed: {e}")
        return 1

    # Load calibration
    if front_offset is not None:
        front_offset_deg = front_offset
        print(f"Using command-line front offset: {front_offset_deg}")
    else:
        front_offset_deg = load_calibration()

    # Connect LiDAR
    print("Connecting to LiDAR...")
    try:
        from pathfinder.lidar_ld19 import LD19Lidar
        lidar = LD19Lidar(port=port)
        if not lidar.connect():
            print("ERROR: Could not connect to LiDAR")
            return 1
        print("LD19 LiDAR connected")
    except ImportError:
        print("ERROR: LD19 driver not available")
        return 1

    # Shared scan state
    scan_state = {"points": [], "lock": threading.Lock(), "running": True}

    def lidar_collect():
        for scan in lidar.iter_scans(min_points=50):
            if not scan_state["running"]:
                break
            with scan_state["lock"]:
                scan_state["points"] = scan

    lidar_thread = threading.Thread(target=lidar_collect, daemon=True)
    lidar_thread.start()

    # Wait for data
    print("Waiting for LiDAR data...")
    for _ in range(50):
        time.sleep(0.1)
        with scan_state["lock"]:
            if scan_state["points"]:
                break

    if headless:
        scan_count = 0
        try:
            while True:
                time.sleep(1.0)
                with scan_state["lock"]:
                    points = list(scan_state["points"])
                if not points:
                    continue

                scan_count += 1
                max_angle, max_clear = find_max_clearance(points, front_offset_deg)
                near_angle, near_dist = find_nearest_obstacle(points, front_offset_deg)

                # Create snapshot
                fig = plt.figure(figsize=(10, 10))
                ax = fig.add_subplot(111, projection='polar')

                angles = np.array([math.radians(apply_offset(p.angle, front_offset_deg)) for p in points])
                distances = np.array([p.distance for p in points])
                ax.scatter(angles, distances, s=2, c='green', alpha=0.6)

                # Front marker
                ax.plot([0, 0], [0, 6000], 'r-', linewidth=2, alpha=0.5, label='FRONT')
                ax.scatter([0], [500], s=200, c='red', marker='^', zorder=20)

                # Max clearance arrow
                if max_angle is not None:
                    mr = math.radians(max_angle)
                    ax.plot([mr, mr], [0, min(max_clear, 6000)], 'c-', linewidth=3, alpha=0.8)
                    ax.scatter([mr], [min(max_clear, 6000)], s=300, c='cyan', marker='*', zorder=15)

                # Nearest obstacle
                if near_angle is not None and near_dist < 6000:
                    nr = math.radians(near_angle)
                    ax.scatter([nr], [near_dist], s=200, c='red', marker='o', edgecolors='white',
                               linewidths=2, zorder=15)

                ax.set_rmax(6000)
                ax.set_title(f"LiDAR Nav #{scan_count} | {len(points)} pts | Front offset: {front_offset_deg:.0f}deg")
                ax.grid(True, alpha=0.3)
                ax.legend(loc='upper right', fontsize=8)

                fname = RESULTS_DIR / f"lidar_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                fig.savefig(fname, dpi=120)
                plt.close(fig)

                clear_str = f"{max_clear:.0f}mm@{max_angle:.0f}deg" if max_angle else "N/A"
                near_str = f"{near_dist:.0f}mm@{near_angle:.0f}deg" if near_angle else "N/A"
                print(f"Scan {scan_count}: {len(points)} pts | MaxClear: {clear_str} | Nearest: {near_str} | Saved: {fname}")

                if max_scans > 0 and scan_count >= max_scans:
                    break

        except KeyboardInterrupt:
            print("\nStopped")

    else:
        # Interactive GUI mode
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='polar')

        scatter = ax.scatter([], [], s=2, c='green', alpha=0.6, zorder=5)

        # Front marker (static red line at 0 degrees)
        front_line, = ax.plot([0, 0], [0, 6000], 'r-', linewidth=2, alpha=0.5, label='FRONT')
        front_marker = ax.scatter([0], [500], s=200, c='red', marker='^', zorder=20)

        # Max clearance arrow
        clear_line, = ax.plot([], [], 'c-', linewidth=3, alpha=0.8, label='Max Clearance')
        clear_marker = ax.scatter([], [], s=300, c='cyan', marker='*', zorder=15)

        # Nearest obstacle
        nearest_marker = ax.scatter([], [], s=200, c='red', marker='o',
                                    edgecolors='white', linewidths=2, zorder=15, label='Nearest')
        nearest_line, = ax.plot([], [], 'r--', linewidth=1, alpha=0.5)

        # Movement intention arrow (thicker, from center toward max clearance, shorter)
        intent_arrow, = ax.plot([], [], color='yellow', linewidth=5, alpha=0.7, zorder=18)

        ax.set_rmax(6000)
        ax.set_title("LiDAR Navigation", fontsize=14, pad=20)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)

        # Info texts
        nav_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                           verticalalignment='top', fontsize=9,
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                           family='monospace')

        controls_text = ax.text(0.98, 0.02, '', transform=ax.transAxes,
                                verticalalignment='bottom', horizontalalignment='right', fontsize=8,
                                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7),
                                family='monospace')
        controls_text.set_text(
            '[C]alibrate front  [F]ront toggle\n'
            '[P]ause  [R]eset  [S]ave  [Q]uit'
        )

        state = {
            "paused": False,
            "show_front": True,
            "frame_count": 0,
            "front_offset": front_offset_deg,
        }

        def update(frame):
            if state["paused"]:
                return (scatter, front_line, front_marker, clear_line, clear_marker,
                        nearest_marker, nearest_line, intent_arrow, nav_text)

            with scan_state["lock"]:
                points = list(scan_state["points"])

            if not points:
                return (scatter, front_line, front_marker, clear_line, clear_marker,
                        nearest_marker, nearest_line, intent_arrow, nav_text)

            state["frame_count"] += 1
            offset = state["front_offset"]

            # Plot scan points in robot-relative coordinates
            angles = np.array([math.radians(apply_offset(p.angle, offset)) for p in points])
            distances = np.array([p.distance for p in points])
            scatter.set_offsets(np.column_stack([angles, distances]))

            # Color by distance
            from pathfinder import config as pf_config
            colors = []
            for d in distances:
                d_m = d / 1000.0
                if d_m <= pf_config.STOP_DISTANCE:
                    colors.append('red')
                elif d_m <= pf_config.SLOW_DISTANCE:
                    colors.append('orange')
                elif d_m <= pf_config.WARN_DISTANCE:
                    colors.append('yellow')
                else:
                    colors.append('green')
            scatter.set_color(colors)

            # Front marker visibility
            if state["show_front"]:
                front_line.set_data([0, 0], [0, 6000])
                front_marker.set_offsets([[0, 500]])
            else:
                front_line.set_data([], [])
                front_marker.set_offsets([])

            # Max clearance
            max_angle, max_clear = find_max_clearance(points, offset)
            if max_angle is not None:
                mr = math.radians(max_angle)
                mc = min(max_clear, 6000)
                clear_line.set_data([mr, mr], [0, mc])
                clear_marker.set_offsets([[mr, mc]])

                # Movement intention: shorter arrow in the max clearance direction
                intent_len = min(1500, mc * 0.4)
                intent_arrow.set_data([mr, mr], [0, intent_len])
            else:
                clear_line.set_data([], [])
                clear_marker.set_offsets([])
                intent_arrow.set_data([], [])

            # Nearest obstacle
            near_angle, near_dist = find_nearest_obstacle(points, offset)
            if near_angle is not None and near_dist < 6000:
                nr = math.radians(near_angle)
                nearest_marker.set_offsets([[nr, near_dist]])
                nearest_line.set_data([nr, nr], [0, near_dist])
            else:
                nearest_marker.set_offsets([])
                nearest_line.set_data([], [])

            # Compute steering from max clearance
            steer_str = "N/A"
            if max_angle is not None:
                # Normalize max_angle to -180..180 (0 = front)
                rel = max_angle if max_angle <= 180 else max_angle - 360
                if abs(rel) < 15:
                    steer_str = "STRAIGHT"
                elif rel > 0:
                    steer_str = f"TURN RIGHT {rel:.0f}deg"
                else:
                    steer_str = f"TURN LEFT {abs(rel):.0f}deg"

            clear_str = f"{max_clear:.0f}mm @ {max_angle:.0f}deg" if max_angle else "N/A"
            near_str = f"{near_dist:.0f}mm @ {near_angle:.0f}deg" if near_angle else "N/A"

            nav_text.set_text(
                f'NAVIGATION\n'
                f'Points: {len(points)}\n'
                f'Front offset: {offset:.0f}deg\n'
                f'Max clear: {clear_str}\n'
                f'Nearest: {near_str}\n'
                f'Intent: {steer_str}\n'
                f'Frame: {state["frame_count"]}'
            )

            return (scatter, front_line, front_marker, clear_line, clear_marker,
                    nearest_marker, nearest_line, intent_arrow, nav_text)

        def on_key(event):
            if event.key == 'q':
                plt.close()
            elif event.key == 'p':
                state["paused"] = not state["paused"]
                print(f"{'Paused' if state['paused'] else 'Resumed'}")
            elif event.key == 'f':
                state["show_front"] = not state["show_front"]
                print(f"Front marker: {'shown' if state['show_front'] else 'hidden'}")
            elif event.key == 'c':
                # Calibrate: find nearest cluster and set as front
                with scan_state["lock"]:
                    cal_points = list(scan_state["points"])
                if cal_points:
                    raw_angle = find_nearest_cluster(cal_points, state["front_offset"])
                    if raw_angle is not None:
                        state["front_offset"] = raw_angle
                        save_calibration(raw_angle)
                        print(f"CALIBRATED: front_offset_deg = {raw_angle:.1f}")
                        print("Place objects in front of robot and press 'c' again to recalibrate")
                    else:
                        print("Calibration failed: no valid points found")
                else:
                    print("Calibration failed: no scan data")
            elif event.key == 'r':
                state["front_offset"] = 0.0
                print("Reset front offset to 0")
            elif event.key == 's':
                fname = RESULTS_DIR / f"lidar_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                fig.savefig(fname, dpi=150)
                print(f"Saved: {fname}")

        fig.canvas.mpl_connect('key_press_event', on_key)

        try:
            anim = FuncAnimation(fig, update, interval=150, blit=False, cache_frame_data=False)
            plt.show()
        except KeyboardInterrupt:
            print("\nStopped")
        finally:
            scan_state["running"] = False

    # Cleanup
    scan_state["running"] = False
    lidar.disconnect()
    print("Done!")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="LiDAR Navigation GUI — front calibration + movement intention",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Calibration:
  1. Place a flat object (book, box) directly in front of the robot
  2. Start gui_lidar_nav.py
  3. Press 'c' — the nearest cluster becomes the new "front"
  4. Calibration saved to tests/results/lidar_calibration.json
  5. Future runs load this automatically

The red triangle at 0deg always points "forward" after calibration.
Cyan star = max clearance direction (where robot would go).
Yellow arrow = movement intention vector.
        """
    )
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="LiDAR port (default: /dev/ttyUSB0)")
    parser.add_argument("--headless", action="store_true", help="No GUI, save snapshots")
    parser.add_argument("--scans", "-n", type=int, default=0, help="Max scans in headless (0=unlimited)")
    parser.add_argument("--front-offset", type=float, default=None,
                        help="Override front offset in degrees (default: load from calibration)")

    args = parser.parse_args()

    print("=" * 50)
    print("AMBOT LiDAR Navigation")
    print("=" * 50)

    try:
        import serial
        print(f"PySerial: {serial.__version__}")
    except ImportError:
        print("ERROR: pyserial not installed")
        return 1

    if not args.headless:
        display = os.environ.get("DISPLAY")
        if not display:
            print("No display, falling back to headless")
            args.headless = True
        else:
            print(f"Display: {display}")

    print(f"Port: {args.port}")
    print()

    return run_lidar_nav(
        port=args.port,
        headless=args.headless,
        max_scans=args.scans,
        front_offset=args.front_offset,
    )


if __name__ == "__main__":
    sys.exit(main())
