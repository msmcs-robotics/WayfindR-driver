#!/usr/bin/env python3
"""
LiDAR Navigation GUI — Front orientation calibration + movement intention.

Shows live LiDAR polar plot with:
  - Scan points colored by safety zone (red/orange/yellow/green)
  - Front marker (red triangle at 0 degrees = physical front of robot)
  - Smoothed heading arrow (where robot would go, yellow, EMA-filtered)
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
    Find the nearest cluster of points for calibration.
    Place an object directly in front, then call this to find
    the angle where that object appears in the LiDAR data.
    Returns the average angle (raw LiDAR degrees) of the nearest cluster.
    """
    if not scan_points:
        return None

    min_dist = float('inf')
    nearest_angle = None
    for p in scan_points:
        if p.distance < min_dist and p.distance > 50:
            min_dist = p.distance
            nearest_angle = p.angle

    if nearest_angle is None:
        return None

    # Average the cluster around that point
    cluster_angles = []
    threshold = min_dist * 1.5
    for p in scan_points:
        if p.distance > 50 and p.distance < threshold:
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

    best_angle = None
    best_clearance = 0
    for i, b in enumerate(bins):
        if b:
            avg = sum(b) / len(b)
            if avg > best_clearance:
                best_clearance = avg
                best_angle = (i + 0.5) * bin_size
    return best_angle, best_clearance


def smooth_angle(current, target, alpha=0.15):
    """
    Exponentially smooth an angle (handles 0/360 wraparound).
    alpha: smoothing factor (0=no change, 1=instant snap).
    """
    if current is None:
        return target
    if target is None:
        return current

    # Compute shortest angular difference
    diff = (target - current + 180) % 360 - 180
    return (current + alpha * diff) % 360


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

                fig = plt.figure(figsize=(8, 8))
                ax = fig.add_subplot(111, projection='polar')

                angles = np.array([math.radians(apply_offset(p.angle, front_offset_deg)) for p in points])
                distances = np.array([p.distance for p in points])
                ax.scatter(angles, distances, s=2, c='green', alpha=0.6)

                # Front marker
                ax.plot([0, 0], [0, 6000], 'r-', linewidth=2, alpha=0.5)
                ax.scatter([0], [500], s=200, c='red', marker='^', zorder=20)

                # Heading arrow
                if max_angle is not None:
                    mr = math.radians(max_angle)
                    intent_len = min(2000, max_clear * 0.5)
                    ax.annotate('', xy=(mr, intent_len), xytext=(0, 0),
                                arrowprops=dict(arrowstyle='->', color='gold', lw=3))

                ax.set_rmax(6000)
                ax.set_title(f"LiDAR Nav #{scan_count} | {len(points)} pts")
                ax.grid(True, alpha=0.3)

                fname = RESULTS_DIR / f"lidar_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                fig.savefig(fname, dpi=100)
                plt.close(fig)

                clear_str = f"{max_clear:.0f}mm@{max_angle:.0f}deg" if max_angle else "N/A"
                print(f"Scan {scan_count}: {len(points)} pts | Heading: {clear_str} | Saved: {fname}")

                if max_scans > 0 and scan_count >= max_scans:
                    break

        except KeyboardInterrupt:
            print("\nStopped")

    else:
        # Interactive GUI mode
        fig = plt.figure(figsize=(8, 8), facecolor='black')
        ax = fig.add_subplot(111, projection='polar', facecolor='#111111')

        # Scan points
        scatter = ax.scatter([], [], s=3, alpha=0.7, zorder=5)

        # Front marker (red line at 0 degrees)
        front_line, = ax.plot([0, 0], [0, 6000], 'r-', linewidth=2, alpha=0.4)
        front_marker = ax.scatter([0], [400], s=150, c='red', marker='^', zorder=20)

        # Smoothed heading arrow (single yellow arrow from center)
        heading_arrow, = ax.plot([], [], color='gold', linewidth=4, alpha=0.9,
                                 zorder=18, solid_capstyle='round')
        heading_tip = ax.scatter([], [], s=200, c='gold', marker='o', zorder=19)

        ax.set_rmax(6000)
        ax.set_facecolor('#111111')
        ax.tick_params(colors='#888888', labelsize=7)
        ax.grid(True, alpha=0.15, color='#444444')
        ax.set_title("LiDAR Navigation", fontsize=12, pad=15, color='white')

        # Minimal info text
        info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                            verticalalignment='top', fontsize=9, color='#cccccc',
                            family='monospace')

        state = {
            "paused": False,
            "show_front": True,
            "frame_count": 0,
            "front_offset": front_offset_deg,
            "smooth_heading": None,  # EMA-smoothed heading angle
        }

        # Print controls once at startup (not cluttering the plot)
        print("\nControls: [C]alibrate  [F]ront toggle  [P]ause  [S]ave  [R]eset  [Q]uit\n")

        def update(frame):
            if state["paused"]:
                return (scatter, front_line, front_marker, heading_arrow, heading_tip, info_text)

            with scan_state["lock"]:
                points = list(scan_state["points"])

            if not points:
                return (scatter, front_line, front_marker, heading_arrow, heading_tip, info_text)

            state["frame_count"] += 1
            offset = state["front_offset"]

            # Plot scan points
            angles = np.array([math.radians(apply_offset(p.angle, offset)) for p in points])
            distances = np.array([p.distance for p in points])
            scatter.set_offsets(np.column_stack([angles, distances]))

            # Color by safety zone
            from pathfinder import config as pf_config
            colors = []
            for d in distances:
                d_m = d / 1000.0
                if d_m <= pf_config.STOP_DISTANCE:
                    colors.append('#ff3333')
                elif d_m <= pf_config.SLOW_DISTANCE:
                    colors.append('#ff8800')
                elif d_m <= pf_config.WARN_DISTANCE:
                    colors.append('#ffdd00')
                else:
                    colors.append('#33cc33')
            scatter.set_color(colors)

            # Front marker
            if state["show_front"]:
                front_line.set_data([0, 0], [0, 6000])
                front_marker.set_offsets([[0, 400]])
                front_line.set_visible(True)
                front_marker.set_visible(True)
            else:
                front_line.set_visible(False)
                front_marker.set_visible(False)

            # Max clearance → smoothed heading
            max_angle, max_clear = find_max_clearance(points, offset)
            if max_angle is not None:
                state["smooth_heading"] = smooth_angle(
                    state["smooth_heading"], max_angle, alpha=0.15
                )
                heading = state["smooth_heading"]
                hr = math.radians(heading)
                arrow_len = min(2000, max_clear * 0.4)
                heading_arrow.set_data([hr, hr], [0, arrow_len])
                heading_tip.set_offsets([[hr, arrow_len]])
                heading_arrow.set_visible(True)
                heading_tip.set_visible(True)
            else:
                heading_arrow.set_visible(False)
                heading_tip.set_visible(False)

            # Steering text
            steer_str = ""
            if state["smooth_heading"] is not None:
                rel = state["smooth_heading"]
                if rel > 180:
                    rel -= 360
                if abs(rel) < 15:
                    steer_str = "STRAIGHT"
                elif rel > 0:
                    steer_str = f"RIGHT {rel:.0f}°"
                else:
                    steer_str = f"LEFT {abs(rel):.0f}°"

            clear_str = f"{max_clear/1000:.1f}m" if max_clear else "?"
            info_text.set_text(
                f'{len(points)} pts  |  Clear: {clear_str}  |  {steer_str}'
            )

            return (scatter, front_line, front_marker, heading_arrow, heading_tip, info_text)

        def on_key(event):
            if event.key == 'q':
                plt.close()
            elif event.key == 'p':
                state["paused"] = not state["paused"]
                print(f"{'Paused' if state['paused'] else 'Resumed'}")
            elif event.key == 'f':
                state["show_front"] = not state["show_front"]
            elif event.key == 'c':
                with scan_state["lock"]:
                    cal_points = list(scan_state["points"])
                if cal_points:
                    raw_angle = find_nearest_cluster(cal_points, state["front_offset"])
                    if raw_angle is not None:
                        state["front_offset"] = raw_angle
                        state["smooth_heading"] = None  # reset smoothing
                        save_calibration(raw_angle)
                        print(f"CALIBRATED: front_offset = {raw_angle:.1f}°")
                    else:
                        print("Calibration failed: no valid points")
                else:
                    print("Calibration failed: no scan data")
            elif event.key == 'r':
                state["front_offset"] = 0.0
                state["smooth_heading"] = None
                print("Reset front offset to 0")
            elif event.key == 's':
                fname = RESULTS_DIR / f"lidar_nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                fig.savefig(fname, dpi=150, facecolor='black')
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

Red triangle = front. Gold arrow = smoothed heading intention.
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
