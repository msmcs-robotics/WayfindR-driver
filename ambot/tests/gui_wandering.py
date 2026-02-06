#!/usr/bin/env python3
"""
Wandering Behavior Visualization GUI

Shows live LiDAR data with NaturalWanderBehavior overlay:
- Raw scan points (green dots)
- Clearance bins (colored wedges: green=clear, yellow=slow, red=stop)
- Target queue (numbered markers showing where the robot wants to go)
- Current target (bold arrow from center)
- Safety zones (concentric circles: STOP/SLOW/WARN)
- Motor command output (what the behavior would send to motors)

Usage (run ON RPi with display):
    python3 tests/gui_wandering.py                     # Live LiDAR + behavior viz
    python3 tests/gui_wandering.py --headless           # Save snapshots (no GUI)
    python3 tests/gui_wandering.py --headless --scans 5 # Save 5 snapshots

Via deploy.sh:
    ./deploy.sh rpi --test=wandering-viz

Controls (GUI mode):
    q - Quit
    p - Pause/resume
    n - Force next target (advance queue)
    r - Reset behavior state
    s - Save current frame
"""

import sys
import os
import time
import math
import argparse
import threading
from datetime import datetime
from pathlib import Path

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
sys.path.insert(0, str(SCRIPT_DIR.parent))

from pathfinder.obstacle_detector import SectorBasedDetector, SafetyLevel
from pathfinder.behaviors import NaturalWanderBehavior, MotorCommand, ClearanceTarget
from pathfinder import config


def run_wandering_gui(port="/dev/ttyUSB0", headless=False, max_scans=0):
    """Run the wandering behavior visualization."""
    import numpy as np

    try:
        import matplotlib
        if headless:
            matplotlib.use('Agg')
        else:
            matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.patches import Wedge
        import matplotlib.patches as mpatches
    except ImportError as e:
        print(f"ERROR: matplotlib not installed: {e}")
        return 1

    # Connect to LiDAR
    print("Connecting to LiDAR...")
    try:
        from pathfinder.lidar_ld19 import LD19Lidar
        lidar = LD19Lidar(port=port)
        if not lidar.connect():
            print("ERROR: Could not connect to LD19 LiDAR")
            return 1
        print("LD19 LiDAR connected")
    except ImportError:
        print("ERROR: LD19 driver not available")
        return 1

    # Create behavior and detector
    detector = SectorBasedDetector()
    behavior = NaturalWanderBehavior(
        forward_speed=0.4,
        turn_speed=0.5,
        bin_size=10.0,
        num_targets=10,
        min_angular_separation=30.0,
        target_hold_time=5.0,
    )

    # Shared state for LiDAR thread
    scan_state = {
        "points": [],
        "lock": threading.Lock(),
        "running": True,
    }

    # Background thread to collect scans
    def lidar_collect():
        for scan in lidar.iter_scans(min_points=50):
            if not scan_state["running"]:
                break
            with scan_state["lock"]:
                scan_state["points"] = scan

    lidar_thread = threading.Thread(target=lidar_collect, daemon=True)
    lidar_thread.start()

    # Wait for first scan
    print("Waiting for LiDAR data...")
    for _ in range(50):
        time.sleep(0.1)
        with scan_state["lock"]:
            if scan_state["points"]:
                break
    else:
        print("WARNING: No LiDAR data received after 5s")

    # Safety zone radii in mm
    stop_mm = config.STOP_DISTANCE * 1000
    slow_mm = config.SLOW_DISTANCE * 1000
    warn_mm = config.WARN_DISTANCE * 1000

    if headless:
        # Headless mode: process scans and save images
        scan_count = 0
        try:
            for scan in lidar.iter_scans(min_points=100):
                scan_count += 1
                detection = detector.process_scan(scan)
                command = behavior.step(detection)

                fig = _create_wandering_frame(
                    scan, detection, behavior, command,
                    stop_mm, slow_mm, warn_mm, scan_count
                )

                filename = RESULTS_DIR / f"wandering_viz_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                fig.savefig(filename, dpi=120)
                plt.close(fig)
                print(f"Scan {scan_count}: {len(scan)} pts | {command} | Saved: {filename}")

                if max_scans > 0 and scan_count >= max_scans:
                    break
        except KeyboardInterrupt:
            print("\nStopped by user")
    else:
        # Interactive GUI mode
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='polar')

        # Scan points scatter
        scatter = ax.scatter([], [], s=2, c='green', alpha=0.6, zorder=5)

        # Safety zone circles
        theta_circle = np.linspace(0, 2 * np.pi, 100)
        ax.plot(theta_circle, [stop_mm] * 100, 'r-', linewidth=1, alpha=0.4, label=f'STOP ({config.STOP_DISTANCE}m)')
        ax.plot(theta_circle, [slow_mm] * 100, color='orange', linewidth=1, alpha=0.4, label=f'SLOW ({config.SLOW_DISTANCE}m)')
        ax.plot(theta_circle, [warn_mm] * 100, 'y-', linewidth=1, alpha=0.3, label=f'WARN ({config.WARN_DISTANCE}m)')

        # Current target arrow (will be updated)
        target_arrow, = ax.plot([], [], 'c-', linewidth=3, alpha=0.9, zorder=15)
        target_marker = ax.scatter([], [], s=300, c='cyan', marker='*',
                                   edgecolors='black', linewidths=1, zorder=16)

        # Target queue markers
        queue_scatter = ax.scatter([], [], s=100, c='magenta', marker='D',
                                   edgecolors='white', linewidths=1, alpha=0.7, zorder=14)

        # Clearance bin wedges (will be drawn as bar chart)
        bin_bars = ax.bar([], [], width=0, bottom=0, alpha=0.3, zorder=3)

        ax.set_rmax(6000)
        ax.set_title("Wandering Behavior Visualization", fontsize=14, pad=20)
        ax.grid(True, alpha=0.3)

        # Info text panels
        state_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                             verticalalignment='top', fontsize=9,
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                             family='monospace')

        cmd_text = ax.text(0.98, 0.98, '', transform=ax.transAxes,
                           verticalalignment='top', horizontalalignment='right', fontsize=9,
                           bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.8),
                           family='monospace')

        legend = ax.legend(loc='lower left', fontsize=7, framealpha=0.7)

        paused = [False]
        frame_count = [0]

        def update(frame):
            if paused[0]:
                return (scatter, target_arrow, target_marker, queue_scatter,
                        state_text, cmd_text)

            # Get latest scan
            with scan_state["lock"]:
                scan_points = list(scan_state["points"])

            if not scan_points:
                return (scatter, target_arrow, target_marker, queue_scatter,
                        state_text, cmd_text)

            frame_count[0] += 1

            # Run detector and behavior
            detection = detector.process_scan(scan_points)
            command = behavior.step(detection)

            # --- Update scan points ---
            angles = np.array([math.radians(p.angle) for p in scan_points])
            distances = np.array([p.distance for p in scan_points])  # mm
            scatter.set_offsets(np.column_stack([angles, distances]))

            # Color by safety zone
            colors = []
            for d in distances:
                d_m = d / 1000.0
                if d_m <= config.STOP_DISTANCE:
                    colors.append('red')
                elif d_m <= config.SLOW_DISTANCE:
                    colors.append('orange')
                elif d_m <= config.WARN_DISTANCE:
                    colors.append('yellow')
                else:
                    colors.append('green')
            scatter.set_color(colors)

            # --- Update clearance bins overlay ---
            # Remove old bars
            for bar in ax.patches[:]:
                if hasattr(bar, '_is_clearance_bin'):
                    bar.remove()

            clearance_bins = behavior._build_clearance_profile(detection)
            bin_width = math.radians(behavior.bin_size) * 0.9
            for angle_deg, clearance_m in clearance_bins:
                angle_rad = math.radians(angle_deg)
                clearance_mm = clearance_m * 1000
                clearance_mm = min(clearance_mm, 6000)

                if clearance_m <= config.STOP_DISTANCE:
                    color = 'red'
                elif clearance_m <= config.SLOW_DISTANCE:
                    color = 'orange'
                elif clearance_m <= config.WARN_DISTANCE:
                    color = '#CCCC00'
                else:
                    color = '#00AA00'

                bar = ax.bar(angle_rad, clearance_mm, width=bin_width,
                             bottom=0, alpha=0.15, color=color, zorder=2)
                for b in bar:
                    b._is_clearance_bin = True

            # --- Update current target ---
            current = behavior._current_target
            if current:
                t_rad = math.radians(current.angle)
                t_mm = min(current.clearance * 1000, 6000)
                target_arrow.set_data([t_rad, t_rad], [0, t_mm])
                target_marker.set_offsets([[t_rad, t_mm]])
            else:
                target_arrow.set_data([], [])
                target_marker.set_offsets([])

            # --- Update target queue ---
            queue = behavior._target_queue
            if queue:
                q_angles = [math.radians(t.angle) for t in queue]
                q_dists = [min(t.clearance * 1000, 6000) for t in queue]
                queue_scatter.set_offsets(np.column_stack([q_angles, q_dists]))
            else:
                queue_scatter.set_offsets(np.empty((0, 2)))

            # --- Update info text ---
            target_info = f"Target: {current.angle:.0f}deg @ {current.clearance:.2f}m" if current else "Target: None"
            queue_info = f"Queue: {len(queue)} remaining"
            hold_elapsed = time.time() - behavior._target_start_time if current else 0
            hold_info = f"Hold: {hold_elapsed:.1f}s / {behavior.target_hold_time:.0f}s"

            state_text.set_text(
                f'BEHAVIOR STATE\n'
                f'{target_info}\n'
                f'{queue_info}\n'
                f'{hold_info}\n'
                f'Points: {len(scan_points)}\n'
                f'Safety: {detection.overall_safety.value}\n'
                f'Frame: {frame_count[0]}\n'
                f'[P]ause [N]ext [R]eset [S]ave [Q]uit'
            )

            # Motor command info
            if command.left_speed == 0 and command.right_speed == 0:
                cmd_str = "STOP"
            elif command.left_speed == command.right_speed:
                cmd_str = f"FWD {command.left_speed:.0%}"
            elif command.left_speed == -command.right_speed:
                if command.right_speed > 0:
                    cmd_str = f"TURN R {command.right_speed:.0%}"
                else:
                    cmd_str = f"TURN L {-command.right_speed:.0%}"
            else:
                cmd_str = f"L={command.left_speed:.0%} R={command.right_speed:.0%}"

            cmd_text.set_text(
                f'MOTOR CMD\n'
                f'{cmd_str}\n'
                f'Closest: {detection.closest_distance:.2f}m\n'
                f'Sector: {detection.closest_sector}'
            )

            return (scatter, target_arrow, target_marker, queue_scatter,
                    state_text, cmd_text)

        def on_key(event):
            if event.key == 'q':
                plt.close()
            elif event.key == 'p':
                paused[0] = not paused[0]
                print(f"{'Paused' if paused[0] else 'Resumed'}")
            elif event.key == 'n':
                behavior._advance_target(time.time())
                print(f"Advanced to next target: {behavior._current_target}")
            elif event.key == 'r':
                behavior.reset()
                print("Behavior reset")
            elif event.key == 's':
                filename = RESULTS_DIR / f"wandering_viz_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                fig.savefig(filename, dpi=150)
                print(f"Saved: {filename}")

        fig.canvas.mpl_connect('key_press_event', on_key)

        try:
            anim = FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
            plt.show()
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            scan_state["running"] = False

    # Cleanup
    scan_state["running"] = False
    lidar.disconnect()
    print("Done!")
    return 0


def _create_wandering_frame(scan_points, detection, behavior, command,
                             stop_mm, slow_mm, warn_mm, scan_num):
    """Create a single visualization frame for headless mode."""
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='polar')

    # Scan points
    if scan_points:
        angles = [math.radians(p.angle) for p in scan_points]
        distances = [p.distance for p in scan_points]
        colors = []
        for d in distances:
            d_m = d / 1000.0
            if d_m <= config.STOP_DISTANCE:
                colors.append('red')
            elif d_m <= config.SLOW_DISTANCE:
                colors.append('orange')
            elif d_m <= config.WARN_DISTANCE:
                colors.append('yellow')
            else:
                colors.append('green')
        ax.scatter(angles, distances, s=2, c=colors, alpha=0.6, zorder=5)

    # Safety circles
    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot(theta, [stop_mm] * 100, 'r-', linewidth=1, alpha=0.4)
    ax.plot(theta, [slow_mm] * 100, color='orange', linewidth=1, alpha=0.4)
    ax.plot(theta, [warn_mm] * 100, 'y-', linewidth=1, alpha=0.3)

    # Clearance bins
    clearance_bins = behavior._build_clearance_profile(detection)
    bin_width = math.radians(behavior.bin_size) * 0.9
    for angle_deg, clearance_m in clearance_bins:
        angle_rad = math.radians(angle_deg)
        clearance_mm = min(clearance_m * 1000, 6000)
        if clearance_m <= config.STOP_DISTANCE:
            color = 'red'
        elif clearance_m <= config.SLOW_DISTANCE:
            color = 'orange'
        else:
            color = '#00AA00'
        ax.bar(angle_rad, clearance_mm, width=bin_width, bottom=0,
               alpha=0.15, color=color, zorder=2)

    # Current target
    current = behavior._current_target
    if current:
        t_rad = math.radians(current.angle)
        t_mm = min(current.clearance * 1000, 6000)
        ax.plot([t_rad, t_rad], [0, t_mm], 'c-', linewidth=3, zorder=15)
        ax.scatter([t_rad], [t_mm], s=300, c='cyan', marker='*',
                   edgecolors='black', linewidths=1, zorder=16)

    # Queue targets
    queue = behavior._target_queue
    if queue:
        q_angles = [math.radians(t.angle) for t in queue]
        q_dists = [min(t.clearance * 1000, 6000) for t in queue]
        ax.scatter(q_angles, q_dists, s=100, c='magenta', marker='D',
                   edgecolors='white', linewidths=1, alpha=0.7, zorder=14)

    ax.set_rmax(6000)
    target_str = f"{current.angle:.0f}deg" if current else "None"
    ax.set_title(f"Wandering Viz - Scan #{scan_num} | Target: {target_str} | {command}",
                 fontsize=11)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(
        description="Wandering Behavior Visualization - shows NaturalWanderBehavior decisions on live LiDAR",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Shows the NaturalWanderBehavior's decision-making overlaid on live LiDAR:
  - Green dots: scan points (colored by safety zone)
  - Colored wedges: clearance bins (36 bins, 10 deg each)
  - Cyan star + arrow: current target direction
  - Magenta diamonds: queued targets
  - Concentric circles: STOP/SLOW/WARN safety zones

Controls (GUI mode):
  q - Quit           n - Next target (advance queue)
  p - Pause/resume   r - Reset behavior state
  s - Save frame
        """
    )
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0",
                        help="LiDAR serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--headless", action="store_true",
                        help="No GUI, save snapshot images")
    parser.add_argument("--scans", "-n", type=int, default=0,
                        help="Max scans in headless mode (0=unlimited)")

    args = parser.parse_args()

    print("=" * 50)
    print("AMBOT Wandering Behavior Visualization")
    print("=" * 50)

    return run_wandering_gui(
        port=args.port,
        headless=args.headless,
        max_scans=args.scans,
    )


if __name__ == "__main__":
    sys.exit(main())
