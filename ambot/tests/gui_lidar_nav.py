#!/usr/bin/env python3
"""
LiDAR Navigation GUI — Front orientation calibration + movement intention.

Shows live LiDAR polar plot with:
  - Scan points colored by safety zone (red/orange/yellow/green)
  - Smoothed heading arrow (where robot would go, gold, EMA-filtered)
  - Front calibration mode: place object in front, press 'c' to calibrate
  - Optional motor output driven by behavior (max_clearance or natural_wander)

The front_offset_deg is saved to tests/results/lidar_calibration.json and can
be loaded automatically in future runs.

Usage:
    python3 tests/gui_lidar_nav.py                      # Live GUI
    python3 tests/gui_lidar_nav.py --headless -n 3       # Save 3 snapshots
    python3 tests/gui_lidar_nav.py --front-offset 90     # Override front angle
    python3 tests/gui_lidar_nav.py --motors              # Enable motor output
    python3 tests/gui_lidar_nav.py --motors --behavior natural_wander
    python3 tests/gui_lidar_nav.py --motors --max-speed 30 --swap-motors

Controls:
    c - Calibrate front (place object directly in front first!)
    p - Pause/resume
    s - Save screenshot
    r - Reset view
    q - Quit
    m - Toggle motors on/off (when --motors enabled)
    n - Cycle behavior (max_clearance / natural_wander)
    x - Toggle swap L/R motors
    v - Toggle invert left motor direction
    b - Toggle invert right motor direction
"""

import sys
import os
import time
import math
import json
import signal
import atexit
import argparse
import threading
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
CALIBRATION_FILE = RESULTS_DIR / "lidar_calibration.json"

sys.path.insert(0, str(SCRIPT_DIR.parent))

# Motor safety — global reference for emergency shutdown
_active_robot = None


def _emergency_motor_stop():
    """Stop motors on any exit — atexit, signal, or crash."""
    global _active_robot
    if _active_robot is not None:
        try:
            _active_robot.stop()
            _active_robot.cleanup()
        except Exception:
            pass
        _active_robot = None


def _signal_handler(signum, frame):
    """Handle SIGTERM/SIGINT — stop motors and exit cleanly."""
    _emergency_motor_stop()
    sys.exit(0)


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


def create_motor_robot(driver_type="L298N"):
    """Create a DifferentialDrive robot for motor output.

    Returns (robot, driver_name) or (None, error_string) on failure.
    """
    try:
        from locomotion.rpi_motors.drivers import DriverType
        from locomotion.rpi_motors.factory import create_robot

        driver_map = {
            "L298N": DriverType.L298N,
            "TB6612FNG": DriverType.TB6612FNG,
            "DRV8833": DriverType.DRV8833,
        }
        dt = driver_map.get(driver_type.upper())
        if dt is None:
            return None, f"Unknown driver: {driver_type}"

        robot = create_robot(driver_type=dt)
        return robot, driver_type.upper()
    except Exception as e:
        return None, str(e)


BEHAVIOR_NAMES = ["max_clearance", "natural_wander"]


def create_behavior(behavior_name):
    """Create a raw behavior instance (no SafetyWrapper for responsive testing)."""
    from pathfinder.behaviors import (
        MaxClearanceBehavior,
        NaturalWanderBehavior,
    )

    if behavior_name == "max_clearance":
        return MaxClearanceBehavior(forward_speed=0.4, turn_speed=0.5)
    elif behavior_name == "natural_wander":
        return NaturalWanderBehavior(forward_speed=0.4, turn_speed=0.5)
    else:
        raise ValueError(f"Unknown behavior: {behavior_name}")


def make_offset_points(raw_points, front_offset_deg):
    """Create scan points with front_offset applied to angles.

    The SectorBasedDetector expects 0 = front. Raw LiDAR angles may not
    align with the robot's physical front, so we adjust them here.
    """
    from pathfinder.lidar_ld19 import ScanPoint
    return [
        ScanPoint(
            angle=apply_offset(p.angle, front_offset_deg),
            distance=p.distance,
            quality=p.quality,
        )
        for p in raw_points
    ]


# Maximum points to display (subsample if more for performance)
MAX_DISPLAY_POINTS = 150


def run_lidar_nav(port="/dev/ttyUSB0", headless=False, max_scans=0, front_offset=None,
                  motors=False, driver_type="L298N", max_speed=40,
                  behavior_name="max_clearance",
                  swap_motors=False, invert_left=False, invert_right=False):
    """Run the LiDAR navigation GUI with optional motor output."""
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

    # Pre-import config outside animation loop
    from pathfinder import config as pf_config

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

    # Motor setup with safety handlers
    global _active_robot
    robot = None
    motors_active = False
    detector = None
    behavior = None

    if motors:
        print(f"Initializing motors ({driver_type}, max {max_speed}%)...")
        robot, info = create_motor_robot(driver_type)
        if robot is None:
            print(f"WARNING: Motor init failed: {info}")
            print("Continuing without motors")
        else:
            robot.stop()
            print(f"Motors ready: {info} (stopped on startup)")
            motors_active = True
            _active_robot = robot
            atexit.register(_emergency_motor_stop)
            signal.signal(signal.SIGTERM, _signal_handler)
            signal.signal(signal.SIGINT, _signal_handler)

        # Set up behavior pipeline: SectorBasedDetector -> Behavior -> MotorCommand
        try:
            from pathfinder.obstacle_detector import SectorBasedDetector
            detector = SectorBasedDetector()
            behavior = create_behavior(behavior_name)
            print(f"Behavior: {behavior_name}")
        except Exception as e:
            print(f"WARNING: Behavior init failed: {e}")
            print("Continuing without behavior-driven motors")
            motors_active = False

    # Shared scan state
    scan_state = {"points": [], "lock": threading.Lock(), "running": True}

    def lidar_collect():
        try:
            for scan in lidar.iter_scans(min_points=50):
                if not scan_state["running"]:
                    break
                with scan_state["lock"]:
                    scan_state["points"] = scan
        except Exception as e:
            print(f"\nLiDAR thread error: {e}")

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

        # Scan points (start with empty placeholder)
        scatter = ax.scatter([0], [0], s=3, alpha=0.7, zorder=5, c='#33cc33')

        # Smoothed heading arrow (gold line from center = "go this way")
        heading_arrow, = ax.plot([], [], color='gold', linewidth=4, alpha=0.9,
                                 zorder=18, solid_capstyle='round')
        heading_tip = ax.scatter([], [], s=200, c='gold', marker='o', zorder=19)

        ax.set_rmax(6000)
        ax.set_facecolor('#111111')
        ax.tick_params(colors='#888888', labelsize=7)
        ax.grid(True, alpha=0.15, color='#444444')
        ax.set_title("LiDAR Navigation", fontsize=12, pad=15, color='white')

        # Info text (top-left)
        info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                            verticalalignment='top', fontsize=9, color='#cccccc',
                            family='monospace')

        # Motor info text (bottom-left)
        motor_text = ax.text(0.02, 0.02, '', transform=ax.transAxes,
                             verticalalignment='bottom', fontsize=9, color='#cccccc',
                             family='monospace')

        state = {
            "paused": False,
            "frame_count": 0,
            "front_offset": front_offset_deg,
            "smooth_heading": None,
            "motors_active": motors_active,
            "behavior_name": behavior_name,
            "swap": swap_motors,
            "invert_left": invert_left,
            "invert_right": invert_right,
            "last_motor_time": time.time(),
            "last_cmd": (0, 0),
        }

        # Print controls once at startup
        controls = "\nControls: [C]alibrate  [P]ause  [S]ave  [R]eset  [Q]uit"
        if motors:
            controls += "\n          [M]otors toggle  [N]ext behavior  [X]swap L/R  [V]invert L  [B]invert R"
        print(controls + "\n")

        def update(frame_num):
            try:
                return _do_update(frame_num)
            except Exception as e:
                # Don't let exceptions kill the animation loop
                print(f"\rUpdate error: {e}       ", end="")
                return (scatter, heading_arrow, heading_tip, info_text, motor_text)

        def _do_update(frame_num):
            if state["paused"]:
                return (scatter, heading_arrow, heading_tip, info_text, motor_text)

            with scan_state["lock"]:
                points = list(scan_state["points"])

            if not points:
                info_text.set_text("Waiting for LiDAR data...")
                return (scatter, heading_arrow, heading_tip, info_text, motor_text)

            state["frame_count"] += 1
            offset = state["front_offset"]
            now = time.time()

            # Subsample for display performance (keep every Nth point)
            if len(points) > MAX_DISPLAY_POINTS:
                step = len(points) // MAX_DISPLAY_POINTS
                display_points = points[::step]
            else:
                display_points = points

            # Plot scan points with offset-adjusted angles
            angles = np.array([math.radians(apply_offset(p.angle, offset))
                               for p in display_points])
            distances = np.array([p.distance for p in display_points])

            if len(angles) == 0:
                return (scatter, heading_arrow, heading_tip, info_text, motor_text)

            scatter.set_offsets(np.column_stack([angles, distances]))

            # Color by safety zone
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

            # Max clearance -> smoothed heading arrow
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
                    steer_str = f"RIGHT {rel:.0f}\u00b0"
                else:
                    steer_str = f"LEFT {abs(rel):.0f}\u00b0"

            clear_str = f"{max_clear/1000:.1f}m" if max_clear else "?"
            info_text.set_text(
                f'{len(points)} pts  |  Clear: {clear_str}  |  {steer_str}'
            )

            # --- Motor control via behavior ---
            if (state["motors_active"] and robot is not None
                    and detector is not None and behavior is not None):
                try:
                    # Apply front_offset so detector sees 0 = robot front
                    adjusted = make_offset_points(points, offset)
                    detection = detector.process_scan(adjusted)
                    command = behavior.step(detection)

                    left_pwm = int(command.left_speed * max_speed)
                    right_pwm = int(command.right_speed * max_speed)
                    left_pwm = max(-max_speed, min(max_speed, left_pwm))
                    right_pwm = max(-max_speed, min(max_speed, right_pwm))

                    # Apply motor orientation adjustments
                    if state["swap"]:
                        left_pwm, right_pwm = right_pwm, left_pwm
                    if state["invert_left"]:
                        left_pwm = -left_pwm
                    if state["invert_right"]:
                        right_pwm = -right_pwm

                    robot.drive(left_pwm, right_pwm)
                    state["last_motor_time"] = now
                    state["last_cmd"] = (left_pwm, right_pwm)
                except Exception as e:
                    # Don't let behavior errors kill the animation
                    robot.stop()
                    state["last_cmd"] = (0, 0)
                    print(f"\rBehavior error: {e}       ", end="")

            elif state["motors_active"] and robot is not None:
                # Watchdog: stop if no valid command for 2 seconds
                if now - state["last_motor_time"] > 2.0:
                    robot.stop()
                    state["last_cmd"] = (0, 0)

            # Motor info display
            if motors:
                lp, rp = state["last_cmd"]
                active_str = "ON" if state["motors_active"] else "OFF"
                orient_parts = []
                if state["swap"]:
                    orient_parts.append("SWAP")
                if state["invert_left"]:
                    orient_parts.append("INV-L")
                if state["invert_right"]:
                    orient_parts.append("INV-R")
                orient_str = " " + " ".join(orient_parts) if orient_parts else ""
                motor_text.set_text(
                    f'Motor {active_str} ({max_speed}%)  L:{lp:+d} R:{rp:+d}'
                    f'  [{state["behavior_name"]}]{orient_str}'
                )
                motor_text.set_color('#ff9900' if state["motors_active"] else '#666666')
            else:
                motor_text.set_text('')

            return (scatter, heading_arrow, heading_tip, info_text, motor_text)

        def on_key(event):
            nonlocal behavior
            if event.key == 'q':
                if robot is not None:
                    robot.stop()
                plt.close()
            elif event.key == 'p':
                state["paused"] = not state["paused"]
                if state["paused"] and robot is not None:
                    robot.stop()
                    state["last_cmd"] = (0, 0)
                print(f"{'Paused' if state['paused'] else 'Resumed'}")
            elif event.key == 'c':
                with scan_state["lock"]:
                    cal_points = list(scan_state["points"])
                if cal_points:
                    raw_angle = find_nearest_cluster(cal_points, state["front_offset"])
                    if raw_angle is not None:
                        state["front_offset"] = raw_angle
                        state["smooth_heading"] = None
                        save_calibration(raw_angle)
                        print(f"CALIBRATED: front_offset = {raw_angle:.1f}\u00b0")
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
            elif event.key == 'm' and robot is not None:
                state["motors_active"] = not state["motors_active"]
                if not state["motors_active"]:
                    robot.stop()
                    state["last_cmd"] = (0, 0)
                print(f"Motors: {'ON' if state['motors_active'] else 'OFF'}")
            elif event.key == 'n' and robot is not None:
                cur_idx = BEHAVIOR_NAMES.index(state["behavior_name"])
                next_idx = (cur_idx + 1) % len(BEHAVIOR_NAMES)
                state["behavior_name"] = BEHAVIOR_NAMES[next_idx]
                behavior = create_behavior(state["behavior_name"])
                robot.stop()
                state["last_cmd"] = (0, 0)
                print(f"Behavior: {state['behavior_name']}")
            elif event.key == 'x' and robot is not None:
                state["swap"] = not state["swap"]
                print(f"Swap motors: {'ON' if state['swap'] else 'OFF'}")
            elif event.key == 'v' and robot is not None:
                state["invert_left"] = not state["invert_left"]
                print(f"Invert left motor: {'ON' if state['invert_left'] else 'OFF'}")
            elif event.key == 'b' and robot is not None:
                state["invert_right"] = not state["invert_right"]
                print(f"Invert right motor: {'ON' if state['invert_right'] else 'OFF'}")

        fig.canvas.mpl_connect('key_press_event', on_key)

        try:
            # 250ms interval (was 150) — better performance on RPi
            anim = FuncAnimation(fig, update, interval=250, blit=False, cache_frame_data=False)
            plt.show()
        except KeyboardInterrupt:
            print("\nStopped")
        finally:
            scan_state["running"] = False
            if robot is not None:
                print("Stopping motors...")
                _emergency_motor_stop()

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
  3. Press 'c' -- the nearest cluster becomes the new "front"
  4. Calibration saved to tests/results/lidar_calibration.json
  5. Future runs load this automatically

Gold arrow = smoothed heading (direction of max clearance).

Behaviors (--behavior):
  max_clearance    -- Always move toward longest distance (simple)
  natural_wander   -- Cycle through top clearance directions (explores more)

Motor orientation:
  Press 'x' to swap L/R, 'v' to invert left, 'b' to invert right.
  Press 'n' to cycle through behaviors at runtime.
        """
    )
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="LiDAR port (default: /dev/ttyUSB0)")
    parser.add_argument("--headless", action="store_true", help="No GUI, save snapshots")
    parser.add_argument("--scans", "-n", type=int, default=0, help="Max scans in headless (0=unlimited)")
    parser.add_argument("--front-offset", type=float, default=None,
                        help="Override front offset in degrees (default: load from calibration)")
    parser.add_argument("--motors", action="store_true", help="Enable motor output (requires GPIO)")
    parser.add_argument("--driver", type=str, default="L298N",
                        choices=["L298N", "TB6612FNG", "DRV8833"],
                        help="Motor driver type (default: L298N)")
    parser.add_argument("--max-speed", type=int, default=40,
                        help="Max motor speed %% (default: 40)")
    parser.add_argument("--behavior", type=str, default="max_clearance",
                        choices=BEHAVIOR_NAMES,
                        help="Navigation behavior (default: max_clearance)")
    parser.add_argument("--swap-motors", action="store_true",
                        help="Swap left/right motor assignments")
    parser.add_argument("--invert-left", action="store_true",
                        help="Invert left motor direction")
    parser.add_argument("--invert-right", action="store_true",
                        help="Invert right motor direction")

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

    if args.motors:
        print(f"Motor output: {args.driver} (max {args.max_speed}%)")
        print(f"Behavior: {args.behavior}")
        orient = []
        if args.swap_motors:
            orient.append("swap-LR")
        if args.invert_left:
            orient.append("invert-L")
        if args.invert_right:
            orient.append("invert-R")
        if orient:
            print(f"Motor orientation: {', '.join(orient)}")

    print()

    return run_lidar_nav(
        port=args.port,
        headless=args.headless,
        max_scans=args.scans,
        front_offset=args.front_offset,
        motors=args.motors,
        driver_type=args.driver,
        max_speed=args.max_speed,
        behavior_name=args.behavior,
        swap_motors=args.swap_motors,
        invert_left=args.invert_left,
        invert_right=args.invert_right,
    )


if __name__ == "__main__":
    sys.exit(main())
