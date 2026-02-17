#!/usr/bin/env python3
"""
Face Tracking GUI — Camera feed with direction vectors and motor intention.

Optimized for RPi 3B (906MB RAM, Cortex-A53). Detects faces every Nth frame
and reuses results in between. Selects the face closest to horizontal center
for tracking. Draws a crosshair, bounding boxes on ALL faces, and direction
vectors from frame center to each face.

Usage:
    python3 tests/gui_face_tracker.py                  # Live GUI (no motors)
    python3 tests/gui_face_tracker.py --motors         # Live GUI + motor output
    python3 tests/gui_face_tracker.py --motors --max-speed 30  # Cap at 30%
    python3 tests/gui_face_tracker.py --motors --swap-motors   # Swap L/R assignment
    python3 tests/gui_face_tracker.py --headless -n 5  # Save 5 frames
    python3 tests/gui_face_tracker.py --dead-zone 50   # Wider dead zone

Controls:
    q - Quit
    s - Save screenshot
    m - Toggle motors on/off (when --motors enabled)
    x - Toggle swap L/R motors
    v - Toggle invert left motor direction
    b - Toggle invert right motor direction
    +/= - Increase tracking gain
    -/_ - Decrease tracking gain
"""

import sys
import os
import time
import math
import signal
import atexit
import argparse
from datetime import datetime
from pathlib import Path

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
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

# Detection tuning constants for RPi performance
DETECT_EVERY_N = 3        # Run face detection every Nth frame
DETECT_SCALE = 0.5        # Scale factor for detection (0.5 = half res)
DETECT_MIN_SIZE = (50, 50)  # Minimum face size at detection scale
DETECT_SCALE_FACTOR = 1.2  # Haar cascade scaleFactor (higher = faster, less accurate)
DETECT_MIN_NEIGHBORS = 4   # Haar cascade minNeighbors


def load_face_cascade():
    """Load Haar cascade for face detection."""
    import cv2

    search_paths = [
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
    ]
    try:
        data_dir = getattr(cv2, 'data', None)
        if data_dir and hasattr(data_dir, 'haarcascades'):
            search_paths.insert(0, data_dir.haarcascades + "haarcascade_frontalface_default.xml")
    except Exception:
        pass

    for path in search_paths:
        if Path(path).exists():
            cascade = cv2.CascadeClassifier(path)
            if not cascade.empty():
                return cascade
    return None


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


def draw_motor_intention(frame, steer, base_speed=0.5, bar_width=120, bar_height=20,
                         motors_active=False, max_speed=40):
    """
    Draw motor speed bars at bottom of frame.

    steer: -1.0 (full left) to +1.0 (full right), 0 = straight.
    Returns (left_speed, right_speed) as floats in [-1, 1].
    """
    import cv2
    h, w = frame.shape[:2]

    # Compute wheel speeds (differential drive)
    # Positive steer = face is RIGHT = turn RIGHT = left faster, right slower
    left_speed = base_speed + steer
    right_speed = base_speed - steer
    left_speed = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))

    # Panel background
    panel_y = h - 60
    cv2.rectangle(frame, (0, panel_y), (w, h), (30, 30, 30), -1)

    # Left motor bar
    lx = 20
    ly = panel_y + 8
    cv2.putText(frame, "L", (lx - 2, ly + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.rectangle(frame, (lx + 15, ly), (lx + 15 + bar_width, ly + bar_height), (80, 80, 80), -1)
    fill_w = int(abs(left_speed) * bar_width)
    color = (0, 200, 0) if left_speed >= 0 else (0, 0, 200)
    if left_speed >= 0:
        cv2.rectangle(frame, (lx + 15, ly), (lx + 15 + fill_w, ly + bar_height), color, -1)
    else:
        cv2.rectangle(frame, (lx + 15 + bar_width - fill_w, ly), (lx + 15 + bar_width, ly + bar_height), color, -1)
    cv2.rectangle(frame, (lx + 15, ly), (lx + 15 + bar_width, ly + bar_height), (150, 150, 150), 1)
    cv2.putText(frame, f"{left_speed:+.2f}", (lx + 15 + bar_width + 5, ly + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # Right motor bar
    rx = w - bar_width - 80
    ry = panel_y + 8
    cv2.putText(frame, "R", (rx - 2, ry + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.rectangle(frame, (rx + 15, ry), (rx + 15 + bar_width, ry + bar_height), (80, 80, 80), -1)
    fill_w = int(abs(right_speed) * bar_width)
    color = (0, 200, 0) if right_speed >= 0 else (0, 0, 200)
    if right_speed >= 0:
        cv2.rectangle(frame, (rx + 15, ry), (rx + 15 + fill_w, ry + bar_height), color, -1)
    else:
        cv2.rectangle(frame, (rx + 15 + bar_width - fill_w, ry), (rx + 15 + bar_width, ry + bar_height), color, -1)
    cv2.rectangle(frame, (rx + 15, ry), (rx + 15 + bar_width, ry + bar_height), (150, 150, 150), 1)
    cv2.putText(frame, f"{right_speed:+.2f}", (rx + 15 + bar_width + 5, ry + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # Steering direction text centered
    if abs(steer) < 0.05:
        direction = "STRAIGHT"
        dir_color = (0, 255, 0)
    elif steer < 0:
        direction = f"LEFT ({abs(steer):.2f})"
        dir_color = (0, 200, 255)
    else:
        direction = f"RIGHT ({abs(steer):.2f})"
        dir_color = (255, 200, 0)

    text_size = cv2.getTextSize(direction, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    tx = (w - text_size[0]) // 2
    cv2.putText(frame, direction, (tx, panel_y + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, dir_color, 1)

    # Motor status indicator
    if motors_active:
        motor_label = f"MOTORS ON ({max_speed}%)"
        motor_color = (0, 180, 255)  # orange
    else:
        motor_label = "MOTORS OFF"
        motor_color = (120, 120, 120)
    cv2.putText(frame, motor_label, (w - 160, panel_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, motor_color, 1)

    return left_speed, right_speed


def run_face_tracker(device=0, headless=False, max_captures=0, dead_zone=40, gain=0.8,
                     motors=False, driver_type="L298N", max_speed=40,
                     swap_motors=False, invert_left=False, invert_right=False):
    """Run the face tracking GUI with optional motor output."""
    import cv2

    # Open camera
    print(f"Opening camera device {device}...")
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera device {device}")
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera: {width}x{height}")

    frame_cx = width // 2
    frame_cy = height // 2
    half_w = width / 2.0

    # Load face cascade
    cascade = load_face_cascade()
    if cascade is None:
        print("ERROR: No face cascade available")
        cap.release()
        return 1
    print("Face cascade loaded")

    # Motor setup with safety handlers
    global _active_robot
    robot = None
    motors_active = False
    if motors:
        print(f"Initializing motors ({driver_type}, max {max_speed}%)...")
        robot, info = create_motor_robot(driver_type)
        if robot is None:
            print(f"WARNING: Motor init failed: {info}")
            print("Continuing without motors")
        else:
            # Immediately stop motors in case they're still running from a previous crash
            robot.stop()
            print(f"Motors ready: {info} (stopped on startup)")
            motors_active = True
            # Register safety handlers so motors stop on ANY exit
            _active_robot = robot
            atexit.register(_emergency_motor_stop)
            signal.signal(signal.SIGTERM, _signal_handler)
            signal.signal(signal.SIGINT, _signal_handler)

    # Print controls once at startup (instead of on-screen help overlay)
    print()
    controls = "Controls: q=Quit  s=Screenshot  +/-=Gain  [/]=Dead zone"
    if motors:
        controls += "  m=Toggle motors  x=Swap L/R  v=Invert L  b=Invert R"
    print(controls)
    print()

    # State
    frame_count = 0
    capture_count = 0
    start_time = time.time()
    last_fps_time = start_time
    actual_fps = 0.0
    last_motor_command_time = start_time  # Watchdog: stop motors if no command for 2s
    current_gain = gain
    current_dead_zone = dead_zone
    current_swap = swap_motors
    current_invert_left = invert_left
    current_invert_right = invert_right

    # Cached face detections (reused between detection frames)
    cached_faces = []

    window_name = "Ambot Face Tracker"

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("ERROR: Failed to capture frame")
                break

            frame_count += 1
            now = time.time()

            # FPS calculation
            if frame_count % 10 == 0:
                actual_fps = 10.0 / max(now - last_fps_time, 0.001)
                last_fps_time = now

            # --- Face detection (every Nth frame for performance) ---
            if frame_count % DETECT_EVERY_N == 1 or DETECT_EVERY_N == 1:
                # Scale down for faster detection
                small = cv2.resize(frame, None, fx=DETECT_SCALE, fy=DETECT_SCALE,
                                   interpolation=cv2.INTER_AREA)
                gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                detections = cascade.detectMultiScale(
                    gray,
                    scaleFactor=DETECT_SCALE_FACTOR,
                    minNeighbors=DETECT_MIN_NEIGHBORS,
                    minSize=DETECT_MIN_SIZE,
                )

                # Scale detections back to full resolution
                inv_scale = 1.0 / DETECT_SCALE
                cached_faces = []
                for (x, y, w, h) in detections:
                    fx = int(x * inv_scale)
                    fy = int(y * inv_scale)
                    fw = int(w * inv_scale)
                    fh = int(h * inv_scale)
                    cx = fx + fw // 2
                    cy = fy + fh // 2
                    # Distance from horizontal center (for tracking selection)
                    h_dist = abs(cx - frame_cx)
                    cached_faces.append((cx, cy, fw, fh, h_dist, fx, fy))

            faces = cached_faces

            # --- Draw crosshair (full frame, always visible) ---
            crosshair_color = (0, 255, 0)  # bright green
            cv2.line(frame, (frame_cx, 0), (frame_cx, height), crosshair_color, 1)
            cv2.line(frame, (0, frame_cy), (width, frame_cy), crosshair_color, 1)

            # --- Draw bounding boxes on ALL faces + vectors ---
            # Select tracked face: closest to horizontal center
            tracked_idx = -1
            if faces:
                tracked_idx = min(range(len(faces)), key=lambda i: faces[i][4])

            steer = 0.0

            for i, (cx, cy, fw, fh, h_dist, fx, fy) in enumerate(faces):
                is_tracked = (i == tracked_idx)

                # Bounding box
                if is_tracked:
                    box_color = (0, 255, 255)   # yellow for tracked face
                    box_thickness = 3
                else:
                    box_color = (0, 255, 0)     # green for other faces
                    box_thickness = 2

                cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), box_color, box_thickness)

                # Direction vector (arrow from frame center to face center)
                if is_tracked:
                    arrow_color = (0, 255, 255)  # yellow
                    arrow_thickness = 2
                else:
                    arrow_color = (0, 180, 0)    # darker green
                    arrow_thickness = 1

                cv2.arrowedLine(frame, (frame_cx, frame_cy), (cx, cy),
                                arrow_color, arrow_thickness, tipLength=0.06)

                # Label on tracked face
                if is_tracked:
                    cv2.putText(frame, "TRACK", (fx, fy - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # --- Compute steering from tracked face ---
            face_centered = False
            if tracked_idx >= 0:
                tcx, tcy, tfw, tfh, th_dist, tfx, tfy = faces[tracked_idx]

                # Check if frame center is inside the tracked face's bounding box.
                # If so, the face is "close enough" — stop motors.
                if (tfx <= frame_cx <= tfx + tfw) and (tfy <= frame_cy <= tfy + tfh):
                    face_centered = True
                    steer = 0.0
                else:
                    error_px = tcx - frame_cx
                    error_norm = error_px / half_w  # [-1, 1]

                    # Dead zone
                    if abs(error_px) < current_dead_zone:
                        steer = 0.0
                    else:
                        steer = error_norm * current_gain

                # Draw centered indicator
                if face_centered:
                    cv2.putText(frame, "CENTERED", (tfx, tfy - 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Motor intention panel + get computed speeds
            left_spd, right_spd = draw_motor_intention(
                frame, steer, motors_active=motors_active, max_speed=max_speed)

            # Drive motors if active
            if motors_active and robot is not None:
                if len(faces) == 0 or face_centered:
                    # No face or face centered — stop motors
                    robot.stop()
                else:
                    # Scale float speeds to int PWM, capped by max_speed
                    left_pwm = int(left_spd * max_speed)
                    right_pwm = int(right_spd * max_speed)
                    left_pwm = max(-max_speed, min(max_speed, left_pwm))
                    right_pwm = max(-max_speed, min(max_speed, right_pwm))

                    # Apply motor orientation adjustments
                    if current_swap:
                        left_pwm, right_pwm = right_pwm, left_pwm
                    if current_invert_left:
                        left_pwm = -left_pwm
                    if current_invert_right:
                        right_pwm = -right_pwm

                    robot.drive(left_pwm, right_pwm)
                    last_motor_command_time = now

                # Watchdog: stop motors if no drive command for 2 seconds
                if now - last_motor_command_time > 2.0:
                    robot.stop()

            # Show motor orientation config if any adjustments active
            if motors and (current_swap or current_invert_left or current_invert_right):
                orient_parts = []
                if current_swap:
                    orient_parts.append("SWAP")
                if current_invert_left:
                    orient_parts.append("INV-L")
                if current_invert_right:
                    orient_parts.append("INV-R")
                orient_text = "Motor: " + " ".join(orient_parts)
                cv2.putText(frame, orient_text, (8, height - 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 180, 255), 1)

            # Minimal info overlay (top-left, no background box to save draw time)
            info = f"FPS:{actual_fps:.0f} Faces:{len(faces)} Gain:{current_gain:.1f} DZ:{current_dead_zone}"
            cv2.putText(frame, info, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            # --- Display or save ---
            if not headless:
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('s'):
                    fname = RESULTS_DIR / f"face_track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(str(fname), frame)
                    print(f"Saved: {fname}")
                elif key in (ord('+'), ord('=')):
                    current_gain = min(2.0, current_gain + 0.1)
                    print(f"Gain: {current_gain:.2f}")
                elif key in (ord('-'), ord('_')):
                    current_gain = max(0.1, current_gain - 0.1)
                    print(f"Gain: {current_gain:.2f}")
                elif key == ord('m') and robot is not None:
                    motors_active = not motors_active
                    if not motors_active:
                        robot.stop()
                    print(f"Motors: {'ON' if motors_active else 'OFF'}")
                elif key == ord('x') and robot is not None:
                    current_swap = not current_swap
                    print(f"Swap motors: {'ON' if current_swap else 'OFF'}")
                elif key == ord('v') and robot is not None:
                    current_invert_left = not current_invert_left
                    print(f"Invert left motor: {'ON' if current_invert_left else 'OFF'}")
                elif key == ord('b') and robot is not None:
                    current_invert_right = not current_invert_right
                    print(f"Invert right motor: {'ON' if current_invert_right else 'OFF'}")
                elif key == ord(']'):
                    current_dead_zone = min(200, current_dead_zone + 10)
                    print(f"Dead zone: {current_dead_zone}px")
                elif key == ord('['):
                    current_dead_zone = max(10, current_dead_zone - 10)
                    print(f"Dead zone: {current_dead_zone}px")
            else:
                # Headless: save periodically
                if frame_count == 1 or frame_count % 30 == 0:
                    fname = RESULTS_DIR / f"face_track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(str(fname), frame)
                    capture_count += 1
                    print(f"Captured #{capture_count}: {fname} (faces: {len(faces)}, steer: {steer:+.3f})")
                    if max_captures > 0 and capture_count >= max_captures:
                        break
                time.sleep(0.033)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        elapsed = time.time() - start_time
        print(f"\nSession: {elapsed:.1f}s, {frame_count} frames, {frame_count/max(elapsed,0.1):.1f} avg fps")
        if robot is not None:
            print("Stopping motors...")
            _emergency_motor_stop()
        cap.release()
        if not headless:
            cv2.destroyAllWindows()

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Face Tracking GUI — direction vectors and motor intention",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--device", "-d", type=int, default=0, help="Camera device (default: 0)")
    parser.add_argument("--headless", action="store_true", help="No GUI, save frames only")
    parser.add_argument("--captures", "-n", type=int, default=0, help="Max captures in headless (0=unlimited)")
    parser.add_argument("--dead-zone", type=int, default=40, help="Dead zone in pixels (default: 40)")
    parser.add_argument("--gain", "-g", type=float, default=0.8, help="Steering gain (default: 0.8)")
    parser.add_argument("--motors", action="store_true", help="Enable motor output (requires GPIO)")
    parser.add_argument("--driver", type=str, default="L298N",
                        choices=["L298N", "TB6612FNG", "DRV8833"],
                        help="Motor driver type (default: L298N)")
    parser.add_argument("--max-speed", type=int, default=40,
                        help="Max motor speed %% (default: 40)")
    parser.add_argument("--swap-motors", action="store_true",
                        help="Swap left/right motor assignments")
    parser.add_argument("--invert-left", action="store_true",
                        help="Invert left motor direction")
    parser.add_argument("--invert-right", action="store_true",
                        help="Invert right motor direction")

    args = parser.parse_args()

    print("=" * 50)
    print("AMBOT Face Tracker")
    print("=" * 50)

    try:
        import cv2
        print(f"OpenCV: {cv2.__version__}")
    except ImportError:
        print("ERROR: OpenCV not installed")
        return 1

    if not args.headless:
        display = os.environ.get("DISPLAY")
        if not display:
            print("No display detected, falling back to headless")
            args.headless = True
        else:
            print(f"Display: {display}")

    if args.motors:
        print(f"Motor output: {args.driver} (max {args.max_speed}%)")
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
    return run_face_tracker(
        device=args.device,
        headless=args.headless,
        max_captures=args.captures,
        dead_zone=args.dead_zone,
        gain=args.gain,
        motors=args.motors,
        driver_type=args.driver,
        max_speed=args.max_speed,
        swap_motors=args.swap_motors,
        invert_left=args.invert_left,
        invert_right=args.invert_right,
    )


if __name__ == "__main__":
    sys.exit(main())
