#!/usr/bin/env python3
"""
Face Tracking GUI — Camera feed with direction vectors and motor intention.

Shows live camera feed with face detection. For the nearest face, draws
a direction vector showing where the robot would orient itself, plus
motor speed intention indicators.

The robot is on a stand (not actually moving) — this visualizes INTENTION only.

Usage:
    python3 tests/gui_face_tracker.py                # Live GUI
    python3 tests/gui_face_tracker.py --headless -n 5  # Save 5 frames
    python3 tests/gui_face_tracker.py --dead-zone 50   # Wider dead zone

Controls:
    q - Quit
    s - Save screenshot
    d - Toggle dead-zone indicator
    h - Toggle help overlay
    +/= - Increase tracking gain
    -/_ - Decrease tracking gain
"""

import sys
import os
import time
import math
import argparse
from datetime import datetime
from pathlib import Path

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
sys.path.insert(0, str(SCRIPT_DIR.parent))


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


def draw_motor_intention(frame, steer, base_speed=0.5, bar_width=120, bar_height=20):
    """
    Draw motor speed bars at bottom of frame.

    steer: -1.0 (full left) to +1.0 (full right), 0 = straight.
    """
    import cv2
    h, w = frame.shape[:2]

    # Compute wheel speeds (differential drive)
    left_speed = base_speed - steer
    right_speed = base_speed + steer
    left_speed = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))

    # Panel background
    panel_y = h - 70
    cv2.rectangle(frame, (0, panel_y), (w, h), (30, 30, 30), -1)

    # Left motor bar
    lx = 20
    ly = panel_y + 10
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
    ry = panel_y + 10
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

    # Steering direction text center
    if abs(steer) < 0.05:
        direction = "STRAIGHT"
        dir_color = (0, 255, 0)
    elif steer < 0:
        direction = f"TURN LEFT ({abs(steer):.2f})"
        dir_color = (0, 200, 255)
    else:
        direction = f"TURN RIGHT ({abs(steer):.2f})"
        dir_color = (255, 200, 0)

    text_size = cv2.getTextSize(direction, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
    tx = (w - text_size[0]) // 2
    cv2.putText(frame, direction, (tx, panel_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, dir_color, 2)

    # Steering arrow at center-bottom
    arrow_cx = w // 2
    arrow_cy = panel_y + 50
    arrow_len = 40
    angle = -steer * math.pi / 3  # map steer to ±60 degrees
    arrow_tip_x = int(arrow_cx + arrow_len * math.sin(angle))
    arrow_tip_y = int(arrow_cy - arrow_len * math.cos(angle))
    cv2.arrowedLine(frame, (arrow_cx, arrow_cy), (arrow_tip_x, arrow_tip_y), dir_color, 2, tipLength=0.3)


def draw_direction_vector(frame, frame_cx, frame_cy, face_cx, face_cy, dead_zone_px, show_dead_zone=True):
    """
    Draw the direction vector from frame center to nearest face.
    Returns the normalized horizontal steering value [-1, 1].
    """
    import cv2
    h, w = frame.shape[:2]
    half_w = w / 2

    # Horizontal error normalized to [-1, 1]
    error_px = face_cx - frame_cx
    error_norm = error_px / half_w

    # Dead zone
    in_dead_zone = abs(error_px) < dead_zone_px

    # Draw center crosshair (robot forward direction)
    cv2.line(frame, (frame_cx, 0), (frame_cx, h), (100, 100, 100), 1)
    cv2.line(frame, (0, frame_cy), (w, frame_cy), (100, 100, 100), 1)

    # Dead zone indicator
    if show_dead_zone:
        cv2.line(frame, (frame_cx - dead_zone_px, 0), (frame_cx - dead_zone_px, h), (80, 80, 80), 1)
        cv2.line(frame, (frame_cx + dead_zone_px, 0), (frame_cx + dead_zone_px, h), (80, 80, 80), 1)
        # Shaded dead zone
        overlay = frame.copy()
        cv2.rectangle(overlay, (frame_cx - dead_zone_px, 0), (frame_cx + dead_zone_px, h), (0, 100, 0), -1)
        cv2.addWeighted(overlay, 0.1, frame, 0.9, 0, frame)

    # Direction vector (arrow from center to face)
    if in_dead_zone:
        arrow_color = (0, 255, 0)  # Green = centered
        steer = 0.0
    else:
        arrow_color = (0, 0, 255)  # Red = needs correction
        steer = error_norm

    # Main direction arrow
    cv2.arrowedLine(frame, (frame_cx, frame_cy), (face_cx, face_cy),
                    arrow_color, 3, tipLength=0.08)

    # Horizontal offset bar at top
    bar_y = 30
    bar_half = w // 4
    bar_cx = w // 2
    # Background
    cv2.rectangle(frame, (bar_cx - bar_half, bar_y - 5), (bar_cx + bar_half, bar_y + 5), (60, 60, 60), -1)
    # Dead zone
    dz_bar = int(dead_zone_px / half_w * bar_half)
    cv2.rectangle(frame, (bar_cx - dz_bar, bar_y - 5), (bar_cx + dz_bar, bar_y + 5), (0, 80, 0), -1)
    # Indicator
    indicator_x = int(bar_cx + error_norm * bar_half)
    indicator_x = max(bar_cx - bar_half, min(bar_cx + bar_half, indicator_x))
    cv2.circle(frame, (indicator_x, bar_y), 8, arrow_color, -1)
    cv2.circle(frame, (indicator_x, bar_y), 8, (255, 255, 255), 1)

    return steer


def run_face_tracker(device=0, headless=False, max_captures=0, dead_zone=40, gain=0.8):
    """Run the face tracking GUI."""
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

    # Load face cascade
    cascade = load_face_cascade()
    if cascade is None:
        print("ERROR: No face cascade available")
        cap.release()
        return 1
    print("Face cascade loaded")

    # State
    show_help = True
    show_dead_zone = True
    frame_count = 0
    capture_count = 0
    start_time = time.time()
    last_fps_time = start_time
    actual_fps = 0.0
    current_gain = gain
    current_dead_zone = dead_zone

    window_name = "Ambot Face Tracker"

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("ERROR: Failed to capture frame")
                break

            frame_count += 1
            now = time.time()

            # FPS
            if frame_count % 10 == 0:
                actual_fps = 10.0 / (now - last_fps_time)
                last_fps_time = now

            # Face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            # Parse faces: list of (cx, cy, w, h, area)
            faces = []
            for (x, y, w, h) in detections:
                cx = x + w // 2
                cy = y + h // 2
                faces.append((cx, cy, w, h, w * h))

            # Draw all face bounding boxes
            for (cx, cy, w, h, area) in faces:
                x = cx - w // 2
                y = cy - h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, f"({cx},{cy})", (cx + 8, cy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

            # Track nearest face (largest area = closest)
            steer = 0.0
            if faces:
                nearest = max(faces, key=lambda f: f[4])
                ncx, ncy = nearest[0], nearest[1]

                # Highlight nearest face
                nx = ncx - nearest[2] // 2
                ny = ncy - nearest[3] // 2
                cv2.rectangle(frame, (nx, ny), (nx + nearest[2], ny + nearest[3]), (0, 255, 255), 3)
                cv2.putText(frame, "TRACKING", (nx, ny - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                # Draw direction vector and get steering value
                raw_steer = draw_direction_vector(
                    frame, frame_cx, frame_cy, ncx, ncy,
                    current_dead_zone, show_dead_zone
                )
                steer = raw_steer * current_gain
            else:
                # No face — just draw center crosshair and dead zone
                cv2.line(frame, (frame_cx, 0), (frame_cx, height), (100, 100, 100), 1)
                cv2.line(frame, (0, frame_cy), (width, frame_cy), (100, 100, 100), 1)
                if show_dead_zone:
                    cv2.line(frame, (frame_cx - current_dead_zone, 0),
                             (frame_cx - current_dead_zone, height), (80, 80, 80), 1)
                    cv2.line(frame, (frame_cx + current_dead_zone, 0),
                             (frame_cx + current_dead_zone, height), (80, 80, 80), 1)

            # Motor intention panel
            draw_motor_intention(frame, steer)

            # Info overlay (top-left)
            overlay = frame.copy()
            cv2.rectangle(overlay, (5, 45), (220, 140), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

            info_lines = [
                f"FPS: {actual_fps:.1f}  Faces: {len(faces)}",
                f"Gain: {current_gain:.2f}  Dead zone: {current_dead_zone}px",
                f"Steer: {steer:+.3f}",
                f"Frame: {frame_count}",
            ]
            y_pos = 62
            for line in info_lines:
                cv2.putText(frame, line, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                y_pos += 18

            # Help overlay (top-right)
            if show_help:
                overlay2 = frame.copy()
                cv2.rectangle(overlay2, (width - 230, 45), (width - 5, 155), (0, 0, 0), -1)
                cv2.addWeighted(overlay2, 0.5, frame, 0.5, 0, frame)
                help_lines = [
                    "q:Quit  s:Screenshot",
                    "d:Toggle dead zone",
                    "h:Toggle help",
                    "+/-: Adjust gain",
                    "[/]: Adjust dead zone",
                ]
                y_pos = 62
                for line in help_lines:
                    cv2.putText(frame, line, (width - 225, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
                    y_pos += 18

            # Display or save
            if not headless:
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('s'):
                    fname = RESULTS_DIR / f"face_track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(str(fname), frame)
                    print(f"Saved: {fname}")
                elif key == ord('d'):
                    show_dead_zone = not show_dead_zone
                elif key == ord('h'):
                    show_help = not show_help
                elif key in (ord('+'), ord('=')):
                    current_gain = min(2.0, current_gain + 0.1)
                    print(f"Gain: {current_gain:.2f}")
                elif key in (ord('-'), ord('_')):
                    current_gain = max(0.1, current_gain - 0.1)
                    print(f"Gain: {current_gain:.2f}")
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

    print()
    return run_face_tracker(
        device=args.device,
        headless=args.headless,
        max_captures=args.captures,
        dead_zone=args.dead_zone,
        gain=args.gain,
    )


if __name__ == "__main__":
    sys.exit(main())
