#!/usr/bin/env python3
"""
Camera GUI Diagnostic with Face Detection

Displays live camera feed with OpenCV face detection.
Can be run with X11 forwarding or on local display.

Usage:
    python3 gui_camera.py                  # Default /dev/video0
    python3 gui_camera.py --device 1       # Use /dev/video1
    python3 gui_camera.py --headless       # No GUI, just save frames
    python3 gui_camera.py --cascade haarcascade_frontalface_alt.xml

Dependencies:
    pip3 install opencv-python-headless  # or opencv-python for full GUI

Controls:
    q - Quit
    s - Save screenshot
    f - Toggle face detection
    h - Toggle help overlay
"""

import sys
import time
import argparse
from datetime import datetime
from pathlib import Path

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)


def detect_display():
    """Check if display is available."""
    import os
    display = os.environ.get("DISPLAY")
    if display:
        print(f"Display detected: {display}")
        return True
    print("No display detected (DISPLAY env var not set)")
    return False


def load_face_cascade(cascade_path=None):
    """Load Haar cascade for face detection."""
    import cv2

    # Default cascades to try
    cascade_paths = [
        cascade_path,
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        str(Path(cv2.__file__).parent / "data" / "haarcascade_frontalface_default.xml"),
    ]

    for path in cascade_paths:
        if path and Path(path).exists():
            cascade = cv2.CascadeClassifier(path)
            if not cascade.empty():
                print(f"Loaded face cascade: {path}")
                return cascade

    print("WARNING: No face cascade found. Face detection disabled.")
    print("Install with: sudo apt-get install opencv-data")
    return None


def draw_info_overlay(frame, info_dict, show_help=False):
    """Draw information overlay on frame."""
    import cv2

    h, w = frame.shape[:2]

    # Semi-transparent background for text
    overlay = frame.copy()
    cv2.rectangle(overlay, (5, 5), (300, 120), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    # Draw info
    y = 25
    for key, value in info_dict.items():
        text = f"{key}: {value}"
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y += 20

    # Help overlay
    if show_help:
        help_text = [
            "Controls:",
            "  q - Quit",
            "  s - Save screenshot",
            "  f - Toggle face detection",
            "  h - Toggle this help",
        ]
        cv2.rectangle(overlay, (w-220, 5), (w-5, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        y = 25
        for line in help_text:
            cv2.putText(frame, line, (w-210, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            y += 18

    return frame


def run_camera_gui(device=0, cascade_path=None, headless=False, fps_limit=30, max_captures=0):
    """Run the camera GUI with face detection."""
    import cv2

    # Open camera
    print(f"Opening camera device {device}...")
    cap = cv2.VideoCapture(device)

    if not cap.isOpened():
        print(f"ERROR: Could not open camera device {device}")
        return 1

    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, fps_limit)

    # Get actual properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"Camera opened: {width}x{height} @ {fps}fps")

    # Load face cascade
    face_cascade = load_face_cascade(cascade_path)
    face_detection_enabled = face_cascade is not None

    # State
    show_help = True
    frame_count = 0
    face_count = 0
    capture_count = 0
    start_time = time.time()
    last_frame_time = start_time
    actual_fps = 0

    # Main loop
    print("Starting camera feed..." if not headless else "Running in headless mode...")
    if max_captures > 0 and headless:
        print(f"Will stop after {max_captures} captures")
    else:
        print("Press 'q' to quit")

    window_name = "Ambot Camera Diagnostic"

    try:
        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("ERROR: Failed to capture frame")
                break

            frame_count += 1
            current_time = time.time()

            # Calculate FPS
            if frame_count % 10 == 0:
                actual_fps = 10 / (current_time - last_frame_time)
                last_frame_time = current_time

            # Face detection
            faces = []
            if face_detection_enabled:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=(30, 30)
                )

                # Draw rectangles around faces
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(frame, "Face", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                face_count = len(faces)

            # Info overlay
            info = {
                "Device": f"/dev/video{device}",
                "Resolution": f"{width}x{height}",
                "FPS": f"{actual_fps:.1f}",
                "Faces": f"{face_count}" if face_detection_enabled else "disabled",
                "Frame": frame_count,
            }
            frame = draw_info_overlay(frame, info, show_help)

            # Display or save
            if not headless:
                cv2.imshow(window_name, frame)

                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    print("Quit requested")
                    break
                elif key == ord('s'):
                    filename = RESULTS_DIR / f"screenshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(str(filename), frame)
                    print(f"Screenshot saved: {filename}")
                elif key == ord('f'):
                    face_detection_enabled = not face_detection_enabled and face_cascade is not None
                    print(f"Face detection: {'enabled' if face_detection_enabled else 'disabled'}")
                elif key == ord('h'):
                    show_help = not show_help
            else:
                # Headless mode - save periodic snapshots
                # First capture at frame 1, then every 5 seconds (fps_limit * 5)
                capture_interval = fps_limit * 5
                if frame_count == 1 or frame_count % capture_interval == 0:
                    filename = RESULTS_DIR / f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(str(filename), frame)
                    capture_count += 1
                    print(f"Captured #{capture_count}: {filename} (faces: {face_count})")

                    if max_captures > 0 and capture_count >= max_captures:
                        print(f"Reached {max_captures} captures, stopping")
                        break

                # Check for Ctrl+C via timeout
                time.sleep(0.033)  # ~30fps

            # Frame rate limiting
            elapsed = time.time() - current_time
            sleep_time = max(0, (1/fps_limit) - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        # Cleanup
        elapsed = time.time() - start_time
        print(f"\nSession stats:")
        print(f"  Duration: {elapsed:.1f}s")
        print(f"  Frames: {frame_count}")
        print(f"  Average FPS: {frame_count/elapsed:.1f}")

        cap.release()
        if not headless:
            cv2.destroyAllWindows()

    return 0


def main():
    parser = argparse.ArgumentParser(description="Camera GUI Diagnostic with Face Detection")
    parser.add_argument("--device", "-d", type=int, default=0, help="Camera device number (default: 0)")
    parser.add_argument("--cascade", "-c", type=str, help="Path to Haar cascade XML file")
    parser.add_argument("--headless", action="store_true", help="Run without GUI (save frames only)")
    parser.add_argument("--fps", type=int, default=30, help="FPS limit (default: 30)")
    parser.add_argument("--captures", "-n", type=int, default=0, help="Max captures in headless mode (0=unlimited)")

    args = parser.parse_args()

    print("=" * 50)
    print("AMBOT Camera Diagnostic")
    print("=" * 50)
    print()

    # Check for OpenCV
    try:
        import cv2
        print(f"OpenCV version: {cv2.__version__}")
    except ImportError:
        print("ERROR: OpenCV not installed")
        print("Install with: pip3 install opencv-python-headless")
        return 1

    # Check display
    if not args.headless:
        if not detect_display():
            print("Falling back to headless mode")
            args.headless = True

    print()

    return run_camera_gui(
        device=args.device,
        cascade_path=args.cascade,
        headless=args.headless,
        fps_limit=args.fps,
        max_captures=args.captures
    )


if __name__ == "__main__":
    sys.exit(main())
