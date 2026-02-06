#!/usr/bin/env python3
"""
Wandering Demo 2 - LiDAR + Camera Face Tracking + Motor Control

Builds on Demo 1 by adding camera-based face tracking to the wandering loop.
When a face is detected, the robot pauses wandering and pivots to center
the face horizontally on screen. When the face leaves or is lost for a
configurable timeout, the robot resumes autonomous wandering.

State machine:
    WANDERING  -> (face detected)    -> TRACKING
    TRACKING   -> (face lost > 2s)   -> WANDERING
    Both states: LiDAR safety override (STOP = emergency stop)

Components: LiDAR (LD19), Camera (OpenCV face detection), Motors (L298N)
See wandering_demo_1.py for the LiDAR-only version.

Usage:
    # Simulation mode (no motors, prints commands)
    python3 wandering_demo_2.py --simulate

    # Real mode with motors
    python3 wandering_demo_2.py

    # Disable camera (falls back to LiDAR-only wandering)
    python3 wandering_demo_2.py --no-camera

    # Adjust face tracking parameters
    python3 wandering_demo_2.py --dead-zone 50 --face-timeout 3.0
"""

import argparse
import logging
import sys
import threading
import time
from datetime import datetime
from enum import Enum
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Reuse RobotAdapter and helpers from demo 1
sys.path.insert(0, str(Path(__file__).parent))
from wandering_demo_1 import RobotAdapter, create_lidar, create_behavior


# =============================================================================
# Camera Face Detector Thread
# =============================================================================

class FaceData:
    """Thread-safe container for face detection results."""

    def __init__(self):
        self._lock = threading.Lock()
        self._faces = []  # list of (center_x, center_y, width, height)
        self._timestamp = 0.0
        self._frame_count = 0

    def update(self, faces, timestamp):
        with self._lock:
            self._faces = list(faces)
            self._timestamp = timestamp
            self._frame_count += 1

    def get(self):
        """Returns (faces, timestamp, frame_count)."""
        with self._lock:
            return list(self._faces), self._timestamp, self._frame_count


class CameraFaceThread(threading.Thread):
    """Background thread for camera face detection."""

    def __init__(self, face_data: FaceData, device: int = 0):
        super().__init__(daemon=True)
        self.face_data = face_data
        self.device = device
        self.running = False
        self.connected = False
        self.frame_width = 640
        self.frame_height = 480

    def run(self):
        self.running = True
        try:
            import cv2
        except ImportError:
            logger.error("OpenCV not available - camera disabled")
            return

        # Open camera
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            logger.error(f"Failed to open camera device {self.device}")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.connected = True
        logger.info(f"Camera opened: {self.frame_width}x{self.frame_height}")

        # Load face cascade
        face_cascade = self._load_cascade(cv2)
        if face_cascade is None:
            logger.error("Failed to load face cascade - camera disabled")
            cap.release()
            return

        logger.info("Face detection ready")

        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                # Detect faces
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = face_cascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=(30, 30),
                )

                faces = []
                if len(detections) > 0:
                    for (x, y, w, h) in detections:
                        center_x = int(x + w // 2)
                        center_y = int(y + h // 2)
                        faces.append((center_x, center_y, int(w), int(h)))

                self.face_data.update(faces, time.time())

                # ~10 fps for face detection
                time.sleep(0.1)

        finally:
            cap.release()
            logger.info("Camera released")

    def stop(self):
        self.running = False

    def _load_cascade(self, cv2):
        """Load Haar cascade for face detection."""
        search_paths = [
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ]

        # Try OpenCV package data
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
                    logger.info(f"Loaded face cascade: {path}")
                    return cascade

        return None


# =============================================================================
# State Machine
# =============================================================================

class DemoState(Enum):
    WANDERING = "wandering"
    TRACKING = "tracking"


# =============================================================================
# Main Demo
# =============================================================================

def run_wandering_v2(
    simulate: bool = False,
    behavior_name: str = "safe_wanderer",
    forward_speed: float = 0.4,
    turn_speed: float = 0.5,
    track_speed: float = 0.35,
    duration: float = 0,
    dead_zone: int = 40,
    face_timeout: float = 2.0,
    no_camera: bool = False,
    output_file: str = None,
):
    """
    Run the v2 wandering demo with face tracking.

    Args:
        simulate: If True, don't use real motors
        behavior_name: Wandering behavior to use
        forward_speed: Forward speed during wandering (0.0-1.0)
        turn_speed: Turn speed during wandering (0.0-1.0)
        track_speed: Max pivot speed during face tracking (0.0-1.0)
        duration: Run duration in seconds (0 = forever)
        dead_zone: Pixels from center considered "centered" (default 40)
        face_timeout: Seconds without face before resuming wander (default 2.0)
        no_camera: If True, disable camera (basic wandering only)
        output_file: File to write results to
    """
    from pathfinder.obstacle_detector import SectorBasedDetector, SafetyLevel
    from pathfinder.behaviors import MotorCommand

    FRAME_CENTER_X = 320  # 640 / 2

    print("=" * 60)
    print("AMBOT WANDERING DEMO V2 (Camera + LiDAR)")
    print("=" * 60)
    print(f"Mode: {'SIMULATION' if simulate else 'REAL HARDWARE'}")
    print(f"Behavior: {behavior_name}")
    print(f"Camera: {'DISABLED' if no_camera else 'ENABLED'}")
    print(f"Speeds: forward={forward_speed}, turn={turn_speed}, track={track_speed}")
    print(f"Face tracking: dead_zone={dead_zone}px, timeout={face_timeout}s")
    print(f"Duration: {duration}s" if duration > 0 else "Duration: until Ctrl+C")
    print("=" * 60)

    robot = None
    robot_adapter = None
    lidar = None
    camera_thread = None
    start_time = time.time()

    try:
        # Create robot adapter
        if not simulate:
            try:
                from locomotion.rpi_motors.factory import create_robot
                from locomotion.rpi_motors.drivers import DriverType

                logger.info("Creating motor controller...")
                robot = create_robot(driver_type=DriverType.L298N)
                robot_adapter = RobotAdapter(robot=robot, simulate=False)
                logger.info("Motor controller ready")
            except Exception as e:
                logger.warning(f"Failed to create motors: {e}")
                logger.info("Falling back to simulation mode")
                simulate = True

        if simulate:
            robot_adapter = RobotAdapter(robot=None, simulate=True)
            print("\n[SIMULATION MODE - Commands will be printed]\n")

        # Start camera thread
        face_data = FaceData()
        if not no_camera:
            logger.info("Starting camera thread...")
            camera_thread = CameraFaceThread(face_data)
            camera_thread.start()
            # Wait briefly for camera to initialize
            time.sleep(1.0)
            if not camera_thread.connected:
                logger.warning("Camera not connected - running without face tracking")
                no_camera = True

        # Create LiDAR
        logger.info("Connecting to LiDAR...")
        lidar = create_lidar()
        if not lidar.connect():
            raise RuntimeError("Failed to connect to LiDAR")
        logger.info("LiDAR connected")

        # Create detector and wandering behavior
        detector = SectorBasedDetector()
        wander_behavior = create_behavior(behavior_name, forward_speed, turn_speed)

        # State machine
        state = DemoState.WANDERING
        last_face_time = 0.0  # last time a face was seen
        scan_count = 0
        state_changes = 0
        face_track_count = 0

        print("\nStarting wandering loop... (Ctrl+C to stop)\n")

        end_time = start_time + duration if duration > 0 else float('inf')

        # Main loop - iterate over LiDAR scans
        for scan in lidar.iter_scans():
            now = time.time()
            if now > end_time:
                break

            loop_start = now
            scan_count += 1

            # Process LiDAR scan
            detection = detector.process_scan(scan)

            # Safety check - always applies
            if detection.overall_safety == SafetyLevel.STOP:
                robot_adapter.set_motors(0.0, 0.0)
                if scan_count % 10 == 0:
                    print(f"\r[SAFETY STOP] Obstacle at {detection.closest_distance:.2f}m", end="")
                # Rate limit
                elapsed = now - loop_start
                if elapsed < 0.1:
                    time.sleep(0.1 - elapsed)
                continue

            # Get face data
            faces, face_ts, _ = face_data.get()
            face_age = now - face_ts if face_ts > 0 else float('inf')
            has_fresh_face = len(faces) > 0 and face_age < 0.5  # face data < 500ms old

            # State machine
            if state == DemoState.WANDERING:
                if has_fresh_face and not no_camera:
                    # Face detected - switch to tracking
                    state = DemoState.TRACKING
                    last_face_time = now
                    state_changes += 1
                    logger.info(f"Face detected! Switching to TRACKING ({len(faces)} face(s))")
                else:
                    # Normal wandering
                    command = wander_behavior.step(detection)
                    robot_adapter.set_motors(command.left_speed, command.right_speed)

            elif state == DemoState.TRACKING:
                if has_fresh_face:
                    last_face_time = now
                    face_track_count += 1

                    # Pick largest face (closest person)
                    largest = max(faces, key=lambda f: f[2] * f[3])
                    face_cx = largest[0]  # center x

                    # Calculate horizontal offset from frame center
                    offset = face_cx - FRAME_CENTER_X  # positive = face is right of center

                    if abs(offset) <= dead_zone:
                        # Face is centered - hold position
                        robot_adapter.set_motors(0.0, 0.0)
                        if scan_count % 10 == 0:
                            print(f"\r[TRACKING] Face centered (offset={offset:+d}px)    ", end="")
                    else:
                        # Pivot toward face (proportional control)
                        # Normalize offset to -1.0..1.0 range
                        norm_offset = offset / FRAME_CENTER_X
                        pivot = track_speed * norm_offset
                        pivot = max(-track_speed, min(track_speed, pivot))

                        # Pivot: positive offset = face right = turn right
                        # turn_right = MotorCommand(+speed, -speed)
                        robot_adapter.set_motors(pivot, -pivot)
                        if scan_count % 10 == 0:
                            print(f"\r[TRACKING] Pivoting (offset={offset:+d}px, pivot={pivot:+.2f})", end="")

                else:
                    # No face - check timeout
                    time_since_face = now - last_face_time
                    if time_since_face > face_timeout:
                        # Face lost for too long - resume wandering
                        state = DemoState.WANDERING
                        state_changes += 1
                        wander_behavior.reset()
                        logger.info(f"Face lost for {time_since_face:.1f}s - resuming WANDERING")
                    else:
                        # Still waiting - hold position
                        robot_adapter.set_motors(0.0, 0.0)
                        if scan_count % 10 == 0:
                            remaining = face_timeout - time_since_face
                            print(f"\r[TRACKING] Face lost, waiting {remaining:.1f}s...     ", end="")

            # Periodic status
            if scan_count % 50 == 0:
                elapsed = now - start_time
                print(f"\n[{elapsed:.0f}s] State={state.value}, Scans={scan_count}, "
                      f"StateChanges={state_changes}, FaceTracks={face_track_count}")

            # Rate limit to ~10Hz
            elapsed = time.time() - loop_start
            if elapsed < 0.1:
                time.sleep(0.1 - elapsed)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    except Exception as e:
        logger.error(f"Error: {e}")
        raise

    finally:
        elapsed = time.time() - start_time
        print(f"\n\nRan for {elapsed:.1f} seconds")
        print(f"State changes: {state_changes}")
        print(f"Face tracking commands: {face_track_count}")

        if robot_adapter:
            stats = robot_adapter.get_stats()
            print(f"Total motor commands: {stats['command_count']}")
            robot_adapter.stop()
            robot_adapter.cleanup()

        if camera_thread:
            camera_thread.stop()

        if lidar:
            lidar.disconnect()

        # Save results
        if output_file:
            import json
            results = {
                "timestamp": datetime.now().isoformat(),
                "version": "v2",
                "duration_seconds": elapsed,
                "behavior": behavior_name,
                "simulate": simulate,
                "camera_enabled": not no_camera,
                "state_changes": state_changes,
                "face_track_count": face_track_count,
                "command_count": stats["command_count"] if robot_adapter else 0,
            }

            output_path = Path(output_file)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_path, "w") as f:
                json.dump(results, f, indent=2)
            print(f"Results saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Wandering Demo 2 - LiDAR + Camera Face Tracking",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
State machine:
  WANDERING  -> face detected    -> TRACKING (pivot to center face)
  TRACKING   -> face lost > 2s   -> WANDERING (resume obstacle avoidance)

Both states: LiDAR safety override (STOP = emergency stop)

Examples:
  python3 wandering_demo_2.py --simulate
  python3 wandering_demo_2.py --simulate --no-camera
  python3 wandering_demo_2.py --dead-zone 50 --face-timeout 3.0
        """
    )

    parser.add_argument("--simulate", "-s", action="store_true",
                        help="Simulation mode (no real motors)")
    parser.add_argument("--behavior", "-b", default="safe_wanderer",
                        choices=["safe_wanderer", "max_clearance", "wall_follower_right",
                                 "wall_follower_left", "random_wander", "avoid_and_go"],
                        help="Wandering behavior (default: safe_wanderer)")
    parser.add_argument("--forward-speed", type=float, default=0.4,
                        help="Forward speed 0.0-1.0 (default: 0.4)")
    parser.add_argument("--turn-speed", type=float, default=0.5,
                        help="Turn speed for wandering 0.0-1.0 (default: 0.5)")
    parser.add_argument("--track-speed", type=float, default=0.35,
                        help="Max pivot speed for face tracking 0.0-1.0 (default: 0.35)")
    parser.add_argument("--duration", "-d", type=float, default=0,
                        help="Run duration in seconds (0 = until Ctrl+C)")
    parser.add_argument("--dead-zone", type=int, default=40,
                        help="Face centering dead zone in pixels (default: 40)")
    parser.add_argument("--face-timeout", type=float, default=2.0,
                        help="Seconds without face before resuming wander (default: 2.0)")
    parser.add_argument("--no-camera", action="store_true",
                        help="Disable camera (falls back to basic wandering)")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Output file for results (JSON)")

    args = parser.parse_args()

    run_wandering_v2(
        simulate=args.simulate,
        behavior_name=args.behavior,
        forward_speed=args.forward_speed,
        turn_speed=args.turn_speed,
        track_speed=args.track_speed,
        duration=args.duration,
        dead_zone=args.dead_zone,
        face_timeout=args.face_timeout,
        no_camera=args.no_camera,
        output_file=args.output,
    )


if __name__ == "__main__":
    main()
