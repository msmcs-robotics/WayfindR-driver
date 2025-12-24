"""
Sheep Helper - Livestock Health Screening System
=================================================

A visual screening system for detecting potential health issues in sheep
using facial detection and pixel analysis. Designed for farmers working
at gates to receive audio alerts about sheep that may need attention.

Features:
- Face detection with unique ID tracking
- Compares new faces against saved known_faces database
- Pixel darkness analysis for necrotic tissue detection
- Audio alerts: Case 1 (healthy) vs Case 2 (alert)
- Automatic face image saving for identification

Detection Logic:
- Dark pixels (< threshold) indicate potential necrotic tissue
- Light pixels (> threshold) indicate healthy tissue
- More dark than light = Case 2 (ALERT) - play warning sound
- More light than dark = Case 1 (HEALTHY) - play confirmation

Run from Windows (not WSL) for webcam access:
    python main.py

Controls:
    q - Quit
    s - Save screenshot
    r - Refresh audio files
    m - Toggle mute
    c - Clear known faces tracking (reset alerts)
"""

import cv2
import time
import os
import sys
from datetime import datetime

from camera import Camera, print_camera_debug_info, get_system_info
from face_detector import FaceDetector
from visualizer import Visualizer
from pixel_analyzer import PixelAnalyzer, CASE_COLORS
from audio_player import AudioPlayer
from face_tracker import FaceTracker


def main():
    print("=" * 60)
    print("Sheep Helper - Livestock Health Screening System")
    print("=" * 60)
    print()

    # Check for debug flag
    if "--debug" in sys.argv or "-d" in sys.argv:
        print_camera_debug_info()
        return 0

    # Quick WSL check before trying camera
    sys_info = get_system_info()
    if sys_info["is_wsl"]:
        print("WARNING: Running in WSL!")
        print("WSL cannot access Windows webcams directly.")
        print()
        print("Options:")
        print("  1. Run on Windows: Open PowerShell, cd to this folder, run 'python main.py'")
        print("  2. Run with --debug to see full diagnostics")
        print()

    # Initialize all components
    print("Initializing components...")
    camera = Camera(camera_index=0, width=640, height=480)
    detector = FaceDetector(scale_factor=1.1, min_neighbors=5)
    visualizer = Visualizer()
    analyzer = PixelAnalyzer(darkness_threshold=80, light_threshold=170)
    audio = AudioPlayer()
    tracker = FaceTracker()

    # Settings
    BORDER_PERCENT = 0.10
    muted = False

    # Open camera
    print()
    print("Opening camera...")
    if not camera.open():
        print("ERROR: Could not open camera!")
        print()
        print("Run with --debug flag for full diagnostics:")
        print("  python main.py --debug")
        print()
        print_camera_debug_info()
        return 1

    width, height = camera.get_dimensions()
    print(f"Camera opened: {width}x{height}")
    print()
    print("Controls:")
    print("  q - Quit and save session log")
    print("  s - Save screenshot")
    print("  r - Refresh audio files")
    print("  m - Toggle mute (current: ON)")
    print("  c - Clear alert history (re-alert on same faces)")
    print()
    print("Screening active. Watching for faces...")
    print()

    # Create screenshots folder
    screenshots_dir = "screenshots"
    if not os.path.exists(screenshots_dir):
        os.makedirs(screenshots_dir)

    # FPS calculation
    fps_start_time = time.time()
    fps_frame_count = 0
    current_fps = 0

    # Track which faces have been alerted THIS SESSION (avoid repeat alerts)
    alerted_faces = set()

    try:
        while True:
            # Read frame
            ret, frame = camera.read()
            if not ret:
                print("Failed to read frame")
                break

            # Detect faces
            faces = detector.detect_faces(frame)

            # Process each detected face
            for (x, y, w, h) in faces:
                # Crop face image
                face_image = frame[y:y+h, x:x+w].copy()

                # Crop with border for pixel analysis
                cropped_with_border = analyzer.crop_with_border(frame, x, y, w, h, BORDER_PERCENT)
                analysis_result = analyzer.analyze_region(cropped_with_border)
                case = analysis_result['case']

                # Process face through tracker (compare with known faces)
                track_result = tracker.process_face(face_image, (x, y, w, h), frame)
                face_id = track_result['face_id']
                is_new = track_result['is_new']
                is_known = track_result['is_known']
                similarity = track_result['similarity']

                # Record detection
                tracker.record_detection(face_id, case)

                # Get color based on case
                color = CASE_COLORS.get(case, (255, 255, 255))

                # Draw face box with case-appropriate color
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

                # Draw border box (yellow)
                bx, by, bw, bh = detector.get_face_with_border(x, y, w, h, BORDER_PERCENT)
                visualizer.draw_border_box(frame, bx, by, bw, bh, width, height)

                # Draw face ID and case info
                case_name = analyzer.get_case_name(case)
                if is_known and not is_new:
                    label = f"ID:{face_id} {case_name} ({similarity:.0%})"
                else:
                    label = f"ID:{face_id} {case_name}"
                    if is_new:
                        label += " [NEW]"

                cv2.putText(frame, label, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Draw analysis stats
                stats = f"D:{analysis_result['dark_ratio']:.0%} L:{analysis_result['light_ratio']:.0%}"
                cv2.putText(frame, stats, (x, y + h + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

                # Play audio alert for NEW faces only (not previously alerted)
                if not muted and face_id not in alerted_faces:
                    if is_new:
                        # Brand new face we've never seen before
                        if case == PixelAnalyzer.CASE_ALERT:
                            print(f"[ALERT] NEW Face {face_id}: Potential issue detected! Playing alert...")
                            audio.play_alert()
                        elif case == PixelAnalyzer.CASE_HEALTHY:
                            print(f"[OK] NEW Face {face_id}: Healthy scan. Playing confirmation...")
                            audio.play_healthy()
                        alerted_faces.add(face_id)
                    elif is_known:
                        # Known face re-detected - just log, don't re-alert
                        # (unless you want to re-alert, remove this condition)
                        pass

            # Calculate FPS
            fps_frame_count += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 1.0:
                current_fps = fps_frame_count / elapsed
                fps_frame_count = 0
                fps_start_time = time.time()

            # Draw info overlay
            face_count = len(faces)
            known_count = tracker.get_known_face_count()
            mute_status = "[MUTED]" if muted else ""
            info_text = f"Faces: {face_count} | Known: {known_count} | FPS: {current_fps:.1f} {mute_status}"
            cv2.putText(frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Draw instructions
            cv2.putText(frame, "q:Quit s:Screenshot r:Refresh m:Mute c:Clear",
                       (10, height - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

            # Draw legend
            cv2.putText(frame, "GREEN=Healthy RED=Alert YELLOW=Border",
                       (10, height - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

            # Display frame
            cv2.imshow('Sheep Helper - Health Screening', frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print()
                print("Quitting...")
                break

            elif key == ord('s'):
                # Save screenshot
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(screenshots_dir, f"screening_{timestamp}.png")
                cv2.imwrite(filename, frame)
                print(f"Screenshot saved: {filename}")

            elif key == ord('r'):
                # Refresh audio files
                audio.refresh_files()
                print(f"Audio files refreshed: Case1={len(audio.case1_files)}, Case2={len(audio.case2_files)}")

            elif key == ord('m'):
                # Toggle mute
                muted = not muted
                print(f"Audio {'MUTED' if muted else 'UNMUTED'}")

            elif key == ord('c'):
                # Clear alerted faces (allow re-alerting)
                alerted_faces.clear()
                tracker.frame_tracks.clear()
                print("Alert history cleared - will re-alert on all faces")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        # Save session log
        print()
        print("Saving session data...")
        tracker.save_session_log()

        # Cleanup
        camera.release()
        cv2.destroyAllWindows()

    print()
    print("=" * 60)
    print("Session Summary:")
    print(f"  Total known faces: {tracker.get_known_face_count()}")
    print(f"  Faces alerted this session: {len(alerted_faces)}")
    print(f"  Images in known_faces folder: {len(list(tracker.known_faces_path.glob('sheep_*.png')))}")
    print("=" * 60)
    print()
    print("Done!")
    return 0


if __name__ == "__main__":
    exit(main())
