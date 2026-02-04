#!/usr/bin/env python3
"""
USB Camera Test Script

Tests USB camera connectivity and basic capture on Raspberry Pi.
Outputs results to tests/results/camera_test_results.txt
"""

import os
import sys
import subprocess
from datetime import datetime
from pathlib import Path

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
RESULT_FILE = RESULTS_DIR / "camera_test_results.txt"


def log(message: str, file=None):
    """Log to both stdout and file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{timestamp}] {message}"
    print(line)
    if file:
        file.write(line + "\n")
        file.flush()


def run_command(cmd: str) -> tuple:
    """Run a shell command and return (success, output)."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=30
        )
        return result.returncode == 0, result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return False, "Command timed out"
    except Exception as e:
        return False, str(e)


def test_video_devices(f):
    """Check for video devices."""
    log("Checking for video devices...", f)
    success, output = run_command("ls -la /dev/video* 2>&1")
    log(f"  /dev/video* devices:\n{output}", f)
    return success


def test_v4l2(f):
    """Test v4l2 tools."""
    log("Checking v4l2 tools...", f)

    # Check if v4l2-ctl is installed
    success, output = run_command("which v4l2-ctl")
    if not success:
        log("  v4l2-ctl not found. Install with: sudo apt install v4l-utils", f)
        return False

    # List devices
    success, output = run_command("v4l2-ctl --list-devices 2>&1")
    log(f"  v4l2-ctl --list-devices:\n{output}", f)

    return success


def test_camera_capture(f):
    """Test capturing a frame."""
    log("Testing camera capture...", f)

    # Check for OpenCV
    try:
        import cv2
        log(f"  OpenCV version: {cv2.__version__}", f)
    except ImportError:
        log("  OpenCV not installed. Install with: pip3 install opencv-python-headless", f)
        return False

    # Try to open camera
    for device_id in [0, 1, 2]:
        log(f"  Trying /dev/video{device_id}...", f)
        cap = cv2.VideoCapture(device_id)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                height, width = frame.shape[:2]
                log(f"  SUCCESS: Captured frame {width}x{height} from /dev/video{device_id}", f)

                # Save test image
                test_image_path = RESULTS_DIR / f"test_capture_video{device_id}.jpg"
                cv2.imwrite(str(test_image_path), frame)
                log(f"  Saved test image to: {test_image_path}", f)

                cap.release()
                return True
            cap.release()

    log("  FAILED: Could not capture from any video device", f)
    return False


def test_camera_info(f):
    """Get camera info via v4l2."""
    log("Getting camera info...", f)

    for device_id in [0, 1, 2]:
        device = f"/dev/video{device_id}"
        if os.path.exists(device):
            log(f"  Device: {device}", f)
            success, output = run_command(f"v4l2-ctl -d {device} --all 2>&1 | head -30")
            if success:
                log(f"{output}", f)


def main():
    """Run all camera tests."""
    with open(RESULT_FILE, "w") as f:
        log("=" * 60, f)
        log("USB CAMERA TEST - Raspberry Pi", f)
        log("=" * 60, f)
        log("", f)

        results = {}

        # Test 1: Video devices
        results["video_devices"] = test_video_devices(f)
        log("", f)

        # Test 2: v4l2 tools
        results["v4l2"] = test_v4l2(f)
        log("", f)

        # Test 3: Camera info
        test_camera_info(f)
        log("", f)

        # Test 4: Capture test
        results["capture"] = test_camera_capture(f)
        log("", f)

        # Summary
        log("=" * 60, f)
        log("TEST SUMMARY", f)
        log("=" * 60, f)
        for test_name, passed in results.items():
            status = "PASS" if passed else "FAIL"
            log(f"  {test_name}: {status}", f)

        all_passed = all(results.values())
        log("", f)
        log(f"Overall: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}", f)
        log(f"Results saved to: {RESULT_FILE}", f)

        return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
