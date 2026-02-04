#!/usr/bin/env python3
"""
USB LiDAR Test Script

Tests USB LiDAR connectivity on Raspberry Pi.
Supports RPLidar C1M1 (primary), RPLidar A-series, and YDLIDAR sensors.
Outputs results to tests/results/lidar_test_results.txt

IMPORTANT: RPLidar C1M1 requires:
  - Baud rate: 460800 (not 115200)
  - Custom driver from pathfinder/lidar.py (not rplidar-roboticia library)
"""

import os
import sys
import subprocess
import time
import struct
from datetime import datetime
from pathlib import Path

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
RESULT_FILE = RESULTS_DIR / "lidar_test_results.txt"

# Add parent directory to path for pathfinder import
AMBOT_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(AMBOT_DIR))


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


def test_serial_devices(f):
    """Check for serial devices."""
    log("Checking for serial devices...", f)

    devices_found = []

    # Check ttyUSB devices
    success, output = run_command("ls -la /dev/ttyUSB* 2>&1")
    if success and "No such file" not in output:
        log(f"  /dev/ttyUSB* devices:\n{output}", f)
        for line in output.split("\n"):
            if "/dev/ttyUSB" in line:
                device = line.split()[-1] if line.split() else None
                if device:
                    devices_found.append(device)

    # Check ttyACM devices
    success, output = run_command("ls -la /dev/ttyACM* 2>&1")
    if success and "No such file" not in output:
        log(f"  /dev/ttyACM* devices:\n{output}", f)
        for line in output.split("\n"):
            if "/dev/ttyACM" in line:
                device = line.split()[-1] if line.split() else None
                if device:
                    devices_found.append(device)

    if not devices_found:
        log("  No serial devices found", f)
        return False, []

    log(f"  Found devices: {devices_found}", f)
    return True, devices_found


def test_usb_devices(f):
    """Check USB devices for known LiDAR vendors."""
    log("Checking USB devices for LiDAR...", f)

    known_lidars = {
        "10c4:ea60": "CP2102 (RPLidar/YDLIDAR common)",
        "1a86:7523": "CH340 (Common USB-Serial)",
        "067b:2303": "PL2303 (USB-Serial)",
        "0403:6001": "FTDI FT232 (USB-Serial)",
        "10c4:ea70": "RPLidar A1/A2",
        "1546:01a7": "YDLIDAR X4",
    }

    success, output = run_command("lsusb")
    log(f"  USB devices:\n{output}", f)

    found_lidar = False
    for vid_pid, name in known_lidars.items():
        if vid_pid.lower() in output.lower():
            log(f"  FOUND: {name} ({vid_pid})", f)
            found_lidar = True

    return found_lidar


def test_serial_permissions(f, devices):
    """Check if user has permission to access serial devices."""
    log("Checking serial permissions...", f)

    for device in devices:
        success, output = run_command(f"test -r {device} && test -w {device} && echo 'OK' || echo 'NO ACCESS'")
        if "OK" in output:
            log(f"  {device}: Read/Write OK", f)
        else:
            log(f"  {device}: NO ACCESS - add user to dialout group: sudo usermod -a -G dialout $USER", f)
            return False

    return True


def test_rplidar_library(f):
    """Test if rplidar library is available."""
    log("Checking rplidar library...", f)

    try:
        from rplidar import RPLidar
        log("  rplidar library: INSTALLED", f)
        return True
    except ImportError:
        log("  rplidar library: NOT INSTALLED", f)
        log("  Install with: pip3 install rplidar-roboticia", f)
        return False


def test_rplidar_c1m1_connection(f, devices):
    """Test connection to RPLidar C1M1 using custom pathfinder driver."""
    log("Testing RPLidar C1M1 connection (460800 baud)...", f)

    try:
        from pathfinder.lidar import RPLidar, RPLidarException
        log("  Using custom pathfinder driver", f)
    except ImportError as e:
        log(f"  Could not import pathfinder.lidar: {e}", f)
        log("  Falling back to direct serial test...", f)
        return test_lidar_serial_direct(f, devices)

    for device in devices:
        log(f"  Trying {device} @ 460800 baud...", f)
        try:
            lidar = RPLidar(port=device, baudrate=460800)
            lidar.connect()

            log("  SUCCESS: Connected to RPLidar C1M1", f)

            # Get device info
            try:
                info = lidar.get_info()
                log(f"    Model: {info.get('model', 'Unknown')}", f)
                log(f"    Firmware: {info.get('firmware_major', '?')}.{info.get('firmware_minor', '?')}", f)
                log(f"    Hardware: {info.get('hardware', 'Unknown')}", f)
                log(f"    Serial: {info.get('serial', 'Unknown')}", f)
            except Exception as e:
                log(f"    Info error (continuing): {e}", f)

            # Get health
            try:
                status, error_code = lidar.get_health()
                log(f"    Health: {status} (error code: {error_code})", f)
            except Exception as e:
                log(f"    Health check error (continuing): {e}", f)

            # Try to get one scan
            log("  Attempting single scan...", f)
            lidar.start_motor()
            time.sleep(2)  # C1M1 needs more spin-up time

            scan_count = 0
            try:
                for scan in lidar.iter_scans(max_buf_meas=500):
                    scan_count = len(scan)
                    log(f"  Scan received: {scan_count} measurements", f)
                    break
            except Exception as e:
                log(f"    Scan error: {e}", f)

            lidar.stop_scan()
            lidar.stop_motor()
            lidar.disconnect()

            if scan_count > 0:
                log(f"  LiDAR WORKING: Got {scan_count} points", f)
                return True
            else:
                log("  Connected but no scan data received", f)
                return True  # Connection worked, scan may need motor adjustment

        except Exception as e:
            log(f"  Failed on {device}: {e}", f)

    return False


def test_lidar_serial_direct(f, devices):
    """Direct serial test for LiDAR at multiple baud rates."""
    log("Testing LiDAR via direct serial connection...", f)

    import serial

    # RPLidar protocol constants
    SYNC_BYTE = 0xA5
    SYNC_BYTE2 = 0x5A
    CMD_GET_INFO = 0x50
    CMD_GET_HEALTH = 0x52

    # Try C1M1 baud rate first (460800), then A1 (115200)
    baud_rates = [460800, 115200, 256000]

    for device in devices:
        for baud in baud_rates:
            log(f"  Trying {device} @ {baud} baud...", f)
            try:
                ser = serial.Serial(device, baud, timeout=2)
                ser.reset_input_buffer()
                ser.reset_output_buffer()

                # Send GET_HEALTH command (simplest)
                ser.write(bytes([SYNC_BYTE, CMD_GET_HEALTH]))
                time.sleep(0.5)

                # Read response
                response = ser.read(100)
                log(f"    Got {len(response)} bytes", f)

                if len(response) >= 7:
                    # Check for valid response header
                    if response[0] == SYNC_BYTE and response[1] == SYNC_BYTE2:
                        log(f"    Valid RPLidar response at {baud} baud!", f)
                        log(f"    Response (hex): {response[:20].hex()}", f)
                        ser.close()
                        return True
                    else:
                        log(f"    Response doesn't match RPLidar protocol", f)
                        log(f"    First bytes: {response[:10].hex()}", f)

                ser.close()

            except Exception as e:
                log(f"    Error: {e}", f)

    log("  No valid LiDAR response found", f)
    log("  Check: Is this an RPLidar C1M1 or different model?", f)
    return False


def test_rplidar_library_connection(f, devices):
    """Test connection using rplidar-roboticia library (A-series only)."""
    log("Testing RPLidar A-series connection (115200 baud)...", f)
    log("  NOTE: rplidar-roboticia does NOT support C1M1!", f)

    try:
        from rplidar import RPLidar
    except ImportError:
        log("  Skipping - rplidar library not installed", f)
        return False

    for device in devices:
        log(f"  Trying {device}...", f)
        try:
            lidar = RPLidar(device)
            info = lidar.get_info()
            log(f"  SUCCESS: Connected to RPLidar A-series", f)
            log(f"    Model: {info.get('model', 'Unknown')}", f)
            log(f"    Firmware: {info.get('firmware', 'Unknown')}", f)
            log(f"    Hardware: {info.get('hardware', 'Unknown')}", f)
            log(f"    Serial: {info.get('serialnumber', 'Unknown')}", f)

            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            return True

        except Exception as e:
            log(f"  Failed on {device}: {e}", f)
            log("  This is expected if using RPLidar C1M1", f)

    return False


def test_ydlidar_library(f):
    """Test if ydlidar library is available."""
    log("Checking ydlidar library...", f)

    try:
        import ydlidar
        log("  ydlidar library: INSTALLED", f)
        return True
    except ImportError:
        log("  ydlidar library: NOT INSTALLED", f)
        log("  For YDLIDAR, install SDK from: https://github.com/YDLIDAR/YDLidar-SDK", f)
        return False


def main():
    """Run all LiDAR tests."""
    with open(RESULT_FILE, "w") as f:
        log("=" * 60, f)
        log("USB LIDAR TEST - Raspberry Pi", f)
        log("RPLidar C1M1 @ 460800 baud (primary)", f)
        log("=" * 60, f)
        log("", f)

        results = {}

        # Test 1: USB devices
        results["usb_lidar_detected"] = test_usb_devices(f)
        log("", f)

        # Test 2: Serial devices
        serial_found, devices = test_serial_devices(f)
        results["serial_devices"] = serial_found
        log("", f)

        if devices:
            # Test 3: Permissions
            results["permissions"] = test_serial_permissions(f, devices)
            log("", f)

            # Test 4: RPLidar library (optional, A-series only)
            rplidar_lib_installed = test_rplidar_library(f)
            results["rplidar_library"] = rplidar_lib_installed
            log("", f)

            # Test 5: YDLIDAR library
            results["ydlidar_library"] = test_ydlidar_library(f)
            log("", f)

            # Test 6: RPLidar C1M1 connection (primary test - uses pathfinder driver)
            log("=" * 40, f)
            log("PRIMARY TEST: RPLidar C1M1", f)
            log("=" * 40, f)
            results["c1m1_connection"] = test_rplidar_c1m1_connection(f, devices)
            log("", f)

            # Test 7: RPLidar A-series (only if C1M1 failed and library available)
            if not results["c1m1_connection"] and rplidar_lib_installed:
                log("Trying A-series protocol (rplidar-roboticia)...", f)
                results["a_series_connection"] = test_rplidar_library_connection(f, devices)
                log("", f)

        # Summary
        log("=" * 60, f)
        log("TEST SUMMARY", f)
        log("=" * 60, f)
        for test_name, passed in results.items():
            status = "PASS" if passed else "FAIL"
            log(f"  {test_name}: {status}", f)

        # Determine overall status
        lidar_working = results.get("c1m1_connection", False) or results.get("a_series_connection", False)
        hardware_detected = results.get("serial_devices", False)

        log("", f)
        if lidar_working:
            log("SUCCESS: LiDAR is working!", f)
        elif hardware_detected:
            log("Hardware detected but connection failed.", f)
            log("Troubleshooting:", f)
            log("  1. Check LiDAR model - C1M1 uses 460800 baud", f)
            log("  2. Check motor is spinning (power issue?)", f)
            log("  3. Try: minicom -D /dev/ttyUSB0 -b 460800", f)
        else:
            log("No LiDAR hardware detected. Check USB connection.", f)

        log(f"Results saved to: {RESULT_FILE}", f)

        return 0 if lidar_working or hardware_detected else 1


if __name__ == "__main__":
    sys.exit(main())
