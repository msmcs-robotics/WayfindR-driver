#!/usr/bin/env python3
"""
LD19 LiDAR Test Script

Tests YOUYEETOO LD19 / LDRobot LD19 LiDAR connectivity.
This is the primary LiDAR test for the ambot project.

Protocol: LDRobot proprietary (NOT RPLidar)
Baud rate: 230400
Packet size: 47 bytes

Outputs results to tests/results/ld19_test_results.txt
"""

import os
import sys
import time
from datetime import datetime
from pathlib import Path

# Setup paths
SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
RESULT_FILE = RESULTS_DIR / "ld19_test_results.txt"

# Add parent directory for pathfinder import
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


def test_serial_port(f, port="/dev/ttyUSB0"):
    """Check if serial port exists."""
    log(f"Checking serial port {port}...", f)

    if os.path.exists(port):
        log(f"  Port exists: {port}", f)
        if os.access(port, os.R_OK | os.W_OK):
            log(f"  Permissions: OK (read/write)", f)
            return True
        else:
            log(f"  Permissions: DENIED - add user to dialout group", f)
            log(f"  Fix: sudo usermod -a -G dialout $USER && logout", f)
            return False
    else:
        log(f"  Port not found: {port}", f)
        return False


def test_ld19_raw_connection(f, port="/dev/ttyUSB0"):
    """Test raw serial connection at 230400 baud."""
    log("Testing LD19 raw connection (230400 baud)...", f)

    try:
        import serial
    except ImportError:
        log("  ERROR: pyserial not installed", f)
        log("  Fix: pip3 install pyserial", f)
        return False

    try:
        ser = serial.Serial(port, 230400, timeout=2)
        ser.reset_input_buffer()

        log("  Waiting for data (2 seconds)...", f)
        time.sleep(0.5)

        # Read some data
        data = ser.read(1000)
        log(f"  Received: {len(data)} bytes", f)

        if len(data) < 47:
            log("  WARNING: Not enough data received", f)
            ser.close()
            return False

        # Check for LD19 packet header (0x54 0x2C)
        header_count = 0
        for i in range(len(data) - 1):
            if data[i] == 0x54 and data[i+1] == 0x2C:
                header_count += 1

        log(f"  Found {header_count} LD19 packet headers (0x54 0x2C)", f)

        if header_count > 0:
            log("  SUCCESS: LD19 protocol detected!", f)
            ser.close()
            return True
        else:
            log("  WARNING: No LD19 headers found", f)
            log(f"  First bytes: {data[:20].hex()}", f)
            ser.close()
            return False

    except Exception as e:
        log(f"  ERROR: {e}", f)
        return False


def test_ld19_driver(f, port="/dev/ttyUSB0"):
    """Test the LD19 driver from pathfinder."""
    log("Testing LD19 driver (pathfinder.lidar_ld19)...", f)

    try:
        from pathfinder.lidar_ld19 import LD19Lidar, LD19LidarException
        log("  Driver imported successfully", f)
    except ImportError as e:
        log(f"  ERROR: Could not import driver: {e}", f)
        return False

    try:
        lidar = LD19Lidar(port=port)
        lidar.connect()
        log("  Connected to LD19", f)

        # Read packets
        log("  Reading packets...", f)
        packet_count = 0
        point_count = 0
        min_distance = float('inf')
        max_distance = 0

        start_time = time.time()
        timeout = 5.0  # 5 second timeout

        while time.time() - start_time < timeout:
            points = lidar.read_packet()
            if points:
                packet_count += 1
                point_count += len(points)

                for p in points:
                    if p.distance > 0:
                        min_distance = min(min_distance, p.distance)
                        max_distance = max(max_distance, p.distance)

                # Stop after 100 packets
                if packet_count >= 100:
                    break

        lidar.disconnect()

        log(f"  Packets received: {packet_count}", f)
        log(f"  Points received: {point_count}", f)

        if packet_count > 0:
            log(f"  Distance range: {min_distance:.0f}mm - {max_distance:.0f}mm", f)
            log("  SUCCESS: LD19 driver working!", f)
            return True
        else:
            log("  WARNING: No packets received", f)
            return False

    except LD19LidarException as e:
        log(f"  Driver error: {e}", f)
        return False
    except Exception as e:
        log(f"  ERROR: {e}", f)
        return False


def test_ld19_scan(f, port="/dev/ttyUSB0"):
    """Test full 360-degree scan."""
    log("Testing LD19 full scan...", f)

    try:
        from pathfinder.lidar_ld19 import LD19Lidar
    except ImportError:
        log("  Skipping - driver not available", f)
        return False

    try:
        with LD19Lidar(port=port) as lidar:
            log("  Waiting for complete scan...", f)

            scan_count = 0
            for scan in lidar.iter_scans(min_points=200):
                scan_count += 1
                log(f"  Scan {scan_count}: {len(scan)} points", f)

                if scan:
                    # Find closest point
                    closest = min(scan, key=lambda p: p.distance)
                    log(f"    Closest: {closest.distance:.0f}mm at {closest.angle:.1f}°", f)

                    # Coverage check
                    angles = [p.angle for p in scan]
                    min_angle = min(angles)
                    max_angle = max(angles)
                    log(f"    Angle range: {min_angle:.1f}° - {max_angle:.1f}°", f)

                # Only need one scan for test
                if scan_count >= 3:
                    break

            if scan_count > 0:
                log("  SUCCESS: Full scans working!", f)
                return True
            else:
                log("  WARNING: No complete scans received", f)
                return False

    except Exception as e:
        log(f"  ERROR: {e}", f)
        return False


def main():
    """Run all LD19 LiDAR tests."""
    port = "/dev/ttyUSB0"

    # Check command line for port override
    if len(sys.argv) > 1:
        port = sys.argv[1]

    with open(RESULT_FILE, "w") as f:
        log("=" * 60, f)
        log("LD19 LIDAR TEST - YOUYEETOO / LDRobot LD19", f)
        log("Protocol: LDRobot (NOT RPLidar)", f)
        log("Baud: 230400 | Packets: 47 bytes | Points: 12/packet", f)
        log("=" * 60, f)
        log("", f)

        results = {}

        # Test 1: Serial port
        results["serial_port"] = test_serial_port(f, port)
        log("", f)

        if results["serial_port"]:
            # Test 2: Raw connection
            results["raw_connection"] = test_ld19_raw_connection(f, port)
            log("", f)

            # Test 3: Driver
            results["driver"] = test_ld19_driver(f, port)
            log("", f)

            # Test 4: Full scan
            if results["driver"]:
                results["full_scan"] = test_ld19_scan(f, port)
                log("", f)

        # Summary
        log("=" * 60, f)
        log("TEST SUMMARY", f)
        log("=" * 60, f)
        for test_name, passed in results.items():
            status = "PASS" if passed else "FAIL"
            log(f"  {test_name}: {status}", f)

        log("", f)
        all_passed = all(results.values())
        if all_passed:
            log("SUCCESS: All LD19 tests passed!", f)
        elif results.get("raw_connection"):
            log("PARTIAL: LD19 detected, some tests failed", f)
        else:
            log("FAILED: LD19 not detected or not working", f)
            log("", f)
            log("Troubleshooting:", f)
            log("  1. Check USB connection", f)
            log("  2. Verify baud rate is 230400 (not 460800)", f)
            log("  3. Check permissions: sudo usermod -a -G dialout $USER", f)
            log("  4. Wait 2-3 seconds after power for motor spin-up", f)

        log(f"Results saved to: {RESULT_FILE}", f)

        return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
