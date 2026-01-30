#!/usr/bin/env python3
"""
Pathfinder LiDAR Test Script

Test and diagnostic utilities for the RPLidar C1M1.

Usage:
    python3 test_lidar.py --check       # Verify device and permissions
    python3 test_lidar.py --scan        # Read and display one scan
    python3 test_lidar.py --visualize   # Simple matplotlib polar plot
    python3 test_lidar.py --continuous  # Continuous obstacle detection
"""

import argparse
import sys
import time
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pathfinder import config
from pathfinder.lidar import RPLidar, RPLidarException, check_device_exists, check_permissions
from pathfinder.obstacle_detector import SectorBasedDetector, SafetyLevel


def cmd_check():
    """Verify device exists and permissions are correct."""
    print("=" * 50)
    print("  LiDAR Device Check")
    print("=" * 50)
    print()

    all_ok = True

    # Check device exists
    exists, path = check_device_exists()
    if exists:
        print(f"[OK] Device found: {path}")
    else:
        print(f"[FAIL] No LiDAR device found")
        print(f"       Checked: {config.SERIAL_PORT}, {config.SERIAL_PORT_FALLBACK}, /dev/ttyUSB*")
        all_ok = False

    if exists:
        # Check permissions
        if check_permissions(path):
            print(f"[OK] Read/Write permissions on {path}")
        else:
            print(f"[FAIL] No read/write permissions on {path}")
            print(f"       Run: sudo chmod 666 {path}")
            print(f"       Or:  sudo usermod -a -G dialout $USER (then logout/login)")
            all_ok = False

        # Check symlink
        if os.path.exists(config.SERIAL_PORT) and os.path.islink(config.SERIAL_PORT):
            target = os.path.realpath(config.SERIAL_PORT)
            print(f"[OK] Symlink {config.SERIAL_PORT} -> {target}")
        elif config.SERIAL_PORT != path:
            print(f"[WARN] No symlink at {config.SERIAL_PORT}")
            print(f"       Run: sudo ./scripts/setup-udev.sh")

    # Try to connect and get info
    if exists and check_permissions(path):
        print()
        print("Attempting connection...")
        try:
            lidar = RPLidar(port=path)
            lidar.connect()

            info = lidar.get_info()
            print(f"[OK] Connected to LiDAR")
            print(f"     Model: {info['model']}")
            print(f"     Firmware: {info['firmware_major']}.{info['firmware_minor']}")
            print(f"     Hardware: {info['hardware']}")
            print(f"     Serial: {info['serial']}")

            health, error = lidar.get_health()
            if health == "Good":
                print(f"[OK] Health: {health}")
            else:
                print(f"[WARN] Health: {health} (error code: {error})")
                all_ok = False

            lidar.disconnect()

        except RPLidarException as e:
            print(f"[FAIL] Connection failed: {e}")
            all_ok = False

    print()
    print("=" * 50)
    if all_ok:
        print("  All checks passed!")
    else:
        print("  Some checks failed. See above for details.")
    print("=" * 50)

    return 0 if all_ok else 1


def cmd_scan():
    """Read and display one scan."""
    print("=" * 50)
    print("  LiDAR Single Scan")
    print("=" * 50)
    print()

    try:
        with RPLidar() as lidar:
            print(f"Connected to {lidar.port}")
            print("Starting scan (press Ctrl+C to abort)...")
            print()

            lidar.start_motor()
            time.sleep(0.5)  # Let motor spin up

            scan = lidar.get_scan(timeout=5.0)

            if not scan:
                print("[FAIL] No scan data received")
                return 1

            print(f"Received {len(scan)} points")
            print()
            print(f"{'Angle':>8}  {'Distance':>10}  {'Quality':>8}")
            print("-" * 30)

            # Show first 20 points
            for point in scan[:20]:
                print(f"{point.angle:>8.1f}  {point.distance:>10.1f}  {point.quality:>8}")

            if len(scan) > 20:
                print(f"... and {len(scan) - 20} more points")

            print()
            print(f"Closest point: {min(p.distance for p in scan if p.distance > 0):.1f}mm")
            print(f"Farthest point: {max(p.distance for p in scan):.1f}mm")

    except RPLidarException as e:
        print(f"[FAIL] {e}")
        return 1
    except KeyboardInterrupt:
        print("\nAborted by user")
        return 0

    return 0


def cmd_visualize():
    """Simple matplotlib polar plot."""
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        print("[FAIL] matplotlib not installed")
        print("       Run: pip install matplotlib numpy")
        return 1

    print("=" * 50)
    print("  LiDAR Visualization")
    print("=" * 50)
    print()
    print("Controls:")
    print("  - Close window to exit")
    print("  - Ctrl+C to abort")
    print()

    lidar = None

    try:
        lidar = RPLidar()
        lidar.connect()
        lidar.start_motor()
        time.sleep(0.5)
        lidar.start_scan()

        # Setup plot
        fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
        scatter = ax.scatter([], [], s=5, c='red', alpha=0.6)

        ax.set_ylim(0, config.MAX_DISTANCE_MM / 1000)  # In meters
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_title('LiDAR Scan (press Ctrl+C to exit)')
        ax.grid(True, alpha=0.3)

        # Data buffer
        angles = []
        distances = []

        def update(frame):
            nonlocal angles, distances

            # Read available points
            try:
                for _ in range(100):  # Read up to 100 points per frame
                    if lidar._serial.in_waiting < 5:
                        break
                    point = lidar._parse_scan_response()
                    if point and point.distance > 0:
                        angles.append(point.angle_rad)
                        distances.append(point.distance_m)
            except Exception:
                pass

            # Keep last 2000 points
            if len(angles) > 2000:
                angles = angles[-2000:]
                distances = distances[-2000:]

            if angles:
                data = np.column_stack((angles, distances))
                scatter.set_offsets(data)
                ax.set_title(f'LiDAR Scan - {len(angles)} points')

            return scatter,

        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
        plt.show()

    except RPLidarException as e:
        print(f"[FAIL] {e}")
        return 1
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if lidar:
            lidar.disconnect()

    return 0


def cmd_continuous():
    """Continuous obstacle detection output."""
    print("=" * 50)
    print("  Continuous Obstacle Detection")
    print("=" * 50)
    print()
    print("Safety zones:")
    print(f"  STOP: < {config.STOP_DISTANCE}m")
    print(f"  SLOW: < {config.SLOW_DISTANCE}m")
    print(f"  WARN: < {config.WARN_DISTANCE}m")
    print()
    print("Press Ctrl+C to stop")
    print()

    detector = SectorBasedDetector()
    lidar = None

    try:
        lidar = RPLidar()
        lidar.connect()
        print(f"Connected to {lidar.port}")

        for scan in lidar.iter_scans():
            result = detector.process_scan(scan)

            # Clear screen and print status
            print("\033[H\033[J", end="")  # ANSI clear screen
            print(f"Time: {time.strftime('%H:%M:%S')}")
            print(f"Points: {len(scan)}")
            print()

            # Print sector readings
            print(f"{'Sector':<12} {'Distance':>10} {'Level':>8}")
            print("-" * 32)

            for name in ["front", "left", "back", "right"]:
                sector = result.get_sector(name)
                if sector:
                    dist_str = f"{sector.min_distance:.2f}m" if sector.min_distance < float('inf') else "---"
                    level = sector.safety_level.value.upper()

                    # Color based on safety level
                    if sector.safety_level == SafetyLevel.STOP:
                        color = "\033[91m"  # Red
                    elif sector.safety_level == SafetyLevel.SLOW:
                        color = "\033[93m"  # Yellow
                    elif sector.safety_level == SafetyLevel.WARN:
                        color = "\033[94m"  # Blue
                    else:
                        color = "\033[92m"  # Green

                    print(f"{name:<12} {dist_str:>10} {color}{level:>8}\033[0m")

            print()
            print(f"Closest: {result.closest_distance:.2f}m ({result.closest_sector})")
            print(f"Overall: {result.overall_safety.value.upper()}")

            # Movement recommendation
            rec = detector.get_movement_recommendation(result)
            print()
            print("Movement:")
            print(f"  Forward:  {'OK' if rec['can_move_forward'] else 'BLOCKED'}")
            print(f"  Backward: {'OK' if rec['can_move_backward'] else 'BLOCKED'}")
            print(f"  Left:     {'OK' if rec['can_turn_left'] else 'BLOCKED'}")
            print(f"  Right:    {'OK' if rec['can_turn_right'] else 'BLOCKED'}")

            if rec['warning']:
                print(f"\n  WARNING: {rec['warning']}")

            time.sleep(0.1)  # Limit update rate

    except RPLidarException as e:
        print(f"[FAIL] {e}")
        return 1
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if lidar:
            lidar.disconnect()

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Pathfinder LiDAR Test Script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --check       Verify device and permissions
  %(prog)s --scan        Read single scan
  %(prog)s --visualize   Real-time polar plot
  %(prog)s --continuous  Continuous obstacle detection
        """
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--check", action="store_true",
                       help="Verify device exists and permissions")
    group.add_argument("--scan", action="store_true",
                       help="Read and display one scan")
    group.add_argument("--visualize", action="store_true",
                       help="Simple matplotlib polar plot")
    group.add_argument("--continuous", action="store_true",
                       help="Continuous obstacle detection output")

    parser.add_argument("--port", type=str, default=None,
                        help=f"Serial port (default: {config.SERIAL_PORT})")

    args = parser.parse_args()

    # Override port if specified
    if args.port:
        config.SERIAL_PORT = args.port

    if args.check:
        return cmd_check()
    elif args.scan:
        return cmd_scan()
    elif args.visualize:
        return cmd_visualize()
    elif args.continuous:
        return cmd_continuous()


if __name__ == "__main__":
    sys.exit(main())
