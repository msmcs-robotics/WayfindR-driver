#!/usr/bin/env python3
"""
Motor Test Script for Raspberry Pi

Test script for FAGM25-370 motors with various motor drivers.
Supports TB6612FNG (default), L298N, and DRV8833.

Usage:
    # Basic test with TB6612FNG (default)
    python3 test_motors.py

    # Test with specific driver
    python3 test_motors.py --driver tb6612fng
    python3 test_motors.py --driver l298n
    python3 test_motors.py --driver drv8833

    # Interactive mode (keyboard control)
    python3 test_motors.py --interactive

    # Custom speed and duration
    python3 test_motors.py --speed 50 --duration 2.0

    # Check platform only
    python3 test_motors.py --check
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import Optional

# Setup path for direct execution
if __name__ == "__main__":
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from locomotion.rpi_motors import (
    create_robot,
    cleanup_gpio,
    DriverType,
    TB6612FNG_CONFIG,
    L298N_CONFIG,
    DRV8833_CONFIG,
)
from locomotion.rpi_motors.config import get_gpio_library

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


DRIVER_MAP = {
    "tb6612fng": DriverType.TB6612FNG,
    "tb6612": DriverType.TB6612FNG,
    "l298n": DriverType.L298N,
    "l298": DriverType.L298N,
    "drv8833": DriverType.DRV8833,
}


def check_platform():
    """Check platform and GPIO availability."""
    print("=" * 50)
    print("Platform Check")
    print("=" * 50)

    try:
        gpio, platform = get_gpio_library()
        print(f"Platform: {platform}")
        print(f"GPIO Library: {gpio.__name__}")
        print("GPIO access: OK")
        return True
    except ImportError as e:
        print(f"ERROR: {e}")
        print("\nInstall the appropriate GPIO library:")
        print("  Raspberry Pi: pip3 install RPi.GPIO")
        print("  Jetson:       pip3 install Jetson.GPIO")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def run_basic_test(driver_type: DriverType, speed: int = 50, duration: float = 2.0):
    """Run basic motor test sequence."""
    print("=" * 50)
    print(f"Basic Motor Test - {driver_type.name}")
    print(f"Speed: {speed}%, Duration: {duration}s per movement")
    print("=" * 50)
    print()

    robot = None
    try:
        print("Creating robot...")
        robot = create_robot(driver_type=driver_type)
        print("Robot created successfully")
        print()

        # Test sequence
        tests = [
            ("Forward", lambda: robot.forward(speed)),
            ("Stop", lambda: robot.stop()),
            ("Reverse", lambda: robot.reverse(speed)),
            ("Stop", lambda: robot.stop()),
            ("Turn Left", lambda: robot.turn_left(speed)),
            ("Turn Right", lambda: robot.turn_right(speed)),
            ("Arc Left", lambda: robot.arc_left(speed, 0.5)),
            ("Arc Right", lambda: robot.arc_right(speed, 0.5)),
            ("Stop", lambda: robot.stop()),
        ]

        for name, action in tests:
            print(f"  {name}...")
            action()
            if "Stop" in name:
                time.sleep(0.5)
            else:
                time.sleep(duration)

        print()
        print("Test complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"ERROR: {e}")
        raise
    finally:
        if robot:
            print("Cleaning up...")
            robot.cleanup()
        cleanup_gpio()
        print("Done")


def run_individual_test(driver_type: DriverType, speed: int = 50, duration: float = 2.0):
    """Test each motor individually."""
    print("=" * 50)
    print(f"Individual Motor Test - {driver_type.name}")
    print("=" * 50)
    print()

    robot = None
    try:
        robot = create_robot(driver_type=driver_type)

        print("Testing LEFT motor only...")
        robot.left.forward(speed)
        time.sleep(duration)
        robot.left.stop()
        time.sleep(0.5)

        robot.left.reverse(speed)
        time.sleep(duration)
        robot.left.stop()
        time.sleep(1)

        print("Testing RIGHT motor only...")
        robot.right.forward(speed)
        time.sleep(duration)
        robot.right.stop()
        time.sleep(0.5)

        robot.right.reverse(speed)
        time.sleep(duration)
        robot.right.stop()

        print()
        print("Individual test complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if robot:
            robot.cleanup()
        cleanup_gpio()


def run_interactive(driver_type: DriverType, speed: int = 50):
    """Interactive keyboard control mode."""
    print("=" * 50)
    print(f"Interactive Mode - {driver_type.name}")
    print("=" * 50)
    print()
    print("Controls:")
    print("  w/W - Forward")
    print("  s/S - Reverse")
    print("  a/A - Turn Left")
    print("  d/D - Turn Right")
    print("  q/Q - Arc Left")
    print("  e/E - Arc Right")
    print("  +/= - Increase speed")
    print("  -/_ - Decrease speed")
    print("  SPACE - Stop")
    print("  x/X - Exit")
    print()
    print(f"Starting speed: {speed}%")
    print()

    # Check for required module
    try:
        import tty
        import termios
    except ImportError:
        print("ERROR: Interactive mode requires Unix terminal")
        print("Use --basic for non-interactive testing")
        return

    robot = None
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        robot = create_robot(driver_type=driver_type)

        tty.setraw(fd)

        current_speed = speed

        while True:
            char = sys.stdin.read(1)

            if char in ('x', 'X', '\x03'):  # x or Ctrl+C
                break
            elif char in ('w', 'W'):
                robot.forward(current_speed)
            elif char in ('s', 'S'):
                robot.reverse(current_speed)
            elif char in ('a', 'A'):
                robot.turn_left(current_speed)
            elif char in ('d', 'D'):
                robot.turn_right(current_speed)
            elif char in ('q', 'Q'):
                robot.arc_left(current_speed, 0.5)
            elif char in ('e', 'E'):
                robot.arc_right(current_speed, 0.5)
            elif char == ' ':
                robot.stop()
            elif char in ('+', '='):
                current_speed = min(100, current_speed + 10)
            elif char in ('-', '_'):
                current_speed = max(10, current_speed - 10)

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if robot:
            robot.cleanup()
        cleanup_gpio()
        print("\nExited interactive mode")


def print_pinout(driver_type: DriverType):
    """Print pinout for the selected driver."""
    from locomotion.rpi_motors.config import DRIVER_CONFIGS

    config = DRIVER_CONFIGS[driver_type]

    print("=" * 50)
    print(f"Pinout Reference - {config.name}")
    print("=" * 50)
    print()
    print(f"Pin Mode: {config.pin_mode}")
    print()
    print("Left Motor:")
    print(f"  IN1: Pin {config.left_motor.in1}")
    print(f"  IN2: Pin {config.left_motor.in2}")
    print(f"  PWM: Pin {config.left_motor.pwm}")
    print()
    print("Right Motor:")
    print(f"  IN1: Pin {config.right_motor.in1}")
    print(f"  IN2: Pin {config.right_motor.in2}")
    print(f"  PWM: Pin {config.right_motor.pwm}")
    print()
    if config.stby_pin:
        print(f"Standby Pin: {config.stby_pin}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Motor test script for Raspberry Pi",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_motors.py --check           # Check platform
  python3 test_motors.py --basic           # Basic test (default)
  python3 test_motors.py --individual      # Test each motor
  python3 test_motors.py --interactive     # Keyboard control
  python3 test_motors.py --pinout          # Show pin assignments
  python3 test_motors.py --driver l298n    # Use L298N driver
        """
    )

    parser.add_argument(
        "--driver", "-d",
        choices=list(DRIVER_MAP.keys()),
        default="tb6612fng",
        help="Motor driver type (default: tb6612fng)"
    )
    parser.add_argument(
        "--speed", "-s",
        type=int,
        default=50,
        help="Motor speed 0-100 (default: 50)"
    )
    parser.add_argument(
        "--duration", "-t",
        type=float,
        default=2.0,
        help="Duration per movement in seconds (default: 2.0)"
    )

    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--check", "-c",
        action="store_true",
        help="Check platform and GPIO only"
    )
    group.add_argument(
        "--basic", "-b",
        action="store_true",
        help="Run basic motor test (default)"
    )
    group.add_argument(
        "--individual", "-i",
        action="store_true",
        help="Test each motor individually"
    )
    group.add_argument(
        "--interactive",
        action="store_true",
        help="Interactive keyboard control"
    )
    group.add_argument(
        "--pinout", "-p",
        action="store_true",
        help="Print pin assignments"
    )

    args = parser.parse_args()

    driver_type = DRIVER_MAP[args.driver.lower()]

    if args.check:
        check_platform()
    elif args.pinout:
        print_pinout(driver_type)
    elif args.individual:
        if check_platform():
            run_individual_test(driver_type, args.speed, args.duration)
    elif args.interactive:
        if check_platform():
            run_interactive(driver_type, args.speed)
    else:
        # Default to basic test
        if check_platform():
            run_basic_test(driver_type, args.speed, args.duration)


if __name__ == "__main__":
    main()
