#!/usr/bin/env python3
"""
Motor Test Script

Top-level convenience script for testing motors on the Raspberry Pi.
Defaults to L298N driver (the wired driver). Delegates to the full
test suite in locomotion/rpi_motors/test_motors.py.

Usage (run on RPi from ambot/):
    python3 tests/test_motors.py                  # Platform check + basic test at 30%
    python3 tests/test_motors.py --check          # Platform/GPIO check only
    python3 tests/test_motors.py --basic          # Forward/reverse/turn sequence
    python3 tests/test_motors.py --individual     # Test each motor separately
    python3 tests/test_motors.py --interactive    # WASD keyboard control
    python3 tests/test_motors.py --pinout         # Show L298N pin assignments
    python3 tests/test_motors.py --speed 50       # Set speed (default: 30)
    python3 tests/test_motors.py --driver tb6612fng  # Use different driver

Via deploy.sh from dev machine:
    ./deploy.sh rpi --test=motors
"""

import sys
import os

# Ensure ambot/ is on the path for imports (go up from tests/ to ambot/)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from locomotion.rpi_motors.test_motors import main, DRIVER_MAP

if __name__ == "__main__":
    # Default to L298N and 30% speed if no args given
    if len(sys.argv) == 1:
        # No arguments: run platform check + basic test at safe speed
        sys.argv.extend(["--driver", "l298n", "--speed", "30", "--basic"])
    else:
        # If user didn't specify --driver, default to l298n
        has_driver = any(arg in ("--driver", "-d") for arg in sys.argv)
        if not has_driver:
            sys.argv.insert(1, "--driver")
            sys.argv.insert(2, "l298n")

        # If user didn't specify --speed, default to 30
        has_speed = any(arg in ("--speed", "-s") for arg in sys.argv)
        if not has_speed:
            sys.argv.insert(1, "--speed")
            sys.argv.insert(2, "30")

    main()
