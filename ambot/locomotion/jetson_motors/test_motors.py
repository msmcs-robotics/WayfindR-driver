#!/usr/bin/env python3
"""
Motor Test Script for Jetson Orin Nano

Tests L298N motor control via Jetson.GPIO.  Automatically falls back
to simulation mode when Jetson.GPIO is not installed (e.g., running
on a laptop for development).

Usage (on Jetson via SSH):
    # Run full test sequence
    python3 test_motors.py

    # Test a single motor
    python3 test_motors.py --motor A --speed 40 --duration 3

    # Specific test patterns
    python3 test_motors.py --test basic
    python3 test_motors.py --test individual
    python3 test_motors.py --test ramp

    # Simulation mode (forced, even if GPIO is available)
    python3 test_motors.py --simulate

    # Emergency stop (kill any running motor process)
    python3 test_motors.py --kill

    # Show pin mapping
    python3 test_motors.py --pinout
"""

from __future__ import annotations

import argparse
import atexit
import logging
import os
import signal
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Path setup for direct execution (e.g., `python3 test_motors.py`)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    _here = os.path.dirname(os.path.abspath(__file__))
    # Add ambot/ to path so "from locomotion.jetson_motors..." works
    _ambot = os.path.dirname(os.path.dirname(_here))
    if _ambot not in sys.path:
        sys.path.insert(0, _ambot)

from locomotion.jetson_motors.config import JETSON_L298N_CONFIG, PWM_FREQ

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Global reference for signal-handler cleanup
# ---------------------------------------------------------------------------
_motors = None


def _signal_handler(signum, frame):
    """Handle SIGINT / SIGTERM — stop motors and exit cleanly."""
    sig_name = signal.Signals(signum).name
    print(f"\n[{sig_name}] Stopping motors...")
    if _motors is not None:
        _motors.cleanup()
    sys.exit(0)


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


# ---------------------------------------------------------------------------
# Helper: create driver (real or forced-sim)
# ---------------------------------------------------------------------------

def create_driver(force_simulate: bool = False):
    """
    Create a JetsonL298N instance.

    If force_simulate is True, temporarily patches the module so the driver
    always uses simulation mode regardless of GPIO availability.
    """
    global _motors

    if force_simulate:
        # Patch the module-level flag before importing the class
        import locomotion.jetson_motors.driver as _mod
        _orig = _mod._SIMULATE
        _mod._SIMULATE = True
        from locomotion.jetson_motors.driver import JetsonL298N
        motors = JetsonL298N()
        _mod._SIMULATE = _orig
    else:
        from locomotion.jetson_motors.driver import JetsonL298N
        motors = JetsonL298N()

    _motors = motors
    return motors


# ---------------------------------------------------------------------------
# Test routines
# ---------------------------------------------------------------------------

def test_basic(motors, speed: int, duration: float):
    """Run through all compound movement commands."""
    print("=" * 55)
    print(f"  BASIC TEST  |  speed={speed}%  duration={duration}s")
    print("=" * 55)

    steps = [
        ("Forward",    lambda: motors.forward(speed)),
        ("Stop",       lambda: motors.stop()),
        ("Backward",   lambda: motors.backward(speed)),
        ("Stop",       lambda: motors.stop()),
        ("Spin left",  lambda: motors.spin_left(speed)),
        ("Stop",       lambda: motors.stop()),
        ("Spin right", lambda: motors.spin_right(speed)),
        ("Stop",       lambda: motors.stop()),
    ]

    for label, action in steps:
        wait = 0.5 if label == "Stop" else duration
        print(f"  {label:.<40s}", end="", flush=True)
        action()
        time.sleep(wait)
        print(f" {wait:.1f}s")

    print("\nBasic test complete.")


def test_individual(motors, speed: int, duration: float):
    """Test each motor one at a time, forward then reverse."""
    print("=" * 55)
    print(f"  INDIVIDUAL MOTOR TEST  |  speed={speed}%  duration={duration}s")
    print("=" * 55)

    for label, motor_fn in [("Motor A", motors.motor_a), ("Motor B", motors.motor_b)]:
        print(f"\n  --- {label} ---")

        print(f"    Forward  ({speed}%)...", end="", flush=True)
        motor_fn(speed)
        time.sleep(duration)
        motors.stop()
        print(" done")
        time.sleep(0.5)

        print(f"    Reverse  ({speed}%)...", end="", flush=True)
        motor_fn(-speed)
        time.sleep(duration)
        motors.stop()
        print(" done")
        time.sleep(0.5)

    print("\nIndividual test complete.")


def test_single_motor(motors, motor: str, speed: int, duration: float):
    """Run one motor at the specified speed for the given duration."""
    motor = motor.upper()
    motor_fn = motors.motor_a if motor == "A" else motors.motor_b

    print(f"Motor {motor}: speed={speed}% for {duration}s")
    motor_fn(speed)
    time.sleep(duration)
    motors.stop()
    print("Stopped.")


def test_ramp(motors, max_speed: int, duration: float):
    """
    Ramp both motors from 0 to max_speed and back down.
    Useful for checking that PWM duty cycle changes smoothly.
    """
    print("=" * 55)
    print(f"  RAMP TEST  |  0 -> {max_speed}% -> 0  over {duration}s each")
    print("=" * 55)

    steps = 20
    step_time = duration / steps

    # Ramp up
    print("  Ramping up...", flush=True)
    for i in range(steps + 1):
        spd = int(max_speed * i / steps)
        motors.forward(spd)
        time.sleep(step_time)

    # Ramp down
    print("  Ramping down...", flush=True)
    for i in range(steps, -1, -1):
        spd = int(max_speed * i / steps)
        motors.forward(spd)
        time.sleep(step_time)

    motors.stop()
    print("\nRamp test complete.")


def print_pinout():
    """Print the Jetson L298N pin mapping."""
    print("=" * 55)
    print("  L298N Pin Mapping  (Jetson Orin Nano, BOARD mode)")
    print("=" * 55)
    print()
    print("  Motor A (left):")
    print(f"    ENA (PWM) : pin {JETSON_L298N_CONFIG['ENA']}")
    print(f"    IN1       : pin {JETSON_L298N_CONFIG['IN1']}")
    print(f"    IN2       : pin {JETSON_L298N_CONFIG['IN2']}")
    print()
    print("  Motor B (right):")
    print(f"    ENB (PWM) : pin {JETSON_L298N_CONFIG['ENB']}")
    print(f"    IN3       : pin {JETSON_L298N_CONFIG['IN3']}")
    print(f"    IN4       : pin {JETSON_L298N_CONFIG['IN4']}")
    print()
    print(f"  PWM frequency: {PWM_FREQ} Hz")
    print()
    print("  Power:")
    print("    L298N VCC  -> external 12V (NOT from Jetson)")
    print("    L298N GND  -> Jetson GND (pin 6/9/14/20/25/30/34/39)")
    print("    L298N 5V   -> leave disconnected")
    print()


def kill_motors():
    """
    Emergency stop: kill any running test_motors.py process
    and attempt a GPIO cleanup.
    """
    print("Emergency stop — killing motor processes...")
    my_pid = os.getpid()

    # Find other instances of this script
    try:
        result = subprocess.run(
            ["pgrep", "-f", "test_motors.py"],
            capture_output=True, text=True
        )
        pids = [int(p) for p in result.stdout.strip().split() if int(p) != my_pid]
    except Exception:
        pids = []

    if pids:
        for pid in pids:
            print(f"  Sending SIGTERM to PID {pid}")
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(0.5)
    else:
        print("  No other motor processes found.")

    # Also do a direct GPIO cleanup in case nothing else is running
    try:
        import Jetson.GPIO as GPIO
        GPIO.setmode(GPIO.BOARD)
        cfg = JETSON_L298N_CONFIG
        for pin in [cfg['ENA'], cfg['IN1'], cfg['IN2'],
                     cfg['ENB'], cfg['IN3'], cfg['IN4']]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.cleanup()
        print("  GPIO pins set LOW and cleaned up.")
    except Exception as e:
        print(f"  GPIO cleanup skipped: {e}")

    print("Done.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Jetson Orin Nano L298N motor test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  python3 test_motors.py                        # basic test, 50%% speed
  python3 test_motors.py --test ramp            # smooth ramp test
  python3 test_motors.py --motor A --speed 40   # single motor
  python3 test_motors.py --simulate             # no hardware needed
  python3 test_motors.py --kill                 # emergency stop
  python3 test_motors.py --pinout               # show wiring
"""
    )

    parser.add_argument(
        "--test", "-t",
        choices=["basic", "individual", "ramp"],
        default="basic",
        help="Test pattern to run (default: basic)"
    )
    parser.add_argument(
        "--motor", "-m",
        choices=["A", "B", "a", "b"],
        default=None,
        help="Test a single motor (A or B)"
    )
    parser.add_argument(
        "--speed", "-s",
        type=int,
        default=50,
        help="Speed 0-100 (default: 50). Negative for reverse with --motor."
    )
    parser.add_argument(
        "--duration", "-d",
        type=float,
        default=2.0,
        help="Seconds per movement step (default: 2.0)"
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Force simulation mode (no GPIO)"
    )
    parser.add_argument(
        "--kill",
        action="store_true",
        help="Emergency stop: kill motor processes and cleanup GPIO"
    )
    parser.add_argument(
        "--pinout",
        action="store_true",
        help="Print Jetson L298N pin mapping and exit"
    )

    args = parser.parse_args()

    # --kill and --pinout are standalone actions
    if args.kill:
        kill_motors()
        return

    if args.pinout:
        print_pinout()
        return

    # Create driver
    motors = create_driver(force_simulate=args.simulate)
    mode = "SIMULATION" if motors.simulate else "HARDWARE"
    print(f"\nDriver ready ({mode})\n")

    try:
        if args.motor:
            test_single_motor(motors, args.motor, args.speed, args.duration)
        elif args.test == "individual":
            test_individual(motors, args.speed, args.duration)
        elif args.test == "ramp":
            test_ramp(motors, args.speed, args.duration)
        else:
            test_basic(motors, args.speed, args.duration)
    finally:
        motors.cleanup()
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
