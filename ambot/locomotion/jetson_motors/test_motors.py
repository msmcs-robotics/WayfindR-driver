#!/usr/bin/env python3
"""
Motor Test Script for Jetson Orin Nano

Tests L298N motor control via gpiod (libgpiod v2). Automatically falls back
to simulation mode when gpiod is not installed (e.g., running on a laptop
for development).

Requires root/sudo for GPIO access on Jetson.

Usage (on Jetson via SSH):
    # Run full test sequence
    sudo python3 test_motors.py

    # Test a single motor
    sudo python3 test_motors.py --motor A --speed 40 --duration 3

    # Specific test patterns
    sudo python3 test_motors.py --test basic
    sudo python3 test_motors.py --test individual
    sudo python3 test_motors.py --test ramp

    # Simulation mode (forced, even if gpiod is available)
    python3 test_motors.py --simulate

    # Emergency stop (kill any running motor process)
    sudo python3 test_motors.py --kill

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
    always uses simulation mode regardless of gpiod availability.
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
    print("  NOTE: No PWM — any nonzero speed = full power")
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
    print("  NOTE: No PWM — any nonzero speed = full power")
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
    print("  (No PWM — any nonzero speed = full power)")
    motor_fn(speed)
    time.sleep(duration)
    motors.stop()
    print("Stopped.")


def test_ramp(motors, max_speed: int, duration: float):
    """
    Step both motors on/off to simulate ramp behavior.

    Note: Without PWM, this just toggles between full speed and stop.
    Retained for API compatibility — will be useful once sysfs PWM is added.
    """
    print("=" * 55)
    print(f"  RAMP TEST  |  0 -> {max_speed}% -> 0  over {duration}s each")
    print("  NOTE: No PWM — this will just toggle on/off")
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
    print("  L298N Pin Mapping  (Jetson Orin Nano, gpiod)")
    print("=" * 55)
    print()
    print("  Motor A (left):")
    chip, line = JETSON_L298N_CONFIG['ENA']
    print(f"    ENA (enable) : {chip}, line {line}")
    chip, line = JETSON_L298N_CONFIG['IN1']
    print(f"    IN1          : {chip}, line {line}")
    chip, line = JETSON_L298N_CONFIG['IN2']
    print(f"    IN2          : {chip}, line {line}")
    print()
    print("  Motor B (right):")
    chip, line = JETSON_L298N_CONFIG['ENB']
    print(f"    ENB (enable) : {chip}, line {line}")
    chip, line = JETSON_L298N_CONFIG['IN3']
    print(f"    IN3          : {chip}, line {line}")
    chip, line = JETSON_L298N_CONFIG['IN4']
    print(f"    IN4          : {chip}, line {line}")
    print()
    print("  PWM: Not available via gpiod (ENA/ENB = HIGH/LOW only)")
    print("  For variable speed, use sysfs PWM or Jetson hardware PWM.")
    print()
    print("  Power:")
    print("    L298N VCC  -> external 12V (NOT from Jetson)")
    print("    L298N GND  -> Jetson GND (pin 6/9/14/20/25/30/34/39)")
    print("    L298N 5V   -> leave disconnected")
    print()


def kill_motors():
    """
    Emergency stop: kill any running test_motors.py process
    and attempt to set all motor pins LOW via gpiod.
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

    # Also do a direct gpiod cleanup in case nothing else is running
    try:
        import gpiod
        from gpiod.line import Direction, Value
        from collections import defaultdict

        # Group pins by chip
        chip_lines = defaultdict(list)
        for pin_name, (chip_path, line_offset) in JETSON_L298N_CONFIG.items():
            if line_offset not in chip_lines[chip_path]:
                chip_lines[chip_path].append(line_offset)

        for chip_path, lines in chip_lines.items():
            settings = gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE,
            )
            req = gpiod.request_lines(
                chip_path,
                consumer="ambot-motors-kill",
                config={tuple(lines): settings},
            )
            # All lines are already INACTIVE from the settings
            req.release()

        print("  All motor GPIO lines set LOW and released.")
    except Exception as e:
        print(f"  GPIO cleanup skipped: {e}")

    print("Done.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Jetson Orin Nano L298N motor test (gpiod)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  sudo python3 test_motors.py                        # basic test, 50%% speed
  sudo python3 test_motors.py --test ramp            # ramp test (on/off toggle)
  sudo python3 test_motors.py --motor A --speed 40   # single motor
  python3 test_motors.py --simulate                   # no hardware needed
  sudo python3 test_motors.py --kill                  # emergency stop
  python3 test_motors.py --pinout                     # show wiring
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
        help="Force simulation mode (no gpiod)"
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
