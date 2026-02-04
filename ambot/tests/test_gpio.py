#!/usr/bin/env python3
"""
GPIO Test Script for Raspberry Pi

Tests GPIO availability for motor control.
Outputs results to tests/results/gpio_test_results.txt
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
RESULT_FILE = RESULTS_DIR / "gpio_test_results.txt"


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


def test_gpio_devices(f):
    """Check for GPIO devices."""
    log("Checking for GPIO devices...", f)
    success, output = run_command("ls -la /dev/gpio* 2>&1")
    log(f"  GPIO devices:\n{output}", f)
    return "/dev/gpiochip" in output or "/dev/gpiomem" in output


def test_gpio_permissions(f):
    """Check GPIO permissions."""
    log("Checking GPIO permissions...", f)

    # Check if user is in gpio group
    success, output = run_command("groups")
    log(f"  User groups: {output.strip()}", f)

    if "gpio" in output:
        log("  User is in gpio group: YES", f)
        return True
    else:
        log("  User is in gpio group: NO", f)
        log("  Fix with: sudo usermod -a -G gpio $USER", f)
        return False


def test_rpi_gpio_library(f):
    """Test if RPi.GPIO library is available."""
    log("Checking RPi.GPIO library...", f)

    try:
        import RPi.GPIO as GPIO
        log(f"  RPi.GPIO: INSTALLED (version {GPIO.VERSION})", f)
        return True
    except ImportError:
        log("  RPi.GPIO: NOT INSTALLED", f)
        log("  Install with: pip3 install RPi.GPIO", f)
        return False
    except RuntimeError as e:
        log(f"  RPi.GPIO: INSTALLED but runtime error: {e}", f)
        return True  # Library is there, just can't init


def test_gpiozero_library(f):
    """Test if gpiozero library is available."""
    log("Checking gpiozero library...", f)

    try:
        from gpiozero import Device
        log("  gpiozero: INSTALLED", f)
        return True
    except ImportError:
        log("  gpiozero: NOT INSTALLED", f)
        log("  Install with: pip3 install gpiozero", f)
        return False


def test_lgpio_library(f):
    """Test if lgpio library is available (for newer Pi OS)."""
    log("Checking lgpio library...", f)

    try:
        import lgpio
        log("  lgpio: INSTALLED", f)
        return True
    except ImportError:
        log("  lgpio: NOT INSTALLED", f)
        log("  Install with: pip3 install lgpio", f)
        return False


def test_gpio_read(f):
    """Try to read GPIO state."""
    log("Testing GPIO read capability...", f)

    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Try to read a safe pin (GPIO4 is usually not connected)
        GPIO.setup(4, GPIO.IN)
        state = GPIO.input(4)
        log(f"  Read GPIO4: {state}", f)
        GPIO.cleanup()
        log("  GPIO read: SUCCESS", f)
        return True
    except Exception as e:
        log(f"  GPIO read failed: {e}", f)
        return False


def test_pinctrl(f):
    """Test pinctrl command (Pi 5 / newer kernels)."""
    log("Checking pinctrl (Pi 5 / newer kernels)...", f)

    success, output = run_command("which pinctrl")
    if success:
        success, output = run_command("pinctrl get 4 2>&1")
        log(f"  pinctrl available:\n{output}", f)
        return True
    else:
        log("  pinctrl not available (older Pi or kernel)", f)
        return False


def test_motor_driver_pins(f):
    """Document the motor driver pins we'll use."""
    log("Motor Driver Pin Configuration (L298N/TB6612FNG):", f)
    log("", f)
    log("  Physical Pin | BCM GPIO | Function", f)
    log("  -------------|----------|----------", f)
    log("  Pin 6        | GND      | Common ground", f)
    log("  Pin 11       | GPIO17   | STBY/Enable", f)
    log("  Pin 13       | GPIO27   | Motor A IN1", f)
    log("  Pin 15       | GPIO22   | Motor A IN2", f)
    log("  Pin 16       | GPIO23   | Motor B IN1", f)
    log("  Pin 18       | GPIO24   | Motor B IN2", f)
    log("  Pin 32       | GPIO12   | Motor B PWM (HW PWM0)", f)
    log("  Pin 33       | GPIO13   | Motor A PWM (HW PWM1)", f)
    log("", f)
    log("  See: https://pinout.xyz/", f)


def main():
    """Run all GPIO tests."""
    with open(RESULT_FILE, "w") as f:
        log("=" * 60, f)
        log("GPIO TEST - Raspberry Pi", f)
        log("=" * 60, f)
        log("", f)

        results = {}

        # Test 1: GPIO devices
        results["gpio_devices"] = test_gpio_devices(f)
        log("", f)

        # Test 2: Permissions
        results["gpio_permissions"] = test_gpio_permissions(f)
        log("", f)

        # Test 3: RPi.GPIO library
        results["rpi_gpio"] = test_rpi_gpio_library(f)
        log("", f)

        # Test 4: gpiozero library
        results["gpiozero"] = test_gpiozero_library(f)
        log("", f)

        # Test 5: lgpio library
        results["lgpio"] = test_lgpio_library(f)
        log("", f)

        # Test 6: pinctrl
        results["pinctrl"] = test_pinctrl(f)
        log("", f)

        # Test 7: GPIO read (only if library available)
        if results["rpi_gpio"]:
            results["gpio_read"] = test_gpio_read(f)
            log("", f)

        # Document motor pins
        test_motor_driver_pins(f)
        log("", f)

        # Summary
        log("=" * 60, f)
        log("TEST SUMMARY", f)
        log("=" * 60, f)
        for test_name, passed in results.items():
            status = "PASS" if passed else "FAIL"
            log(f"  {test_name}: {status}", f)

        # Critical check
        critical = results.get("gpio_devices", False)
        log("", f)
        if critical:
            log("GPIO hardware available. Install libraries to complete setup.", f)
        else:
            log("GPIO hardware NOT detected. Check if running on Raspberry Pi.", f)

        log(f"Results saved to: {RESULT_FILE}", f)

        return 0 if critical else 1


if __name__ == "__main__":
    sys.exit(main())
