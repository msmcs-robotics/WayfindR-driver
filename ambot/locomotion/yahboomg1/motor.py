#!/usr/bin/env python3
"""
YAHBOOM G1 Motor Control Module

Controls DC motors via TB6612FNG motor driver for differential drive robots.
Compatible with Jetson Orin Nano (Jetson.GPIO) and Raspberry Pi (RPi.GPIO).

Usage:
    from motor import Motor, DifferentialDrive
    import Jetson.GPIO as GPIO  # or RPi.GPIO

    GPIO.setmode(GPIO.BOARD)

    left = Motor(in1=13, in2=15, pwm=33)
    right = Motor(in1=16, in2=18, pwm=32)
    robot = DifferentialDrive(left, right, stby=11)

    robot.forward(50)
    robot.stop()
    robot.cleanup()
    GPIO.cleanup()
"""

from __future__ import annotations

import time
import logging
from typing import Optional, Tuple

# Try to import GPIO library
try:
    from .config import get_gpio_library, PWM_FREQUENCY, LEFT_OFFSET, RIGHT_OFFSET
    GPIO, PLATFORM = get_gpio_library()
except ImportError:
    # Fallback for direct execution
    import sys
    import os
    sys.path.insert(0, os.path.dirname(__file__))
    from config import get_gpio_library, PWM_FREQUENCY, LEFT_OFFSET, RIGHT_OFFSET
    GPIO, PLATFORM = get_gpio_library()

logger = logging.getLogger(__name__)


class Motor:
    """
    Control a single DC motor via TB6612FNG H-bridge driver.

    Attributes:
        in1: GPIO pin for direction control 1
        in2: GPIO pin for direction control 2
        pwm_pin: GPIO pin for PWM speed control
        offset: Direction offset (1 or -1) to correct motor wiring
    """

    def __init__(
        self,
        in1: int,
        in2: int,
        pwm: int,
        pwm_freq: int = PWM_FREQUENCY,
        offset: int = 1,
    ):
        """
        Initialize motor control.

        Args:
            in1: IN1 direction pin (BOARD numbering)
            in2: IN2 direction pin (BOARD numbering)
            pwm: PWM speed control pin (BOARD numbering)
            pwm_freq: PWM frequency in Hz (default 1000)
            offset: Direction correction (1 or -1)
        """
        self.in1 = in1
        self.in2 = in2
        self.pwm_pin = pwm
        self.offset = offset
        self._speed = 0

        # Setup direction pins as output
        GPIO.setup(self.in1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.in2, GPIO.OUT, initial=GPIO.LOW)

        # Setup PWM pin
        GPIO.setup(self.pwm_pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm = GPIO.PWM(self.pwm_pin, pwm_freq)
        self.pwm.start(0)

        logger.debug(f"Motor initialized: IN1={in1}, IN2={in2}, PWM={pwm}")

    @property
    def speed(self) -> int:
        """Current motor speed (0-100)."""
        return self._speed

    def _set_direction(self, forward: bool):
        """Set motor direction pins."""
        if self.offset == -1:
            forward = not forward

        if forward:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def drive(self, speed: int):
        """
        Drive motor at specified speed.

        Args:
            speed: -100 to 100 (negative = reverse)
        """
        speed = max(-100, min(100, speed))

        if speed > 0:
            self._set_direction(forward=True)
            self.pwm.ChangeDutyCycle(speed)
        elif speed < 0:
            self._set_direction(forward=False)
            self.pwm.ChangeDutyCycle(abs(speed))
        else:
            self.stop()

        self._speed = speed

    def forward(self, speed: int = 100):
        """Run motor forward at specified speed (0-100)."""
        self.drive(abs(speed))

    def reverse(self, speed: int = 100):
        """Run motor in reverse at specified speed (0-100)."""
        self.drive(-abs(speed))

    def stop(self):
        """Stop motor (coast)."""
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        self._speed = 0

    def brake(self):
        """Brake motor (short circuit)."""
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(0)
        self._speed = 0

    def cleanup(self):
        """Stop PWM and cleanup."""
        self.stop()
        self.pwm.stop()


class DifferentialDrive:
    """
    Control a differential drive robot (two-wheeled tank-style).

    Provides high-level commands for forward, reverse, turning, and arc movements.
    """

    def __init__(
        self,
        left_motor: Motor,
        right_motor: Motor,
        stby_pin: Optional[int] = None,
    ):
        """
        Initialize differential drive controller.

        Args:
            left_motor: Left Motor instance
            right_motor: Right Motor instance
            stby_pin: Standby pin for motor driver (optional)
        """
        self.left = left_motor
        self.right = right_motor
        self.stby = stby_pin
        self._enabled = False

        # Setup standby pin if provided
        if self.stby is not None:
            GPIO.setup(self.stby, GPIO.OUT, initial=GPIO.LOW)

        logger.info(f"DifferentialDrive initialized on {PLATFORM}")

    @property
    def enabled(self) -> bool:
        """Whether motor driver is enabled."""
        return self._enabled

    def enable(self):
        """Enable motor driver (bring out of standby)."""
        if self.stby is not None:
            GPIO.output(self.stby, GPIO.HIGH)
        self._enabled = True
        logger.debug("Motor driver enabled")

    def disable(self):
        """Disable motor driver (standby mode)."""
        self.stop()
        if self.stby is not None:
            GPIO.output(self.stby, GPIO.LOW)
        self._enabled = False
        logger.debug("Motor driver disabled")

    def forward(self, speed: int = 100):
        """
        Move forward.

        Args:
            speed: Speed 0-100
        """
        if not self._enabled:
            self.enable()
        self.left.forward(speed)
        self.right.forward(speed)

    def reverse(self, speed: int = 100):
        """
        Move backward.

        Args:
            speed: Speed 0-100
        """
        if not self._enabled:
            self.enable()
        self.left.reverse(speed)
        self.right.reverse(speed)

    def turn_left(self, speed: int = 100):
        """
        Pivot turn left (in place).

        Args:
            speed: Speed 0-100
        """
        if not self._enabled:
            self.enable()
        self.left.reverse(speed)
        self.right.forward(speed)

    def turn_right(self, speed: int = 100):
        """
        Pivot turn right (in place).

        Args:
            speed: Speed 0-100
        """
        if not self._enabled:
            self.enable()
        self.left.forward(speed)
        self.right.reverse(speed)

    def arc_left(self, speed: int = 100, ratio: float = 0.5):
        """
        Arc turn left (wider turn).

        Args:
            speed: Base speed 0-100
            ratio: Inner wheel ratio (0 = pivot, 1 = straight)
        """
        if not self._enabled:
            self.enable()
        inner_speed = int(speed * ratio)
        self.left.forward(inner_speed)
        self.right.forward(speed)

    def arc_right(self, speed: int = 100, ratio: float = 0.5):
        """
        Arc turn right (wider turn).

        Args:
            speed: Base speed 0-100
            ratio: Inner wheel ratio (0 = pivot, 1 = straight)
        """
        if not self._enabled:
            self.enable()
        inner_speed = int(speed * ratio)
        self.left.forward(speed)
        self.right.forward(inner_speed)

    def drive(self, left_speed: int, right_speed: int):
        """
        Direct control of both motors.

        Args:
            left_speed: Left motor speed (-100 to 100)
            right_speed: Right motor speed (-100 to 100)
        """
        if not self._enabled:
            self.enable()
        self.left.drive(left_speed)
        self.right.drive(right_speed)

    def stop(self):
        """Stop both motors (coast)."""
        self.left.stop()
        self.right.stop()

    def brake(self):
        """Brake both motors."""
        self.left.brake()
        self.right.brake()

    def cleanup(self):
        """Cleanup GPIO and disable driver."""
        self.disable()
        self.left.cleanup()
        self.right.cleanup()


# =============================================================================
# Factory function
# =============================================================================

def create_robot(
    left_pins: Tuple[int, int, int] = (13, 15, 33),
    right_pins: Tuple[int, int, int] = (16, 18, 32),
    stby_pin: int = 11,
    setup_gpio: bool = True,
) -> DifferentialDrive:
    """
    Factory function to create a configured robot.

    Args:
        left_pins: (IN1, IN2, PWM) for left motor
        right_pins: (IN1, IN2, PWM) for right motor
        stby_pin: Standby pin
        setup_gpio: Whether to call GPIO.setmode()

    Returns:
        Configured DifferentialDrive instance
    """
    if setup_gpio:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

    left = Motor(
        in1=left_pins[0],
        in2=left_pins[1],
        pwm=left_pins[2],
        offset=LEFT_OFFSET,
    )
    right = Motor(
        in1=right_pins[0],
        in2=right_pins[1],
        pwm=right_pins[2],
        offset=RIGHT_OFFSET,
    )

    return DifferentialDrive(left, right, stby_pin)


# =============================================================================
# CLI for testing
# =============================================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Motor control test")
    parser.add_argument("--speed", type=int, default=50, help="Motor speed (0-100)")
    parser.add_argument("--duration", type=float, default=2.0, help="Test duration")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG)

    print(f"Platform: {PLATFORM}")
    print(f"Testing motors at speed {args.speed} for {args.duration}s")
    print()

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    try:
        robot = create_robot(setup_gpio=False)

        print("Forward...")
        robot.forward(args.speed)
        time.sleep(args.duration)

        print("Stop...")
        robot.stop()
        time.sleep(0.5)

        print("Reverse...")
        robot.reverse(args.speed)
        time.sleep(args.duration)

        print("Stop...")
        robot.stop()
        time.sleep(0.5)

        print("Turn left...")
        robot.turn_left(args.speed)
        time.sleep(args.duration / 2)

        print("Turn right...")
        robot.turn_right(args.speed)
        time.sleep(args.duration / 2)

        print("Stop...")
        robot.stop()

        print("\nTest complete!")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        robot.cleanup()
        GPIO.cleanup()
        print("Cleanup done")
