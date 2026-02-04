"""
Motor and DifferentialDrive Classes

High-level motor control abstraction that works with any supported driver.
"""

from __future__ import annotations

import logging
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .drivers import MotorDriver

logger = logging.getLogger(__name__)


class Motor:
    """
    High-level motor control wrapper.

    Wraps a MotorDriver instance and provides direction offset correction.
    """

    def __init__(self, driver: MotorDriver, offset: int = 1):
        """
        Initialize motor with a driver.

        Args:
            driver: MotorDriver instance (TB6612FNG, L298N, or DRV8833)
            offset: Direction offset (1 or -1) to correct wiring
        """
        self.driver = driver
        self.offset = offset
        self._speed = 0

    def setup(self):
        """Setup the underlying driver."""
        self.driver.setup()

    @property
    def speed(self) -> int:
        """Current motor speed (-100 to 100)."""
        return self._speed

    def drive(self, speed: int):
        """
        Drive motor at specified speed.

        Args:
            speed: -100 to 100 (negative = reverse)
        """
        speed = max(-100, min(100, speed))

        # Apply direction offset
        if self.offset == -1:
            speed = -speed

        if speed > 0:
            self.driver.forward(abs(speed))
        elif speed < 0:
            self.driver.reverse(abs(speed))
        else:
            self.driver.stop()

        self._speed = speed

    def forward(self, speed: int = 100):
        """Run motor forward at specified speed (0-100)."""
        self.drive(abs(speed))

    def reverse(self, speed: int = 100):
        """Run motor in reverse at specified speed (0-100)."""
        self.drive(-abs(speed))

    def stop(self):
        """Stop motor (coast)."""
        self.driver.stop()
        self._speed = 0

    def brake(self):
        """Brake motor (short circuit windings)."""
        self.driver.brake()
        self._speed = 0

    def cleanup(self):
        """Cleanup driver resources."""
        self.driver.cleanup()


class DifferentialDrive:
    """
    Differential drive (tank-style) robot controller.

    Controls two motors for forward, reverse, turning, and arc movements.
    """

    def __init__(
        self,
        left_motor: Motor,
        right_motor: Motor,
        gpio=None,
        stby_pin: Optional[int] = None,
    ):
        """
        Initialize differential drive controller.

        Args:
            left_motor: Left Motor instance
            right_motor: Right Motor instance
            gpio: GPIO library reference (for STBY pin control)
            stby_pin: Standby pin for motor driver (optional)
        """
        self.left = left_motor
        self.right = right_motor
        self.GPIO = gpio
        self.stby = stby_pin
        self._enabled = False

        logger.info("DifferentialDrive initialized")

    def setup(self):
        """Setup motors and standby pin."""
        self.left.setup()
        self.right.setup()

        if self.stby is not None and self.GPIO is not None:
            self.GPIO.setup(self.stby, self.GPIO.OUT, initial=self.GPIO.LOW)

    @property
    def enabled(self) -> bool:
        """Whether motor driver is enabled."""
        return self._enabled

    def enable(self):
        """Enable motor driver (bring out of standby)."""
        if self.stby is not None and self.GPIO is not None:
            self.GPIO.output(self.stby, self.GPIO.HIGH)
        self._enabled = True
        logger.debug("Motor driver enabled")

    def disable(self):
        """Disable motor driver (standby mode)."""
        self.stop()
        if self.stby is not None and self.GPIO is not None:
            self.GPIO.output(self.stby, self.GPIO.LOW)
        self._enabled = False
        logger.debug("Motor driver disabled")

    def _ensure_enabled(self):
        """Auto-enable if not already enabled."""
        if not self._enabled:
            self.enable()

    def forward(self, speed: int = 100):
        """
        Move forward.

        Args:
            speed: Speed 0-100
        """
        self._ensure_enabled()
        self.left.forward(speed)
        self.right.forward(speed)

    def reverse(self, speed: int = 100):
        """
        Move backward.

        Args:
            speed: Speed 0-100
        """
        self._ensure_enabled()
        self.left.reverse(speed)
        self.right.reverse(speed)

    def turn_left(self, speed: int = 100):
        """
        Pivot turn left (in place).

        Args:
            speed: Speed 0-100
        """
        self._ensure_enabled()
        self.left.reverse(speed)
        self.right.forward(speed)

    def turn_right(self, speed: int = 100):
        """
        Pivot turn right (in place).

        Args:
            speed: Speed 0-100
        """
        self._ensure_enabled()
        self.left.forward(speed)
        self.right.reverse(speed)

    def arc_left(self, speed: int = 100, ratio: float = 0.5):
        """
        Arc turn left (wider turn).

        Args:
            speed: Base speed 0-100
            ratio: Inner wheel ratio (0 = pivot, 1 = straight)
        """
        self._ensure_enabled()
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
        self._ensure_enabled()
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
        self._ensure_enabled()
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
