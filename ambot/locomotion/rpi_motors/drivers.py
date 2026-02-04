"""
Motor Driver Implementations

Provides abstraction layer for different motor driver chips:
- TB6612FNG (recommended for Raspberry Pi + FAGM25-370)
- L298N (budget option, less efficient)
- DRV8833 (low voltage only, max 10.8V)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .config import DriverConfig


class DriverType(Enum):
    """Supported motor driver types."""
    TB6612FNG = auto()
    L298N = auto()
    DRV8833 = auto()


@dataclass
class DriverSpecs:
    """Motor driver specifications."""
    name: str
    min_voltage: float
    max_voltage: float
    continuous_current: float
    peak_current: float
    voltage_drop: float
    has_standby: bool
    pwm_max_freq: int


class MotorDriver(ABC):
    """
    Abstract base class for motor drivers.

    Handles the low-level GPIO control for a single motor channel.
    """

    specs: DriverSpecs

    def __init__(self, gpio, in1: int, in2: int, pwm_pin: int, pwm_freq: int = 1000):
        """
        Initialize motor driver channel.

        Args:
            gpio: GPIO library (RPi.GPIO or Jetson.GPIO)
            in1: Direction control pin 1
            in2: Direction control pin 2
            pwm_pin: PWM speed control pin
            pwm_freq: PWM frequency in Hz
        """
        self.GPIO = gpio
        self.in1 = in1
        self.in2 = in2
        self.pwm_pin = pwm_pin
        self.pwm_freq = pwm_freq
        self._speed = 0
        self._pwm = None

    def setup(self):
        """Setup GPIO pins."""
        self.GPIO.setup(self.in1, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.in2, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.pwm_pin, self.GPIO.OUT, initial=self.GPIO.LOW)
        self._pwm = self.GPIO.PWM(self.pwm_pin, self.pwm_freq)
        self._pwm.start(0)

    @abstractmethod
    def forward(self, speed: int):
        """Run motor forward at given speed (0-100)."""
        pass

    @abstractmethod
    def reverse(self, speed: int):
        """Run motor in reverse at given speed (0-100)."""
        pass

    @abstractmethod
    def stop(self):
        """Stop motor (coast)."""
        pass

    @abstractmethod
    def brake(self):
        """Brake motor (short circuit windings)."""
        pass

    def cleanup(self):
        """Cleanup GPIO resources."""
        self.stop()
        if self._pwm:
            self._pwm.stop()

    @property
    def speed(self) -> int:
        """Current motor speed."""
        return self._speed


class TB6612FNG(MotorDriver):
    """
    TB6612FNG motor driver implementation.

    Recommended driver for Raspberry Pi with FAGM25-370 motors.
    - Voltage: 4.5V - 13.5V
    - Current: 1.2A continuous, 3.2A peak
    - Efficiency: ~95% (MOSFET-based)
    """

    specs = DriverSpecs(
        name="TB6612FNG",
        min_voltage=4.5,
        max_voltage=13.5,
        continuous_current=1.2,
        peak_current=3.2,
        voltage_drop=0.2,
        has_standby=True,
        pwm_max_freq=100000,
    )

    def forward(self, speed: int):
        """Run motor forward."""
        speed = max(0, min(100, speed))
        self.GPIO.output(self.in1, self.GPIO.HIGH)
        self.GPIO.output(self.in2, self.GPIO.LOW)
        self._pwm.ChangeDutyCycle(speed)
        self._speed = speed

    def reverse(self, speed: int):
        """Run motor in reverse."""
        speed = max(0, min(100, speed))
        self.GPIO.output(self.in1, self.GPIO.LOW)
        self.GPIO.output(self.in2, self.GPIO.HIGH)
        self._pwm.ChangeDutyCycle(speed)
        self._speed = -speed

    def stop(self):
        """Stop motor (coast - both LOW)."""
        self.GPIO.output(self.in1, self.GPIO.LOW)
        self.GPIO.output(self.in2, self.GPIO.LOW)
        self._pwm.ChangeDutyCycle(0)
        self._speed = 0

    def brake(self):
        """Brake motor (short brake - both HIGH)."""
        self.GPIO.output(self.in1, self.GPIO.HIGH)
        self.GPIO.output(self.in2, self.GPIO.HIGH)
        self._pwm.ChangeDutyCycle(0)
        self._speed = 0


class L298N(MotorDriver):
    """
    L298N motor driver implementation.

    Budget option, less efficient than TB6612FNG.
    - Voltage: 5V - 35V
    - Current: 2A continuous
    - Efficiency: ~75% (BJT-based, 2V+ voltage drop)

    Note: L298N uses ENA/ENB pins for PWM (directly connects to motor enable).
    """

    specs = DriverSpecs(
        name="L298N",
        min_voltage=5.0,
        max_voltage=35.0,
        continuous_current=2.0,
        peak_current=3.5,
        voltage_drop=2.5,
        has_standby=False,  # Uses ENA/ENB instead
        pwm_max_freq=25000,
    )

    def forward(self, speed: int):
        """Run motor forward."""
        speed = max(0, min(100, speed))
        self.GPIO.output(self.in1, self.GPIO.HIGH)
        self.GPIO.output(self.in2, self.GPIO.LOW)
        self._pwm.ChangeDutyCycle(speed)
        self._speed = speed

    def reverse(self, speed: int):
        """Run motor in reverse."""
        speed = max(0, min(100, speed))
        self.GPIO.output(self.in1, self.GPIO.LOW)
        self.GPIO.output(self.in2, self.GPIO.HIGH)
        self._pwm.ChangeDutyCycle(speed)
        self._speed = -speed

    def stop(self):
        """Stop motor."""
        self.GPIO.output(self.in1, self.GPIO.LOW)
        self.GPIO.output(self.in2, self.GPIO.LOW)
        self._pwm.ChangeDutyCycle(0)
        self._speed = 0

    def brake(self):
        """Brake motor (same as stop for L298N)."""
        # L298N doesn't have true brake mode like TB6612FNG
        self.stop()


class DRV8833(MotorDriver):
    """
    DRV8833 motor driver implementation.

    Efficient but limited to low voltage (max 10.8V).
    NOT recommended for 12V FAGM25-370 motors.

    - Voltage: 2.7V - 10.8V
    - Current: 1.2A continuous, 2A peak
    - Efficiency: ~95%

    Note: DRV8833 uses PWM on xIN pins directly (no separate PWM pin).
    This implementation uses PWM on IN1 for forward, IN2 for reverse.
    """

    specs = DriverSpecs(
        name="DRV8833",
        min_voltage=2.7,
        max_voltage=10.8,
        continuous_current=1.2,
        peak_current=2.0,
        voltage_drop=0.18,
        has_standby=False,  # Has sleep pin instead
        pwm_max_freq=50000,
    )

    def __init__(self, gpio, in1: int, in2: int, pwm_pin: int = None, pwm_freq: int = 1000):
        """
        DRV8833 uses IN1/IN2 directly for PWM control.
        pwm_pin is ignored - PWM is applied to direction pins.
        """
        self.GPIO = gpio
        self.in1 = in1
        self.in2 = in2
        self.pwm_pin = pwm_pin  # Not used for DRV8833
        self.pwm_freq = pwm_freq
        self._speed = 0
        self._pwm1 = None
        self._pwm2 = None

    def setup(self):
        """Setup GPIO pins with PWM on both direction pins."""
        self.GPIO.setup(self.in1, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.in2, self.GPIO.OUT, initial=self.GPIO.LOW)
        self._pwm1 = self.GPIO.PWM(self.in1, self.pwm_freq)
        self._pwm2 = self.GPIO.PWM(self.in2, self.pwm_freq)
        self._pwm1.start(0)
        self._pwm2.start(0)

    def forward(self, speed: int):
        """Run motor forward (PWM on IN1, IN2 LOW)."""
        speed = max(0, min(100, speed))
        self._pwm1.ChangeDutyCycle(speed)
        self._pwm2.ChangeDutyCycle(0)
        self._speed = speed

    def reverse(self, speed: int):
        """Run motor in reverse (IN1 LOW, PWM on IN2)."""
        speed = max(0, min(100, speed))
        self._pwm1.ChangeDutyCycle(0)
        self._pwm2.ChangeDutyCycle(speed)
        self._speed = -speed

    def stop(self):
        """Stop motor (coast - both LOW)."""
        self._pwm1.ChangeDutyCycle(0)
        self._pwm2.ChangeDutyCycle(0)
        self._speed = 0

    def brake(self):
        """Brake motor (slow decay - both HIGH)."""
        self._pwm1.ChangeDutyCycle(100)
        self._pwm2.ChangeDutyCycle(100)
        self._speed = 0

    def cleanup(self):
        """Cleanup GPIO resources."""
        self.stop()
        if self._pwm1:
            self._pwm1.stop()
        if self._pwm2:
            self._pwm2.stop()


# Driver class lookup
DRIVER_CLASSES = {
    DriverType.TB6612FNG: TB6612FNG,
    DriverType.L298N: L298N,
    DriverType.DRV8833: DRV8833,
}


def get_driver_class(driver_type: DriverType) -> type:
    """Get the driver class for a given driver type."""
    return DRIVER_CLASSES[driver_type]
