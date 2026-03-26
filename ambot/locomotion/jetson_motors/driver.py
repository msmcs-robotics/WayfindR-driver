"""
Jetson L298N Motor Driver

Controls two DC motors via an L298N H-bridge using Jetson.GPIO.
Falls back to simulation mode if Jetson.GPIO is not available
(useful for development on a laptop).

Usage:
    from locomotion.jetson_motors.driver import JetsonL298N

    motors = JetsonL298N()
    motors.forward(50)    # both motors forward at 50%
    motors.stop()
    motors.cleanup()
"""

from __future__ import annotations

import atexit
import logging
import sys

from .config import JETSON_L298N_CONFIG, PWM_FREQ

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# GPIO import with simulation fallback
# ---------------------------------------------------------------------------

_SIMULATE = False

try:
    import Jetson.GPIO as GPIO
except ImportError:
    _SIMULATE = True
    GPIO = None
    logger.warning("Jetson.GPIO not available — running in SIMULATION mode")


class _SimGPIO:
    """
    Minimal GPIO stub for simulation mode.
    Logs calls instead of touching hardware.
    """
    BOARD = "BOARD"
    OUT = "OUT"
    HIGH = 1
    LOW = 0

    _warned = False

    @classmethod
    def _log(cls, msg):
        if not cls._warned:
            logger.info("[SIM] Simulation mode active — no hardware output")
            cls._warned = True
        logger.debug("[SIM] %s", msg)

    @classmethod
    def setmode(cls, mode):
        cls._log(f"setmode({mode})")

    @classmethod
    def setup(cls, pin, direction, initial=None):
        cls._log(f"setup(pin={pin}, dir={direction}, initial={initial})")

    @classmethod
    def output(cls, pin, value):
        cls._log(f"output(pin={pin}, value={value})")

    @classmethod
    def cleanup(cls):
        cls._log("cleanup()")

    class PWM:
        def __init__(self, pin, freq):
            _SimGPIO._log(f"PWM(pin={pin}, freq={freq})")
            self.pin = pin
            self.duty = 0

        def start(self, duty):
            self.duty = duty
            _SimGPIO._log(f"PWM.start(pin={self.pin}, duty={duty})")

        def ChangeDutyCycle(self, duty):
            self.duty = duty

        def stop(self):
            _SimGPIO._log(f"PWM.stop(pin={self.pin})")


# ---------------------------------------------------------------------------
# JetsonL298N driver
# ---------------------------------------------------------------------------

class JetsonL298N:
    """
    Two-motor L298N driver for Jetson Orin Nano.

    Speed values are -100..100:
        positive = forward
        negative = reverse
        0        = stop

    Attributes:
        simulate (bool): True when running without real GPIO.
    """

    def __init__(self, config: dict | None = None, pwm_freq: int = PWM_FREQ):
        """
        Initialize the motor driver.

        Args:
            config:   Pin mapping dict (defaults to JETSON_L298N_CONFIG).
            pwm_freq: PWM frequency in Hz (default 1000).
        """
        self.simulate = _SIMULATE
        self._gpio = GPIO if not _SIMULATE else _SimGPIO
        self._config = config or JETSON_L298N_CONFIG
        self._pwm_freq = pwm_freq
        self._pwm_a = None
        self._pwm_b = None
        self._speed_a = 0
        self._speed_b = 0
        self._cleaned_up = False

        self._setup()

        # Register cleanup so motors stop even on unhandled exit
        atexit.register(self.cleanup)

    # ----- internal setup --------------------------------------------------

    def _setup(self):
        """Configure GPIO pins and start PWM channels."""
        self._gpio.setmode(self._gpio.BOARD)

        # Motor A pins
        self._gpio.setup(self._config['IN1'], self._gpio.OUT, initial=self._gpio.LOW)
        self._gpio.setup(self._config['IN2'], self._gpio.OUT, initial=self._gpio.LOW)
        self._gpio.setup(self._config['ENA'], self._gpio.OUT, initial=self._gpio.LOW)

        # Motor B pins
        self._gpio.setup(self._config['IN3'], self._gpio.OUT, initial=self._gpio.LOW)
        self._gpio.setup(self._config['IN4'], self._gpio.OUT, initial=self._gpio.LOW)
        self._gpio.setup(self._config['ENB'], self._gpio.OUT, initial=self._gpio.LOW)

        # PWM on enable pins
        self._pwm_a = self._gpio.PWM(self._config['ENA'], self._pwm_freq)
        self._pwm_b = self._gpio.PWM(self._config['ENB'], self._pwm_freq)
        self._pwm_a.start(0)
        self._pwm_b.start(0)

        mode_label = "SIMULATION" if self.simulate else "HARDWARE"
        logger.info("JetsonL298N initialized (%s), PWM %d Hz", mode_label, self._pwm_freq)

    # ----- single-motor control -------------------------------------------

    def motor_a(self, speed: int):
        """
        Set Motor A speed.

        Args:
            speed: -100..100 (negative = reverse, 0 = stop).
        """
        speed = max(-100, min(100, speed))
        self._speed_a = speed
        duty = abs(speed)

        if speed > 0:
            self._gpio.output(self._config['IN1'], self._gpio.HIGH)
            self._gpio.output(self._config['IN2'], self._gpio.LOW)
        elif speed < 0:
            self._gpio.output(self._config['IN1'], self._gpio.LOW)
            self._gpio.output(self._config['IN2'], self._gpio.HIGH)
        else:
            self._gpio.output(self._config['IN1'], self._gpio.LOW)
            self._gpio.output(self._config['IN2'], self._gpio.LOW)

        if self._pwm_a:
            self._pwm_a.ChangeDutyCycle(duty)

    def motor_b(self, speed: int):
        """
        Set Motor B speed.

        Args:
            speed: -100..100 (negative = reverse, 0 = stop).
        """
        speed = max(-100, min(100, speed))
        self._speed_b = speed
        duty = abs(speed)

        if speed > 0:
            self._gpio.output(self._config['IN3'], self._gpio.HIGH)
            self._gpio.output(self._config['IN4'], self._gpio.LOW)
        elif speed < 0:
            self._gpio.output(self._config['IN3'], self._gpio.LOW)
            self._gpio.output(self._config['IN4'], self._gpio.HIGH)
        else:
            self._gpio.output(self._config['IN3'], self._gpio.LOW)
            self._gpio.output(self._config['IN4'], self._gpio.LOW)

        if self._pwm_b:
            self._pwm_b.ChangeDutyCycle(duty)

    # ----- compound movement commands -------------------------------------

    def forward(self, speed: int = 50):
        """Both motors forward at the given speed (0..100)."""
        speed = max(0, min(100, speed))
        self.motor_a(speed)
        self.motor_b(speed)

    def backward(self, speed: int = 50):
        """Both motors reverse at the given speed (0..100)."""
        speed = max(0, min(100, speed))
        self.motor_a(-speed)
        self.motor_b(-speed)

    def spin_left(self, speed: int = 50):
        """Spin in place: left motor reverse, right motor forward."""
        speed = max(0, min(100, speed))
        self.motor_a(-speed)
        self.motor_b(speed)

    def spin_right(self, speed: int = 50):
        """Spin in place: left motor forward, right motor reverse."""
        speed = max(0, min(100, speed))
        self.motor_a(speed)
        self.motor_b(-speed)

    def stop(self):
        """Stop both motors (coast stop)."""
        # Zero PWM first, then clear direction pins
        if self._pwm_a:
            self._pwm_a.ChangeDutyCycle(0)
        if self._pwm_b:
            self._pwm_b.ChangeDutyCycle(0)

        self._gpio.output(self._config['IN1'], self._gpio.LOW)
        self._gpio.output(self._config['IN2'], self._gpio.LOW)
        self._gpio.output(self._config['IN3'], self._gpio.LOW)
        self._gpio.output(self._config['IN4'], self._gpio.LOW)

        self._speed_a = 0
        self._speed_b = 0

    # ----- cleanup --------------------------------------------------------

    def cleanup(self):
        """
        Stop motors and release GPIO resources.

        Safe to call multiple times. Registered with atexit so motors
        stop even if the program crashes or is killed.
        """
        if self._cleaned_up:
            return
        self._cleaned_up = True

        logger.info("JetsonL298N cleanup — stopping motors")
        self.stop()

        if self._pwm_a:
            self._pwm_a.stop()
            self._pwm_a = None
        if self._pwm_b:
            self._pwm_b.stop()
            self._pwm_b = None

        self._gpio.cleanup()

    # ----- status ---------------------------------------------------------

    @property
    def speed_a(self) -> int:
        """Current Motor A speed (-100..100)."""
        return self._speed_a

    @property
    def speed_b(self) -> int:
        """Current Motor B speed (-100..100)."""
        return self._speed_b

    def __repr__(self):
        mode = "SIM" if self.simulate else "HW"
        return f"JetsonL298N({mode}, A={self._speed_a}, B={self._speed_b})"
