"""
Jetson L298N Motor Driver — Jetson.GPIO + gpiod fallback

Controls two DC motors via an L298N H-bridge.

Backend priority:
  1. Jetson.GPIO (BOARD mode) — works on original DTB, supports hardware PWM
  2. gpiod (libgpiod v2) — fallback if Jetson.GPIO fails (e.g., custom DTB)
  3. Simulation mode — if neither GPIO library is available

Usage:
    from locomotion.jetson_motors.driver import JetsonL298N

    motors = JetsonL298N()
    motors.forward(100)
    motors.stop()
    motors.cleanup()
"""

from __future__ import annotations

import atexit
import logging
import sys
from collections import defaultdict

from .config import (
    JETSON_L298N_BOARD_PINS,
    JETSON_L298N_GPIOD,
    PWM_FREQ,
)

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Backend detection
# ---------------------------------------------------------------------------

_BACKEND = "simulate"  # "jetson_gpio", "gpiod", or "simulate"

_JetsonGPIO = None
_gpiod = None

try:
    import Jetson.GPIO as _JetsonGPIO
    # Quick sanity check — model property works only on real Jetson
    _model = _JetsonGPIO.model
    _BACKEND = "jetson_gpio"
    logger.info("Jetson.GPIO available (model: %s) — using as primary backend", _model)
except Exception as e:
    logger.debug("Jetson.GPIO not usable: %s", e)
    try:
        import gpiod as _gpiod
        from gpiod.line import Direction, Value
        _BACKEND = "gpiod"
        logger.info("gpiod available — using as fallback backend")
    except ImportError:
        logger.warning("No GPIO library available — running in SIMULATION mode")


# ---------------------------------------------------------------------------
# Simulation stubs
# ---------------------------------------------------------------------------

class _SimValue:
    """Stub for gpiod.line.Value."""
    ACTIVE = 1
    INACTIVE = 0


class _SimLineRequest:
    """Minimal gpiod LineRequest stub for simulation mode."""

    _warned = False

    def __init__(self, chip_path, lines, consumer):
        if not _SimLineRequest._warned:
            logger.info("[SIM] Simulation mode active — no hardware output")
            _SimLineRequest._warned = True
        logger.debug("[SIM] request_lines(%s, lines=%s, consumer=%s)",
                     chip_path, lines, consumer)

    def set_value(self, line, value):
        logger.debug("[SIM] set_value(line=%s, value=%s)", line, value)

    def release(self):
        logger.debug("[SIM] release()")


# ---------------------------------------------------------------------------
# JetsonL298N driver
# ---------------------------------------------------------------------------

class JetsonL298N:
    """
    Two-motor L298N driver for Jetson Orin Nano.

    Tries Jetson.GPIO first (BOARD mode), falls back to gpiod, then simulation.

    Speed values are -100..100:
        positive = forward
        negative = reverse
        0        = stop

    With Jetson.GPIO backend: ENA/ENB use hardware PWM for variable speed.
    With gpiod backend: ENA/ENB are HIGH/LOW only (any nonzero = full speed).

    Attributes:
        simulate (bool): True when running without real GPIO.
        backend (str): "jetson_gpio", "gpiod", or "simulate".
    """

    def __init__(self, simulate: bool = False, config: dict | None = None,
                 board_pins: dict | None = None, pwm_freq: int = PWM_FREQ):
        """
        Initialize the motor driver.

        Args:
            simulate:   Force simulation mode (no hardware access).
            config:     gpiod pin mapping dict (defaults to JETSON_L298N_GPIOD).
            board_pins: BOARD pin mapping dict (defaults to JETSON_L298N_BOARD_PINS).
            pwm_freq:   PWM frequency in Hz for Jetson.GPIO backend.
        """
        self._board_pins = board_pins or JETSON_L298N_BOARD_PINS
        self._gpiod_config = config or JETSON_L298N_GPIOD
        self._pwm_freq = pwm_freq
        self._speed_a = 0
        self._speed_b = 0
        self._cleaned_up = False

        if simulate:
            self.backend = "simulate"
        else:
            self.backend = _BACKEND

        self.simulate = (self.backend == "simulate")

        # Backend-specific state
        self._pwm_a = None  # Jetson.GPIO PWM for ENA
        self._pwm_b = None  # Jetson.GPIO PWM for ENB
        self._requests: dict[str, object] = {}  # gpiod line requests
        self._pins: dict[str, tuple[str, int]] = {}  # gpiod pin lookup

        self._setup()

        # Register cleanup so motors stop even on unhandled exit
        atexit.register(self.cleanup)

    # ----- internal setup --------------------------------------------------

    def _setup(self):
        """Configure GPIO lines using the active backend."""
        if self.backend == "jetson_gpio":
            self._setup_jetson_gpio()
        elif self.backend == "gpiod":
            self._setup_gpiod()
        else:
            self._setup_simulate()

        logger.info("JetsonL298N initialized (backend=%s)", self.backend)

    def _setup_jetson_gpio(self):
        """Configure pins via Jetson.GPIO in BOARD mode."""
        GPIO = _JetsonGPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        pins = self._board_pins
        # Direction pins as outputs, initially LOW
        for name in ['IN1', 'IN2', 'IN3', 'IN4']:
            GPIO.setup(pins[name], GPIO.OUT, initial=GPIO.LOW)

        # Enable pins with hardware PWM
        GPIO.setup(pins['ENA'], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(pins['ENB'], GPIO.OUT, initial=GPIO.LOW)
        self._pwm_a = GPIO.PWM(pins['ENA'], self._pwm_freq)
        self._pwm_b = GPIO.PWM(pins['ENB'], self._pwm_freq)
        self._pwm_a.start(0)
        self._pwm_b.start(0)

    def _setup_gpiod(self):
        """Configure pins via gpiod (libgpiod v2)."""
        from gpiod.line import Direction, Value

        chip_lines: dict[str, list[int]] = defaultdict(list)
        for pin_name in ['ENA', 'IN1', 'IN2', 'ENB', 'IN3', 'IN4']:
            chip_path, line_offset = self._gpiod_config[pin_name]
            self._pins[pin_name] = (chip_path, line_offset)
            if line_offset not in chip_lines[chip_path]:
                chip_lines[chip_path].append(line_offset)

        for chip_path, lines in chip_lines.items():
            settings = _gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE,
            )
            config = {tuple(lines): settings}
            self._requests[chip_path] = _gpiod.request_lines(
                chip_path,
                consumer="ambot-motors",
                config=config,
            )

    def _setup_simulate(self):
        """Configure simulation stubs."""
        chip_lines: dict[str, list[int]] = defaultdict(list)
        for pin_name in ['ENA', 'IN1', 'IN2', 'ENB', 'IN3', 'IN4']:
            chip_path, line_offset = self._gpiod_config[pin_name]
            self._pins[pin_name] = (chip_path, line_offset)
            if line_offset not in chip_lines[chip_path]:
                chip_lines[chip_path].append(line_offset)

        for chip_path, lines in chip_lines.items():
            self._requests[chip_path] = _SimLineRequest(
                chip_path, lines, "ambot-motors"
            )

    # ----- pin control -----------------------------------------------------

    def _set_pin(self, pin_name: str, high: bool):
        """Set a single pin HIGH or LOW."""
        if self.backend == "jetson_gpio":
            GPIO = _JetsonGPIO
            pin = self._board_pins[pin_name]
            GPIO.output(pin, GPIO.HIGH if high else GPIO.LOW)
        elif self.backend == "gpiod":
            from gpiod.line import Value
            chip_path, line_offset = self._pins[pin_name]
            val = Value.ACTIVE if high else Value.INACTIVE
            self._requests[chip_path].set_value(line_offset, val)
        else:
            chip_path, line_offset = self._pins[pin_name]
            val = _SimValue.ACTIVE if high else _SimValue.INACTIVE
            self._requests[chip_path].set_value(line_offset, val)

    def _set_enable(self, motor: str, duty_cycle: int):
        """Set enable pin — PWM duty cycle (Jetson.GPIO) or HIGH/LOW (gpiod/sim)."""
        if self.backend == "jetson_gpio":
            pwm = self._pwm_a if motor == 'A' else self._pwm_b
            pwm.ChangeDutyCycle(abs(duty_cycle))
        else:
            pin_name = 'ENA' if motor == 'A' else 'ENB'
            self._set_pin(pin_name, duty_cycle != 0)

    # ----- single-motor control -------------------------------------------

    def motor_a(self, speed: int):
        """
        Set Motor A speed.

        Args:
            speed: -100..100 (negative = reverse, 0 = stop).
        """
        speed = max(-100, min(100, speed))
        self._speed_a = speed

        if speed > 0:
            self._set_pin('IN1', True)
            self._set_pin('IN2', False)
            self._set_enable('A', speed)
        elif speed < 0:
            self._set_pin('IN1', False)
            self._set_pin('IN2', True)
            self._set_enable('A', -speed)
        else:
            self._set_pin('IN1', False)
            self._set_pin('IN2', False)
            self._set_enable('A', 0)

    def motor_b(self, speed: int):
        """
        Set Motor B speed.

        Args:
            speed: -100..100 (negative = reverse, 0 = stop).
        """
        speed = max(-100, min(100, speed))
        self._speed_b = speed

        if speed > 0:
            self._set_pin('IN3', True)
            self._set_pin('IN4', False)
            self._set_enable('B', speed)
        elif speed < 0:
            self._set_pin('IN3', False)
            self._set_pin('IN4', True)
            self._set_enable('B', -speed)
        else:
            self._set_pin('IN3', False)
            self._set_pin('IN4', False)
            self._set_enable('B', 0)

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
        self._set_enable('A', 0)
        self._set_enable('B', 0)
        self._set_pin('IN1', False)
        self._set_pin('IN2', False)
        self._set_pin('IN3', False)
        self._set_pin('IN4', False)

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

        logger.info("JetsonL298N cleanup — stopping motors and releasing GPIO")

        # Best-effort stop before releasing
        try:
            self.stop()
        except Exception:
            pass

        if self.backend == "jetson_gpio":
            # Stop PWM channels
            for pwm in (self._pwm_a, self._pwm_b):
                if pwm is not None:
                    try:
                        pwm.stop()
                    except Exception:
                        pass
            self._pwm_a = None
            self._pwm_b = None
            # Release all pins
            try:
                _JetsonGPIO.cleanup()
            except Exception as e:
                logger.debug("Error in GPIO.cleanup(): %s", e)
        else:
            # Release gpiod / sim line requests
            for chip_path, request in self._requests.items():
                try:
                    request.release()
                except Exception as e:
                    logger.debug("Error releasing %s: %s", chip_path, e)
            self._requests.clear()

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
        return f"JetsonL298N({self.backend}, A={self._speed_a}, B={self._speed_b})"
