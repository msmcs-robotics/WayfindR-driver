"""
Jetson L298N Motor Driver (gpiod / libgpiod v2)

Controls two DC motors via an L298N H-bridge using the gpiod Python library.
Replaces the previous Jetson.GPIO-based driver which is broken on JetPack 6.x
(R36.4.4) — GPIO.output() silently fails to drive pins.

Falls back to simulation mode if gpiod is not available (useful for
development on a laptop).

PWM note: gpiod does not support PWM. ENA/ENB are driven HIGH/LOW for
full-speed on/off. For variable speed control, software PWM or sysfs
hardware PWM (/sys/class/pwm/) can be added later.

Usage:
    from locomotion.jetson_motors.driver import JetsonL298N

    motors = JetsonL298N()
    motors.forward(100)   # full speed (PWM not available — any nonzero = full)
    motors.stop()
    motors.cleanup()
"""

from __future__ import annotations

import atexit
import logging
import sys
from collections import defaultdict

from .config import JETSON_L298N_CONFIG, PWM_FREQ

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# gpiod import with simulation fallback
# ---------------------------------------------------------------------------

_SIMULATE = False

try:
    import gpiod
    from gpiod.line import Direction, Value
except ImportError:
    _SIMULATE = True
    gpiod = None
    logger.warning("gpiod not available — running in SIMULATION mode")


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
    Two-motor L298N driver for Jetson Orin Nano using gpiod (libgpiod v2).

    Speed values are -100..100:
        positive = forward
        negative = reverse
        0        = stop

    Note: Without PWM, any nonzero speed drives the motor at full power.
    The speed value is still tracked for API compatibility and future PWM.

    Attributes:
        simulate (bool): True when running without real GPIO.
    """

    def __init__(self, config: dict | None = None, pwm_freq: int = PWM_FREQ):
        """
        Initialize the motor driver.

        Args:
            config:   Pin mapping dict (defaults to JETSON_L298N_CONFIG).
                      Each value is a (chip_path, line_offset) tuple.
            pwm_freq: PWM frequency in Hz (retained for API compat, currently unused).
        """
        self.simulate = _SIMULATE
        self._config = config or JETSON_L298N_CONFIG
        self._pwm_freq = pwm_freq
        self._speed_a = 0
        self._speed_b = 0
        self._cleaned_up = False

        # gpiod line requests, keyed by chip_path
        self._requests: dict[str, object] = {}
        # Map from pin name to (chip_path, line_offset) for quick lookup
        self._pins: dict[str, tuple[str, int]] = {}

        self._setup()

        # Register cleanup so motors stop even on unhandled exit
        atexit.register(self.cleanup)

    # ----- internal setup --------------------------------------------------

    def _setup(self):
        """Configure GPIO lines via gpiod."""
        # Group lines by chip so we can make one request per chip
        chip_lines: dict[str, list[int]] = defaultdict(list)
        for pin_name in ['ENA', 'IN1', 'IN2', 'ENB', 'IN3', 'IN4']:
            chip_path, line_offset = self._config[pin_name]
            self._pins[pin_name] = (chip_path, line_offset)
            if line_offset not in chip_lines[chip_path]:
                chip_lines[chip_path].append(line_offset)

        if self.simulate:
            for chip_path, lines in chip_lines.items():
                self._requests[chip_path] = _SimLineRequest(
                    chip_path, lines, "ambot-motors"
                )
        else:
            for chip_path, lines in chip_lines.items():
                settings = gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE,
                )
                config = {tuple(lines): settings}
                self._requests[chip_path] = gpiod.request_lines(
                    chip_path,
                    consumer="ambot-motors",
                    config=config,
                )

        mode_label = "SIMULATION" if self.simulate else "HARDWARE"
        logger.info("JetsonL298N initialized (%s) via gpiod", mode_label)

    def _set_pin(self, pin_name: str, high: bool):
        """Set a single pin HIGH or LOW."""
        chip_path, line_offset = self._pins[pin_name]
        if self.simulate:
            val = _SimValue.ACTIVE if high else _SimValue.INACTIVE
            self._requests[chip_path].set_value(line_offset, val)
        else:
            val = Value.ACTIVE if high else Value.INACTIVE
            self._requests[chip_path].set_value(line_offset, val)

    # ----- single-motor control -------------------------------------------

    def motor_a(self, speed: int):
        """
        Set Motor A speed.

        Args:
            speed: -100..100 (negative = reverse, 0 = stop).
                   Without PWM, any nonzero value = full speed.
        """
        speed = max(-100, min(100, speed))
        self._speed_a = speed

        if speed > 0:
            self._set_pin('IN1', True)
            self._set_pin('IN2', False)
            self._set_pin('ENA', True)
        elif speed < 0:
            self._set_pin('IN1', False)
            self._set_pin('IN2', True)
            self._set_pin('ENA', True)
        else:
            self._set_pin('IN1', False)
            self._set_pin('IN2', False)
            self._set_pin('ENA', False)

    def motor_b(self, speed: int):
        """
        Set Motor B speed.

        Args:
            speed: -100..100 (negative = reverse, 0 = stop).
                   Without PWM, any nonzero value = full speed.
        """
        speed = max(-100, min(100, speed))
        self._speed_b = speed

        if speed > 0:
            self._set_pin('IN3', True)
            self._set_pin('IN4', False)
            self._set_pin('ENB', True)
        elif speed < 0:
            self._set_pin('IN3', False)
            self._set_pin('IN4', True)
            self._set_pin('ENB', True)
        else:
            self._set_pin('IN3', False)
            self._set_pin('IN4', False)
            self._set_pin('ENB', False)

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
        # Disable enable pins first, then clear direction
        self._set_pin('ENA', False)
        self._set_pin('ENB', False)
        self._set_pin('IN1', False)
        self._set_pin('IN2', False)
        self._set_pin('IN3', False)
        self._set_pin('IN4', False)

        self._speed_a = 0
        self._speed_b = 0

    # ----- cleanup --------------------------------------------------------

    def cleanup(self):
        """
        Stop motors and release gpiod line requests.

        Safe to call multiple times. Registered with atexit so motors
        stop even if the program crashes or is killed.
        """
        if self._cleaned_up:
            return
        self._cleaned_up = True

        logger.info("JetsonL298N cleanup — stopping motors and releasing lines")

        # Best-effort stop before releasing
        try:
            self.stop()
        except Exception:
            pass

        # Release all gpiod line requests
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
        mode = "SIM" if self.simulate else "HW"
        return f"JetsonL298N({mode}, A={self._speed_a}, B={self._speed_b})"
