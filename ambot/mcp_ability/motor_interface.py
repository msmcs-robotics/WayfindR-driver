"""
Motor Interface — Abstraction layer between MCP tools and hardware.

Provides thread-safe motor control with simulation mode and auto-stop safety.
Supports both Jetson (JetsonL298N) and RPi (DifferentialDrive) platforms.
"""

import time
import logging
import platform
import threading
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)

AUTO_STOP_TIMEOUT = 30.0  # seconds with no command before auto-stop


@dataclass
class MotorState:
    """Current state of the motors."""
    left_speed: int = 0
    right_speed: int = 0
    direction: str = "stopped"
    last_command: str = "none"
    last_command_time: float = 0.0
    start_time: float = field(default_factory=time.time)
    command_count: int = 0

    def to_dict(self) -> dict:
        uptime = time.time() - self.start_time
        idle = time.time() - self.last_command_time if self.last_command_time > 0 else uptime
        return {
            "left_speed": self.left_speed,
            "right_speed": self.right_speed,
            "direction": self.direction,
            "last_command": self.last_command,
            "idle_seconds": round(idle, 1),
            "uptime_seconds": round(uptime, 1),
            "command_count": self.command_count,
        }


class MotorInterface:
    """
    Thread-safe motor control interface.

    Args:
        simulate: If True, log commands without touching hardware.
        auto_stop_timeout: Seconds of inactivity before auto-stop (0 to disable).
    """

    def __init__(self, simulate: bool = True, auto_stop_timeout: float = AUTO_STOP_TIMEOUT):
        self.simulate = simulate
        self.auto_stop_timeout = auto_stop_timeout
        self._lock = threading.Lock()
        self._state = MotorState()
        self._robot = None  # DifferentialDrive instance
        self._duration_timer: Optional[threading.Timer] = None
        self._watchdog_timer: Optional[threading.Timer] = None

        if not simulate:
            self._init_hardware()

    # ------------------------------------------------------------------
    # Hardware init
    # ------------------------------------------------------------------

    def _init_hardware(self):
        """Import and create the appropriate motor driver for the current platform."""
        # Try Jetson first (aarch64 with Jetson.GPIO or gpiod)
        if platform.machine() == "aarch64":
            try:
                from locomotion.jetson_motors.driver import JetsonL298N
                jetson_driver = JetsonL298N(simulate=False)
                if not jetson_driver.simulate:
                    # Wrap JetsonL298N to provide drive(left, right) interface
                    self._robot = _JetsonDriveAdapter(jetson_driver)
                    logger.info("Jetson motor driver initialized (backend=%s)", jetson_driver.backend)
                    return
                else:
                    logger.info("JetsonL298N fell back to simulate, trying RPi driver...")
            except Exception as exc:
                logger.debug("Jetson motor driver not available: %s", exc)

        # Fall back to RPi DifferentialDrive
        try:
            from locomotion.rpi_motors.factory import create_robot
            from locomotion.rpi_motors.drivers import DriverType
            self._robot = create_robot(driver_type=DriverType.L298N)
            logger.info("RPi motor driver initialized (L298N)")
        except Exception as exc:
            logger.error("Failed to init hardware motors: %s — falling back to simulate", exc)
            self.simulate = True

    # ------------------------------------------------------------------
    # Core drive
    # ------------------------------------------------------------------

    def drive(self, left_speed: int, right_speed: int,
              duration: float = 0, label: str = "drive") -> dict:
        """
        Set motor speeds. Thread-safe.

        Args:
            left_speed: -100..100
            right_speed: -100..100
            duration: Seconds to run (0 = continuous).
            label: Human-readable action name for status.

        Returns:
            Status dict.
        """
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        with self._lock:
            self._cancel_timers()

            # Determine direction label
            direction = self._direction_label(left_speed, right_speed)

            # Update state
            self._state.left_speed = left_speed
            self._state.right_speed = right_speed
            self._state.direction = direction
            self._state.last_command = label
            self._state.last_command_time = time.time()
            self._state.command_count += 1

            # Execute
            if self.simulate:
                logger.info("[SIM] %s  L=%+d  R=%+d  dur=%.1f",
                            label, left_speed, right_speed, duration)
            elif self._robot:
                self._robot.drive(left_speed, right_speed)

            # Schedule auto-stop after duration
            if duration > 0:
                self._duration_timer = threading.Timer(duration, self._auto_stop,
                                                       args=("duration",))
                self._duration_timer.daemon = True
                self._duration_timer.start()

            # Schedule watchdog auto-stop
            if self.auto_stop_timeout > 0:
                self._watchdog_timer = threading.Timer(self.auto_stop_timeout,
                                                       self._auto_stop,
                                                       args=("watchdog",))
                self._watchdog_timer.daemon = True
                self._watchdog_timer.start()

        return {
            "success": True,
            "action": label,
            "speed": max(abs(left_speed), abs(right_speed)),
            "duration": duration if duration > 0 else "continuous",
            "motor_state": self._state.to_dict(),
        }

    def stop(self, reason: str = "manual") -> dict:
        """Stop all motors immediately."""
        with self._lock:
            self._cancel_timers()

            self._state.left_speed = 0
            self._state.right_speed = 0
            self._state.direction = "stopped"
            self._state.last_command = f"stop ({reason})"
            self._state.last_command_time = time.time()
            self._state.command_count += 1

            if self.simulate:
                logger.info("[SIM] STOP (%s)", reason)
            elif self._robot:
                self._robot.stop()

        return {
            "success": True,
            "action": "stop",
            "reason": reason,
            "motor_state": self._state.to_dict(),
        }

    def get_status(self) -> dict:
        """Get current motor state."""
        with self._lock:
            return {
                "success": True,
                "simulate": self.simulate,
                "motor_state": self._state.to_dict(),
            }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _auto_stop(self, reason: str):
        """Called by timer threads to stop motors."""
        logger.warning("Auto-stop triggered: %s", reason)
        self.stop(reason=reason)

    def _cancel_timers(self):
        """Cancel pending duration and watchdog timers. Must hold _lock."""
        if self._duration_timer is not None:
            self._duration_timer.cancel()
            self._duration_timer = None
        if self._watchdog_timer is not None:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None

    @staticmethod
    def _direction_label(left: int, right: int) -> str:
        if left == 0 and right == 0:
            return "stopped"
        if left > 0 and right > 0:
            return "forward"
        if left < 0 and right < 0:
            return "backward"
        if left < 0 and right > 0:
            return "turning_left"
        if left > 0 and right < 0:
            return "turning_right"
        return "mixed"

    def cleanup(self):
        """Release hardware resources."""
        with self._lock:
            self._cancel_timers()
        if self._robot:
            self._robot.cleanup()
            logger.info("Hardware motor driver cleaned up")


class _JetsonDriveAdapter:
    """Adapts JetsonL298N (motor_a/motor_b) to the drive(left, right) interface."""

    def __init__(self, driver):
        self._driver = driver

    def drive(self, left_speed: int, right_speed: int):
        self._driver.motor_a(left_speed)
        self._driver.motor_b(right_speed)

    def stop(self):
        self._driver.stop()

    def cleanup(self):
        self._driver.cleanup()
