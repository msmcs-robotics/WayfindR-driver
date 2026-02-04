"""
Modular Motor Control for Raspberry Pi

Supports multiple motor drivers:
- TB6612FNG (recommended)
- L298N
- DRV8833

Usage:
    from locomotion.rpi_motors import create_robot, DriverType

    robot = create_robot(driver_type=DriverType.TB6612FNG)
    robot.forward(50)
    robot.stop()
    robot.cleanup()
"""

from .drivers import DriverType, MotorDriver, TB6612FNG, L298N, DRV8833
from .motor import Motor, DifferentialDrive
from .config import (
    TB6612FNG_CONFIG,
    L298N_CONFIG,
    DRV8833_CONFIG,
    get_config,
)
from .factory import create_robot, create_motor, cleanup_gpio

__all__ = [
    "DriverType",
    "MotorDriver",
    "TB6612FNG",
    "L298N",
    "DRV8833",
    "Motor",
    "DifferentialDrive",
    "TB6612FNG_CONFIG",
    "L298N_CONFIG",
    "DRV8833_CONFIG",
    "get_config",
    "create_robot",
    "create_motor",
    "cleanup_gpio",
]
