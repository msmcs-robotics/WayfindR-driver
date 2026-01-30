"""
YAHBOOM G1 Motor Driver Module

Motor control for TB6612FNG-based differential drive robots.
Compatible with Jetson Orin Nano and Raspberry Pi.
"""

from .motor import Motor, DifferentialDrive, create_robot
from .config import (
    STBY_PIN,
    LEFT_MOTOR,
    RIGHT_MOTOR,
    PWM_FREQUENCY,
)

__all__ = [
    "Motor",
    "DifferentialDrive",
    "create_robot",
    "STBY_PIN",
    "LEFT_MOTOR",
    "RIGHT_MOTOR",
    "PWM_FREQUENCY",
]

__version__ = "0.1.0"
