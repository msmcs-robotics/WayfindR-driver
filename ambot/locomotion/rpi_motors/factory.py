"""
Factory Functions for Motor Creation

Provides easy-to-use factory functions to create configured motor systems.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple

from .drivers import DriverType, get_driver_class
from .config import get_config, get_gpio_library, DriverConfig, MotorPins
from .motor import Motor, DifferentialDrive

logger = logging.getLogger(__name__)


def create_motor(
    driver_type: DriverType,
    pins: MotorPins,
    gpio=None,
    pwm_frequency: int = 1000,
) -> Motor:
    """
    Create a single motor with the specified driver.

    Args:
        driver_type: Type of motor driver (TB6612FNG, L298N, DRV8833)
        pins: MotorPins configuration
        gpio: GPIO library (auto-detected if not provided)
        pwm_frequency: PWM frequency in Hz

    Returns:
        Configured Motor instance
    """
    if gpio is None:
        gpio, platform = get_gpio_library()
        logger.info(f"Auto-detected platform: {platform}")

    driver_class = get_driver_class(driver_type)
    driver = driver_class(
        gpio=gpio,
        in1=pins.in1,
        in2=pins.in2,
        pwm_pin=pins.pwm,
        pwm_freq=pwm_frequency,
    )

    motor = Motor(driver=driver, offset=pins.offset)
    return motor


def create_robot(
    driver_type: DriverType = DriverType.TB6612FNG,
    config: Optional[DriverConfig] = None,
    left_pins: Optional[Tuple[int, int, int]] = None,
    right_pins: Optional[Tuple[int, int, int]] = None,
    stby_pin: Optional[int] = None,
    setup_gpio: bool = True,
) -> DifferentialDrive:
    """
    Factory function to create a configured differential drive robot.

    Args:
        driver_type: Type of motor driver (default: TB6612FNG)
        config: Full DriverConfig (overrides driver_type default)
        left_pins: Override (in1, in2, pwm) for left motor
        right_pins: Override (in1, in2, pwm) for right motor
        stby_pin: Override standby pin
        setup_gpio: Whether to call GPIO.setmode()

    Returns:
        Configured DifferentialDrive instance

    Example:
        # Using defaults (TB6612FNG)
        robot = create_robot()

        # Specify driver type
        robot = create_robot(driver_type=DriverType.L298N)

        # Custom pins
        robot = create_robot(
            driver_type=DriverType.TB6612FNG,
            left_pins=(13, 15, 33),
            right_pins=(16, 18, 32),
            stby_pin=11,
        )
    """
    # Get GPIO library
    gpio, platform = get_gpio_library()
    logger.info(f"Platform: {platform}")

    # Get base configuration
    if config is None:
        config = get_config(driver_type)

    # Setup GPIO mode
    if setup_gpio:
        if config.pin_mode == "BOARD":
            gpio.setmode(gpio.BOARD)
        else:
            gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)

    # Apply pin overrides
    left_motor_pins = config.left_motor
    right_motor_pins = config.right_motor

    if left_pins:
        left_motor_pins = MotorPins(
            in1=left_pins[0],
            in2=left_pins[1],
            pwm=left_pins[2],
            offset=config.left_motor.offset,
        )

    if right_pins:
        right_motor_pins = MotorPins(
            in1=right_pins[0],
            in2=right_pins[1],
            pwm=right_pins[2],
            offset=config.right_motor.offset,
        )

    stby = stby_pin if stby_pin is not None else config.stby_pin

    # Create motors
    left_motor = create_motor(
        driver_type=driver_type,
        pins=left_motor_pins,
        gpio=gpio,
        pwm_frequency=config.pwm_frequency,
    )

    right_motor = create_motor(
        driver_type=driver_type,
        pins=right_motor_pins,
        gpio=gpio,
        pwm_frequency=config.pwm_frequency,
    )

    # Create differential drive
    robot = DifferentialDrive(
        left_motor=left_motor,
        right_motor=right_motor,
        gpio=gpio,
        stby_pin=stby,
    )

    # Setup all components
    robot.setup()

    logger.info(f"Robot created with {config.name} driver")
    return robot


def cleanup_gpio():
    """Cleanup all GPIO resources."""
    try:
        gpio, _ = get_gpio_library()
        gpio.cleanup()
        logger.info("GPIO cleanup complete")
    except Exception as e:
        logger.warning(f"GPIO cleanup failed: {e}")
