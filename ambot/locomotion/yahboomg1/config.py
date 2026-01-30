"""
YAHBOOM G1 Motor Driver Configuration

Pin assignments for TB6612FNG motor driver on Jetson Orin Nano.
Adjust these values based on your wiring.

Pin numbering mode: BOARD (physical pin numbers)
"""

# =============================================================================
# Platform Detection
# =============================================================================
import platform

def get_gpio_library():
    """Auto-detect platform and return appropriate GPIO library."""
    machine = platform.machine()

    if 'aarch64' in machine:
        # Jetson (ARM64)
        try:
            import Jetson.GPIO as GPIO
            return GPIO, "jetson"
        except ImportError:
            pass

    # Fallback to RPi.GPIO
    try:
        import RPi.GPIO as GPIO
        return GPIO, "raspberry_pi"
    except ImportError:
        pass

    raise ImportError("No GPIO library found. Install Jetson.GPIO or RPi.GPIO")


# =============================================================================
# Pin Configuration (BOARD numbering)
# =============================================================================

# Motor Driver Standby (enable/disable)
STBY_PIN = 11

# Left Motor (Motor A)
LEFT_MOTOR = {
    "in1": 13,      # Direction pin 1
    "in2": 15,      # Direction pin 2
    "pwm": 33,      # PWM speed control (most reliable on Orin Nano)
}

# Right Motor (Motor B)
RIGHT_MOTOR = {
    "in1": 16,      # Direction pin 1
    "in2": 18,      # Direction pin 2
    "pwm": 32,      # PWM speed control
}

# =============================================================================
# Motor Parameters
# =============================================================================

# PWM frequency (Hz)
PWM_FREQUENCY = 1000

# Speed limits (0-100)
MIN_SPEED = 0
MAX_SPEED = 100
DEFAULT_SPEED = 50

# Motor direction offsets (1 or -1)
# Change to -1 if motor spins the wrong way
LEFT_OFFSET = 1
RIGHT_OFFSET = 1

# =============================================================================
# Safety
# =============================================================================

# Startup delay (seconds) - wait before enabling motors
STARTUP_DELAY = 0.5

# Auto-disable after no commands (seconds, 0 to disable)
IDLE_TIMEOUT = 0

# =============================================================================
# Alternative Pin Configurations
# =============================================================================

# BCM Pin numbers (for reference, not used by default)
# BCM mode uses GPIO numbers instead of physical pin numbers
BCM_PINS = {
    "stby": 17,
    "left_in1": 27,
    "left_in2": 22,
    "left_pwm": 13,
    "right_in1": 23,
    "right_in2": 24,
    "right_pwm": 12,
}

# Alternative wiring for PCA9685 I2C PWM controller
# (if hardware PWM is insufficient)
PCA9685_CONFIG = {
    "enabled": False,
    "i2c_address": 0x40,
    "pwm_frequency": 1000,
    "left_pwm_channel": 0,
    "right_pwm_channel": 1,
}
