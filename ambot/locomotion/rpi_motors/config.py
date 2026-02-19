"""
Motor Driver Pin Configurations for Raspberry Pi

Defines pin mappings for different motor drivers on Raspberry Pi.

================================================================================
PIN NUMBERING MODES - IMPORTANT!
================================================================================

There are TWO ways to refer to GPIO pins on Raspberry Pi:

1. BOARD (Physical) - The actual pin number printed on the board (1-40)
   - Pin 1 is top-left (3.3V), Pin 2 is top-right (5V)
   - This is what you count when looking at the physical header
   - RECOMMENDED for beginners - matches what you see

2. BCM (Broadcom) - The GPIO number used by the chip (GPIO0-GPIO27)
   - These are the "GPIO17", "GPIO27" etc. numbers
   - Used in most online tutorials and pinout diagrams
   - More commonly used in software documentation

Example: The same pin can be called:
   - BOARD Pin 11  =  BCM GPIO17  (same physical pin!)
   - BOARD Pin 13  =  BCM GPIO27  (same physical pin!)

PINOUT REFERENCE IMAGE:
   https://pinout.xyz/
   https://www.raspberrypi.com/documentation/computers/raspberry-pi.html

================================================================================
RASPBERRY PI 40-PIN HEADER DIAGRAM
================================================================================

Looking at the Pi with USB ports facing down, GPIO header at top:

                    3.3V [1]  [2]  5V
          (GPIO2)  SDA1 [3]  [4]  5V
          (GPIO3)  SCL1 [5]  [6]  GND
          (GPIO4)       [7]  [8]  TXD (GPIO14)
                    GND [9]  [10] RXD (GPIO15)
  --> STBY (GPIO17)    [11] [12] (GPIO18) PWM0
  --> AIN1 (GPIO27)    [13] [14] GND
  --> AIN2 (GPIO22)    [15] [16] (GPIO23) BIN1 <--
          (GPIO24) BIN2 [17] [18] (GPIO24) BIN2 <--
          (GPIO10) MOSI [19] [20] GND
          (GPIO9)  MISO [21] [22] (GPIO25)
          (GPIO11) SCLK [23] [24] (GPIO8)  CE0
                    GND [25] [26] (GPIO7)  CE1
          (GPIO0)  ID_SD[27] [28] ID_SC (GPIO1)
          (GPIO5)       [29] [30] GND
          (GPIO6)       [31] [32] (GPIO12) PWMB <-- Hardware PWM0
  --> PWMA (GPIO13)    [33] [34] GND
          (GPIO19) PWM1 [35] [36] (GPIO16)
          (GPIO26)      [37] [38] (GPIO20)
                    GND [39] [40] (GPIO21)

Pins marked with --> are used in our default motor driver configurations.

================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Dict, Any
from enum import Enum
import platform


# =============================================================================
# Platform Detection
# =============================================================================

def get_gpio_library():
    """
    Auto-detect platform and return appropriate GPIO library.

    Returns:
        tuple: (GPIO_library, platform_name)

    Raises:
        ImportError: If no GPIO library is found
    """
    machine = platform.machine()

    if 'aarch64' in machine:
        # Jetson (ARM64) - try Jetson.GPIO first
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
# Configuration Data Classes
# =============================================================================

@dataclass
class MotorPins:
    """
    Pin configuration for a single motor.

    Attributes:
        in1: Direction control pin 1 (HIGH/LOW determines direction)
        in2: Direction control pin 2 (HIGH/LOW determines direction)
        pwm: PWM speed control pin (duty cycle 0-100%)
        offset: Direction correction (1 or -1) if motor wired backwards
    """
    in1: int          # Direction control pin 1
    in2: int          # Direction control pin 2
    pwm: int          # PWM speed control pin
    offset: int = 1   # Direction correction (1 or -1)


@dataclass
class DriverConfig:
    """
    Complete configuration for a motor driver setup.

    Attributes:
        name: Human-readable name for this configuration
        left_motor: Pin configuration for left/Motor A
        right_motor: Pin configuration for right/Motor B
        stby_pin: Standby pin - must be HIGH to enable (TB6612FNG)
        enable_a: Enable pin for motor A (L298N - same as PWM)
        enable_b: Enable pin for motor B (L298N - same as PWM)
        sleep_pin: Sleep pin - must be HIGH to enable (DRV8833)
        pwm_frequency: PWM frequency in Hz (1000 is good for DC motors)
        pin_mode: "BOARD" for physical pins, "BCM" for GPIO numbers
    """
    name: str
    left_motor: MotorPins
    right_motor: MotorPins
    stby_pin: Optional[int] = None
    enable_a: Optional[int] = None
    enable_b: Optional[int] = None
    sleep_pin: Optional[int] = None
    pwm_frequency: int = 1000
    pin_mode: str = "BOARD"


# =============================================================================
# TB6612FNG Configuration (SparkFun Board)
# =============================================================================
#
# TB6612FNG is a dual H-bridge MOSFET driver. Very efficient (~95%).
# Datasheet: https://www.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf
# SparkFun Product: https://www.sparkfun.com/products/14451
#
# WIRING TABLE (BOARD pin mode - physical pin numbers):
# ┌─────────────────┬───────────────────┬──────────────┬─────────────────────┐
# │ TB6612FNG Pin   │ RPi Physical Pin  │ BCM GPIO     │ Function            │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ VCC             │ Pin 1             │ 3.3V         │ Logic power         │
# │ GND             │ Pin 6             │ GND          │ Common ground       │
# │ STBY            │ Pin 11            │ GPIO17       │ Standby (HIGH=on)   │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ AIN1            │ Pin 13            │ GPIO27       │ Motor A direction 1 │
# │ AIN2            │ Pin 15            │ GPIO22       │ Motor A direction 2 │
# │ PWMA            │ Pin 33            │ GPIO13       │ Motor A speed (PWM) │
# │ AO1             │ Motor A wire 1    │ -            │ Motor A output      │
# │ AO2             │ Motor A wire 2    │ -            │ Motor A output      │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ BIN1            │ Pin 16            │ GPIO23       │ Motor B direction 1 │
# │ BIN2            │ Pin 18            │ GPIO24       │ Motor B direction 2 │
# │ PWMB            │ Pin 32            │ GPIO12       │ Motor B speed (PWM) │
# │ BO1             │ Motor B wire 1    │ -            │ Motor B output      │
# │ BO2             │ Motor B wire 2    │ -            │ Motor B output      │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ VM              │ External Battery+ │ -            │ Motor power (6-12V) │
# │ GND (VM side)   │ External Battery- │ -            │ Motor power ground  │
# └─────────────────┴───────────────────┴──────────────┴─────────────────────┘
#
# IMPORTANT: VM GND and RPi GND must be connected (common ground)!
#
# Control Truth Table:
#   IN1=H, IN2=L, PWM=H  →  Forward
#   IN1=L, IN2=H, PWM=H  →  Reverse
#   IN1=H, IN2=H, PWM=X  →  Brake (short)
#   IN1=L, IN2=L, PWM=X  →  Coast (stop)
#   STBY=L               →  Standby (disabled)
#

TB6612FNG_CONFIG = DriverConfig(
    name="TB6612FNG (SparkFun)",
    left_motor=MotorPins(
        in1=13,    # Physical Pin 13 → TB6612 AIN1 (GPIO27)
        in2=15,    # Physical Pin 15 → TB6612 AIN2 (GPIO22)
        pwm=33,    # Physical Pin 33 → TB6612 PWMA (GPIO13) - Hardware PWM1
        offset=1,
    ),
    right_motor=MotorPins(
        in1=16,    # Physical Pin 16 → TB6612 BIN1 (GPIO23)
        in2=18,    # Physical Pin 18 → TB6612 BIN2 (GPIO24)
        pwm=32,    # Physical Pin 32 → TB6612 PWMB (GPIO12) - Hardware PWM0
        offset=1,
    ),
    stby_pin=11,   # Physical Pin 11 → TB6612 STBY (GPIO17) - Must be HIGH!
    pwm_frequency=1000,
    pin_mode="BOARD",
)

# Alternative TB6612FNG config using BCM numbering (GPIO numbers)
TB6612FNG_BCM_CONFIG = DriverConfig(
    name="TB6612FNG (BCM)",
    left_motor=MotorPins(
        in1=27,    # GPIO27 (Physical Pin 13) → TB6612 AIN1
        in2=22,    # GPIO22 (Physical Pin 15) → TB6612 AIN2
        pwm=13,    # GPIO13 (Physical Pin 33) → TB6612 PWMA - Hardware PWM1
        offset=1,
    ),
    right_motor=MotorPins(
        in1=23,    # GPIO23 (Physical Pin 16) → TB6612 BIN1
        in2=24,    # GPIO24 (Physical Pin 18) → TB6612 BIN2
        pwm=12,    # GPIO12 (Physical Pin 32) → TB6612 PWMB - Hardware PWM0
        offset=1,
    ),
    stby_pin=17,   # GPIO17 (Physical Pin 11) → TB6612 STBY
    pwm_frequency=1000,
    pin_mode="BCM",
)


# =============================================================================
# L298N Configuration
# =============================================================================
#
# L298N is a dual H-bridge BJT driver. Less efficient (~75%) but robust.
# Common red PCB module with heatsink.
# Tutorial: https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/
#
# WIRING TABLE (BOARD pin mode - physical pin numbers):
# ┌─────────────────┬───────────────────┬──────────────┬─────────────────────┐
# │ L298N Pin       │ RPi Physical Pin  │ BCM GPIO     │ Function            │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ GND             │ Pin 6             │ GND          │ Common ground       │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ IN1             │ Pin 13            │ GPIO27       │ Motor A direction 1 │
# │ IN2             │ Pin 15            │ GPIO22       │ Motor A direction 2 │
# │ ENA             │ Pin 33            │ GPIO13       │ Motor A enable/PWM  │
# │ OUT1            │ Motor A wire 1    │ -            │ Motor A output      │
# │ OUT2            │ Motor A wire 2    │ -            │ Motor A output      │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ IN3             │ Pin 16            │ GPIO23       │ Motor B direction 1 │
# │ IN4             │ Pin 18            │ GPIO24       │ Motor B direction 2 │
# │ ENB             │ Pin 32            │ GPIO12       │ Motor B enable/PWM  │
# │ OUT3            │ Motor B wire 1    │ -            │ Motor B output      │
# │ OUT4            │ Motor B wire 2    │ -            │ Motor B output      │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ +12V (VS)       │ External Battery+ │ -            │ Motor power (6-35V) │
# │ GND             │ External Battery- │ -            │ Motor power ground  │
# │ +5V (output)    │ (Leave floating)  │ -            │ Regulator output*   │
# └─────────────────┴───────────────────┴──────────────┴─────────────────────┘
#
# * The 5V pin is an OUTPUT when the jumper is installed and VS > 7V.
#   With 4xAA batteries (~6V), REMOVE the 5V jumper!
#
# IMPORTANT NOTES:
# 1. L298N has ~2V voltage drop. With 6V input, motors get ~4V.
# 2. Remove ENA/ENB jumpers to use PWM speed control.
# 3. L298N GND and RPi GND must be connected (common ground)!
#
# Control Truth Table:
#   IN1=H, IN2=L, ENA=PWM  →  Forward at PWM speed
#   IN1=L, IN2=H, ENA=PWM  →  Reverse at PWM speed
#   IN1=X, IN2=X, ENA=L    →  Stop (coast)
#

L298N_CONFIG = DriverConfig(
    name="L298N",
    left_motor=MotorPins(
        in1=13,    # Physical Pin 13 → L298N IN1 (GPIO27)
        in2=15,    # Physical Pin 15 → L298N IN2 (GPIO22)
        pwm=33,    # Physical Pin 33 → L298N ENA (GPIO13) - Remove jumper!
        offset=-1, # Left motor wired in reverse — negate direction
    ),
    right_motor=MotorPins(
        in1=16,    # Physical Pin 16 → L298N IN3 (GPIO23)
        in2=18,    # Physical Pin 18 → L298N IN4 (GPIO24)
        pwm=32,    # Physical Pin 32 → L298N ENB (GPIO12) - Remove jumper!
        offset=1,
    ),
    # L298N uses ENA/ENB for PWM enable, no STBY pin
    stby_pin=None,
    pwm_frequency=1000,
    pin_mode="BOARD",
)

# Alternative L298N config using BCM numbering
L298N_BCM_CONFIG = DriverConfig(
    name="L298N (BCM)",
    left_motor=MotorPins(
        in1=27,    # GPIO27 (Physical Pin 13) → L298N IN1
        in2=22,    # GPIO22 (Physical Pin 15) → L298N IN2
        pwm=13,    # GPIO13 (Physical Pin 33) → L298N ENA
        offset=1,
    ),
    right_motor=MotorPins(
        in1=23,    # GPIO23 (Physical Pin 16) → L298N IN3
        in2=24,    # GPIO24 (Physical Pin 18) → L298N IN4
        pwm=12,    # GPIO12 (Physical Pin 32) → L298N ENB
        offset=1,
    ),
    stby_pin=None,
    pwm_frequency=1000,
    pin_mode="BCM",
)


# =============================================================================
# DRV8833 Configuration
# =============================================================================
#
# DRV8833 is a dual H-bridge MOSFET driver. Efficient but LOW VOLTAGE ONLY!
# MAX INPUT: 10.8V - Do NOT use with 12V batteries!
# Datasheet: https://www.ti.com/product/DRV8833
#
# KEY DIFFERENCE: DRV8833 applies PWM directly to the IN pins!
# There is no separate PWM pin. Speed control is done via PWM on IN1/IN2.
#
# WIRING TABLE (BOARD pin mode - physical pin numbers):
# ┌─────────────────┬───────────────────┬──────────────┬─────────────────────┐
# │ DRV8833 Pin     │ RPi Physical Pin  │ BCM GPIO     │ Function            │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ VCC             │ (tied to VM)      │ -            │ Logic = Motor power │
# │ GND             │ Pin 6             │ GND          │ Common ground       │
# │ SLP (Sleep)     │ Pin 11            │ GPIO17       │ Sleep (HIGH=awake)  │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ AIN1            │ Pin 13            │ GPIO27       │ Motor A PWM/dir 1   │
# │ AIN2            │ Pin 15            │ GPIO22       │ Motor A PWM/dir 2   │
# │ AOUT1           │ Motor A wire 1    │ -            │ Motor A output      │
# │ AOUT2           │ Motor A wire 2    │ -            │ Motor A output      │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ BIN1            │ Pin 16            │ GPIO23       │ Motor B PWM/dir 1   │
# │ BIN2            │ Pin 18            │ GPIO24       │ Motor B PWM/dir 2   │
# │ BOUT1           │ Motor B wire 1    │ -            │ Motor B output      │
# │ BOUT2           │ Motor B wire 2    │ -            │ Motor B output      │
# ├─────────────────┼───────────────────┼──────────────┼─────────────────────┤
# │ VM              │ External Battery+ │ -            │ Motor power (MAX 10.8V!) │
# │ GND             │ External Battery- │ -            │ Motor power ground  │
# └─────────────────┴───────────────────┴──────────────┴─────────────────────┘
#
# Control Truth Table (PWM applied to IN pins directly):
#   AIN1=PWM, AIN2=L    →  Forward at PWM speed
#   AIN1=L,   AIN2=PWM  →  Reverse at PWM speed
#   AIN1=H,   AIN2=H    →  Brake (slow decay)
#   AIN1=L,   AIN2=L    →  Coast (fast decay)
#

DRV8833_CONFIG = DriverConfig(
    name="DRV8833",
    left_motor=MotorPins(
        in1=13,    # Physical Pin 13 → DRV8833 AIN1 (GPIO27) - PWM for forward
        in2=15,    # Physical Pin 15 → DRV8833 AIN2 (GPIO22) - PWM for reverse
        pwm=0,     # NOT USED - DRV8833 uses IN pins for PWM!
        offset=1,
    ),
    right_motor=MotorPins(
        in1=16,    # Physical Pin 16 → DRV8833 BIN1 (GPIO23)
        in2=18,    # Physical Pin 18 → DRV8833 BIN2 (GPIO24)
        pwm=0,     # NOT USED - DRV8833 uses IN pins for PWM!
        offset=1,
    ),
    sleep_pin=11,  # Physical Pin 11 → DRV8833 SLP (GPIO17) - Must be HIGH!
    pwm_frequency=1000,
    pin_mode="BOARD",
)


# =============================================================================
# Configuration Registry
# =============================================================================

from .drivers import DriverType

DRIVER_CONFIGS: Dict[DriverType, DriverConfig] = {
    DriverType.TB6612FNG: TB6612FNG_CONFIG,
    DriverType.L298N: L298N_CONFIG,
    DriverType.DRV8833: DRV8833_CONFIG,
}


def get_config(driver_type: DriverType) -> DriverConfig:
    """Get the default configuration for a driver type."""
    return DRIVER_CONFIGS[driver_type]


def create_custom_config(
    driver_type: DriverType,
    left_pins: tuple = None,
    right_pins: tuple = None,
    stby_pin: int = None,
    pwm_frequency: int = None,
    pin_mode: str = None,
) -> DriverConfig:
    """
    Create a custom configuration based on a driver type.

    Args:
        driver_type: Base driver type
        left_pins: (in1, in2, pwm) for left motor
        right_pins: (in1, in2, pwm) for right motor
        stby_pin: Override standby pin
        pwm_frequency: Override PWM frequency
        pin_mode: "BOARD" or "BCM"

    Returns:
        Customized DriverConfig
    """
    base = get_config(driver_type)

    return DriverConfig(
        name=f"{base.name} (Custom)",
        left_motor=MotorPins(
            in1=left_pins[0] if left_pins else base.left_motor.in1,
            in2=left_pins[1] if left_pins else base.left_motor.in2,
            pwm=left_pins[2] if left_pins else base.left_motor.pwm,
            offset=base.left_motor.offset,
        ),
        right_motor=MotorPins(
            in1=right_pins[0] if right_pins else base.right_motor.in1,
            in2=right_pins[1] if right_pins else base.right_motor.in2,
            pwm=right_pins[2] if right_pins else base.right_motor.pwm,
            offset=base.right_motor.offset,
        ),
        stby_pin=stby_pin if stby_pin is not None else base.stby_pin,
        pwm_frequency=pwm_frequency if pwm_frequency else base.pwm_frequency,
        pin_mode=pin_mode if pin_mode else base.pin_mode,
    )


# =============================================================================
# Pinout Reference
# =============================================================================
#
# QUICK REFERENCE - BOARD to BCM Mapping for Motor Control Pins:
#
# ┌──────────────┬──────────┬─────────────────────────────────────┐
# │ Physical Pin │ BCM GPIO │ Our Usage                           │
# ├──────────────┼──────────┼─────────────────────────────────────┤
# │ Pin 6        │ GND      │ Common ground (REQUIRED)            │
# │ Pin 11       │ GPIO17   │ STBY (TB6612) / SLP (DRV8833)       │
# │ Pin 13       │ GPIO27   │ Left motor IN1 / AIN1               │
# │ Pin 15       │ GPIO22   │ Left motor IN2 / AIN2               │
# │ Pin 16       │ GPIO23   │ Right motor IN1 / BIN1              │
# │ Pin 18       │ GPIO24   │ Right motor IN2 / BIN2              │
# │ Pin 32       │ GPIO12   │ Right motor PWM (Hardware PWM0)     │
# │ Pin 33       │ GPIO13   │ Left motor PWM (Hardware PWM1)      │
# └──────────────┴──────────┴─────────────────────────────────────┘
#
# Hardware PWM Pins (smoother motor control):
#   - GPIO12 (Pin 32) - PWM0 channel
#   - GPIO13 (Pin 33) - PWM1 channel
#   - GPIO18 (Pin 12) - PWM0 channel (alternative)
#   - GPIO19 (Pin 35) - PWM1 channel (alternative)
#
# PINOUT REFERENCE LINKS:
#   https://pinout.xyz/                                    (Interactive)
#   https://www.raspberrypi.com/documentation/computers/   (Official)
#   https://pinout.xyz/pinout/ground                       (Ground pins)
#

PINOUT_REFERENCE = """
================================================================================
RASPBERRY PI 40-PIN HEADER - COMPLETE REFERENCE
================================================================================

     3V3 Power [1]  [2]  5V Power
  (SDA1) GPIO2 [3]  [4]  5V Power
  (SCL1) GPIO3 [5]  [6]  Ground        <-- COMMON GROUND
         GPIO4 [7]  [8]  GPIO14 (TXD)
        Ground [9]  [10] GPIO15 (RXD)
 STBY→ GPIO17 [11] [12] GPIO18 (PWM0)
 AIN1→ GPIO27 [13] [14] Ground
 AIN2→ GPIO22 [15] [16] GPIO23 ←BIN1
        3V3   [17] [18] GPIO24 ←BIN2
 (MOSI) GPIO10[19] [20] Ground
 (MISO) GPIO9 [21] [22] GPIO25
 (SCLK) GPIO11[23] [24] GPIO8 (CE0)
        Ground[25] [26] GPIO7 (CE1)
 (ID_SD) GPIO0[27] [28] GPIO1 (ID_SC)
         GPIO5[29] [30] Ground
         GPIO6[31] [32] GPIO12 ←PWMB (Hardware PWM0)
 PWMA→ GPIO13 [33] [34] Ground
 (PWM1) GPIO19[35] [36] GPIO16
        GPIO26[37] [38] GPIO20
        Ground[39] [40] GPIO21

Arrows (→/←) indicate pins used in default motor driver configurations.

================================================================================
"""
