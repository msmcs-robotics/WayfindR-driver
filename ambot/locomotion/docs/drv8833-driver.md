# DRV8833 Motor Driver Wiring Guide

Complete pinout and wiring guide for connecting a DRV8833 dual H-bridge motor driver to a Raspberry Pi.

## Overview

The DRV8833 is a compact, efficient dual H-bridge motor driver from Texas Instruments. It is designed for low-voltage applications and features built-in protection circuits.

**Important Voltage Limitation**: The DRV8833 has a maximum voltage of **10.8V**. This makes it ideal for 6V-9V battery packs but **NOT suitable for 12V motors**.

## Specifications

| Parameter | Value |
|-----------|-------|
| Motor Voltage (VM) | 2.7V - 10.8V |
| Logic Voltage | 2V - 7V (3.3V compatible) |
| Continuous Current | 1.2A per channel (1.5A with heatsink) |
| Peak Current | 2A per channel |
| Voltage Drop | ~0.18V (excellent efficiency) |
| PWM Frequency | Up to 50kHz |
| Channels | 2 (dual H-bridge) |

## Key Difference from TB6612FNG

**The DRV8833 does NOT have separate PWM pins.**

| Driver | PWM Method | Pins per Motor |
|--------|------------|----------------|
| TB6612FNG | Separate PWM pin + direction pins | 3 pins (PWM, IN1, IN2) |
| DRV8833 | PWM applied directly to IN pins | 2 pins (IN1, IN2) |

With the DRV8833:
- **Forward**: Apply PWM to IN1, hold IN2 LOW
- **Reverse**: Hold IN1 LOW, apply PWM to IN2
- **Brake**: Both IN1 and IN2 HIGH (100% duty cycle)
- **Coast/Stop**: Both IN1 and IN2 LOW (0% duty cycle)

This means you need to set up **software PWM on both direction pins** for each motor.

---

## Physical Wiring Diagram (Text-Based)

```
                    RASPBERRY PI 40-PIN HEADER
    ┌─────────────────────────────────────────────────────┐
    │  (Pin 1) 3.3V  ●────────────────┐                   │
    │  (Pin 2) 5V    ○                │                   │
    │  (Pin 3) SDA   ○                │                   │
    │  (Pin 4) 5V    ○                │                   │
    │  (Pin 5) SCL   ○                │                   │
    │  (Pin 6) GND   ●────────────────│───┐               │
    │  ...                            │   │               │
    │  (Pin 11) GPIO17 ●──────────────│───│───┐           │
    │  (Pin 12) GPIO18 ○              │   │   │           │
    │  (Pin 13) GPIO27 ●──────────────│───│───│───┐       │
    │  ...                            │   │   │   │       │
    │  (Pin 15) GPIO22 ●──────────────│───│───│───│───┐   │
    │  (Pin 16) GPIO23 ●──────────────│───│───│───│───│───┼───┐
    │  ...                            │   │   │   │   │   │   │
    │  (Pin 18) GPIO24 ●──────────────│───│───│───│───│───│───│───┐
    │  ...                            │   │   │   │   │   │   │   │
    └─────────────────────────────────│───│───│───│───│───│───│───│─┘
                                      │   │   │   │   │   │   │   │
                                      │   │   │   │   │   │   │   │
                    DRV8833 MODULE    │   │   │   │   │   │   │   │
    ┌─────────────────────────────────│───│───│───│───│───│───│───│─┐
    │                                 │   │   │   │   │   │   │   │ │
    │  VCC  ●─────────────────────────┘   │   │   │   │   │   │   │ │
    │  GND  ●─────────────────────────────┘   │   │   │   │   │   │ │
    │  SLP  ●─────────────────────────────────┘   │   │   │   │   │ │
    │                                             │   │   │   │   │ │
    │  AIN1 ●─────────────────────────────────────┘   │   │   │   │ │
    │  AIN2 ●─────────────────────────────────────────┘   │   │   │ │
    │  BIN1 ●─────────────────────────────────────────────┘   │   │ │
    │  BIN2 ●─────────────────────────────────────────────────┘   │ │
    │                                                             │ │
    │  AOUT1 ●────────────────────┐                               │ │
    │  AOUT2 ●────────────────────│──┐                            │ │
    │  BOUT1 ●────────────────────│──│──┐                         │ │
    │  BOUT2 ●────────────────────│──│──│──┐                      │ │
    │                             │  │  │  │                      │ │
    │  VM    ●────────────────────│──│──│──│──┐ (External 6-9V)   │ │
    │  GND   ●────────────────────│──│──│──│──│──┐                │ │
    │                             │  │  │  │  │  │                │ │
    └─────────────────────────────│──│──│──│──│──│────────────────┘ │
                                  │  │  │  │  │  │                  │
                                  │  │  │  │  │  │                  │
                   MOTOR A        │  │  │  │  │  │                  │
                 (LEFT MOTOR)     │  │  │  │  │  │                  │
    ┌──────────────────────┐      │  │  │  │  │  │                  │
    │  ●─────────────────────────┘  │  │  │  │  │                  │
    │  M                            │  │  │  │  │                  │
    │  ●────────────────────────────┘  │  │  │  │                  │
    └──────────────────────┘           │  │  │  │                  │
                                       │  │  │  │                  │
                   MOTOR B             │  │  │  │                  │
                 (RIGHT MOTOR)         │  │  │  │                  │
    ┌──────────────────────┐           │  │  │  │                  │
    │  ●───────────────────────────────┘  │  │  │                  │
    │  M                                  │  │  │                  │
    │  ●──────────────────────────────────┘  │  │                  │
    └──────────────────────┘                 │  │                  │
                                             │  │                  │
                   BATTERY PACK              │  │                  │
                   (6V - 9V)                 │  │                  │
    ┌──────────────────────┐                 │  │                  │
    │  (+) ──────────────────────────────────┘  │                  │
    │  6-9V                                     │                  │
    │  (-) ─────────────────────────────────────┴──────────────────┘
    └──────────────────────┘                      (Common Ground)
```

---

## Pin-by-Pin Connection Table

### Control Pins (Raspberry Pi to DRV8833)

| DRV8833 Pin | Function | Raspberry Pi Physical Pin | BCM GPIO | Wire Color (Suggested) |
|-------------|----------|---------------------------|----------|------------------------|
| VCC | Logic power (3.3V) | Pin 1 | 3.3V | Red |
| GND | Ground | Pin 6 | GND | Black |
| SLP (Sleep) | Sleep/Enable | Pin 11 | GPIO17 | Yellow |
| AIN1 | Motor A direction 1 (PWM) | Pin 13 | GPIO27 | Orange |
| AIN2 | Motor A direction 2 (PWM) | Pin 15 | GPIO22 | Orange |
| BIN1 | Motor B direction 1 (PWM) | Pin 16 | GPIO23 | Blue |
| BIN2 | Motor B direction 2 (PWM) | Pin 18 | GPIO24 | Blue |

### Motor Output Pins

| DRV8833 Pin | Function | Connect To |
|-------------|----------|------------|
| AOUT1 | Motor A output + | Left motor terminal 1 |
| AOUT2 | Motor A output - | Left motor terminal 2 |
| BOUT1 | Motor B output + | Right motor terminal 1 |
| BOUT2 | Motor B output - | Right motor terminal 2 |

### Power Pins

| DRV8833 Pin | Function | Connect To |
|-------------|----------|------------|
| VM | Motor voltage (2.7-10.8V) | Battery pack positive (+) |
| GND | Motor ground | Battery pack negative (-) AND Raspberry Pi GND |

---

## Power Supply Requirements

### Voltage Limits

| Parameter | Minimum | Recommended | Maximum |
|-----------|---------|-------------|---------|
| Motor Voltage (VM) | 2.7V | 6V - 9V | **10.8V** |
| Logic Voltage (VCC) | 2.0V | 3.3V | 7.0V |

### Recommended Battery Options

| Battery Type | Nominal Voltage | Charged Voltage | Status |
|--------------|-----------------|-----------------|--------|
| 4x AA NiMH | 4.8V | 5.6V | Good |
| 5x AA NiMH | 6.0V | 7.0V | Excellent |
| 6x AA NiMH | 7.2V | 8.4V | Excellent |
| 2S LiPo | 7.4V | 8.4V | Excellent |
| 6x AA Alkaline | 9.0V | 9.6V | Good (at limit) |

**WARNING**: Do NOT use:
- 3S LiPo (11.1V nominal, 12.6V charged) - **EXCEEDS MAX VOLTAGE**
- 12V lead-acid batteries - **EXCEEDS MAX VOLTAGE**
- 8x AA cells - **EXCEEDS MAX VOLTAGE**

### Current Considerations

| Scenario | Current Draw | DRV8833 Rating | Notes |
|----------|--------------|----------------|-------|
| Light load | 200-500mA | 1.2A | Safe |
| Normal operation | 500-800mA | 1.2A | Safe |
| Heavy load | 800-1200mA | 1.2A | At limit |
| Stall condition | 1.5-2A | 2A peak | Brief only |

The DRV8833 has built-in thermal shutdown and overcurrent protection.

---

## Control Truth Table

| AIN1/BIN1 | AIN2/BIN2 | SLP | Motor State |
|-----------|-----------|-----|-------------|
| PWM | LOW | HIGH | Forward (speed = duty cycle) |
| LOW | PWM | HIGH | Reverse (speed = duty cycle) |
| HIGH (100%) | HIGH (100%) | HIGH | Brake (slow decay) |
| LOW (0%) | LOW (0%) | HIGH | Coast (fast decay) |
| X | X | LOW | Sleep mode (outputs Hi-Z) |

---

## Key Differences from TB6612FNG

| Feature | TB6612FNG | DRV8833 |
|---------|-----------|---------|
| **PWM Method** | Separate PWM pin | PWM on direction pins |
| **Pins per motor** | 3 (PWM, IN1, IN2) | 2 (IN1, IN2) |
| **Maximum voltage** | 13.5V | **10.8V** |
| **Enable method** | STBY pin (active HIGH) | SLP pin (active HIGH) |
| **Peak current** | 3.2A | 2A |
| **Brake mode** | Both IN HIGH, PWM LOW | Both IN HIGH (100% duty) |
| **Built-in protection** | Limited | Thermal + overcurrent |

### Code Implications

**TB6612FNG** - Set direction with digital pins, control speed with PWM:
```python
# Forward at 50% speed
GPIO.output(IN1, HIGH)
GPIO.output(IN2, LOW)
pwm.ChangeDutyCycle(50)  # Separate PWM pin
```

**DRV8833** - PWM is applied directly to direction pins:
```python
# Forward at 50% speed
pwm_in1.ChangeDutyCycle(50)  # PWM on IN1
pwm_in2.ChangeDutyCycle(0)   # IN2 held LOW
```

---

## Step-by-Step Connection Instructions

### Materials Needed

- DRV8833 motor driver module
- Raspberry Pi (any model with 40-pin header)
- 2x DC motors
- Battery pack (6V-9V)
- Jumper wires (female-to-female or female-to-male depending on module)
- (Optional) Breadboard

### Step 1: Prepare the Raspberry Pi

1. **Power off** the Raspberry Pi completely
2. **Disconnect** any power sources
3. Identify the GPIO header pins using the pinout diagram

### Step 2: Connect Power Rails

1. Connect Raspberry Pi **Pin 1 (3.3V)** to DRV8833 **VCC**
2. Connect Raspberry Pi **Pin 6 (GND)** to DRV8833 **GND** (logic ground)
3. Connect Battery pack **negative (-)** to DRV8833 **GND** (motor ground)
4. Connect Battery pack **positive (+)** to DRV8833 **VM**

**Critical**: The Raspberry Pi GND and battery GND must be connected together (common ground).

### Step 3: Connect the Sleep/Enable Pin

1. Connect Raspberry Pi **Pin 11 (GPIO17)** to DRV8833 **SLP**
2. This pin must be pulled HIGH to enable the driver
3. When LOW, the driver enters sleep mode and motor outputs are disabled

### Step 4: Connect Motor A (Left Motor) Control Pins

1. Connect Raspberry Pi **Pin 13 (GPIO27)** to DRV8833 **AIN1**
2. Connect Raspberry Pi **Pin 15 (GPIO22)** to DRV8833 **AIN2**

### Step 5: Connect Motor B (Right Motor) Control Pins

1. Connect Raspberry Pi **Pin 16 (GPIO23)** to DRV8833 **BIN1**
2. Connect Raspberry Pi **Pin 18 (GPIO24)** to DRV8833 **BIN2**

### Step 6: Connect the Motors

1. Connect **Left Motor** to DRV8833 **AOUT1** and **AOUT2**
2. Connect **Right Motor** to DRV8833 **BOUT1** and **BOUT2**

Note: If a motor spins in the wrong direction, swap its two output wires.

### Step 7: Verify Connections

Before powering on, double-check:

- [ ] VCC connected to 3.3V (NOT 5V)
- [ ] All GND connections are made (common ground)
- [ ] Battery voltage is between 6V and 10.8V
- [ ] No short circuits between adjacent pins
- [ ] Motor wires are secure

### Step 8: Power On and Test

1. Connect the battery pack
2. Power on the Raspberry Pi
3. Run the test script (see Python Usage below)

---

## Wiring Summary Diagram

```
RASPBERRY PI                          DRV8833                    EXTERNAL
─────────────                         ───────                    ────────

[Pin 1 ] 3.3V  ──────────────────────> VCC
                                                                 BATTERY
[Pin 6 ] GND   ──────────────────────> GND <───────────────────> GND (-)
                                       VM  <───────────────────> (+) 6-9V
[Pin 11] GPIO17 ─────────────────────> SLP (Sleep/Enable)

[Pin 13] GPIO27 ─────────────────────> AIN1 ─┐                   LEFT
[Pin 15] GPIO22 ─────────────────────> AIN2 ─┼─> AOUT1/AOUT2 ──> MOTOR

[Pin 16] GPIO23 ─────────────────────> BIN1 ─┐                   RIGHT
[Pin 18] GPIO24 ─────────────────────> BIN2 ─┼─> BOUT1/BOUT2 ──> MOTOR
```

---

## Python Usage

### Using the Project's Motor Library

```python
from locomotion.rpi_motors import create_robot, DriverType

# Create robot with DRV8833 driver
robot = create_robot(driver_type=DriverType.DRV8833)

# Enable the driver
robot.enable()

# Control the robot
robot.forward(50)   # 50% speed forward
robot.turn_left(30) # Turn left at 30% speed
robot.stop()

# Cleanup
robot.cleanup()
```

### Direct GPIO Control Example

```python
import RPi.GPIO as GPIO
import time

# Pin definitions (BOARD numbering)
SLP_PIN = 11   # Sleep/Enable
AIN1 = 13      # Motor A direction 1
AIN2 = 15      # Motor A direction 2
BIN1 = 16      # Motor B direction 1
BIN2 = 18      # Motor B direction 2

# Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SLP_PIN, GPIO.OUT, initial=GPIO.HIGH)  # Enable driver
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

# Create PWM on all direction pins
pwm_ain1 = GPIO.PWM(AIN1, 1000)  # 1kHz frequency
pwm_ain2 = GPIO.PWM(AIN2, 1000)
pwm_bin1 = GPIO.PWM(BIN1, 1000)
pwm_bin2 = GPIO.PWM(BIN2, 1000)

# Start all PWM at 0%
pwm_ain1.start(0)
pwm_ain2.start(0)
pwm_bin1.start(0)
pwm_bin2.start(0)

def motor_a_forward(speed):
    """Motor A forward at given speed (0-100)"""
    pwm_ain1.ChangeDutyCycle(speed)
    pwm_ain2.ChangeDutyCycle(0)

def motor_a_reverse(speed):
    """Motor A reverse at given speed (0-100)"""
    pwm_ain1.ChangeDutyCycle(0)
    pwm_ain2.ChangeDutyCycle(speed)

def motor_a_stop():
    """Motor A coast stop"""
    pwm_ain1.ChangeDutyCycle(0)
    pwm_ain2.ChangeDutyCycle(0)

def motor_a_brake():
    """Motor A brake (slow decay)"""
    pwm_ain1.ChangeDutyCycle(100)
    pwm_ain2.ChangeDutyCycle(100)

# Test: Run Motor A forward for 2 seconds
try:
    print("Motor A forward at 50%")
    motor_a_forward(50)
    time.sleep(2)

    print("Motor A reverse at 50%")
    motor_a_reverse(50)
    time.sleep(2)

    print("Stop")
    motor_a_stop()

finally:
    # Cleanup
    pwm_ain1.stop()
    pwm_ain2.stop()
    pwm_bin1.stop()
    pwm_bin2.stop()
    GPIO.cleanup()
```

---

## Troubleshooting

### Motor doesn't run

1. **Check SLP pin** - Must be HIGH to enable driver
2. **Verify voltage** - Is battery connected? Is VM between 2.7V and 10.8V?
3. **Check ground** - Pi GND and battery GND must be connected
4. **Test PWM** - Ensure PWM is actually being output

### Motor runs in wrong direction

1. Swap the motor wires (AOUT1 <-> AOUT2 or BOUT1 <-> BOUT2)
2. Or change the `offset` value in software config to `-1`

### Motor stutters or is weak

1. **Low battery** - Check voltage with multimeter
2. **Loose connections** - Verify all wires are secure
3. **PWM frequency** - Try adjusting between 500Hz and 2000Hz

### Driver gets hot

1. **Current too high** - Motors may be overloaded or stalled
2. **Voltage too high** - Ensure VM is under 10.8V
3. Add a small heatsink to the DRV8833 chip

### Raspberry Pi resets when motors run

1. **Insufficient power supply** - Use a 3A+ power supply for Pi
2. **No common ground** - Verify GND connections
3. **Add capacitor** - Place a 100-470uF capacitor across VM and GND

---

## Common DRV8833 Modules

### Pololu DRV8833 Carrier

- Compact breakout board
- Pin labels: VIN, GND, OUT1, OUT2, IN1, IN2, SLP, FLT
- Includes 0.1uF ceramic capacitors
- [Pololu Product Page](https://www.pololu.com/product/2130)

### SparkFun Motor Driver - Dual TB6612FNG

Note: This is NOT a DRV8833 - different chip, different pinout!

### Generic DRV8833 Modules

- Often have additional pins: EEP (fault output), ULT (fault output)
- May have built-in voltage regulator
- Verify pinout matches your specific module

---

## References

- [Texas Instruments DRV8833 Datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
- [Pololu DRV8833 Hookup Guide](https://www.pololu.com/product/2130/resources)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)

---

## Quick Reference Card

```
DRV8833 Quick Reference for Raspberry Pi
========================================

VOLTAGE: 6V - 9V (MAX 10.8V!)
CURRENT: 1.2A continuous, 2A peak

CONNECTIONS (BOARD numbering):
  Pi Pin 1  (3.3V)   -> VCC
  Pi Pin 6  (GND)    -> GND
  Pi Pin 11 (GPIO17) -> SLP (HIGH = enabled)
  Pi Pin 13 (GPIO27) -> AIN1 (PWM for left motor)
  Pi Pin 15 (GPIO22) -> AIN2 (PWM for left motor)
  Pi Pin 16 (GPIO23) -> BIN1 (PWM for right motor)
  Pi Pin 18 (GPIO24) -> BIN2 (PWM for right motor)

CONTROL (per motor):
  Forward: PWM on IN1, IN2 = 0%
  Reverse: IN1 = 0%, PWM on IN2
  Brake:   IN1 = 100%, IN2 = 100%
  Coast:   IN1 = 0%, IN2 = 0%

REMEMBER: PWM goes on direction pins (no separate PWM pin!)
```
