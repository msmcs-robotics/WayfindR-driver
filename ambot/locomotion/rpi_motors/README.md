# Raspberry Pi Motor Control Module

Modular motor control for FAGM25-370 motors with multiple driver support.

## Overview

This module provides a unified interface for controlling DC motors with different motor drivers. Primary target is the SparkFun TB6612FNG with FAGM25-370 brushed DC gearmotors.

## Supported Motor Drivers

| Driver | Voltage Range | Current | Status |
|--------|---------------|---------|--------|
| **TB6612FNG** | 4.5V - 13.5V | 1.2A cont / 3.2A peak | **Recommended** |
| L298N | 5V - 35V | 2A cont | Supported |
| DRV8833 | 2.7V - 10.8V | 1.2A cont | Supported (6-9V only) |

## Quick Start

### 1. Install Dependencies

```bash
# On Raspberry Pi
pip3 install RPi.GPIO

# On Jetson
pip3 install Jetson.GPIO
```

### 2. Wire the TB6612FNG

See [docs/tb6612fng-driver.md](../docs/tb6612fng-driver.md) for complete wiring guide.

Default pin configuration (BOARD numbering):

| TB6612 Pin | Raspberry Pi Pin | Function |
|------------|------------------|----------|
| VCC | Pin 1 (3.3V) | Logic power |
| GND | Pin 6 (GND) | Ground |
| STBY | Pin 11 | Standby enable |
| AIN1 | Pin 13 | Left motor dir 1 |
| AIN2 | Pin 15 | Left motor dir 2 |
| PWMA | Pin 33 | Left motor speed |
| BIN1 | Pin 16 | Right motor dir 1 |
| BIN2 | Pin 18 | Right motor dir 2 |
| PWMB | Pin 32 | Right motor speed |

Motor power (VM) connects to external 6-12V supply.

### 3. Test Motors

```bash
# Check platform detection
python3 -m locomotion.rpi_motors.test_motors --check

# Show pin assignments
python3 -m locomotion.rpi_motors.test_motors --pinout

# Basic motor test
python3 -m locomotion.rpi_motors.test_motors --basic

# Interactive keyboard control
python3 -m locomotion.rpi_motors.test_motors --interactive
```

## Python API

### Basic Usage

```python
from locomotion.rpi_motors import create_robot, DriverType

# Create robot with TB6612FNG (default)
robot = create_robot()

# Or specify driver type
robot = create_robot(driver_type=DriverType.TB6612FNG)

# Control robot
robot.forward(50)      # Forward at 50% speed
robot.reverse(50)      # Reverse at 50% speed
robot.turn_left(50)    # Pivot turn left
robot.turn_right(50)   # Pivot turn right
robot.arc_left(50, 0.5)  # Arc turn (ratio 0-1)
robot.stop()           # Stop (coast)
robot.brake()          # Stop (brake)

# Direct motor control
robot.drive(left=50, right=30)

# Cleanup when done
robot.cleanup()
```

### Custom Pin Configuration

```python
from locomotion.rpi_motors import create_robot, DriverType

# Custom pins (in1, in2, pwm)
robot = create_robot(
    driver_type=DriverType.TB6612FNG,
    left_pins=(13, 15, 33),
    right_pins=(16, 18, 32),
    stby_pin=11,
)
```

### Using Different Drivers

```python
from locomotion.rpi_motors import create_robot, DriverType

# L298N driver
robot = create_robot(driver_type=DriverType.L298N)

# DRV8833 driver (for 6-9V motors only)
robot = create_robot(driver_type=DriverType.DRV8833)
```

## File Structure

```
rpi_motors/
├── __init__.py       # Module exports
├── config.py         # Pin configurations for each driver
├── drivers.py        # Motor driver implementations
├── motor.py          # Motor and DifferentialDrive classes
├── factory.py        # Factory functions for easy setup
├── test_motors.py    # Test script
├── requirements.txt  # Python dependencies
└── README.md         # This file
```

## Documentation

- [FAGM25-370 Motor Specs](../docs/fagm25-370-motors.md)
- [TB6612FNG Driver Guide](../docs/tb6612fng-driver.md)
- [Motor Driver Comparison](../docs/motor-drivers-comparison.md)

## Troubleshooting

### "No GPIO library found"

Install the appropriate library:
```bash
# Raspberry Pi
pip3 install RPi.GPIO

# Jetson
pip3 install Jetson.GPIO
```

### "Permission denied" on GPIO

Add user to gpio group:
```bash
sudo usermod -a -G gpio $USER
# Log out and back in
```

### Motors don't move

1. Check wiring matches pin configuration (`--pinout`)
2. Verify external power supply is connected to motor driver
3. Run `--check` to verify GPIO access
4. Ensure STBY pin is connected (TB6612FNG)

### Wrong direction

Edit pin offset in config or swap motor wires:
```python
from locomotion.rpi_motors.config import TB6612FNG_CONFIG
TB6612FNG_CONFIG.left_motor.offset = -1  # Reverse left motor
```
