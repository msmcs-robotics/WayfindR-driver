# Locomotion - Motor Control Component

Motor control system for the Ambot robot platform. This component handles low-level motor control, separate from:
- **bootylicious**: LLM/RAG system for natural language processing
- **pathfinder**: LiDAR-based navigation and obstacle avoidance

## Overview

The locomotion module provides differential drive control for wheeled robots using the YAHBOOM G1 Smart Robot Kit with TB6612FNG motor driver.

### Supported Platforms

| Platform | GPIO Library | Status |
|----------|--------------|--------|
| Jetson Orin Nano | Jetson.GPIO | Fully supported |
| Raspberry Pi 3/4/5 | RPi.GPIO | Fully supported |

### Supported Motor Drivers

| Driver | Voltage | Current | Recommended For |
|--------|---------|---------|-----------------|
| **TB6612FNG** | 4.5-13.5V | 1.2A cont | Raspberry Pi + FAGM25-370 |
| L298N | 5-35V | 2A cont | Budget projects |
| DRV8833 | 2.7-10.8V | 1.2A cont | Low voltage (6-9V) only |

### Hardware

- **Motor Driver**: TB6612FNG (dual H-bridge) - SparkFun or similar
- **Motors**: FAGM25-370 / 370 DC Gearmotors x 2
- **Drive Type**: Differential (tank-style)
- **Kit**: YAHBOOM G1 Smart Robot Kit (Jetson) or custom Raspberry Pi build

## Directory Structure

```
locomotion/
├── deploy.sh           # Deployment and diagnostic script
├── README.md           # This file
├── logs/               # Log files (auto-created)
├── docs/               # Hardware documentation
│   ├── fagm25-370-motors.md    # Motor specifications
│   ├── tb6612fng-driver.md     # TB6612FNG wiring guide
│   └── motor-drivers-comparison.md  # Driver comparison
├── rpi_motors/         # Modular Raspberry Pi motor control
│   ├── __init__.py     # Module exports
│   ├── config.py       # Pin configurations for multiple drivers
│   ├── drivers.py      # Driver implementations (TB6612FNG, L298N, DRV8833)
│   ├── motor.py        # Motor and DifferentialDrive classes
│   ├── factory.py      # Factory functions for easy setup
│   └── test_motors.py  # Test script
└── yahboomg1/          # YAHBOOM G1 motor driver (Jetson-focused)
    ├── __init__.py     # Module exports
    ├── config.py       # Pin configuration
    ├── motor.py        # Motor and DifferentialDrive classes
    └── test_motors.py  # Interactive test script
```

## Quick Start

### 1. Check Setup

```bash
cd /path/to/ambot/locomotion
./deploy.sh setup
```

This checks:
- Platform detection (Jetson vs Raspberry Pi)
- GPIO library availability
- User permissions (gpio group)
- GPIO access

### 2. Run Diagnostics

```bash
./deploy.sh diagnose
```

Full diagnostic report including pin availability testing.

### 3. Test Motors

```bash
./deploy.sh test
```

Runs a basic motor test sequence (forward, reverse, turns).

## Pin Configuration

The motor driver uses **BOARD** pin numbering (physical pin numbers on the 40-pin header).

| Function | Pin | Description |
|----------|-----|-------------|
| STBY | 11 | Standby (enable/disable driver) |
| Left IN1 | 13 | Left motor direction 1 |
| Left IN2 | 15 | Left motor direction 2 |
| Left PWM | 33 | Left motor speed (PWM) |
| Right IN1 | 16 | Right motor direction 1 |
| Right IN2 | 18 | Right motor direction 2 |
| Right PWM | 32 | Right motor speed (PWM) |

To change pin assignments, edit `yahboomg1/config.py`.

## Usage

### Raspberry Pi with Modular Drivers (rpi_motors)

```python
from locomotion.rpi_motors import create_robot, DriverType

# Create robot with TB6612FNG (default, recommended)
robot = create_robot()

# Or specify driver type
robot = create_robot(driver_type=DriverType.TB6612FNG)
robot = create_robot(driver_type=DriverType.L298N)

# Control
robot.forward(50)
robot.turn_left(50)
robot.stop()
robot.cleanup()
```

Test with: `python3 -m locomotion.rpi_motors.test_motors --basic`

See [rpi_motors/README.md](rpi_motors/README.md) for full documentation.

### Jetson/YAHBOOM G1 (yahboomg1)

```python
from locomotion.yahboomg1 import create_robot

# Create robot instance (auto-detects platform)
robot = create_robot()

# Basic movements
robot.forward(50)      # Forward at 50% speed
robot.reverse(50)      # Reverse at 50% speed
robot.turn_left(50)    # Pivot turn left
robot.turn_right(50)   # Pivot turn right
robot.arc_left(50, 0.5)  # Arc turn (ratio 0-1)
robot.stop()           # Stop motors (coast)
robot.brake()          # Stop motors (brake)

# Direct motor control
robot.drive(left=50, right=30)  # Different speeds per side

# Cleanup when done
robot.cleanup()
```

### Direct Motor Control

```python
from locomotion.yahboomg1 import Motor, GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Create individual motor
motor = Motor(in1=13, in2=15, pwm=33)

motor.forward(50)
motor.reverse(50)
motor.stop()
motor.cleanup()

GPIO.cleanup()
```

### CLI Testing

```bash
# Platform check only
python3 -m locomotion.yahboomg1.test_motors --check

# Basic motor test
python3 -m locomotion.yahboomg1.test_motors --basic

# Individual motor test (one at a time)
python3 -m locomotion.yahboomg1.test_motors --individual

# Interactive keyboard control
python3 -m locomotion.yahboomg1.test_motors --interactive

# Custom speed and duration
python3 -m locomotion.yahboomg1.test_motors --basic --speed 30 --duration 2.0
```

## Deploy Script Commands

| Command | Description |
|---------|-------------|
| `setup` | Check GPIO permissions, verify gpio group, check dependencies |
| `start` | Info only (motors are controlled on-demand, not as a service) |
| `stop` | Stop motor processes and ensure motors are stopped (safety) |
| `status` | Show GPIO availability, motor driver status, permissions |
| `diagnose` | Comprehensive diagnostics with pin testing |
| `test` | Run basic motor test sequence |
| `help` | Show usage information |

## Troubleshooting

### "No GPIO library found"

Install the appropriate library for your platform:

```bash
# Jetson
sudo pip3 install Jetson.GPIO

# Raspberry Pi
sudo pip3 install RPi.GPIO
```

### "Permission denied" or GPIO access errors

Add your user to the gpio group:

```bash
sudo usermod -a -G gpio $USER
```

Then **log out and log back in** for the change to take effect.

### Motors don't move

1. Check wiring matches pin configuration
2. Verify power supply to motor driver
3. Run `./deploy.sh diagnose` to test pins
4. Check STBY pin is being set HIGH (enabled)

### Wrong direction

Edit `yahboomg1/config.py` and change the offset:

```python
LEFT_OFFSET = -1   # Change from 1 to -1 to reverse direction
RIGHT_OFFSET = -1
```

### PWM not working

Some GPIO pins don't support hardware PWM. The default pins (32, 33) are chosen for PWM compatibility on Jetson Orin Nano. If using Raspberry Pi, you may need to adjust.

## Integration with Other Components

The locomotion system is intentionally separate from the LLM/RAG (bootylicious) and LiDAR (pathfinder) systems. Integration happens at the application level:

```python
# Example: Voice-controlled robot
from locomotion.yahboomg1 import create_robot
from bootylicious.rag.client import ask_question

robot = create_robot()

# Parse natural language command
response = ask_question("Move forward slowly")
# LLM response contains intent

if "forward" in response.lower():
    robot.forward(30)
elif "stop" in response.lower():
    robot.stop()
```

```python
# Example: LiDAR-guided navigation
from locomotion.yahboomg1 import create_robot
from pathfinder.lidar import LidarScanner

robot = create_robot()
lidar = LidarScanner()

while True:
    scan = lidar.get_scan()
    if scan.obstacle_ahead(distance=0.5):
        robot.stop()
    else:
        robot.forward(50)
```

## Safety Notes

1. **Always call `robot.cleanup()`** when done to release GPIO pins
2. **Use `./deploy.sh stop`** to ensure motors are stopped in emergencies
3. Test on a raised platform before running on the floor
4. Start with low speeds (30-50) until confident in wiring
5. The STBY pin must be HIGH for motors to operate

## Files Reference

- `yahboomg1/config.py` - Pin assignments and motor parameters
- `yahboomg1/motor.py` - Motor and DifferentialDrive classes
- `yahboomg1/test_motors.py` - Test and interactive control script
- `deploy.sh` - Deployment, diagnostics, and testing script
