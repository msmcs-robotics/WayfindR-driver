# YAHBOOM G1 Motor Driver - Locomotion Module

Motor control for the YAHBOOM G1 Smart Robot Kit on Jetson Orin Nano.

## Overview

This module controls the differential drive system using:
- **Motor Driver**: TB6612FNG (or compatible)
- **Motors**: 370 DC Motors x 2
- **Control**: PWM speed + GPIO direction

## Folder Structure

```
locomotion/
├── yahboomg1/           # This driver
│   ├── README.md
│   ├── motor.py         # Motor control classes
│   ├── config.py        # Pin configuration
│   ├── test_motors.py   # Test script
│   └── requirements.txt
├── [future_driver]/     # Other motor drivers
└── README.md            # Locomotion overview
```

## Hardware Requirements

- Jetson Orin Nano (or Raspberry Pi)
- TB6612FNG Motor Driver
- 370 DC Motors x 2 (from YAHBOOM G1 kit)
- 7-12V Battery

## Wiring

See [docs/findings/yahboom-g1-motor-driver-jetson.md](../../../docs/findings/yahboom-g1-motor-driver-jetson.md) for detailed wiring diagram.

### Quick Reference (BOARD Pin Numbering)

| Function | Jetson Pin | TB6612FNG |
|----------|------------|-----------|
| 3.3V | 1 | VCC |
| GND | 6 | GND |
| STBY | 11 | STBY |
| Left IN1 | 13 | AIN1 |
| Left IN2 | 15 | AIN2 |
| Left PWM | 33 | PWMA |
| Right IN1 | 16 | BIN1 |
| Right IN2 | 18 | BIN2 |
| Right PWM | 32 | PWMB |
| Motor Power | External | VM |

## Installation

```bash
# Install Jetson.GPIO (on Jetson)
sudo pip3 install Jetson.GPIO

# Or for Raspberry Pi
# sudo pip3 install RPi.GPIO

# Add user to gpio group
sudo usermod -a -G gpio $USER
# Logout/login required
```

## Usage

```python
from motor import Motor, DifferentialDrive
import Jetson.GPIO as GPIO

# Setup
GPIO.setmode(GPIO.BOARD)

# Create motors
left = Motor(in1=13, in2=15, pwm=33)
right = Motor(in1=16, in2=18, pwm=32)
robot = DifferentialDrive(left, right, stby=11)

# Control
robot.forward(speed=50)   # 0-100
robot.turn_left(speed=30)
robot.stop()

# Cleanup
robot.cleanup()
GPIO.cleanup()
```

## Testing

```bash
# Run motor test
python3 test_motors.py

# Test specific function
python3 -c "from motor import Motor; ..."
```

## Platform Compatibility

| Platform | GPIO Library | Status |
|----------|--------------|--------|
| Jetson Orin Nano | Jetson.GPIO | Primary |
| Raspberry Pi | RPi.GPIO | Compatible |

The code auto-detects the platform and uses the appropriate library.

## Configuration

Edit `config.py` to change pin assignments or motor parameters.

## Related

- [Motor Driver Research](../../../docs/findings/yahboom-g1-motor-driver-jetson.md)
- [TB6612FNG Datasheet](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all)
