# TB6612FNG Motor Driver

## Quick Reference — RPi Pin Connections

| TB6612 Pin | Function | Physical Pin | BCM GPIO |
|------------|----------|:------------:|:--------:|
| **VCC** | Logic power | **1** | 3.3V |
| **GND** | Ground | **6** | GND |
| **STBY** | Standby enable | **11** | GPIO17 |
| **PWMA** | Motor A PWM | **33** | GPIO13 |
| **AIN1** | Motor A dir 1 | **13** | GPIO27 |
| **AIN2** | Motor A dir 2 | **15** | GPIO22 |
| **PWMB** | Motor B PWM | **32** | GPIO12 |
| **BIN1** | Motor B dir 1 | **16** | GPIO23 |
| **BIN2** | Motor B dir 2 | **18** | GPIO24 |

---

Primary motor driver for the Ambot project. SparkFun TB6612FNG breakout board.

## Overview

The TB6612FNG is a dual H-bridge MOSFET motor driver - more efficient than the older L298N BJT-based design.

## Specifications

| Parameter | Value |
|-----------|-------|
| Motor Voltage (VM) | 4.5V - 13.5V (max 15V) |
| Logic Voltage (VCC) | 2.7V - 5.5V |
| Continuous Current | 1.2A per channel |
| Peak Current | 3.2A per channel |
| Voltage Drop | ~0.2V (very efficient) |
| PWM Frequency | Up to 100kHz |

## Pin Functions

### Power Pins

| Pin | Function | Connect To |
|-----|----------|------------|
| VM | Motor power | External 6-12V supply |
| VCC | Logic power | Raspberry Pi 3.3V |
| GND | Ground | Common ground |

### Control Pins (Motor A)

| Pin | Function |
|-----|----------|
| PWMA | Speed control (PWM) |
| AIN1 | Direction control 1 |
| AIN2 | Direction control 2 |
| AO1/AO2 | Motor A outputs |

### Control Pins (Motor B)

| Pin | Function |
|-----|----------|
| PWMB | Speed control (PWM) |
| BIN1 | Direction control 1 |
| BIN2 | Direction control 2 |
| BO1/BO2 | Motor B outputs |

### Standby

| Pin | Function |
|-----|----------|
| STBY | Must be HIGH to enable driver |

## Control Truth Table

| IN1 | IN2 | PWM | STBY | Mode |
|-----|-----|-----|------|------|
| H | L | H | H | Forward (CW) |
| L | H | H | H | Reverse (CCW) |
| H | H | X | H | Short Brake |
| L | L | X | H | Coast/Stop |
| X | X | X | L | Standby |

## Recommended Wiring for Raspberry Pi

### Using BOARD Pin Numbers (Physical Pins)

| TB6612 Pin | Raspberry Pi | Physical Pin |
|------------|--------------|--------------|
| VCC | 3.3V | Pin 1 |
| GND | GND | Pin 6 |
| STBY | GPIO17 | Pin 11 |
| PWMA | GPIO13 (PWM1) | Pin 33 |
| AIN1 | GPIO27 | Pin 13 |
| AIN2 | GPIO22 | Pin 15 |
| PWMB | GPIO12 (PWM0) | Pin 32 |
| BIN1 | GPIO23 | Pin 16 |
| BIN2 | GPIO24 | Pin 18 |

### Using BCM GPIO Numbers

| TB6612 Pin | BCM GPIO |
|------------|----------|
| STBY | GPIO17 |
| PWMA | GPIO13 |
| AIN1 | GPIO27 |
| AIN2 | GPIO22 |
| PWMB | GPIO12 |
| BIN1 | GPIO23 |
| BIN2 | GPIO24 |

**Note**: GPIO12 and GPIO13 support hardware PWM on Raspberry Pi.

## Power Wiring

```
External Battery (6-12V)
    (+) -----> TB6612 VM
    (-) -----> TB6612 GND -----> Raspberry Pi GND

Raspberry Pi
    3.3V -----> TB6612 VCC
    GND ------> TB6612 GND (already connected above)
```

## Best Practices

1. **Enable STBY first** before sending motor commands
2. **Use hardware PWM pins** (GPIO12, GPIO13) for smoother control
3. **Soft-start motors** - ramp up speed gradually to reduce current spikes
4. **Use brake mode** (both IN high) for precise stopping
5. **Add decoupling capacitors** (100uF) across VM and GND

## Python Usage

```python
from locomotion.rpi_motors import create_robot, DriverType

# Create robot with TB6612FNG driver
robot = create_robot(driver_type=DriverType.TB6612FNG)

robot.forward(50)   # 50% speed
robot.stop()
robot.cleanup()
```

## SparkFun Board Notes

The SparkFun TB6612FNG breakout board includes:
- Decoupling capacitors
- Clearly labeled pins
- Breadboard-friendly form factor

Product link: [SparkFun Motor Driver - Dual TB6612FNG](https://www.sparkfun.com/products/14451)

## References

- [Toshiba TB6612FNG Datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf)
- [SparkFun Hookup Guide](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide)
