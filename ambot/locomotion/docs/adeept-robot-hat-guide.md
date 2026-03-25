# Adeept Robot HAT — Motor Control Guide

> Target: Raspberry Pi 4 Model B at `ieeeerau@10.33.214.246`
> OS: Ubuntu 25.10, Python 3.13.7, Kernel 6.17
> SSH: `ssh adeept-pi` (config in ~/.ssh/config, key: id_git)
> Motor scripts: `~/motors/` on the Pi

## Hardware Summary

| Component | Value |
|-----------|-------|
| Pi Model | Raspberry Pi 4 Model B Rev 1.5 |
| RAM | 3.7 GiB (1 GiB available) |
| Disk | 14G total, **100% full** (snap=7.2G, /usr=5.8G) |
| HAT | Adeept Robot HAT (old generation) |
| PCA9685 | I2C address 0x40 (standard), all-call 0x70 |
| Motor Driver | L298P (on-HAT, driven via PCA9685 PWM channels) |
| Motors | 2 DC motors (differential drive) |
| LiDAR | LD19 at /dev/ttyUSB0, 230400 baud |

## Critical Issue: Disk Full

The disk is 100% full (14M free). Snap packages consume 7.2G. Before installing
anything, free space:

```bash
# Check what snaps are installed
snap list

# Remove unused snaps (e.g., firefox, chromium, etc.)
sudo snap remove --purge firefox
sudo snap remove --purge chromium

# Or disable snap entirely
sudo systemctl stop snapd
sudo systemctl disable snapd
sudo apt remove --purge snapd
```

## Two Motor Control Approaches

### Approach 1: PCA9685 via I2C (Recommended)

Uses the HAT's onboard PCA9685 chip to drive the L298P motor driver. This is
the correct approach that matches the Adeept HAT's hardware design.

**PCA9685 Channel Mapping** (from `motor.c`):

| Motor | IN1 Channel | IN2 Channel | PWM Channel |
|-------|-------------|-------------|-------------|
| Motor 1 | 8 | 10 | 7 |
| Motor 2 | 13 | 12 | 11 |

**How it works**:
- IN1/IN2 set direction (one HIGH=4095, one LOW=0)
- PWM channel controls speed (0-4095 duty cycle)
- PCA9685 communicates over I2C bus 1 at address 0x40
- No pip packages needed — uses raw I2C via `/dev/i2c-1`

**Script**: `~/motors/motor_pca9685.py`

```bash
python3 ~/motors/motor_pca9685.py --test            # Full test
python3 ~/motors/motor_pca9685.py --motor 1          # Motor 1 only
python3 ~/motors/motor_pca9685.py --motor 2          # Motor 2 only
python3 ~/motors/motor_pca9685.py --speed 50          # 50% speed
```

### Approach 2: Direct GPIO via lgpio

Bypasses the HAT's PCA9685 and drives L298N-style EN+IN1+IN2 pins directly
from the Pi's GPIO. This is the approach used in the original `~/motor.py`.

**GPIO Pin Mapping** (BCM):

| Motor | EN Pin | IN1 Pin | IN2 Pin |
|-------|--------|---------|---------|
| Motor A (right) | 17 | 27 | 22 |
| Motor B (left) | 13 | 6 | 5 |

**Alternative Motor B pins** (from `motor_debug.py`):

| Motor B Alt | EN=4 | IN1=26 | IN2=21 |

**How it works**:
- EN pin gets PWM signal (lgpio.tx_pwm) for speed control
- IN1/IN2 set direction (one HIGH, one LOW)
- Uses `lgpio` library (apt package `python3-lgpio`, already installed)

**Script**: `~/motors/motor_gpio.py`

```bash
python3 ~/motors/motor_gpio.py --test                # Full test
python3 ~/motors/motor_gpio.py --motor A             # Motor A only
python3 ~/motors/motor_gpio.py --motor B             # Motor B only
python3 ~/motors/motor_gpio.py --scan                # Scan all GPIO pins
```

## Diagnostic Tool

```bash
python3 ~/motors/diagnose.py            # Full hardware check
python3 ~/motors/diagnose.py --i2c      # I2C scan only
python3 ~/motors/diagnose.py --gpio     # GPIO check only
python3 ~/motors/diagnose.py --pca      # PCA9685 test
```

### Latest Diagnostic Results (2026-03-24)

```
PCA9685 at 0x40: Found, sleep mode, prescale=30 (~196 Hz)
GPIO chip 0: pinctrl-bcm2711
Motor A EN (17): readable, current=1
Motor A IN1 (27): readable, current=1
Motor A IN2 (22): readable, current=0
Motor B EN (13): readable, current=0        ← Motor B enable is LOW!
Motor B IN1 (6): readable, current=1
Motor B IN2 (5): readable, current=1
```

**Update (2026-03-24)**: Both motors now confirmed working with the GPIO approach.
A GPIO pin change was made in `~/motor.py` and both motors spin. Direction
calibration still needed — run `~/motors/calibrate_direction.py` to verify
forward/backward/left/right match expectations.

## Direction Calibration Tool

```bash
# Full 8-test calibration — have someone watch the robot
python3 ~/motors/calibrate_direction.py

# Slower speed for safety
python3 ~/motors/calibrate_direction.py --speed 30
```

Tests individual motors in each direction, then combined movements (forward,
backward, spin left, spin right). Reports which direction flags need flipping.

## Existing Code on the Pi

| File | Purpose | Approach |
|------|---------|----------|
| `~/motor.py` | 2-motor control | Direct GPIO (lgpio) |
| `~/motor_debug.py` | Pin scanning/debugging | Direct GPIO (lgpio), brute-force pin scan |
| `~/motor.c` | 2-motor control in C | PCA9685 via I2C (correct HAT approach) |
| `~/ld_19_fast.c` | LiDAR viewer (SDL2) | Serial + SDL2 graphics |
| `~/port_test1.c` | LiDAR viewer (earlier) | Serial + SDL2 |

### What Worked Previously

From comments in `~/motor.py`:
- `motor_A(0, 100)` — **Motor A forward works** (direction 0)
- `motor_B(0, 100)` — Comment says "0 works 1 works, broken motor"

The "broken motor" comment suggests Motor B physically works (both directions
were tested) but may be intermittent or have a wiring issue.

## Adeept Robot HAT Architecture

```
┌─────────────────────────────────────────────┐
│            Adeept Robot HAT (old gen)        │
│                                             │
│  ┌──────────┐         ┌──────────┐          │
│  │ PCA9685  │──I2C──→ │  L298P   │──→ Motor │
│  │ 0x40     │         │  H-Bridge│──→ Motor │
│  │ 16-ch PWM│         └──────────┘          │
│  └──────────┘                               │
│       ↑                                     │
│    I2C Bus 1 (SDA=GPIO2, SCL=GPIO3)         │
│                                             │
│  Servo channels: 0-6                        │
│  Motor channels: 7-13                       │
└─────────────────────────────────────────────┘
```

The PCA9685 generates PWM signals that drive the L298P motor controller.
Direct GPIO access (as in motor.py) goes **around** this architecture and
tries to drive the L298P pins directly from the Pi's GPIO header — this
can work but may conflict with the PCA9685 if both try to drive the same lines.

## Troubleshooting Motor B

1. **Try PCA9685 approach first** — `python3 ~/motors/motor_pca9685.py --motor 2`
   This drives Motor 2 through the correct HAT pathway.

2. **If PCA9685 doesn't work**, the HAT's L298P connection to Motor B may be bad.

3. **Check physical connections**:
   - Motor wires firmly in screw terminals
   - HAT fully seated on Pi GPIO header
   - Power supply adequate (motors need more current than Pi can provide via USB)

4. **Check for pin conflicts**: Don't run motor.py (GPIO) and motor_pca9685.py
   (I2C) simultaneously — they may fight over the same motor driver pins.

## Package Status

| Package | Installed | Notes |
|---------|-----------|-------|
| python3-lgpio | Yes | For direct GPIO motor control |
| python3-rpi.gpio | Yes | Legacy, may conflict with lgpio |
| python3-gpiozero | Yes | High-level GPIO |
| i2c-tools | Yes | For `i2cdetect` |
| pip3 | **No** | `pip3: command not found` |
| SDL2 | Yes | Used by LiDAR C programs |

**Note**: pip3 is missing, so no Python packages can be installed via pip.
All Python packages must come from apt. The PCA9685 motor script uses raw
I2C (`/dev/i2c-1`) to avoid needing any pip packages.
