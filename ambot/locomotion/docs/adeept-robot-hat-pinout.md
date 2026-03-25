# Adeept Robot HAT — Motor Pinout Reference

> Also saved on the Pi at `~/motors/PINOUT.md`

## Motor Terminal Connections

```
    ┌──────────────────────────────────────┐
    │         Adeept Robot HAT             │
    │                                      │
    │   [M1+] [M1-]     [M2+] [M2-]       │  ← Motor screw terminals
    │                                      │
    │   ┌──────────────────────────┐       │
    │   │      PCA9685 chip        │       │
    │   └──────────────────────────┘       │
    │                                      │
    │   [Servo headers 0-7]                │
    │                                      │
    │   [Power input: VIN / GND]           │  ← External power for motors
    │                                      │
    └──┬──────────────────────────────┬────┘
       │  GPIO Header (40-pin)        │
       │  (sits on top of Pi)         │
       └──────────────────────────────┘
```

**Motor 1** (right): Red → M1+, Black → M1-
**Motor 2** (left): Red → M2+, Black → M2-
**If wrong direction**: Swap red ↔ black wires.

**Power**: External 6-12V DC to VIN/GND terminals (USB power is NOT enough).

## PCA9685 Channel Mapping (I2C approach)

| Function | PCA9685 Channel | Purpose |
|----------|----------------|---------|
| Motor 1 IN1 | Ch 8 | Direction pin 1 |
| Motor 1 IN2 | Ch 10 | Direction pin 2 |
| Motor 1 PWM | Ch 7 | Speed (0-4095 duty) |
| Motor 2 IN1 | Ch 13 | Direction pin 1 |
| Motor 2 IN2 | Ch 12 | Direction pin 2 |
| Motor 2 PWM | Ch 11 | Speed (0-4095 duty) |
| Servos | Ch 0-6 | Servo outputs |

PCA9685 address: **0x40** (I2C bus 1)

### Direction Truth Table

| IN1 | IN2 | Action |
|-----|-----|--------|
| HIGH (4095) | LOW (0) | Forward |
| LOW (0) | HIGH (4095) | Reverse |
| LOW | LOW | Stop |

## GPIO Pin Mapping (Direct GPIO approach)

### Motor A (Right Wheel)

| Function | BCM Pin | Physical Pin |
|----------|---------|-------------|
| Enable (PWM) | GPIO 17 | Pin 11 |
| IN1 | GPIO 27 | Pin 13 |
| IN2 | GPIO 22 | Pin 15 |

### Motor B (Left Wheel) — UPDATED 2026-03-24

| Function | BCM Pin | Physical Pin | Notes |
|----------|---------|-------------|-------|
| Enable (PWM) | GPIO 4 | Pin 7 | Was GPIO 13, rewired |
| IN1 | GPIO 26 | Pin 37 | Was GPIO 6, rewired |
| IN2 | GPIO 21 | Pin 40 | Was GPIO 5, rewired |

### Old Motor B Pins (no longer used)

| EN=GPIO 13 (Pin 33) | IN1=GPIO 6 (Pin 31) | IN2=GPIO 5 (Pin 29) |

## Pi 4 GPIO Header (40-pin)

```
                    3V3  (1)  (2)  5V
         I2C SDA  GPIO2  (3)  (4)  5V
         I2C SCL  GPIO3  (5)  (6)  GND
                  GPIO4  (7)  (8)  GPIO14  UART TX
                    GND  (9)  (10) GPIO15  UART RX
  Motor A EN →   GPIO17 (11)  (12) GPIO18
  Motor A IN1 →  GPIO27 (13)  (14) GND
  Motor A IN2 →  GPIO22 (15)  (16) GPIO23
                   3V3  (17)  (18) GPIO24
                 GPIO10 (19)  (20) GND
                  GPIO9 (21)  (22) GPIO25
                 GPIO11 (23)  (24) GPIO8
                    GND (25)  (26) GPIO7
                  GPIO0 (27)  (28) GPIO1
  Motor B IN2 →   GPIO5 (29)  (30) GND
  Motor B IN1 →   GPIO6 (31)  (32) GPIO12
  Motor B EN →   GPIO13 (33)  (34) GND
                 GPIO19 (35)  (36) GPIO16
                 GPIO26 (37)  (38) GPIO20
                    GND (39)  (40) GPIO21
```

## I2C Devices Detected

| Address | Device |
|---------|--------|
| 0x40 | PCA9685 (PWM driver for motors + servos) |
| 0x70 | PCA9685 all-call (broadcast, normal) |

## Diagnostic Results (2026-03-24)

| Pin | State | Note |
|-----|-------|------|
| Motor A EN (GPIO 17) | HIGH | Enable active |
| Motor A IN1 (GPIO 27) | HIGH | |
| Motor A IN2 (GPIO 22) | LOW | |
| Motor B EN (GPIO 13) | **LOW** | Enable NOT active — Motor B won't spin |
| Motor B IN1 (GPIO 6) | HIGH | |
| Motor B IN2 (GPIO 5) | HIGH | |

Motor B's enable pin reads LOW. This likely explains why Motor B has been
unreliable — the enable signal isn't reaching the motor driver for Motor B.
