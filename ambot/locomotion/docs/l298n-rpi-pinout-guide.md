# L298N Motor Driver - Raspberry Pi Pinout Guide

## Quick Reference -- AMBOT Primary Pins

| L298N Pin | Function | BOARD Pin | BCM GPIO |
|-----------|----------|:---------:|:--------:|
| **ENA** | Motor A PWM | **33** | GPIO13 |
| **IN1** | Motor A dir 1 | **13** | GPIO27 |
| **IN2** | Motor A dir 2 | **15** | GPIO22 |
| **ENB** | Motor B PWM | **32** | GPIO12 |
| **IN3** | Motor B dir 1 | **16** | GPIO23 |
| **IN4** | Motor B dir 2 | **18** | GPIO24 |
| **GND** | Common ground | **6** | GND |

---

## Adeept Robot HAT Alternative Pins (Tested Session 21)

These BCM pins were tested during the Adeept HAT evaluation. The Adeept HAT was
**abandoned** in favor of direct L298N wiring -- the HAT added unnecessary
complexity and conflicted with other peripherals. Listed here for reference only.

| L298N Pin | Function | Adeept BCM GPIO |
|-----------|----------|:---------------:|
| Motor A EN | PWM | 17 |
| Motor A IN1 | Dir 1 | 27 |
| Motor A IN2 | Dir 2 | 22 |
| Motor B EN | PWM | 4 |
| Motor B IN1 | Dir 1 | 26 |
| Motor B IN2 | Dir 2 | 21 |

**Note**: GPIO4 (Adeept Motor B EN) conflicts with 1-Wire bus. GPIO21 conflicts
with SPI0_MISO. The primary pin mapping above avoids these conflicts.

---

## Control Truth Table

| IN1 | IN2 | EN (PWM%) | Motor Action |
|:---:|:---:|:---------:|--------------|
| HIGH | LOW | 0-100 | Forward at PWM speed |
| LOW | HIGH | 0-100 | Reverse at PWM speed |
| HIGH | HIGH | any | Brake (motor locked) |
| LOW | LOW | any | Coast (motor free-spinning) |
| any | any | 0 | Stopped |

Same logic applies to IN3/IN4/ENB for Motor B.

---

## ASCII Wiring Diagram

```
    4xAA BATTERY (~6V)              RASPBERRY PI (BOARD pins)
    ==================              =========================

    (+) ────────────── L298N [+12V/VS]
                                        Pin 33 (GPIO13) ── L298N [ENA]
    (-) ──┬─────────── L298N [GND]      Pin 13 (GPIO27) ── L298N [IN1]
          |                             Pin 15 (GPIO22) ── L298N [IN2]
          └─────────── RPi  [Pin 6]     Pin 16 (GPIO23) ── L298N [IN3]
                       (GND)            Pin 18 (GPIO24) ── L298N [IN4]
                                        Pin 32 (GPIO12) ── L298N [ENB]
                                        Pin 6  (GND) ───── L298N [GND]

    L298N [OUT1] ── Motor A wire 1
    L298N [OUT2] ── Motor A wire 2      * Remove ENA/ENB jumpers for PWM control
    L298N [OUT3] ── Motor B wire 1      * Remove 5V regulator jumper (VS < 7V)
    L298N [OUT4] ── Motor B wire 2      * Swap motor wires to reverse direction
```

```
    RASPBERRY PI 40-PIN HEADER (pins used marked with *)
    =====================================================

    3.3V [1]  [2]  5V
   GPIO2 [3]  [4]  5V
   GPIO3 [5]  [6]* GND  ──────── L298N GND (common ground)
   GPIO4 [7]  [8]  GPIO14
     GND [9]  [10] GPIO15
  GPIO17 [11] [12] GPIO18
* GPIO27 [13] [14] GND
* GPIO22 [15] [16] GPIO23 *
    3.3V [17] [18] GPIO24 *
  GPIO10 [19] [20] GND
   GPIO9 [21] [22] GPIO25
  GPIO11 [23] [24] GPIO8
     GND [25] [26] GPIO7
   GPIO0 [27] [28] GPIO1
   GPIO5 [29] [30] GND
   GPIO6 [31] [32] GPIO12 *  ── L298N ENB (Motor B PWM)
* GPIO13 [33] [34] GND
  GPIO19 [35] [36] GPIO16
  GPIO26 [37] [38] GPIO20
     GND [39] [40] GPIO21
```

---

## PWM Setup Examples

### RPi.GPIO (legacy, works on all Pi models)

```python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# Pin setup
ENA, IN1, IN2 = 13, 27, 22  # Motor A
ENB, IN3, IN4 = 12, 23, 24  # Motor B

GPIO.setup([ENA, IN1, IN2, ENB, IN3, IN4], GPIO.OUT)

# PWM at 1 kHz
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

# Motor A forward at 50%
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
pwm_a.ChangeDutyCycle(50)

# Cleanup
pwm_a.stop()
pwm_b.stop()
GPIO.cleanup()
```

### lgpio (modern replacement, required on Pi 5)

```python
import lgpio

h = lgpio.gpiochip_open(0)

# Pin setup
ENA, IN1, IN2 = 13, 27, 22  # Motor A
ENB, IN3, IN4 = 12, 23, 24  # Motor B

for pin in [ENA, IN1, IN2, ENB, IN3, IN4]:
    lgpio.gpio_claim_output(h, pin)

# Motor A forward at 50% (1 kHz PWM)
lgpio.gpio_write(h, IN1, 1)
lgpio.gpio_write(h, IN2, 0)
lgpio.tx_pwm(h, ENA, 1000, 50)  # freq_hz, duty_cycle_pct

# Cleanup
lgpio.tx_pwm(h, ENA, 1000, 0)
lgpio.gpiochip_close(h)
```

---

## Common Ground Requirement

The L298N and Raspberry Pi **must** share a common ground connection. Without it,
the 3.3V logic signals from the Pi cannot be referenced by the L298N, and no
motor commands will work.

```
    Battery (-) ──┬── L298N [GND terminal]
                  └── RPi [Pin 6 / any GND pin]
```

Any RPi GND pin works: 6, 9, 14, 20, 25, 30, 34, or 39.

---

## Key Notes

- **ENA/ENB jumpers**: Must be removed for PWM speed control. Factory default
  has jumpers installed (motors always full speed).
- **5V regulator jumper**: Remove when VS < 7V (our 4xAA = ~6V).
- **Voltage drop**: L298N drops 2-2.8V (BJT losses). With 6V in, motors see
  ~3.2-4V. Use fresh batteries or upgrade to 6xAA / 2S LiPo.
- **3.3V logic**: L298N accepts 3.3V inputs from the Pi (5V tolerant).
- **Motor direction**: Swap motor wires at OUT1/OUT2 (or OUT3/OUT4) if a motor
  spins the wrong way. No software change needed.
- **Hardware PWM**: GPIO12 (PWM0) and GPIO13 (PWM1) support hardware PWM for
  jitter-free motor control. Software PWM works but may cause motor whine.

---

## Cross-References

- [L298N Full Wiring Guide](l298n-driver-wiring-guide.md) -- verbose version with step-by-step instructions and troubleshooting
- [TB6612FNG RPi Pinout Guide](tb6612fng-rpi-pinout-guide.md) -- alternative MOSFET-based driver (lower voltage drop)
- [Motor Driver Comparison](motor-drivers-comparison.md) -- L298N vs TB6612FNG vs DRV8833
- [Adeept Robot HAT Guide](adeept-robot-hat-guide.md) -- abandoned HAT approach (reference only)
- [Jetson Motor Driver Pinout](../../docs/findings/yahboom-g1-motor-driver-jetson.md) -- Jetson Orin Nano motor wiring
