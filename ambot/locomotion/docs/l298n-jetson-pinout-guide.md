# L298N Motor Driver Wiring Guide for Jetson Orin Nano

## Quick Reference -- Jetson Pin Connections

| L298N Pin | Function | Jetson BOARD Pin | Jetson GPIO | Notes |
|-----------|----------|:----------------:|:-----------:|-------|
| **ENA** | Motor A PWM | **32** | GPIO07 | Hardware PWM, needs pinmux |
| **IN1** | Motor A dir 1 | **29** | GPIO01 | General GPIO |
| **IN2** | Motor A dir 2 | **31** | GPIO11 | General GPIO |
| **ENB** | Motor B PWM | **33** | GPIO13 | Hardware PWM, needs pinmux |
| **IN3** | Motor B dir 1 | **7** | GPIO09 | General GPIO |
| **IN4** | Motor B dir 2 | **13** | SPI1_SCK | General GPIO (repurposed) |
| **GND** | Common ground | **6** | GND | **CRITICAL** |

---

## Overview

This guide covers wiring an L298N dual H-bridge motor driver to a **Jetson Orin Nano**
developer kit. The Jetson Orin Nano has a 40-pin expansion header that is physically
compatible with the Raspberry Pi header layout, but the GPIO numbering, electrical
characteristics, and pin configuration process differ significantly.

**Key differences from Raspberry Pi:**
- GPIO names use Jetson-specific numbering (GPIO07, GPIO13, etc.), not BCM
- PWM pins require **pinmux configuration** via `jetson-io.py` before use
- GPIO library is `Jetson.GPIO` (API-compatible with `RPi.GPIO`)
- All GPIO are 3.3V logic (same as RPi)

---

## L298N Specifications (Recap)

| Parameter | Value |
|-----------|-------|
| Motor Voltage (VS) | 5V - 35V |
| Logic Voltage (VSS) | 5V (onboard regulator) or 3.3V (direct) |
| Logic Input High (VIH) | min 2.3V |
| Continuous Current | 2A per channel |
| Peak Current | 3.5A per channel |
| Voltage Drop | 2V - 2.8V (BJT losses) |

**Voltage compatibility**: Jetson Orin Nano GPIO outputs 3.3V. The L298N logic input
high threshold (VIH) is 2.3V minimum. Since 3.3V > 2.3V, the L298N will reliably
recognize Jetson GPIO HIGH signals. **No level shifter is required.**

---

## Jetson Orin Nano 40-Pin Header (Annotated)

```
JETSON ORIN NANO 40-PIN EXPANSION HEADER
(Looking at the board with the header at the top edge)

         3.3V [1]  [2]  5V
        SDA.1 [3]  [4]  5V
        SCL.1 [5]  [6]  GND          <-- COMMON GROUND
  IN3-> GPIO09[7]  [8]  UART1_TX
          GND [9]  [10] UART1_RX
       GPIO08[11] [12] I2S0_SCLK
  IN4-> SPI1  [13] [14] GND
       GPIO12[15] [16] SPI1_CS1
         3.3V [17] [18] SPI1_CS0
   SPI0_MOSI [19] [20] GND
   SPI0_MISO [21] [22] GPIO05
   SPI0_SCK  [23] [24] SPI0_CS0
          GND [25] [26] SPI0_CS1
   SDA.0     [27] [28] SCL.0
  IN1-> GPIO01[29] [30] GND
  IN2-> GPIO11[31] [32] GPIO07 <--ENA (PWM)
  ENB-> GPIO13[33] [34] GND
  I2S0_FS    [35] [36] UART1_CTS
       GPIO05 [37] [38] I2S0_DIN
          GND [39] [40] I2S0_DOUT

Arrows (-->) indicate pins used for L298N motor control.
```

### PWM-Capable Pins on the 40-Pin Header

| BOARD Pin | Jetson GPIO | PWM Channel | Our Usage |
|:---------:|:-----------:|:-----------:|-----------|
| 15 | GPIO12 | PWM | Reserved for future |
| 32 | GPIO07 | PWM | **Motor A ENA** |
| 33 | GPIO13 | PWM | **Motor B ENB** |

Only pins 32 and 33 are used for motor PWM. Pin 15 is kept available for future
peripherals (servos, etc.).

---

## Wiring Diagram (ASCII)

```
    BATTERY PACK (6V-12V)                    JETSON ORIN NANO
    =====================                    ================

         (+) ────────────────────┐
                                 │
                                 v
                           [L298N +12V/VS]

         (-) ────────────────────┬──────────────── [GND Pin 6]
                                 │
                                 v
                           [L298N GND]


    L298N CONTROL SIDE                  JETSON 40-PIN HEADER
    ==================                  ====================

    ENA  ─────── (orange) ──────────── Pin 32  (GPIO07, HW PWM)
    IN1  ─────── (yellow) ──────────── Pin 29  (GPIO01)
    IN2  ─────── (green)  ──────────── Pin 31  (GPIO11)

    ENB  ─────── (white)  ──────────── Pin 33  (GPIO13, HW PWM)
    IN3  ─────── (blue)   ──────────── Pin 7   (GPIO09)
    IN4  ─────── (purple) ──────────── Pin 13  (SPI1_SCK as GPIO)

    GND  ─────── (black)  ──────────── Pin 6   (GND)


    L298N OUTPUT SIDE
    =================

    OUT1 ──── Motor A wire 1
    OUT2 ──── Motor A wire 2
    OUT3 ──── Motor B wire 1
    OUT4 ──── Motor B wire 2


                    L298N MODULE (Top View)
    +=======================================================+
    |                                                       |
    |   [Motor A-]  [Motor A+]     [Motor B+]  [Motor B-]  |
    |      OUT1       OUT2           OUT3        OUT4       |
    |                                                       |
    |   +-------------------------------------------+       |
    |   |                                           |       |
    |   |              L298N CHIP                   |       |
    |   |              (with heatsink)              |       |
    |   |                                           |       |
    |   +-------------------------------------------+       |
    |                                                       |
    |   [12V/+VS]  [GND]  [5V]     ENA  IN1  IN2  IN3  IN4  ENB
    |      |        |      |        |    |    |    |    |    |
    +=======|========|======|========|====|====|====|====|====|=+
            |        |      |        |    |    |    |    |    |
         Batt+    Batt-   N/C     Pin32 Pin29 Pin31 Pin7 Pin13 Pin33
                  + Jtsn                  (Jetson BOARD pins)
                  GND(6)
```

---

## Pinmux Configuration (REQUIRED for PWM)

Unlike the Raspberry Pi, the Jetson Orin Nano requires explicit **pinmux configuration**
to enable PWM output on pins 32 and 33. Without this step, those pins default to GPIO
mode and PWM will not work.

### Step 1: Run jetson-io.py

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

### Step 2: Configure PWM Pins

1. Select **"Configure Jetson 40pin Header"**
2. Select **"Configure header pins manually"**
3. Enable PWM on:
   - **Pin 32** (GPIO07) --> select "pwm0" or equivalent
   - **Pin 33** (GPIO13) --> select "pwm2" or equivalent
4. Save and exit
5. **Reboot required** for changes to take effect

```bash
sudo reboot
```

### Step 3: Verify Configuration

After reboot, confirm PWM is available:

```bash
# Check that PWM channels are exported
ls /sys/class/pwm/

# Verify pin configuration
sudo cat /sys/kernel/debug/gpio | grep -E "gpio-0(7|13)"
```

**Note**: The pinmux configuration persists across reboots once set.

---

## Python Library: Jetson.GPIO

The Jetson uses `Jetson.GPIO` (version 2.1.7+), which is API-compatible with `RPi.GPIO`.
This means existing motor driver code written for RPi.GPIO works with minimal changes.

### Installation

```bash
# Usually pre-installed on JetPack. If not:
pip install Jetson.GPIO

# Grant GPIO access without sudo:
sudo groupadd -f -r gpio
sudo usermod -aG gpio $USER
sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# Log out and back in for group changes
```

### BOARD vs TEGRA Numbering

Jetson.GPIO supports multiple pin numbering modes:

| Mode | Example (Pin 32) | Usage |
|------|-------------------|-------|
| `BOARD` | `32` | **Recommended** -- matches physical pin number |
| `TEGRA_SOC` | SOC-specific name | Low-level, not portable |
| `CVM` | Module-specific | Not recommended |

**Use BOARD numbering** for consistency with our RPi codebase.

### Code Compatibility

```python
# This same import pattern works on both RPi and Jetson:
try:
    import Jetson.GPIO as GPIO
except ImportError:
    import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)  # Works identically on both platforms
```

The `config.py` module in `rpi_motors/` already includes `get_gpio_library()` which
auto-detects the platform and returns the correct GPIO library.

---

## Control Logic Truth Table

| IN1 | IN2 | ENA (PWM) | Motor A Behavior |
|:---:|:---:|:---------:|------------------|
| HIGH | LOW | 0-100% | Forward at PWM speed |
| LOW | HIGH | 0-100% | Reverse at PWM speed |
| HIGH | HIGH | Any | Brake (motor locked) |
| LOW | LOW | Any | Coast (motor free) |
| Any | Any | 0% | Stopped |

Same logic applies to IN3/IN4/ENB for Motor B.

---

## RPi vs Jetson Pinout Comparison

This table shows the pin mapping differences between the RPi and Jetson configurations
for the L298N driver:

| L298N Pin | Function | RPi BOARD Pin | RPi BCM | Jetson BOARD Pin | Jetson GPIO | Same Pin? |
|-----------|----------|:------------:|:-------:|:----------------:|:-----------:|:---------:|
| **ENA** | Motor A PWM | 32 | GPIO12 | 32 | GPIO07 | Same |
| **IN1** | Motor A dir 1 | 11 | GPIO17 | 29 | GPIO01 | Different |
| **IN2** | Motor A dir 2 | 13 | GPIO27 | 31 | GPIO11 | Different |
| **ENB** | Motor B PWM | 33 | GPIO13 | 33 | GPIO13 | Same |
| **IN3** | Motor B dir 1 | 15 | GPIO22 | 7 | GPIO09 | Different |
| **IN4** | Motor B dir 2 | 16 | GPIO23 | 13 | SPI1_SCK | Different |
| **GND** | Common ground | 6 | GND | 6 | GND | Same |

**Key observations:**
- **PWM pins 32 and 33 are the same** on both platforms (both have hardware PWM there)
- **Direction pins are different** because the RPi pins used (11, 13, 15, 16) serve
  other functions on the Jetson (I2S, SPI, etc.)
- **GND pin 6 is the same** on both platforms
- The Jetson direction pins (7, 13, 29, 31) were chosen to avoid conflicts with I2C,
  SPI0, UART, and I2S buses

### Why Different Direction Pins?

On the RPi, pins 11/13/15/16 are general-purpose GPIO. On the Jetson Orin Nano, some
of those pins serve specialized functions:

| RPi Pin | RPi Function | Jetson Function | Conflict? |
|:-------:|-------------|-----------------|:---------:|
| 11 | GPIO17 | GPIO08 | Usable, but reserved for other peripherals |
| 13 | GPIO27 | **SPI1_SCK** | Yes -- SPI bus conflict |
| 15 | GPIO22 | **GPIO12/PWM** | Yes -- wastes a PWM-capable pin |
| 16 | GPIO23 | **SPI1_CS1** | Yes -- SPI bus conflict |

The Jetson pinout uses pins 7, 13, 29, and 31 instead, which are available as
general-purpose GPIO without sacrificing important bus interfaces.

---

## Power Supply Connections

### Wiring

```
    BATTERY PACK (6V-12V)
    +================+
    |  (+) ──────────┼──────────────────────> L298N [+12V/VS]
    |                |
    |  (-) ──────────┼───────┬──────────────> L298N [GND]
    +================+       |
                             |
                             └──────────────> Jetson Orin Nano [GND Pin 6]

    Jetson is powered separately via its barrel jack / USB-C power supply.
```

### Critical Power Rules

1. **NEVER power motors from the Jetson 5V pin** -- motor current draw will damage
   the board or cause brownouts
2. **ALWAYS use a separate battery pack** for the L298N motor supply
3. **ALWAYS connect common ground** between battery/L298N and Jetson (Pin 6)
4. **Remove ENA/ENB jumpers** on the L298N to enable PWM speed control
5. **Remove the 5V regulator jumper** if your battery pack is under 7V

### Voltage Drop Reminder

```
Fresh 4xAA alkaline:  4 x 1.5V = 6.0V
L298N voltage drop:   -2.0V to -2.8V
Motor sees:           3.2V to 4.0V

2S LiPo:              7.4V nominal
L298N voltage drop:   -2.0V
Motor sees:           5.4V (recommended for more torque)
```

---

## Step-by-Step Wiring Instructions

### Prerequisites

- Jetson Orin Nano developer kit (JetPack R36.4.x installed)
- L298N motor driver module
- Battery holder with leads (4xAA or 2S LiPo)
- 2x DC motors
- Jumper wires (female-to-female for Jetson header, stripped ends for L298N terminals)
- Small flathead screwdriver for screw terminals

### Step 1: Prepare the L298N Module

1. **Remove the ENA and ENB jumpers** -- these short the enable pins to 5V and prevent
   PWM control
2. **Remove the 5V regulator jumper** if using a battery pack under 7V

### Step 2: Connect Power (Battery Pack)

1. Battery (+) --> L298N **+12V/VS** screw terminal
2. Battery (-) --> L298N **GND** screw terminal
3. **Do not insert batteries yet**

### Step 3: Connect Common Ground (MOST CRITICAL)

1. Run a wire from the L298N **GND** terminal to Jetson **Pin 6** (GND)
2. This connection is shared with the battery ground
3. Without common ground, no control signals will work

### Step 4: Connect Motor A Control Pins

| Wire | From L298N | To Jetson Pin | Color (suggested) |
|------|------------|:-------------:|:-----------------:|
| 1 | ENA | Pin 32 (GPIO07) | Orange |
| 2 | IN1 | Pin 29 (GPIO01) | Yellow |
| 3 | IN2 | Pin 31 (GPIO11) | Green |

### Step 5: Connect Motor B Control Pins

| Wire | From L298N | To Jetson Pin | Color (suggested) |
|------|------------|:-------------:|:-----------------:|
| 4 | ENB | Pin 33 (GPIO13) | White |
| 5 | IN3 | Pin 7 (GPIO09) | Blue |
| 6 | IN4 | Pin 13 (SPI1_SCK) | Purple |

### Step 6: Connect Motors to Output Terminals

1. Motor A wires --> **OUT1** and **OUT2**
2. Motor B wires --> **OUT3** and **OUT4**
3. Polarity determines direction; swap wires later if motor spins backwards

### Step 7: Pre-Power Verification Checklist

Before powering on, verify every connection:

- [ ] Battery (+) connected to L298N +12V/VS
- [ ] Battery (-) connected to L298N GND
- [ ] L298N GND connected to Jetson GND (Pin 6)
- [ ] ENA jumper **removed**, ENA wire to Jetson Pin 32
- [ ] ENB jumper **removed**, ENB wire to Jetson Pin 33
- [ ] IN1 connected to Jetson Pin 29
- [ ] IN2 connected to Jetson Pin 31
- [ ] IN3 connected to Jetson Pin 7
- [ ] IN4 connected to Jetson Pin 13
- [ ] Motor A wires in OUT1 and OUT2
- [ ] Motor B wires in OUT3 and OUT4
- [ ] All screw terminals tight
- [ ] No bare wires touching each other or the heatsink
- [ ] Pinmux configured and Jetson rebooted (see Pinmux section above)

### Step 8: Power On and Test

1. Insert batteries / connect LiPo
2. L298N power LED should illuminate
3. Boot the Jetson
4. Run the test script below

---

## Test Script (Jetson)

```python
#!/usr/bin/env python3
"""L298N Motor Test for Jetson Orin Nano -- tests each motor direction."""

import time

try:
    import Jetson.GPIO as GPIO
except ImportError:
    import RPi.GPIO as GPIO

# Pin definitions (BOARD numbering -- physical pin numbers)
ENA = 32  # Motor A PWM  (GPIO07)
IN1 = 29  # Motor A dir  (GPIO01)
IN2 = 31  # Motor A dir  (GPIO11)
ENB = 33  # Motor B PWM  (GPIO13)
IN3 = 7   # Motor B dir  (GPIO09)
IN4 = 13  # Motor B dir  (SPI1_SCK as GPIO)

# Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup([ENA, IN1, IN2, ENB, IN3, IN4], GPIO.OUT)

# Create PWM objects (1kHz frequency)
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)


def motor_a_forward(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)


def motor_a_backward(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)


def motor_b_forward(speed=50):
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_b.ChangeDutyCycle(speed)


def motor_b_backward(speed=50):
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(speed)


def stop_all():
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)


try:
    print("Testing Motor A forward...")
    motor_a_forward(50)
    time.sleep(2)
    stop_all()
    time.sleep(0.5)

    print("Testing Motor A backward...")
    motor_a_backward(50)
    time.sleep(2)
    stop_all()
    time.sleep(0.5)

    print("Testing Motor B forward...")
    motor_b_forward(50)
    time.sleep(2)
    stop_all()
    time.sleep(0.5)

    print("Testing Motor B backward...")
    motor_b_backward(50)
    time.sleep(2)
    stop_all()

    print("All tests complete!")

finally:
    stop_all()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
```

---

## Troubleshooting

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| No motor response | Missing common ground | Connect L298N GND to Jetson GND (Pin 6) |
| Motors always full speed | ENA/ENB jumpers still installed | Remove both jumpers |
| PWM has no effect | Pinmux not configured | Run `jetson-io.py`, enable PWM on pins 32/33, reboot |
| `GPIO.PWM()` throws error | Pin not configured for PWM | Verify pinmux config, check `/sys/class/pwm/` |
| Permission denied on GPIO | User not in gpio group | `sudo usermod -aG gpio $USER`, log out/in |
| `Jetson.GPIO` import fails | Library not installed | `pip install Jetson.GPIO` |
| One motor works, other not | Wiring error on that channel | Trace wires, check correct Jetson pin numbers |
| Motors spin wrong direction | Normal -- depends on wiring | Swap motor wires at OUT terminals |
| Motors weak/stall easily | Voltage drop + low battery | Use fresh batteries or higher voltage pack |
| Jetson reboots during motor use | Drawing power through Jetson | Use separate motor power supply |
| Erratic motor behavior | Loose connections | Check all screw terminals and jumper wires |
| `GPIO.setup()` warns about pin mode | Pin already in use by kernel | Check pinmux, release pin from other drivers |

---

## Common Mistakes to Avoid

### 1. Forgetting Pinmux Configuration

The single biggest difference from RPi. On the Raspberry Pi, PWM pins work out of the
box. On Jetson, you **must** run `jetson-io.py` to configure pins 32 and 33 for PWM
output. Without this, `GPIO.PWM()` calls will fail or produce no output.

### 2. Using RPi BCM Pin Numbers

If porting code from RPi, do not use BCM GPIO numbers (12, 13, 17, 22, 23, 27). Use
BOARD pin numbers, which refer to the physical header position. The BOARD numbers are
consistent between RPi and Jetson; the GPIO names behind them are different.

### 3. Connecting Motor Power to Jetson

The Jetson Orin Nano draws up to 15W from its power supply. Motor stall current can
cause voltage drops that corrupt the eMMC or damage the power management IC. Always use
a completely separate power supply for the L298N motor voltage.

### 4. Skipping Common Ground

The L298N GND **must** be connected to the Jetson GND. Without a shared ground
reference, the 3.3V control signals from the Jetson have no reference point and the
L298N will not recognize them.

---

## References

- [Jetson Orin Nano 40-Pin Header Pinout](https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/)
- [Jetson.GPIO Library (GitHub)](https://github.com/NVIDIA/jetson-gpio)
- [L298N Datasheet (ST)](https://www.st.com/resource/en/datasheet/l298.pdf)
- [jetson-io.py Pinmux Tool](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonGpioExpansionHeader.html)
- RPi version of this guide: [l298n-driver-wiring-guide.md](l298n-driver-wiring-guide.md)
- Motor driver comparison: [motor-drivers-comparison.md](motor-drivers-comparison.md)
