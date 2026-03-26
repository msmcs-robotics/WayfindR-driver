# Jetson Orin Nano GPIO PWM for Motor Control — Research Findings

**Date:** 2026-03-26
**Context:** Evaluating GPIO PWM options for L298N motor control on Jetson Orin Nano (JetPack R36.4.4 / JetPack 6.x)

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Hardware PWM on Jetson Orin Nano](#hardware-pwm-on-jetson-orin-nano)
3. [Software PWM — Not Available](#software-pwm--not-available)
4. [Pinmux Configuration — Step by Step](#pinmux-configuration--step-by-step)
5. [L298N Wiring with Jetson](#l298n-wiring-with-jetson)
6. [Working Code Examples](#working-code-examples)
7. [PCA9685 External PWM Controller (Plan B)](#pca9685-external-pwm-controller-plan-b)
8. [Gotchas and Common Mistakes](#gotchas-and-common-mistakes)
9. [Recommendations for AMBOT](#recommendations-for-ambot)
10. [Sources](#sources)

---

## Executive Summary

**Key takeaway:** The Jetson Orin Nano does NOT support software PWM. Only hardware PWM is available, and only on specific pins that must be enabled via pinmux configuration. This is fundamentally different from RPi.GPIO which supports software PWM on any GPIO pin.

**Available hardware PWM pins on Orin Nano:**
| BOARD Pin | PWM Channel | Sysfs Path | Status |
|-----------|------------|------------|--------|
| Pin 15 | pwm (GPIO12) | `/sys/devices/3280000.pwm` | Works on some JetPack versions |
| Pin 33 | pwm5 (GPIO13) | `/sys/devices/32c0000.pwm` | Most reliable, confirmed working |
| Pin 32 | pwm7 | Varies | Problematic — many reports of failure |

**Bottom line:** Pin 33 is the most reliable PWM pin. Pin 32 has widespread issues. For L298N motor control (which needs 2 PWM channels for ENA/ENB), this is a problem. A PCA9685 I2C PWM controller is the recommended fallback.

---

## Hardware PWM on Jetson Orin Nano

### How It Works

The Jetson.GPIO library (`import RPi.GPIO as GPIO` or `import Jetson.GPIO as GPIO`) wraps the Linux sysfs PWM interface. When you call `GPIO.PWM(pin, freq)`, it:

1. Looks up which `pwmchip` device corresponds to the pin
2. Exports the PWM channel via `/sys/class/pwm/pwmchipN/export`
3. Writes period and duty_cycle to the sysfs files
4. The hardware PWM controller generates the signal

This means:
- The pinmux MUST be configured to route the PWM hardware to the pin
- If the pinmux is wrong, the pin acts as a simple GPIO (fully on/off, no modulation)
- The Jetson.GPIO library does NOT configure pinmux itself

### Pin 33 — The Reliable One

Pin 33 maps to `pwmchip` at `/sys/devices/platform/bus@0/32c0000.pwm`. This is the default PWM pin in NVIDIA's own sample code (`simple_pwm.py`). It works across JetPack 5.x and 6.x when properly configured.

### Pin 32 — Problematic

Multiple users report Pin 32 does not produce PWM output even after jetson-io configuration. The sysfs shows `duty: 0 ns` despite software requests. This appears to be a pinmux or device tree issue that varies by JetPack version. NVIDIA's GitHub has an open issue (#105) about this.

### Pin 15 — Orin-Specific Addition

Pin 15 is available as a PWM output on Orin Nano/NX (not on older Jetson Nano). Sysfs path: `/sys/devices/3280000.pwm`. Less community testing than Pin 33.

---

## Software PWM — Not Available

**The Jetson.GPIO library does NOT implement software-emulated PWM.** This is explicitly stated in NVIDIA's documentation.

Why not:
- The Tegra SoC runs a non-real-time OS (Ubuntu)
- Software PWM requires precise timing that Linux cannot guarantee
- Jitter from kernel scheduling would cause unreliable motor control
- RPi.GPIO's software PWM works "well enough" on Raspberry Pi but NVIDIA chose not to implement it

**This means you cannot do `GPIO.PWM()` on arbitrary GPIO pins.** Only the 2-3 pins with hardware PWM controllers work. Attempting PWM on other pins raises: `"Channel XX is not a PWM"`.

---

## Pinmux Configuration — Step by Step

### Method 1: jetson-io.py (Easiest, Sometimes Unreliable)

```bash
# Run the configuration tool
sudo /opt/nvidia/jetson-io/jetson-io.py
```

Interactive steps:
1. Select **"Configure Jetson 40pin Header"**
2. Select **"Configure header pins manually"**
3. Enable: **`[*] pwm5 (33)`** and **`[*] pwm7 (32)`**
4. Select **"Save pin changes"**
5. **Reboot**

After reboot, the tool generates a Device Tree Blob Overlay (DTBO) that is applied at boot.

**Verification after reboot:**
```bash
# Check PWM devices exist
ls /sys/class/pwm/

# Check debug info
sudo cat /sys/kernel/debug/pwm

# Verify specific PWM chip
ls /sys/devices/platform/bus@0/32c0000.pwm/
```

**Known issue:** jetson-io.py does not work reliably on all JetPack versions. Multiple users report it fails silently — the tool says it saved, but PWM does not work after reboot.

### Method 2: Reinstall Jetson.GPIO After jetson-io (NVIDIA-Recommended Fix)

NVIDIA staff confirmed this sequence works on JetPack 6.0:

```bash
# 1. Run jetson-io and enable PWM pins (as above)
sudo /opt/nvidia/jetson-io/jetson-io.py
# ... enable pwm5 (33) and pwm7 (32), save, reboot ...

# 2. After reboot, reinstall jetson-gpio from source
git clone https://github.com/NVIDIA/jetson-gpio.git
cd jetson-gpio
sudo python3 setup.py install

# 3. Test
sudo python3 samples/simple_pwm.py
```

The reinstall ensures the GPIO library picks up the new pin configuration.

### Method 3: Manual Device Tree Modification (Most Reliable, Most Complex)

For cases where jetson-io.py fails:

1. **Download the Pinmux spreadsheet** from NVIDIA Developer site for Orin Nano
2. **Convert current DTB to DTS:**
   ```bash
   dtc -I dtb -O dts -o current.dts /boot/dtb/kernel_tegra234-*.dtb
   ```
3. **Edit the DTS file** — find the PWM node and set status to "okay":
   ```
   pwm@32c0000 {
       status = "okay";
   };
   pwm@32e0000 {
       status = "okay";
   };
   ```
4. **Compile back to DTB:**
   ```bash
   dtc -I dts -O dtb -o modified.dtb modified.dts
   ```
5. **Install and reboot**
6. **Back up originals before modifying anything**

### Method 4: Busybox devmem (Jetson Nano Only — NOT for Orin)

The `busybox devmem` commands found in many tutorials are for the **original Jetson Nano** (Tegra X1), NOT the Orin Nano (Tegra T234). The register addresses are completely different:

```bash
# THESE ARE FOR JETSON NANO ONLY — DO NOT USE ON ORIN NANO
busybox devmem 0x700031fc 32 0x45   # Pin 32 / PWM0
busybox devmem 0x6000d504 32 0x2
busybox devmem 0x70003248 32 0x46   # Pin 33 / PWM2
busybox devmem 0x6000d100 32 0x00
```

**WARNING:** Using wrong devmem addresses on the wrong board can brick it or cause hardware damage.

---

## L298N Wiring with Jetson

### Pin Assignments (from JetBot L298N project)

| L298N Pin | Jetson Pin (BOARD) | Function |
|-----------|-------------------|----------|
| ENA | Pin 33 (PWM) | Left motor speed (PWM) |
| IN1 | Pin 35 (GPIO) | Left motor direction A |
| IN2 | Pin 36 (GPIO) | Left motor direction B |
| ENB | Pin 32 (PWM) or PCA9685 | Right motor speed (PWM) |
| IN3 | Pin 37 (GPIO) | Right motor direction A |
| IN4 | Pin 38 (GPIO) | Right motor direction B |
| GND | Pin 6 or 9 (GND) | Common ground |

### Power Notes

- L298N motor power (VCC/+12V terminal): External battery (6-12V), NOT from Jetson
- L298N 5V output: Do NOT use to power Jetson (insufficient current)
- Common ground between Jetson and L298N is required
- L298N logic level is 5V tolerant; Jetson GPIO is 3.3V — this works fine for input

### The Two-PWM Problem

The L298N needs 2 PWM signals (ENA + ENB) for independent speed control of left and right motors. The Orin Nano has Pin 33 (reliable) and Pin 32 (unreliable). Options:

1. **Try both pins** — Pin 32 works for some users; test on your specific board/JetPack
2. **Use PCA9685** for PWM, Jetson GPIO for direction pins only
3. **Tie ENB high** (always full speed on one side) and vary only ENA — crude but works for basic testing
4. **Use a different motor driver** that takes digital direction + single PWM (e.g., DRV8833, TB6612FNG which can be driven with direction pins only at full speed)

---

## Working Code Examples

### NVIDIA Official PWM Sample (Pin 33)

```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO  # Actually Jetson.GPIO, aliased as RPi.GPIO
import time

# Jetson Orin Nano default PWM pin
output_pin = 33  # BOARD numbering

GPIO.setmode(GPIO.BOARD)
GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
p = GPIO.PWM(output_pin, 50)  # 50 Hz
p.start(25)  # 25% duty cycle

try:
    val = 25
    incr = 5
    while True:
        time.sleep(0.25)
        if val >= 100:
            incr = -incr
        if val <= 0:
            incr = -incr
        val += incr
        p.ChangeDutyCycle(val)
finally:
    p.stop()
    GPIO.cleanup()
```

### L298N Motor Control (Adapted from JetBot Project)

```python
#!/usr/bin/env python3
"""
L298N motor control on Jetson Orin Nano.
Requires: pinmux configured for PWM on pin 33 (and pin 32 if available).
"""
import RPi.GPIO as GPIO
import time

# BOARD pin numbering
LEFT_PWM = 33    # ENA — hardware PWM
LEFT_IN1 = 35    # Direction A
LEFT_IN2 = 36    # Direction B

RIGHT_PWM = 32   # ENB — hardware PWM (may not work on all boards)
RIGHT_IN3 = 37   # Direction A
RIGHT_IN4 = 38   # Direction B

GPIO.setmode(GPIO.BOARD)

# Setup direction pins
for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4]:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

# Setup PWM pins
GPIO.setup(LEFT_PWM, GPIO.OUT)
GPIO.setup(RIGHT_PWM, GPIO.OUT)

pwm_left = GPIO.PWM(LEFT_PWM, 1000)   # 1 kHz for motors
pwm_right = GPIO.PWM(RIGHT_PWM, 1000)
pwm_left.start(0)
pwm_right.start(0)

def forward(speed=50):
    """speed: 0-100 duty cycle"""
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def stop():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4]:
        GPIO.output(pin, GPIO.LOW)

try:
    forward(40)
    time.sleep(2)
    stop()
finally:
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
```

### PCA9685 + L298N (Reliable Multi-Channel PWM)

```python
#!/usr/bin/env python3
"""
PCA9685 I2C PWM controller driving L298N motor driver.
Jetson Orin Nano I2C Bus 7: Pin 3 (SDA), Pin 5 (SCL).
PCA9685 default address: 0x40.
"""
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO
import time

# PCA9685 setup (I2C)
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # 1 kHz for DC motors

# PCA9685 channels for PWM speed control
CH_LEFT_PWM = 0   # ENA
CH_RIGHT_PWM = 1  # ENB

# Jetson GPIO for direction (no PWM needed)
LEFT_IN1 = 35
LEFT_IN2 = 36
RIGHT_IN3 = 37
RIGHT_IN4 = 38

GPIO.setmode(GPIO.BOARD)
for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4]:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

def set_speed(channel, percent):
    """Convert 0-100% to 16-bit duty cycle."""
    pca.channels[channel].duty_cycle = int(percent / 100 * 0xFFFF)

def forward(speed=50):
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    set_speed(CH_LEFT_PWM, speed)
    set_speed(CH_RIGHT_PWM, speed)

def stop():
    set_speed(CH_LEFT_PWM, 0)
    set_speed(CH_RIGHT_PWM, 0)

try:
    forward(40)
    time.sleep(2)
    stop()
finally:
    pca.deinit()
    GPIO.cleanup()
```

**Install dependencies for PCA9685:**
```bash
pip3 install adafruit-circuitpython-pca9685 adafruit-blinka
```

---

## PCA9685 External PWM Controller (Plan B)

### Why PCA9685?

- Provides **16 independent PWM channels** via I2C (only needs 2 Jetson pins)
- Built-in oscillator — no CPU involvement for PWM timing
- Works reliably regardless of pinmux configuration
- Widely used in robotics (Adafruit, JetBot, ROS projects)
- ~$3-5 for a breakout board

### Wiring to Jetson Orin Nano

| PCA9685 | Jetson Pin (BOARD) | I2C Bus |
|---------|-------------------|---------|
| SDA | Pin 3 | Bus 7 |
| SCL | Pin 5 | Bus 7 |
| VCC | Pin 1 (3.3V) or Pin 2 (5V) | - |
| GND | Pin 6 or 9 | - |
| V+ | External motor power (6-12V) | - |

**Verify I2C connection:**
```bash
sudo i2cdetect -y -r 7
# Should show device at 0x40
```

### When to Use PCA9685

- You need more than 1 reliable PWM channel
- Pin 32 does not work on your board
- You want to avoid pinmux headaches
- Future expansion (servos, more motors)

---

## Gotchas and Common Mistakes

### 1. No Software PWM — Period
Unlike RPi.GPIO, you CANNOT do `GPIO.PWM()` on arbitrary pins. Only hardware PWM pins (15, 32, 33) work, and only after pinmux configuration.

### 2. busybox devmem Addresses Are Board-Specific
The `busybox devmem 0x700031fc...` commands all over the internet are for the **original Jetson Nano (Tegra X1)**. The Orin Nano uses a completely different SoC (T234) with different register addresses. Using wrong addresses can damage hardware.

### 3. jetson-io.py May Fail Silently
The tool says "saved" but PWM may not work. Always verify with:
```bash
ls /sys/class/pwm/
sudo cat /sys/kernel/debug/pwm
```

### 4. Reinstall Jetson.GPIO After Pinmux Changes
NVIDIA staff explicitly recommends reinstalling the library from GitHub after using jetson-io.py. The packaged version may not pick up the new pin configuration.

### 5. Pin 32 Is Unreliable
Multiple forum threads, GitHub issues, and blog posts report Pin 32 not working. This is an ongoing issue across JetPack versions. Do not depend on it without testing first.

### 6. PWM Frequency for Motors
- Servos: 50 Hz
- DC motors (L298N): 500-2000 Hz (1000 Hz is a good default)
- Higher frequencies reduce audible whine but may reduce torque at low duty cycles

### 7. 3.3V GPIO vs 5V Logic
Jetson GPIO outputs 3.3V. The L298N accepts this fine for logic inputs. However, some cheap motor drivers may need 5V logic — check the datasheet.

### 8. Permissions
PWM sysfs access requires root or proper udev rules:
```bash
# Add user to gpio group
sudo usermod -aG gpio $USER
# May also need:
sudo groupadd -f -r gpio
sudo chown root:gpio /sys/class/pwm/*/export /sys/class/pwm/*/unexport
```

### 9. The Jetson.GPIO Import Alias
The library can be imported as either:
```python
import Jetson.GPIO as GPIO
# or
import RPi.GPIO as GPIO  # Compatibility alias
```
Both work identically on Jetson. The RPi alias helps with code portability.

---

## Recommendations for AMBOT

### Recommended Approach: Hybrid (GPIO Direction + PCA9685 PWM)

Given that:
- Our L298N needs 2 independent PWM channels
- Pin 32 is unreliable on Orin Nano
- We already have I2C working (MPU6050 on the RPi uses I2C bus 1)
- We want reliability over simplicity

**Plan A — Try native PWM first:**
1. Run `jetson-io.py` to enable pwm5 (pin 33) and pwm7 (pin 32)
2. Reinstall jetson-gpio from GitHub
3. Test both pins with `simple_pwm.py` (modified to test each pin)
4. If both work, use direct GPIO PWM (simplest wiring, no extra hardware)

**Plan B — PCA9685 fallback:**
1. Get a PCA9685 breakout board (~$3-5)
2. Connect via I2C Bus 7 (Pin 3 SDA, Pin 5 SCL)
3. Use PCA9685 for all PWM (ENA, ENB)
4. Use Jetson GPIO for direction pins (IN1-IN4) — no PWM needed
5. Install `adafruit-circuitpython-pca9685` and `adafruit-blinka`

**Plan C — Single PWM workaround:**
1. Use Pin 33 (reliable) for one motor's PWM
2. Tie the other motor's enable pin HIGH (always full speed)
3. Control differential steering via duty cycle on one side + full on the other
4. Crude but requires zero extra hardware

### Driver Architecture Note

The existing `drivers.py` motor abstraction should work with minor changes. The PWM interface (`start()`, `ChangeDutyCycle()`, `stop()`) is the same whether using Jetson.GPIO hardware PWM or PCA9685 channels. The adapter pattern in `RobotAdapter` already handles float-to-int conversion.

For PCA9685, we would need a thin wrapper class that mimics the `GPIO.PWM` interface but writes to PCA9685 channels underneath. This keeps the existing motor driver code unchanged.

---

## Sources

### NVIDIA Official
- [NVIDIA/jetson-gpio — GitHub](https://github.com/NVIDIA/jetson-gpio) — Official GPIO library, PWM samples
- [simple_pwm.py sample](https://github.com/NVIDIA/jetson-gpio/blob/master/samples/simple_pwm.py) — Official PWM example code
- [Jetson Orin Nano GPIO Header Pinout — JetsonHacks](https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/) — Pin numbers, I2C buses, PWM sysfs paths
- [NVIDIA Jetson Linux Developer Guide — Orin NX/Nano Series](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html) — Pinmux configuration reference

### NVIDIA Developer Forums
- [PWM can't be set on Jetson expansion header (Orin Nano JetPack 6)](https://forums.developer.nvidia.com/t/pwm-cant-be-set-on-jetson-expansion-header-orin-nano-jetpack-6/284883) — NVIDIA staff confirmed fix: jetson-io + reinstall gpio library
- [Issue with PWM Pins 33 and 32 Not Functioning](https://forums.developer.nvidia.com/t/issue-with-pwm-pins-on-jetson-orin-nano-gpio-pins-33-and-32-not-functioning/280659) — Pin 32 issues, unresolved
- [Python controls GPIO errors](https://forums.developer.nvidia.com/t/python-controls-gpio-errors/260621) — General GPIO troubleshooting
- [Issue with PWM with Jetson.GPIO and Orin Nano](https://forums.developer.nvidia.com/t/issue-with-pwm-with-jetson-gpio-and-orin-nano/261564) — PWM debugging

### GitHub Issues
- [Jetson Orin Nano PWM only works on Pin 33 — Issue #105](https://github.com/NVIDIA/jetson-gpio/issues/105) — Pin 32 confirmed broken for some users

### Tutorials and Guides
- [JetBot using L298N PWM Motor — Medium](https://medium.com/@anandmohan_8991/jetbot-using-l298n-pwm-motor-a6556ed358d6) — L298N wiring, busybox devmem (Nano-specific)
- [Integrate PWM driver for DC motors with Jetson Nano — Medium](https://medium.com/@a.vanmalleghem/integrate-pwm-driver-for-dc-motors-with-jetson-nano-13ead585069a) — Comparison of 4 PWM enable methods
- [Configuring PWM on Jetson Orin Nano — Piveral](https://nvidia-jetson.piveral.com/jetson-orin-nano/configuring-pwm-on-jetson-orin-nano-developer-kit/) — Troubleshooting guide
- [Configuring 6 PWM Motor Control Outputs — Piveral](https://nvidia-jetson.piveral.com/jetson-orin-nano/configuring-6-pwm-motor-control-outputs-on-nvidia-jetson-orin-nano-dev-board/) — PCA9685 approach
- [Changing PINMUX Configuration in Device Tree — Piveral](https://nvidia-jetson.piveral.com/jetson-orin-nano/changing-pinmux-configuration-in-jetson-orin-nano-device-tree/) — Manual DTS editing
- [Building a Custom JetBot — Hackster.io](https://www.hackster.io/gatoninja236/building-a-custom-jetbot-with-jetson-nano-8d41b6) — Full JetBot build with L298N

### PCA9685 / External PWM
- [Motor-Driver-L298N-and-PCA9685 — GitHub](https://github.com/custom-build-robots/Motor-Driver-L298N-and-PCA9685) — Python code for PCA9685 + L298N combo
- [Jetson Nano Using I2C — JetsonHacks](https://jetsonhacks.com/2019/07/22/jetson-nano-using-i2c/) — I2C setup reference
- [I2C with PCA9685 on Orin Nano — NVIDIA Forums](https://forums.developer.nvidia.com/t/i2c-with-pca9685/293393) — Orin Nano specific I2C setup
