# Jetson Motor Debug Log

**Date:** 2026-03-26 (Session 22)
**Status:** Motors NOT spinning despite all code running without errors

## What We Know

### Software — All Working
- Jetson.GPIO 2.1.7 detects Orin Nano correctly
- `GPIO.setup(pin, GPIO.OUT)` completes without error on all 6 pins
- `GPIO.output(pin, HIGH)` completes without error
- `GPIO.PWM(pin, 1000).start(70)` completes without error
- Running with `sudo` makes no difference
- Pin 32 (PBB.01) was stuck in INPUT mode → fixed with `sudo gpioset gpiochip1 9=1`
- After fix, `gpioinfo` confirms Pin 32 is OUTPUT
- 5 of 6 pins were always in OUTPUT mode

### Hardware — Confirmed
- Battery/power supply connected to L298N
- GND wire connects Jetson GND (pin 6) to L298N GND
- ENA/ENB jumpers are ON (motors should be full-speed when direction pins set)
- Motors connected to OUT1-4 on L298N
- Same L298N + motors + power supply worked on RPi

### Pin Mapping Used
| L298N | Jetson BOARD | GPIO Chip | Line | Tegra | State |
|-------|:-----------:|-----------|:----:|-------|-------|
| ENA | 32 | gpiochip1 | 9 | PBB.01 | OUTPUT (after fix) |
| IN1 | 29 | gpiochip0 | 144 | PAC.06 | OUTPUT |
| IN2 | 31 | gpiochip1 | 15 | PCC.03 | OUTPUT |
| ENB | 33 | gpiochip0 | 105 | PQ.05 | OUTPUT |
| IN3 | 7 | gpiochip1 | 12 | PCC.00 | OUTPUT |
| IN4 | 13 | gpiochip0 | 43 | PH.00 | OUTPUT |

## ACTUAL Root Cause: Pinmux Not Configured for GPIO

**All 40-pin header pins default to their alternate functions (SPI, clock, audio, etc.),
NOT GPIO.** The `gpioinfo` command shows pins as "output" but the hardware pinmux routes
them to non-GPIO peripherals. Software (Jetson.GPIO, gpiod, gpioset) all silently
"succeed" but no voltage appears on the physical pins.

**Diagnosis evidence:**
- `Jetson.GPIO.output(pin, HIGH)` → no error, but `gpioget` reads 0
- `gpiod request_lines(OUTPUT, ACTIVE)` → no error, but `gpioget` reads 0
- `gpioset gpiochip1 9=1` → no error, but `gpioget` reads 0
- Even a known "output" pin (PBB.03) reads 0 when set HIGH
- `config-by-function.py -l enabled` → "No functions are enabled" on header

**Fix applied (Session 22):**
1. Attempted DT overlay via jetson-io → overlay didn't load (OVERLAYS not supported in JetPack 6)
2. Merged overlay into DTB via `fdtoverlay` → changed model string, broke Jetson.GPIO detection
3. Reverted to original DTB → Jetson.GPIO works, `gpioinfo` confirms ALL 6 pins as output [used]
4. `gpioget` returns empty because Jetson.GPIO holds the lines (expected behavior)

**Final GPIO state (confirmed working):**
```
line  41: "PG.06"  "Jetson-gpio" output active-high [used]   ← BOARD 32 / ENA
line  43: "PH.00"  "Jetson-gpio" output active-high [used]   ← BOARD 33 / ENB
line 105: "PQ.05"  "Jetson-gpio" output active-high [used]   ← BOARD 29 / IN1
line 106: "PQ.06"  "Jetson-gpio" output active-high [used]   ← BOARD 31 / IN2
line 122: "PY.00"  "Jetson-gpio" output active-high [used]   ← BOARD 13 / IN4
line 144: "PAC.06" "Jetson-gpio" output active-high [used]   ← BOARD  7 / IN3
```

**Motors still don't spin → suspected dead batteries.** GPIO confirmed driving, L298N + motors
known good from RPi testing. User will try fresh batteries.

**Correct gpiochip0 line mapping (ALL pins on tegra234-gpio):**
| BOARD | Tegra | gpiochip0 line | L298N |
|:-----:|-------|:--------------:|-------|
| 7 | PAC.06 | 144 | IN3 |
| 13 | PY.00 | 122 | IN4 |
| 29 | PQ.05 | 105 | IN1 |
| 31 | PQ.06 | 106 | IN2 |
| 32 | PG.06 | 41 | ENA |
| 33 | PH.00 | 43 | ENB |

**Key lesson:** The gpiod driver agent used incorrect chip/line mapping from `gpioinfo`
(which showed PBB/PCC names on gpiochip1). The authoritative source is
`Jetson.GPIO/gpio_pin_data.py` which shows ALL 40-pin header pins are on `tegra234-gpio`
(gpiochip0). The Jetson.GPIO library (BOARD mode) handles the mapping correctly.

**Original extlinux.conf backed up to:** `/boot/extlinux/extlinux.conf.bak`

---

## Previous Theory: ENA/ENB Jumpers Removed

The ENA and ENB jumpers were **removed** from the L298N and those pins connected to
the Jetson GPIO (pins 32 and 33). This means the enable pins are being driven by GPIO.

However, Jetson GPIO Pin 32 (PBB.01) was stuck in **input mode** (pinmux issue), meaning
ENA was never actually going HIGH → Motor A got no power.

Even after fixing Pin 32's pinmux, the motors still didn't spin. Testing with `sudo`
and with `gpioset` directly also failed. The user tested touching IN1 to Pin 1 (3.3V)
but without connecting IN2 to GND — so no current path through the H-bridge.

**Solution**: Put ENA/ENB jumpers back ON for initial testing. This makes enable always
HIGH (full speed from L298N 5V regulator). Then only the 4 direction pins need GPIO
control. Once direction is confirmed working, remove jumpers and use PWM for speed.

## Previous Theory: Physical Pin Location Mismatch

The Jetson.GPIO library reports success for all operations, and `gpioinfo` confirms pins
are in output mode. But motors don't spin. This strongly suggests **the physical jumper
wires are not on the correct header pins**.

The Jetson 40-pin header is physically identical to RPi, but the user may have:
1. Miscounted pin positions (Pin 1 is not always where you expect)
2. Connected to the wrong row (odd vs even pins)
3. Connected to the wrong end of the header

## Verification Steps (need multimeter or LED)

### Step 1: Confirm Header Orientation
- Measure voltage between **Pin 1** and **Pin 6**
- Should read **3.3V** (Pin 1 = 3.3V power, Pin 6 = GND)
- If you read 0V or 5V, pin counting is wrong

### Step 2: Confirm GPIO Output
While running the motor test script:
- Measure voltage on the **IN1 jumper wire** (at the L298N end) vs L298N GND
- Should read **~3.3V** when script says pins are HIGH
- If 0V → wire is on the wrong Jetson pin

### Step 3: Simplest Possible Test
1. Disconnect all Jetson GPIO wires from L298N
2. Put ENA and ENB jumpers back ON
3. Touch IN1 wire directly to **Jetson Pin 1** (3.3V power)
4. Touch IN2 wire to **Jetson Pin 6** (GND)
5. Motor A should spin
6. If it does → L298N is fine, problem is GPIO pin locations

### Step 4: Locate Exact Pin Positions
Pin 1 on the Jetson Orin Nano carrier board is:
- Closest to the board edge
- Marked on the PCB silkscreen
- Top-left when looking at the board with USB ports facing you (check specific carrier)
- See photo: `imgs/jetson-orin-nano-40pin-header-pinout.png`
