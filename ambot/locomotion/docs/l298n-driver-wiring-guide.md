# L298N Motor Driver Wiring Guide for Raspberry Pi

Complete pinout and wiring guide for connecting an L298N dual H-bridge motor driver to a Raspberry Pi using a 4xAA battery pack (~6V).

## Overview

The L298N is a dual H-bridge motor driver based on BJT (bipolar junction transistor) technology. While less efficient than modern MOSFET-based drivers like the TB6612FNG, it remains popular due to its low cost, wide availability, and extensive documentation.

## L298N Specifications

| Parameter | Value |
|-----------|-------|
| Motor Voltage (VS) | 5V - 35V |
| Logic Voltage (VSS) | 5V (onboard regulator) or 3.3V (direct) |
| Continuous Current | 2A per channel |
| Peak Current | 3.5A per channel |
| Voltage Drop | 2V - 2.8V (BJT losses) |
| Logic Input | 5V tolerant, works with 3.3V |

**Important**: With a 4xAA battery pack providing ~6V, the motor will only see approximately **3.2-4V** after the L298N voltage drop. This is normal but limits motor power.

---

## Physical Wiring Diagram (Text-Based)

```
                    L298N MODULE (Top View)
    +=======================================================+
    |                                                       |
    |   [Motor A-]  [Motor A+]     [Motor B+]  [Motor B-]   |
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
            |        |      |        |    |    |    |    |    |
    POWER INPUTS    OUTPUT      CONTROL PINS (to Raspberry Pi)


    RASPBERRY PI 40-PIN HEADER (Relevant Pins)
    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    | 2| 4| 6| 8|10|12|14|16|18|20|22|24|26|28|30|32|34|36|38|40|
    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    | 1| 3| 5| 7| 9|11|13|15|17|19|21|23|25|27|29|31|33|35|37|39|
    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+

    Key Pi Pins Used:
    Pin 2  (5V)     - Can power L298N logic (alternative)
    Pin 6  (GND)    - Common ground **CRITICAL**
    Pin 11 (GPIO17) - IN1 (Motor A direction)
    Pin 13 (GPIO27) - IN2 (Motor A direction)
    Pin 15 (GPIO22) - IN3 (Motor B direction)
    Pin 16 (GPIO23) - IN4 (Motor B direction)
    Pin 32 (GPIO12) - ENA (Motor A speed - Hardware PWM0)
    Pin 33 (GPIO13) - ENB (Motor B speed - Hardware PWM1)
```

### Wiring Connections Overview

```
    4xAA BATTERY PACK (~6V)                RASPBERRY PI
    ==================                     =============
         (+) ─────────────────────┐
                                  │
                                  ▼
                            [L298N +12V/VS]

         (-) ─────────────────────┬────────────── [GND Pin 6]
                                  │
                                  ▼
                            [L298N GND]

                            [L298N 5V] ───────── (Leave disconnected
                                                  OR use to power Pi
                                                  if jumper installed)

    MOTOR A (Left)              MOTOR B (Right)
    ==============              ===============
    Wire 1 ──── [OUT1]          Wire 1 ──── [OUT3]
    Wire 2 ──── [OUT2]          Wire 2 ──── [OUT4]
```

---

## Pin-by-Pin Connection Tables

### L298N Power Connections

| L298N Pin | Connect To | Description |
|-----------|------------|-------------|
| **+12V / VS** | Battery Pack (+) | Motor power supply (4xAA = ~6V) |
| **GND** | Battery Pack (-) AND Raspberry Pi GND | **CRITICAL: Common ground** |
| **+5V** | Leave disconnected | Output when jumper installed (can power Pi) |

**Note on 5V Jumper**: The L298N has a jumper near the +12V input. When installed with VS > 7V, the onboard regulator provides 5V output. With a 6V battery pack, remove this jumper - the voltage is too low for the regulator.

### L298N to Raspberry Pi Control Connections

| L298N Pin | Function | RPi Physical Pin | BCM GPIO | Wire Color (Suggested) |
|-----------|----------|------------------|----------|------------------------|
| **ENA** | Motor A PWM speed | Pin 32 | GPIO12 | Orange |
| **IN1** | Motor A direction 1 | Pin 11 | GPIO17 | Yellow |
| **IN2** | Motor A direction 2 | Pin 13 | GPIO27 | Green |
| **IN3** | Motor B direction 1 | Pin 15 | GPIO22 | Blue |
| **IN4** | Motor B direction 2 | Pin 16 | GPIO23 | Purple |
| **ENB** | Motor B PWM speed | Pin 33 | GPIO13 | White |
| **GND** | Common ground | Pin 6 | GND | Black |

### Raspberry Pi 40-Pin Header Reference (Pins Used)

| Physical Pin | Name | Connected To |
|--------------|------|--------------|
| 6 | GND | L298N GND (common ground) |
| 11 | GPIO17 | L298N IN1 |
| 13 | GPIO27 | L298N IN2 |
| 15 | GPIO22 | L298N IN3 |
| 16 | GPIO23 | L298N IN4 |
| 32 | GPIO12 (PWM0) | L298N ENA |
| 33 | GPIO13 (PWM1) | L298N ENB |

### Motor Output Connections

| L298N Output | Connect To |
|--------------|------------|
| **OUT1** | Motor A terminal 1 |
| **OUT2** | Motor A terminal 2 |
| **OUT3** | Motor B terminal 1 |
| **OUT4** | Motor B terminal 2 |

**Note**: Motor wire polarity determines rotation direction. If a motor spins backwards, swap its two wires.

---

## Power Supply Requirements

### Your Setup: 4xAA Battery Pack (~6V)

| Component | Requirement | Your Battery Pack | Status |
|-----------|-------------|-------------------|--------|
| L298N VS minimum | 5V | ~6V (fresh) / ~4.8V (depleted) | Marginal |
| L298N VS maximum | 35V | 6V | Safe |
| Motor voltage after drop | - | 3.2-4V | Reduced power |

### Power Calculations

```
Fresh 4xAA alkaline:  4 x 1.5V = 6.0V
L298N voltage drop:   -2.0V to -2.8V
Motor sees:           3.2V to 4.0V

Depleted batteries:   4 x 1.2V = 4.8V
L298N voltage drop:   -2.0V
Motor sees:           2.8V (may not spin)
```

### Recommendations for 6V Battery Pack

1. **Consider NiMH rechargeable AA cells** - More consistent voltage under load
2. **Monitor battery voltage** - Performance degrades significantly below 5.5V total
3. **For more power, upgrade to 6xAA pack** (9V) or a 2S LiPo (7.4V nominal)
4. **Remove the 5V regulator jumper** - Not enough voltage for the onboard regulator

### Power Wiring Diagram

```
    4xAA Battery Pack
    +================+
    |  (+) ──────────┼──────────────────────► L298N [+12V/VS]
    |                |
    |  (-) ──────────┼───────┬──────────────► L298N [GND]
    +================+       │
                             │
                             └──────────────► Raspberry Pi [GND Pin 6]

    Raspberry Pi (powered separately via USB-C or dedicated supply)
```

**CRITICAL**: The battery ground MUST connect to BOTH the L298N GND AND the Raspberry Pi GND. Without a common ground, the control signals will not work.

---

## Common Mistakes to Avoid

### 1. Missing Common Ground (Most Common Problem)

**Symptom**: Motors don't respond to any commands.

**Cause**: L298N GND not connected to Raspberry Pi GND.

**Fix**: Run a wire from L298N GND to any Raspberry Pi GND pin (6, 9, 14, 20, 25, 30, 34, or 39).

### 2. ENA/ENB Jumpers Left In Place

**Symptom**: Motors always run at full speed, PWM has no effect.

**Cause**: The L298N ships with jumpers on ENA and ENB pins, shorting them to 5V.

**Fix**: Remove both jumpers and connect ENA/ENB to Pi PWM pins (GPIO12, GPIO13).

### 3. 5V Regulator Jumper Issues

**Symptom**: L298N overheats or 5V output doesn't work.

**Cause**: Using 6V supply with 5V regulator jumper installed.

**Fix**: Remove the jumper next to +12V input when using supplies under 7V.

### 4. Incorrect Pin Numbering

**Symptom**: Motors behave erratically or opposite of expected.

**Cause**: Confusing BCM GPIO numbers with physical pin numbers.

**Fix**:
- Physical Pin 11 = GPIO17 (not GPIO11)
- Physical Pin 13 = GPIO27 (not GPIO13)
- Always double-check using `pinout` command on Pi

### 5. Powering Motors from Raspberry Pi 5V

**Symptom**: Raspberry Pi reboots, behaves erratically, or won't boot.

**Cause**: Drawing motor current through Pi's 5V pin.

**Fix**: Always use a separate battery pack for motors. Never connect L298N VS to Pi 5V.

### 6. No Flyback Protection on Motor Connections

**Symptom**: Erratic behavior, GPIO damage over time.

**Cause**: Motor inductive kickback when switching.

**Fix**: The L298N has built-in flyback diodes, but add external ones for long motor leads.

### 7. Reversed Motor Wires Causing Confusion

**Symptom**: Motor spins opposite direction than expected.

**Cause**: Motor wire polarity determines direction.

**Fix**: This is not a wiring error - simply swap the motor wires at OUT1/OUT2 or OUT3/OUT4, OR swap IN1/IN2 or IN3/IN4 logic in software.

### 8. Using Software PWM Instead of Hardware PWM

**Symptom**: Motors whine, stutter, or have inconsistent speed.

**Cause**: Software PWM has timing jitter.

**Fix**: Use GPIO12 (PWM0) and GPIO13 (PWM1) for hardware PWM support.

### 9. Battery Voltage Too Low

**Symptom**: Motors barely turn or stall easily.

**Cause**: After L298N's 2V+ drop, insufficient voltage reaches motors.

**Fix**: Use fresh batteries or upgrade to higher voltage pack (6xAA = 9V).

### 10. Loose Screw Terminal Connections

**Symptom**: Intermittent operation, motors cut out during movement.

**Cause**: Wires not secured in L298N screw terminals.

**Fix**: Strip 5-7mm of wire, insert fully, and tighten screws firmly.

---

## Step-by-Step Connection Instructions

### Prerequisites

- Raspberry Pi (any model with 40-pin header)
- L298N motor driver module
- 4xAA battery holder with leads
- 2x DC motors
- Jumper wires (female-to-female for Pi, stripped ends for L298N)
- Small flathead screwdriver for screw terminals

### Step 1: Prepare the L298N Module

1. **Remove the ENA and ENB jumpers**
   - Locate the two small jumpers near the ENA and ENB pins
   - Remove both jumpers and set aside (or discard)
   - This allows PWM speed control from the Raspberry Pi

2. **Check the 5V regulator jumper**
   - Locate the jumper near the +12V input terminal
   - **With 6V battery pack**: Remove this jumper
   - This jumper only works with supplies > 7V

### Step 2: Connect Power Supply (Battery Pack)

1. **Connect battery positive (+)**
   - Strip ~5mm of the red battery wire
   - Insert into the **+12V/VS** screw terminal on L298N
   - Tighten the screw firmly

2. **Connect battery negative (-)**
   - Strip ~5mm of the black battery wire
   - Insert into the **GND** screw terminal (middle terminal)
   - Tighten the screw firmly

**Do not insert batteries yet** - complete all wiring first.

### Step 3: Connect Common Ground to Raspberry Pi

1. Take a female-to-bare jumper wire (or strip one end of a female-female wire)
2. Insert the stripped end into the **GND** terminal on L298N (share with battery ground)
3. Connect the female end to **Raspberry Pi Pin 6** (GND)

```
This is the most critical connection - signals won't work without common ground!
```

### Step 4: Connect Motor Control Pins

**Motor A (typically left motor):**

| Wire | From L298N | To RPi Pin |
|------|------------|------------|
| 1 | ENA | Pin 32 (GPIO12) |
| 2 | IN1 | Pin 11 (GPIO17) |
| 3 | IN2 | Pin 13 (GPIO27) |

**Motor B (typically right motor):**

| Wire | From L298N | To RPi Pin |
|------|------------|------------|
| 4 | IN3 | Pin 15 (GPIO22) |
| 5 | IN4 | Pin 16 (GPIO23) |
| 6 | ENB | Pin 33 (GPIO13) |

**Wiring technique:**
- Use female-to-female jumper wires
- One end connects to the L298N header pins
- Other end connects to the corresponding Pi GPIO pin

### Step 5: Connect Motors to Output Terminals

1. **Motor A (left)**
   - Connect motor wire 1 to **OUT1** terminal
   - Connect motor wire 2 to **OUT2** terminal
   - Tighten both screws

2. **Motor B (right)**
   - Connect motor wire 1 to **OUT3** terminal
   - Connect motor wire 2 to **OUT4** terminal
   - Tighten both screws

**Note**: Don't worry about polarity yet - you can swap wires later if motor direction is reversed.

### Step 6: Final Verification Checklist

Before powering on, verify:

- [ ] Battery (+) connected to L298N +12V/VS
- [ ] Battery (-) connected to L298N GND
- [ ] L298N GND connected to Raspberry Pi GND (Pin 6)
- [ ] ENA jumper removed, ENA connected to Pi Pin 32
- [ ] ENB jumper removed, ENB connected to Pi Pin 33
- [ ] IN1 connected to Pi Pin 11
- [ ] IN2 connected to Pi Pin 13
- [ ] IN3 connected to Pi Pin 15
- [ ] IN4 connected to Pi Pin 16
- [ ] Motor A wires in OUT1 and OUT2
- [ ] Motor B wires in OUT3 and OUT4
- [ ] All screw terminals tight
- [ ] No loose or touching wires

### Step 7: Power On and Test

1. Insert batteries into the battery holder
2. The L298N power LED should illuminate (if equipped)
3. Boot the Raspberry Pi
4. Run a simple test script:

```python
#!/usr/bin/env python3
"""L298N Motor Test - tests each motor direction."""

import RPi.GPIO as GPIO
import time

# Pin definitions (BCM numbering)
ENA = 12  # Motor A PWM
IN1 = 17  # Motor A direction
IN2 = 27  # Motor A direction
ENB = 13  # Motor B PWM
IN3 = 22  # Motor B direction
IN4 = 23  # Motor B direction

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([ENA, IN1, IN2, ENB, IN3, IN4], GPIO.OUT)

# Create PWM objects
pwm_a = GPIO.PWM(ENA, 1000)  # 1kHz frequency
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

    print("Test complete!")

finally:
    stop_all()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
```

---

## Control Logic Truth Table

| IN1 | IN2 | ENA (PWM) | Motor A Behavior |
|-----|-----|-----------|------------------|
| HIGH | LOW | 0-100% | Forward at PWM speed |
| LOW | HIGH | 0-100% | Backward at PWM speed |
| HIGH | HIGH | Any | Brake (motor locked) |
| LOW | LOW | Any | Coast (motor free) |
| Any | Any | 0% | Stopped |

Same logic applies to IN3/IN4/ENB for Motor B.

---

## Troubleshooting

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| No motor response | Missing ground connection | Connect L298N GND to Pi GND |
| Motors always full speed | ENA/ENB jumpers still installed | Remove jumpers |
| One motor works, other doesn't | Wiring error on that channel | Check IN3/IN4/ENB connections |
| Motors spin wrong direction | Normal - depends on wiring | Swap motor wires or invert logic |
| Motors weak/slow | Voltage drop + low battery | Use higher voltage or fresh batteries |
| Pi reboots when motors run | Power drawing through Pi | Use separate motor power supply |
| Erratic behavior | Loose connections | Check all screw terminals |

---

## References

- [L298N Datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
- See also: [TB6612FNG Driver Guide](tb6612fng-driver.md) for a more efficient alternative
- See also: [Motor Driver Comparison](motor-drivers-comparison.md)
