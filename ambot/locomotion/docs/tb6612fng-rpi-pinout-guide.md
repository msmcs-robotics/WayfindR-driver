# TB6612FNG Motor Driver - Raspberry Pi Pinout Wiring Guide

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

Complete wiring reference for connecting the SparkFun TB6612FNG dual motor driver to a Raspberry Pi 40-pin header.

## Table of Contents

1. [Physical Wiring Diagram](#physical-wiring-diagram)
2. [Pin-by-Pin Connections](#pin-by-pin-connections)
3. [Power Supply Requirements](#power-supply-requirements)
4. [STBY Pin - Critical Information](#stby-pin---critical-information)
5. [Step-by-Step Connection Instructions](#step-by-step-connection-instructions)
6. [Wiring Verification Checklist](#wiring-verification-checklist)

---

## Physical Wiring Diagram

### Text-Based Wiring Schematic

```
                    RASPBERRY PI 40-PIN HEADER
                    ===========================

    3.3V Power [1]  [2]  5V
         GPIO2 [3]  [4]  5V
         GPIO3 [5]  [6]  GND  ─────────────────────┐
         GPIO4 [7]  [8]  GPIO14                    │
           GND [9]  [10] GPIO15                    │
  STBY─ GPIO17 [11] [12] GPIO18                    │
  AIN1─ GPIO27 [13] [14] GND                       │
  AIN2─ GPIO22 [15] [16] GPIO23 ─BIN1              │
          3.3V [17] [18] GPIO24 ─BIN2              │
        GPIO10 [19] [20] GND                       │
         GPIO9 [21] [22] GPIO25                    │
        GPIO11 [23] [24] GPIO8                     │
           GND [25] [26] GPIO7                     │
         GPIO0 [27] [28] GPIO1                     │
         GPIO5 [29] [30] GND                       │
         GPIO6 [31] [32] GPIO12 ─PWMB (HW PWM0)    │
  PWMA─ GPIO13 [33] [34] GND                       │
        GPIO19 [35] [36] GPIO16                    │
        GPIO26 [37] [38] GPIO20                    │
           GND [39] [40] GPIO21                    │
                                                   │
                                                   │
            TB6612FNG (SparkFun Breakout)          │
            ==============================         │
                                                   │
            ┌─────────────────────────┐            │
            │   VM   VCC  GND  STBY   │            │
            │   │     │    │    │     │            │
            │   │     │    └────│─────│────────────┘
            │   │     │         │     │
            │   │     └─────────│─────│────── Pi Pin 1 (3.3V)
            │   │               │     │
            │   └── External    │     └────── Pi Pin 11 (GPIO17)
            │       Battery     │
            │       6-12V       │
            │                   │
            │  PWMA AIN1 AIN2   │
            │   │    │    │     │
            │   │    │    └─────│────────────── Pi Pin 15 (GPIO22)
            │   │    │          │
            │   │    └──────────│────────────── Pi Pin 13 (GPIO27)
            │   │               │
            │   └───────────────│────────────── Pi Pin 33 (GPIO13)
            │                   │
            │  PWMB BIN1 BIN2   │
            │   │    │    │     │
            │   │    │    └─────│────────────── Pi Pin 18 (GPIO24)
            │   │    │          │
            │   │    └──────────│────────────── Pi Pin 16 (GPIO23)
            │   │               │
            │   └───────────────│────────────── Pi Pin 32 (GPIO12)
            │                   │
            │  AO1  AO2  BO1  BO2                  │
            │   │    │    │    │                   │
            │   └──┬─┘    └──┬─┘                   │
            │      │         │                     │
            │   Motor A   Motor B                  │
            │   (Left)    (Right)                  │
            └─────────────────────────┘
```

### Wire Color Recommendations

| Connection | Suggested Wire Color |
|------------|---------------------|
| 3.3V (VCC) | Red (thin) |
| GND | Black |
| STBY | Yellow |
| Motor A signals (AIN1, AIN2, PWMA) | Blue wires |
| Motor B signals (BIN1, BIN2, PWMB) | Green wires |
| VM (Motor power) | Red (thick) |
| Motor output wires | Red/Black pairs |

---

## Pin-by-Pin Connections

### Complete Connection Table

| TB6612FNG Pin | Function | RPi BOARD Pin | BCM GPIO | Wire Notes |
|---------------|----------|---------------|----------|------------|
| **VCC** | Logic power (2.7-5.5V) | Pin 1 | 3.3V | Red wire, thin gauge |
| **GND** | Ground | Pin 6 | GND | Black wire, common ground |
| **VM** | Motor power (4.5-13.5V) | --- | --- | External battery positive |
| **STBY** | Standby control | Pin 11 | GPIO17 | Yellow wire, MUST be HIGH |
| **PWMA** | Motor A speed (PWM) | Pin 33 | GPIO13 | Hardware PWM1 channel |
| **AIN1** | Motor A direction 1 | Pin 13 | GPIO27 | Direction control |
| **AIN2** | Motor A direction 2 | Pin 15 | GPIO22 | Direction control |
| **PWMB** | Motor B speed (PWM) | Pin 32 | GPIO12 | Hardware PWM0 channel |
| **BIN1** | Motor B direction 1 | Pin 16 | GPIO23 | Direction control |
| **BIN2** | Motor B direction 2 | Pin 18 | GPIO24 | Direction control |
| **AO1** | Motor A output 1 | --- | --- | To left motor + |
| **AO2** | Motor A output 2 | --- | --- | To left motor - |
| **BO1** | Motor B output 1 | --- | --- | To right motor + |
| **BO2** | Motor B output 2 | --- | --- | To right motor - |

### BOARD Numbering Quick Reference

```
BOARD Pin Number = Physical pin position on the 40-pin header
```

| Signal | BOARD Pin | Location on Header |
|--------|-----------|-------------------|
| 3.3V (VCC) | 1 | Top-left corner |
| GND | 6 | Third pin down, right column |
| STBY | 11 | Sixth pin down, left column |
| AIN1 | 13 | Seventh pin down, left column |
| AIN2 | 15 | Eighth pin down, left column |
| BIN1 | 16 | Eighth pin down, right column |
| BIN2 | 18 | Ninth pin down, right column |
| PWMB | 32 | Sixteenth pin down, right column |
| PWMA | 33 | Seventeenth pin down, left column |

### BCM Numbering Quick Reference

```
BCM GPIO Number = Broadcom SoC GPIO designation
```

| Signal | BCM GPIO | Notes |
|--------|----------|-------|
| STBY | GPIO17 | General purpose GPIO |
| PWMA | GPIO13 | Hardware PWM1 - recommended for motor speed |
| AIN1 | GPIO27 | General purpose GPIO |
| AIN2 | GPIO22 | General purpose GPIO |
| PWMB | GPIO12 | Hardware PWM0 - recommended for motor speed |
| BIN1 | GPIO23 | General purpose GPIO |
| BIN2 | GPIO24 | General purpose GPIO |

### Why These Specific Pins?

| Pin Choice | Reason |
|------------|--------|
| GPIO12, GPIO13 for PWM | These are the only hardware PWM pins on the Pi (PWM0, PWM1). Hardware PWM provides smoother motor control than software PWM. |
| GPIO17 for STBY | Convenient location, not used for special functions |
| GPIO22-24, 27 for direction | Clustered together for neat wiring, no special functions |
| Pin 1 for VCC | Provides 3.3V logic voltage, matches TB6612FNG requirements |
| Pin 6 for GND | Close to Pin 1, convenient for power connections |

---

## Power Supply Requirements

### Voltage Requirements

| Power Rail | Voltage Range | Typical Value | Source |
|------------|---------------|---------------|--------|
| **VCC** (Logic) | 2.7V - 5.5V | **3.3V** | Raspberry Pi Pin 1 |
| **VM** (Motor) | 4.5V - 13.5V | **6V - 12V** | External battery pack |

### Current Requirements

| Parameter | Value | Notes |
|-----------|-------|-------|
| Logic current (VCC) | < 10mA | Negligible, Pi can easily supply |
| Motor current (per channel) | 1.2A continuous | TB6612FNG limit |
| Motor peak current | 3.2A | Short duration only |
| Typical FAGM25-370 motor | 0.3-0.8A | Well within limits |

### Power Wiring Diagram

```
                    POWER CONNECTIONS
                    ==================

┌─────────────────┐          ┌─────────────────┐
│  Raspberry Pi   │          │   TB6612FNG     │
├─────────────────┤          ├─────────────────┤
│                 │          │                 │
│  Pin 1 (3.3V)   │──────────│  VCC            │
│                 │          │                 │
│  Pin 6 (GND)    │────┬─────│  GND            │
│                 │    │     │                 │
└─────────────────┘    │     │  VM ────────────│────┐
                       │     │                 │    │
                       │     └─────────────────┘    │
                       │                            │
                       │     ┌─────────────────┐    │
                       │     │ External Battery │    │
                       │     │   (6-12V)       │    │
                       │     ├─────────────────┤    │
                       │     │                 │    │
                       └─────│  (-) Negative   │    │
                             │                 │    │
                             │  (+) Positive ──│────┘
                             │                 │
                             └─────────────────┘
```

### Important Power Notes

1. **Common Ground Required**: The Raspberry Pi GND and battery negative MUST be connected together through the TB6612FNG GND pin.

2. **Never Connect VM to Pi**: The VM (motor voltage) should NEVER connect to the Raspberry Pi. Keep motor power completely separate.

3. **Use 3.3V for VCC**: Although TB6612FNG can handle 5V logic, using 3.3V from the Pi is safer and fully compatible.

4. **Decoupling Capacitor**: Add a 100uF electrolytic capacitor across VM and GND on the TB6612FNG board to suppress voltage spikes from motors. The SparkFun breakout includes some capacitance, but additional capacitance helps.

5. **Battery Selection**:
   - 2S LiPo (7.4V nominal) - Excellent choice
   - 6x AA cells (9V) - Good for testing
   - 12V SLA/lead-acid - Works, near max voltage
   - Avoid batteries above 13.5V

---

## STBY Pin - Critical Information

### What is the STBY Pin?

The STBY (Standby) pin is an **enable pin** that controls whether the motor driver is active or in low-power standby mode.

### STBY Behavior

| STBY State | Driver State | Motors | Current Draw |
|------------|--------------|--------|--------------|
| **LOW (0V)** | Standby/Disabled | **OFF - No output** | ~1uA (microamps) |
| **HIGH (3.3V)** | Active/Enabled | Respond to commands | Normal operation |

### Critical Notes

1. **STBY Must Be HIGH for Motors to Work**
   - If STBY is LOW or floating, motors will NOT respond
   - This is the #1 cause of "motors don't work" issues

2. **Hardware vs Software Control**

   **Option A - Hardwired (Simplest)**:
   ```
   Connect STBY directly to 3.3V (Pin 1) - Always enabled
   ```

   **Option B - GPIO Control (Recommended)**:
   ```
   Connect STBY to GPIO17 (Pin 11) - Software enable/disable
   ```

3. **Software Enable Sequence**
   ```python
   import RPi.GPIO as GPIO

   STBY_PIN = 11  # BOARD numbering (GPIO17)

   GPIO.setmode(GPIO.BOARD)
   GPIO.setup(STBY_PIN, GPIO.OUT)

   # Enable driver - MUST do this before motor commands work
   GPIO.output(STBY_PIN, GPIO.HIGH)

   # ... motor commands work here ...

   # Disable driver (low power mode)
   GPIO.output(STBY_PIN, GPIO.LOW)
   ```

4. **Benefits of GPIO Control**:
   - Emergency stop functionality
   - Power saving when motors not needed
   - Prevents unexpected motor movement during code initialization

5. **When to Use Hardwired STBY**:
   - Simple projects without power management needs
   - When you want motors always ready
   - Testing/debugging

### STBY Pin Quick Test

If motors aren't responding, try this quick test:

```bash
# Connect STBY directly to 3.3V (Pin 1) with a jumper wire
# If motors now respond, your STBY GPIO code needs fixing
```

---

## Step-by-Step Connection Instructions

### Required Materials

- Raspberry Pi (any model with 40-pin header)
- SparkFun TB6612FNG breakout board
- 2x DC motors (e.g., FAGM25-370)
- 6-12V battery pack
- Jumper wires (female-to-female for Pi, male headers on TB6612FNG)
- (Optional) Breadboard for prototyping
- (Optional) 100uF capacitor

### Step 1: Prepare the Raspberry Pi

1. **Power off the Raspberry Pi** completely
2. Disconnect any power sources
3. Place Pi on a static-safe surface
4. Identify Pin 1 (3.3V) - it's marked with a square solder pad on the PCB, located at the corner of the header

### Step 2: Connect Ground (CRITICAL - Do First)

| From | To | Wire |
|------|-----|------|
| Raspberry Pi Pin 6 | TB6612FNG GND | Black wire |

**Why first?** Establishing common ground before any other connections prevents voltage differences that could damage components.

### Step 3: Connect Logic Power

| From | To | Wire |
|------|-----|------|
| Raspberry Pi Pin 1 | TB6612FNG VCC | Red wire (thin) |

**Verify**: VCC should receive 3.3V (can test with multimeter later)

### Step 4: Connect STBY (Standby/Enable)

| From | To | Wire |
|------|-----|------|
| Raspberry Pi Pin 11 | TB6612FNG STBY | Yellow wire |

**Remember**: This pin must be set HIGH in software before motors respond

### Step 5: Connect Motor A Control Pins (Left Motor)

| From | To | Function |
|------|-----|----------|
| Raspberry Pi Pin 13 | TB6612FNG AIN1 | Direction control 1 |
| Raspberry Pi Pin 15 | TB6612FNG AIN2 | Direction control 2 |
| Raspberry Pi Pin 33 | TB6612FNG PWMA | Speed control (PWM) |

### Step 6: Connect Motor B Control Pins (Right Motor)

| From | To | Function |
|------|-----|----------|
| Raspberry Pi Pin 16 | TB6612FNG BIN1 | Direction control 1 |
| Raspberry Pi Pin 18 | TB6612FNG BIN2 | Direction control 2 |
| Raspberry Pi Pin 32 | TB6612FNG PWMB | Speed control (PWM) |

### Step 7: Connect Motors to Driver Outputs

| TB6612FNG Pin | Motor Connection |
|---------------|------------------|
| AO1 | Left motor terminal 1 (+ or red) |
| AO2 | Left motor terminal 2 (- or black) |
| BO1 | Right motor terminal 1 (+ or red) |
| BO2 | Right motor terminal 2 (- or black) |

**Note**: Motor polarity affects direction. If a motor runs backwards, swap its two wires.

### Step 8: Connect Motor Power (Battery)

| From | To |
|------|-----|
| Battery positive (+) | TB6612FNG VM |
| Battery negative (-) | TB6612FNG GND (already connected to Pi GND) |

**Warning**: Double-check polarity. Reverse polarity will damage the TB6612FNG.

### Step 9: Verify Connections

Before powering on, verify:

- [ ] No loose connections
- [ ] No short circuits (wires touching where they shouldn't)
- [ ] GND from Pi connects to GND on driver
- [ ] VCC connects to 3.3V, NOT 5V
- [ ] VM connects to battery positive only
- [ ] Battery negative connects to GND rail
- [ ] All control wires go to correct pins (double-check against table)

### Step 10: Power On and Test

1. Connect the motor battery first
2. Boot the Raspberry Pi
3. Run the test script:
   ```bash
   python3 -m locomotion.rpi_motors.test_motors --check
   python3 -m locomotion.rpi_motors.test_motors --basic
   ```

---

## Wiring Verification Checklist

Use this checklist to verify your wiring before power-on:

### Power Connections

- [ ] Pi Pin 1 (3.3V) connected to TB6612FNG VCC
- [ ] Pi Pin 6 (GND) connected to TB6612FNG GND
- [ ] Battery (+) connected to TB6612FNG VM
- [ ] Battery (-) connected to TB6612FNG GND
- [ ] VCC and VM are NOT connected together
- [ ] No wires connected to Pi 5V pins (Pin 2 or Pin 4)

### Control Connections

- [ ] Pi Pin 11 connected to TB6612FNG STBY
- [ ] Pi Pin 13 connected to TB6612FNG AIN1
- [ ] Pi Pin 15 connected to TB6612FNG AIN2
- [ ] Pi Pin 33 connected to TB6612FNG PWMA
- [ ] Pi Pin 16 connected to TB6612FNG BIN1
- [ ] Pi Pin 18 connected to TB6612FNG BIN2
- [ ] Pi Pin 32 connected to TB6612FNG PWMB

### Motor Connections

- [ ] Left motor wires connected to AO1 and AO2
- [ ] Right motor wires connected to BO1 and BO2
- [ ] Motor wires are secure (no loose connections)

### Final Checks

- [ ] No exposed wire strands that could short
- [ ] Battery polarity is correct
- [ ] All connections are secure
- [ ] No wires pinched or stressed

---

## Quick Reference Card

Print this section for bench reference:

```
TB6612FNG to Raspberry Pi - Quick Reference
============================================

POWER:
  VCC  --> Pi Pin 1  (3.3V)
  GND  --> Pi Pin 6  (GND) + Battery (-)
  VM   --> Battery (+) 6-12V

ENABLE:
  STBY --> Pi Pin 11 (GPIO17) -- SET HIGH TO ENABLE

MOTOR A (Left):
  AIN1 --> Pi Pin 13 (GPIO27)
  AIN2 --> Pi Pin 15 (GPIO22)
  PWMA --> Pi Pin 33 (GPIO13) *Hardware PWM1*
  AO1/AO2 --> Left Motor

MOTOR B (Right):
  BIN1 --> Pi Pin 16 (GPIO23)
  BIN2 --> Pi Pin 18 (GPIO24)
  PWMB --> Pi Pin 32 (GPIO12) *Hardware PWM0*
  BO1/BO2 --> Right Motor

Motor Truth Table:
  IN1=H, IN2=L, PWM=H --> Forward
  IN1=L, IN2=H, PWM=H --> Reverse
  IN1=H, IN2=H, PWM=X --> Brake
  IN1=L, IN2=L, PWM=X --> Coast/Stop
  STBY=L              --> Standby (disabled)
```

---

## Troubleshooting

### Motors Don't Spin

1. **Check STBY pin** - Must be HIGH (most common issue)
2. **Check motor power** - Is battery connected to VM?
3. **Check common ground** - Pi GND and battery GND must connect
4. **Check PWM output** - Test with LED on PWM pin

### Motors Spin Wrong Direction

1. Swap the motor wires (AO1/AO2 or BO1/BO2)
2. Or change the `offset` value in software to -1

### One Motor Works, Other Doesn't

1. Check the non-working motor's connections
2. Test motor directly with battery (bypassing driver)
3. Verify all three control pins for that motor channel

### Motors Stutter or Have Weak Torque

1. Check battery voltage (may be low)
2. Verify PWM frequency setting
3. Check for loose connections
4. Ensure adequate wire gauge for motor current

---

## References

- [SparkFun TB6612FNG Product Page](https://www.sparkfun.com/products/14451)
- [SparkFun Hookup Guide](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide)
- [Toshiba TB6612FNG Datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
