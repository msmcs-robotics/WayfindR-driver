# Mobile Robot Power System Guide

This document provides a comprehensive guide for designing a power system for a mobile robot with the following components:
- Raspberry Pi 3B+/4 (or Jetson Nano)
- L298N motor driver with 2x DC motors
- USB webcam
- USB LiDAR
- Future: MPU6050 IMU

---

## Table of Contents

1. [Power Budget Estimation](#1-power-budget-estimation)
2. [Single Battery vs Separate Batteries](#2-single-battery-vs-separate-batteries)
3. [Battery Types Comparison](#3-battery-types-comparison)
4. [Voltage Conversion and Regulation](#4-voltage-conversion-and-regulation)
5. [Safety Considerations](#5-safety-considerations)
6. [Recommended Configurations](#6-recommended-configurations)
7. [Wiring Diagrams](#7-wiring-diagrams)

---

## 1. Power Budget Estimation

### Component Power Requirements

| Component | Voltage | Current (Typical) | Current (Peak) | Power (Typical) |
|-----------|---------|-------------------|----------------|-----------------|
| **Raspberry Pi 4** | 5V | 600mA (idle) | 1.5-3A (load + USB) | 3-15W |
| **Raspberry Pi 3B+** | 5V | 400mA (idle) | 1.2A (load) | 2-6W |
| **Jetson Nano** | 5V | 1-2A (5W mode) | 4A (10W mode) | 5-20W |
| **L298N Driver** | 5V logic | 36mA | 36mA | 0.18W |
| **DC Motor (each)** | 6-12V | 200-750mA | 1.8A (stall) | 2.4-9W each |
| **USB Webcam** | 5V | 200-250mA | 500mA | 1-2.5W |
| **USB LiDAR (YDLIDAR X4)** | 5V | 180mA | 300mA | 0.9-1.5W |
| **USB LiDAR (RPLIDAR A1)** | 5V | 250mA | 500mA | 1.25-2.5W |
| **MPU6050 IMU** | 3.3V | 3.6mA | 5mA | 0.012W |

### Total Power Budget Calculations

#### Configuration A: Raspberry Pi 4 Based Robot

| Subsystem | Voltage | Current | Power |
|-----------|---------|---------|-------|
| Pi 4 + Peripherals | 5V | 2A avg, 3A peak | 10-15W |
| USB Webcam | 5V | 250mA | 1.25W |
| USB LiDAR | 5V | 250mA | 1.25W |
| MPU6050 | 3.3V | 4mA | 0.015W |
| **Compute Subtotal** | 5V | ~2.5A avg | ~12-17W |
| 2x DC Motors | 12V | 1.5A avg, 3.6A stall | 18-43W |
| L298N Logic | 5V | 36mA | 0.18W |
| **Motor Subtotal** | 12V | 1.5-3.6A | ~18-43W |
| **TOTAL** | Mixed | - | **30-60W** |

#### Configuration B: Jetson Nano Based Robot

| Subsystem | Voltage | Current | Power |
|-----------|---------|---------|-------|
| Jetson Nano (10W mode) | 5V | 2-4A | 10-20W |
| USB Webcam | 5V | 250mA | 1.25W |
| USB LiDAR | 5V | 250mA | 1.25W |
| MPU6050 | 3.3V | 4mA | 0.015W |
| **Compute Subtotal** | 5V | ~3-4.5A | ~12-22W |
| 2x DC Motors | 12V | 1.5A avg, 3.6A stall | 18-43W |
| L298N Logic | 5V | 36mA | 0.18W |
| **Motor Subtotal** | 12V | 1.5-3.6A | ~18-43W |
| **TOTAL** | Mixed | - | **30-65W** |

### Runtime Estimation Formula

```
Runtime (hours) = Battery Capacity (Wh) / Average Power Draw (W) × Efficiency Factor

Where:
- Efficiency Factor = 0.80-0.90 (accounts for voltage conversion losses)
- Battery Capacity (Wh) = Voltage × Amp-hours
```

**Example**: 3S LiPo (11.1V × 5000mAh = 55.5Wh), 40W average draw, 85% efficiency:
```
Runtime = 55.5 × 0.85 / 40 = 1.18 hours (~70 minutes)
```

---

## 2. Single Battery vs Separate Batteries

### Option A: Single Battery System

```
┌─────────────────────────────────────────────────────────────┐
│                    SINGLE BATTERY                           │
│                    (3S LiPo 11.1V)                          │
└─────────────────────────┬───────────────────────────────────┘
                          │
          ┌───────────────┴───────────────┐
          │                               │
          ▼                               ▼
    ┌───────────┐                   ┌───────────┐
    │  L298N    │                   │ Buck Conv │
    │  Motors   │                   │ 12V→5V    │
    │  (Direct) │                   │ (3-5A)    │
    └───────────┘                   └─────┬─────┘
                                          │
                                          ▼
                                    ┌───────────┐
                                    │   Pi/Nano │
                                    │  + Sensors│
                                    └───────────┘
```

**Advantages:**
- Simpler to manage (one battery to charge/monitor)
- Lighter weight overall
- Lower cost
- Easier battery level monitoring

**Disadvantages:**
- Motor noise can affect compute (voltage dips, EMI)
- Single point of failure
- Motors can brown-out the computer during high current draws
- Need good filtering/regulation

**Mitigation Strategies:**
- Use quality buck converter with capacitor filtering
- Add 100-470µF electrolytic capacitors near the Pi power input
- Use ferrite beads on power lines
- Consider LC filter for sensitive electronics

### Option B: Dual Battery System

```
┌───────────────────┐        ┌───────────────────┐
│ MOTOR BATTERY     │        │ COMPUTE BATTERY   │
│ (3S LiPo 11.1V)   │        │ (USB Power Bank   │
│                   │        │  or 2S LiPo)      │
└─────────┬─────────┘        └─────────┬─────────┘
          │                            │
          ▼                            ▼
    ┌───────────┐                ┌───────────┐
    │  L298N    │                │   Pi/Nano │
    │  Motors   │                │  + Sensors│
    └───────────┘                └───────────┘
          │                            │
          └────────── GND ─────────────┘
           (Common ground required)
```

**Advantages:**
- Complete electrical isolation of motor noise
- Compute system protected from motor current spikes
- Can use optimized battery chemistry for each purpose
- Longer compute runtime independent of motor usage

**Disadvantages:**
- Two batteries to manage and charge
- Heavier total weight
- More complex wiring
- Must maintain common ground between systems

### Recommendation

**For beginners or simple robots:** Start with a **single battery system** using a quality buck converter with proper filtering. This is simpler and works well for most hobby robots.

**For reliability-critical applications:** Use a **dual battery system** to completely isolate motor noise from compute. Essential if you experience random reboots or sensor glitches.

---

## 3. Battery Types Comparison

### 3.1 LiPo (Lithium Polymer) Batteries

| Specification | Value |
|---------------|-------|
| Cell Voltage | 3.7V nominal, 4.2V full, 3.0V cutoff |
| Energy Density | 250-300 Wh/kg |
| Discharge Rate | 20C-120C (very high) |
| Cycle Life | 300-500 cycles |
| Cost | $20-100+ per pack |

**Common Configurations:**
- **2S (7.4V)**: Good for 6V motors, needs 5V buck converter
- **3S (11.1V)**: Ideal for 12V motors, good for L298N
- **4S (14.8V)**: Maximum for L298N (within 6-35V range)

**Pros:**
- Highest discharge rates (great for motor start-up surges)
- Lightweight and compact
- Flexible form factors
- Best for high-performance robots

**Cons:**
- Requires careful handling (fire risk if damaged)
- Shorter lifespan than 18650
- Needs balance charger
- Can puff/swell if abused

**Safety Requirements:**
- Always use LiPo-safe charging bag
- Never discharge below 3.0V per cell
- Store at 3.8V per cell for long-term
- Inspect for puffing before each use

### 3.2 18650 Li-Ion Batteries

| Specification | Value |
|---------------|-------|
| Cell Voltage | 3.7V nominal, 4.2V full, 2.5-3.0V cutoff |
| Energy Density | 200-250 Wh/kg |
| Discharge Rate | 1-3C typical, up to 35A for high-drain cells |
| Cycle Life | 500-1000 cycles |
| Cost | $2-5 per cell |

**Common Configurations:**
- **3S (11.1V)**: 3 cells in series, good for motors
- **4S (14.8V)**: Higher voltage, more headroom

**Popular High-Drain Cells:**
- Samsung 30Q (3000mAh, 15A continuous)
- Sony VTC6 (3000mAh, 15A continuous)
- Samsung 25R (2500mAh, 20A continuous)
- LG HG2 (3000mAh, 20A continuous)

**Pros:**
- Longer lifespan (500-1000 cycles)
- More thermally stable (safer)
- Lower cost per Wh
- Widely available, standardized
- More resistant to damage

**Cons:**
- Lower discharge rates than LiPo
- Heavier for same capacity
- Fixed cylindrical form factor
- May need holder/spot welding

**Recommended for:** Long-runtime applications, cost-sensitive projects, beginners who want safer option.

### 3.3 USB Power Banks

| Specification | Value |
|---------------|-------|
| Output Voltage | 5V (some have 9V/12V QC/PD) |
| Capacity | 5,000-30,000 mAh |
| Max Current | 2.1-3A typical |
| Cost | $15-50 |

**Pros:**
- Extremely simple to use
- Built-in protection circuits
- USB-C PD models can deliver higher power
- No charging equipment needed

**Cons:**
- Cannot power motors directly
- Limited current output (usually max 3A)
- Auto-shutoff with low current draw
- Lower efficiency than direct battery

**Best Use Case:** Powering Raspberry Pi only (not motors) in dual-battery configurations.

### 3.4 Battery Comparison Summary

| Feature | LiPo | 18650 Pack | Power Bank |
|---------|------|------------|------------|
| **Discharge Rate** | Excellent | Good | Limited |
| **Energy Density** | Excellent | Good | Good |
| **Cycle Life** | 300-500 | 500-1000 | 300-500 |
| **Safety** | Requires care | Safer | Safest |
| **Cost** | Medium | Low | Medium |
| **Convenience** | Low | Medium | High |
| **Motor Suitability** | Excellent | Good | Poor |
| **Compute Suitability** | Good | Good | Excellent |

### Recommendation by Use Case

| Use Case | Recommended Battery |
|----------|---------------------|
| High-performance robot | 3S LiPo (11.1V, 2200-5000mAh, 25C+) |
| Budget/beginner robot | 3S 18650 pack with BMS |
| Long runtime priority | 4S 18650 pack (high capacity cells) |
| Compute only (dual system) | USB Power Bank (10000+ mAh) |
| Indoor slow robot | 2S LiPo + buck converter |

---

## 4. Voltage Conversion and Regulation

### 4.1 Buck Converter Selection

The buck converter steps down battery voltage (7.4-14.8V) to 5V for the Raspberry Pi/Jetson Nano.

**Requirements:**
- Input: 7V-16V (to support 2S-4S LiPo)
- Output: 5V ± 0.25V (critical for Pi stability)
- Current: 3A continuous, 5A peak (minimum)
- Efficiency: 85%+ preferred

**Recommended Buck Converters:**

| Model | Input | Output | Current | Efficiency | Notes |
|-------|-------|--------|---------|------------|-------|
| **Pololu D24V50F5** | 4.5-38V | 5V | 5A | 90%+ | Premium, excellent regulation |
| **DFRobot Multi-Output** | 7.5-30V | 5V | 3-5A | ~85% | Multiple voltage options |
| **Adafruit UBEC** | 6-16V | 5V | 3A | ~85% | Good for hobby use |
| **MP1584 Module** | 4.5-28V | 5V | 3A | 92% | Compact, cheap |
| **LM2596 Module** | 4-40V | 5V | 3A | 80% | Budget option |

**For Jetson Nano:** Consider higher capacity:
- MEAN WELL SD-25A-5 (25W, 5A)
- Pololu D24V90F5 (9A capacity)

### 4.2 L298N Onboard Regulator

The L298N has a built-in 78M05 voltage regulator:

**When to use the onboard regulator:**
- Input voltage ≤ 12V
- Only powering the L298N logic (36mA)
- Can provide small amount of 5V to microcontroller

**When NOT to use (remove jumper):**
- Input voltage > 12V
- Need 5V output for Pi (insufficient current capacity)
- Use external 5V supply for logic instead

**Recommendation:** Always use a dedicated buck converter for the Pi. The L298N's regulator is only suitable for its own logic.

### 4.3 Voltage Drop Considerations

The L298N has significant internal voltage drop:

| Motor Current | Voltage Drop | Notes |
|---------------|--------------|-------|
| 0.5A | ~1.4V | Light load |
| 1.0A | ~2.0V | Typical operation |
| 2.0A | ~3.5-4.9V | Heavy load |

**Implication:** If your motors are rated for 12V and you supply 12V to the L298N, the motors only receive ~10V at 1A load.

**Solutions:**
- Use 14-15V supply for 12V motors
- Use 9-10V supply for 6V motors
- Consider modern drivers (TB6612FNG, BTS7960) with lower drop

### 4.4 Power Distribution Architecture

**Recommended Single-Battery Architecture:**

```
                    ┌────────────────────────────────────────┐
                    │         3S LiPo (11.1V nominal)        │
                    │         9.0V (discharged) to           │
                    │         12.6V (fully charged)          │
                    └───────────────────┬────────────────────┘
                                        │
                                        │ Main Fuse (10-15A)
                                        │
                    ┌───────────────────┴────────────────────┐
                    │                                        │
              ┌─────┴─────┐                           ┌──────┴──────┐
              │   L298N   │                           │    Buck     │
              │   Motor   │                           │  Converter  │
              │   Driver  │                           │  12V→5V 5A  │
              └─────┬─────┘                           └──────┬──────┘
                    │                                        │
         ┌──────────┼──────────┐                    ┌────────┴────────┐
         │          │          │                    │                 │
    ┌────┴────┐ ┌───┴───┐      │              ┌─────┴─────┐    ┌──────┴──────┐
    │ Motor A │ │Motor B│   Logic 5V          │   Pi/Nano │    │USB Hub      │
    └─────────┘ └───────┘   (optional)        │  + IMU    │    │(powered)    │
                                              └───────────┘    └──────┬──────┘
                                                                      │
                                                          ┌───────────┼───────────┐
                                                          │           │           │
                                                     ┌────┴────┐ ┌────┴────┐      │
                                                     │ Webcam  │ │  LiDAR  │    Other
                                                     └─────────┘ └─────────┘    USB
```

---

## 5. Safety Considerations

### 5.1 Fuse Protection

**Main Fuse (Required):**
- Place between battery and all loads
- Rating: 1.5× maximum expected continuous current
- For 40W system at 12V: 10-15A fuse

**Recommended Fuse Types:**
| Type | Pros | Cons |
|------|------|------|
| Blade fuse (automotive) | Cheap, easy to replace | Not resettable |
| Glass fuse | Cheap, visible break | Fragile |
| PTC resettable fuse | Self-resetting | Higher resistance |
| Circuit breaker | Resettable, reliable | Bulky, expensive |

**Wiring:**
```
Battery (+) ──► Fuse ──► Main switch ──► Distribution
```

### 5.2 Low Voltage Cutoff (LVC)

**Why it's critical:**
- Li-ion/LiPo cells are damaged below ~3.0V per cell
- Damaged cells may catch fire during charging
- Over-discharge reduces cycle life dramatically

**LVC Options:**

| Method | Implementation | Pros | Cons |
|--------|----------------|------|------|
| **BMS Module** | Integrated with battery pack | Automatic, reliable | Fixed threshold |
| **Voltage Alarm** | Audible buzzer at low voltage | Cheap, simple | Requires user action |
| **Software Monitor** | ADC reading in code | Graceful shutdown | Requires coding |
| **LVC Module** | Dedicated cutoff circuit | Adjustable, reliable | Extra component |

**Recommended Thresholds:**
- Warning: 3.5V per cell (10.5V for 3S)
- Cutoff: 3.2V per cell (9.6V for 3S)
- Hard cutoff: 3.0V per cell (9.0V for 3S)

**Software Implementation (Raspberry Pi):**
```python
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Using ADS1115 ADC
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
battery_channel = AnalogIn(ads, ADS.P0)

# Voltage divider: 47k / 10k for 12V → 2.1V range
DIVIDER_RATIO = (47000 + 10000) / 10000  # 5.7

def get_battery_voltage():
    return battery_channel.voltage * DIVIDER_RATIO

def check_battery():
    voltage = get_battery_voltage()
    cell_voltage = voltage / 3  # For 3S battery

    if cell_voltage < 3.0:
        # CRITICAL: Shutdown immediately
        shutdown_robot()
    elif cell_voltage < 3.2:
        # WARNING: Return to charging station
        return_to_base()
    elif cell_voltage < 3.5:
        # CAUTION: Low battery warning
        log_warning("Low battery")
```

### 5.3 Battery Management System (BMS)

A BMS provides comprehensive protection:

**Features:**
- Over-charge protection (cuts charging above 4.2V/cell)
- Over-discharge protection (cuts load below threshold)
- Over-current protection (limits maximum current)
- Short-circuit protection (immediate cutoff)
- Cell balancing (equalizes cell voltages)
- Temperature monitoring (some models)

**BMS Selection:**

| Cells | Continuous Current | Recommended BMS |
|-------|-------------------|-----------------|
| 3S | 15A | 3S 15A BMS |
| 3S | 25A | 3S 25A BMS |
| 4S | 15A | 4S 15A BMS |

**Important:** Size your BMS for **peak** motor current, not average. Motors can draw 2-3× rated current on startup.

### 5.4 Reverse Polarity Protection

Protects against connecting battery backwards:

**Methods:**
1. **Schottky Diode:** Simple but drops 0.3-0.5V
2. **P-channel MOSFET:** Low voltage drop (~0.1V)
3. **Connector keying:** Use polarized connectors (XT60, Deans)

**Recommended:** Use **XT60 connectors** which are keyed and rated for high current.

### 5.5 Safety Checklist

Before each operation:
- [ ] Check battery voltage (above 3.5V per cell)
- [ ] Inspect LiPo for puffing or damage
- [ ] Verify all connections are secure
- [ ] Confirm fuse is in place
- [ ] Test emergency stop function

During charging:
- [ ] Use appropriate charger (balance charger for LiPo)
- [ ] Never leave charging unattended
- [ ] Charge in fire-safe location (LiPo bag)
- [ ] Don't charge immediately after heavy use (let cool)

Storage:
- [ ] Store LiPo at 3.8V per cell (storage charge)
- [ ] Keep in cool, dry location
- [ ] Use fireproof container for LiPo storage

---

## 6. Recommended Configurations

### Configuration 1: Budget Beginner (Raspberry Pi 4)

**Total Cost:** ~$50-80 (excluding Pi and motors)

| Component | Specification | Est. Cost |
|-----------|---------------|-----------|
| Battery | 3S 18650 pack (3× Samsung 30Q) | $15-25 |
| BMS | 3S 15A with balance | $5-10 |
| Buck Converter | LM2596 or MP1584 module | $3-5 |
| Fuse | 10A blade fuse + holder | $3-5 |
| Connectors | XT60 pairs, terminal blocks | $5-10 |
| Wiring | 14-16 AWG silicone wire | $10-15 |
| Battery Holder | 3S 18650 holder | $5-10 |

**Runtime:** ~60-90 minutes with 3× 3000mAh cells

### Configuration 2: Intermediate (Raspberry Pi 4)

**Total Cost:** ~$80-120

| Component | Specification | Est. Cost |
|-----------|---------------|-----------|
| Battery | 3S LiPo 5000mAh 25C | $35-50 |
| Buck Converter | Pololu D24V50F5 or similar | $15-25 |
| BMS/Alarm | Low voltage buzzer + BMS | $10-15 |
| Fuse | 15A automotive fuse | $5 |
| Connectors | XT60, JST | $10 |
| Wiring | 12-14 AWG silicone | $15 |

**Runtime:** ~90-120 minutes

### Configuration 3: High Performance (Jetson Nano)

**Total Cost:** ~$120-180

| Component | Specification | Est. Cost |
|-----------|---------------|-----------|
| Battery | 3S LiPo 6000mAh 35C | $50-70 |
| Buck Converter | Pololu D24V90F5 (9A) | $25-35 |
| BMS | 3S 30A with cutoff | $15-20 |
| Power Monitor | INA219 module | $5-10 |
| Powered USB Hub | 4-port with external power | $15-25 |
| Fuse/Switch | 20A with switch | $10 |
| Connectors/Wiring | Premium | $20 |

**Runtime:** ~60-90 minutes (higher compute power draw)

### Configuration 4: Dual Battery (Maximum Reliability)

**Total Cost:** ~$100-150

| Component | Specification | Est. Cost |
|-----------|---------------|-----------|
| Motor Battery | 3S LiPo 2200mAh 25C | $20-30 |
| Compute Battery | USB-C PD Power Bank 20000mAh | $30-50 |
| USB-C to Barrel | For Jetson Nano (if used) | $10 |
| Motor Fuse | 10A blade fuse | $5 |
| Connectors | XT60, USB-C | $15 |
| Common Ground | Star ground implementation | $5-10 |

**Runtime:**
- Motors: ~45-60 minutes active
- Compute: 4-6+ hours

---

## 7. Wiring Diagrams

### Basic Single Battery Wiring

```
                         ┌─────────────────────────────────────────────────┐
                         │                                                 │
    ┌────────────────────┴────────────────────┐                            │
    │           3S LiPo Battery               │                            │
    │         (11.1V, 5000mAh 25C)            │                            │
    │                                         │                            │
    │  (+) ─────────┬───────────── (-)        │                            │
    └───────────────┼─────────────────────────┘                            │
                    │              │                                       │
             ┌──────┴──────┐       │                                       │
             │  10A Fuse   │       │                                       │
             └──────┬──────┘       │                                       │
                    │              │                                       │
             ┌──────┴──────┐       │                                       │
             │   Switch    │       │                                       │
             └──────┬──────┘       │                                       │
                    │              │                                       │
    ┌───────────────┴───────────────────────────────────────────┐          │
    │                        POWER BUS (+)                      │          │
    └──────────────────┬────────────────────────────┬───────────┘          │
                       │                            │                      │
              ┌────────┴────────┐          ┌────────┴────────┐             │
              │     L298N       │          │  Buck Converter │             │
              │   VCC (12V)     │          │   IN+ (12V)     │             │
              │   GND ──────────┼──────────┼── IN- ──────────┼─────────────┤
              │                 │          │                 │             │
              │   OUT1 ─► Motor A          │   OUT+ (5V) ────┼─► Pi 5V     │
              │   OUT2 ─► Motor A          │   OUT- ─────────┼─► Pi GND    │
              │   OUT3 ─► Motor B          │                 │             │
              │   OUT4 ─► Motor B          └─────────────────┘             │
              │                 │                                          │
              │   ENA ◄── Pi GPIO (PWM)                                    │
              │   ENB ◄── Pi GPIO (PWM)                                    │
              │   IN1-4 ◄── Pi GPIO                                        │
              │                 │                                          │
              │   GND ──────────┼──────────────────────────────────────────┘
              └─────────────────┘

    Note: Remove L298N 5V jumper when using external 5V supply
```

### GPIO Connections (Raspberry Pi)

```
L298N Pin    │    Raspberry Pi GPIO
─────────────┼────────────────────────
   ENA       │    GPIO 18 (PWM0)
   IN1       │    GPIO 23
   IN2       │    GPIO 24
   ENB       │    GPIO 19 (PWM1)
   IN3       │    GPIO 17
   IN4       │    GPIO 27
   GND       │    GND (Pin 6)
```

### MPU6050 I2C Connections

```
MPU6050 Pin  │    Raspberry Pi
─────────────┼────────────────────────
   VCC       │    3.3V (Pin 1)
   GND       │    GND (Pin 9)
   SCL       │    GPIO 3 / SCL (Pin 5)
   SDA       │    GPIO 2 / SDA (Pin 3)
   AD0       │    GND (for address 0x68)
```

---

## References and Further Reading

### Power System Design
- [Articulated Robotics - Power Concepts](https://articulatedrobotics.xyz/mobile-robot-5-power-theory/)
- [Kev's Robots - Power Up Your Robot Projects](https://www.kevsrobots.com/blog/power.html)
- [Husarion - Choosing the Right Battery for Your Robot](https://husarion.com/blog/batteries-for-mobile-robots/)

### Component Specifications
- [L298N Motor Driver Tutorial](https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/)
- [Raspberry Pi 4 Power Requirements](https://raspberryexpert.com/raspberry-pi-4-power-requirements/)
- [Jetson Nano Power Supply Guide](https://piveral.com/jetson-nano-power-supply-guide/)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

### Battery Safety
- [LiPo Battery Protection Circuits Guide](https://www.hanery.com/en/blog/lipo-battery-protection-circuits/)
- [18650 vs LiPo Comparison](https://www.ufinebattery.com/blog/18650-cell-or-lipo-battery-cell-choosing-the-best-battery/)

### Buck Converters
- [Adafruit UBEC](https://www.adafruit.com/product/1385)
- [DFRobot Multi-Output Buck Converter](https://www.dfrobot.com/product-2599.html)

---

*Document created for the WayfindR mobile robot project.*
*Last updated: February 2026*
