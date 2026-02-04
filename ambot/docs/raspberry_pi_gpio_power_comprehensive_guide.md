# Comprehensive Raspberry Pi Power Guide: All Methods Including GPIO Power

This guide covers **ALL** methods to power a Raspberry Pi 3B+ and Pi 4 from batteries, with special focus on GPIO power options for mobile robotics applications.

---

## Table of Contents

1. [Power Methods Overview](#1-power-methods-overview)
2. [USB Power (Standard Method)](#2-usb-power-standard-method)
3. [GPIO 5V Pin Power (Advanced Method)](#3-gpio-5v-pin-power-advanced-method)
4. [Power over Ethernet (PoE)](#4-power-over-ethernet-poe)
5. [GPIO vs USB Comparison](#5-gpio-vs-usb-comparison)
6. [Wiring Diagrams for GPIO Power from LiPo](#6-wiring-diagrams-for-gpio-power-from-lipo)
7. [Buck Converter Recommendations](#7-buck-converter-recommendations)
8. [Safety Considerations for GPIO Power](#8-safety-considerations-for-gpio-power)
9. [Protection Circuits](#9-protection-circuits)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Power Methods Overview

### Summary of All Power Methods

| Method | Pi 3B+ | Pi 4 | Voltage | Protection | Best For |
|--------|--------|------|---------|------------|----------|
| **USB (Micro-USB/USB-C)** | Yes | Yes | 5V | Full (polyfuse + TVS) | General use, beginners |
| **GPIO 5V Pins (Pin 2/4)** | Yes | Yes | 5V regulated | **None** | Robotics, embedded |
| **PoE HAT** | Yes | Yes | 48V (converted) | Full | Network deployments |
| **Battery HATs** | Yes | Yes | 5V | Full | UPS, portable |
| **USB-C PD Trigger** | No | Yes | 5V negotiated | Varies | High-current needs |

---

## 2. USB Power (Standard Method)

### 2.1 Raspberry Pi 3B+ USB Power

**Connector:** Micro-USB (Type B)

**Specifications:**
- Input Voltage: 5V +/- 5% (4.75V - 5.25V)
- Recommended Current: 2.5A minimum
- Maximum Current Draw: ~1.4A (Pi only), 2.5A+ with peripherals

**Power Path:**
```
Micro-USB → Polyfuse (2.5A) → TVS Diode → 5V Rail → SoC + Peripherals
```

**Protection Features:**
- **Polyfuse (MF-MSMF250):** 2.5A hold, trips at ~5A, resets when cooled
- **TVS Diode:** Protects against voltage transients/ESD
- **Reverse Polarity:** USB connector prevents reversal

**Pros:**
- Safest method with full protection
- Standard cables readily available
- Hot-pluggable
- Undervoltage detection and warning (lightning bolt icon)

**Cons:**
- Limited to ~2.5A through polyfuse
- Voltage drop across polyfuse (~0.1-0.2V)
- Micro-USB connectors can be fragile
- Cable quality significantly affects performance

**Recommended Power Sources:**
- Official Raspberry Pi Power Supply
- High-quality 2.5A+ power banks
- USB chargers rated 5V/2.5A minimum

---

### 2.2 Raspberry Pi 4 USB Power

**Connector:** USB Type-C

**Specifications:**
- Input Voltage: 5.1V +/- 5% (4.85V - 5.35V)
- Recommended Current: 3.0A minimum
- Maximum Current Draw: ~1.5A (idle), 3A+ under load with peripherals

**Power Path:**
```
USB-C → Polyfuse (3A) → TVS Diode → PMIC → 5V/3.3V/1.8V Rails
```

**Protection Features:**
- **Polyfuse:** 3A hold current
- **TVS Diode:** ESD protection
- **PMIC (MXL7704):** Power management IC with multiple rails
- **Reverse Polarity:** USB-C prevents reversal

**Important USB-C Note (Early Pi 4 Revisions):**
Early Pi 4 boards (Rev 1.1) had a non-compliant USB-C implementation where CC pins were tied together. This caused some USB-C cables with e-marked chips to fail. **Rev 1.2 and later** fixed this issue.

**Workaround for Rev 1.1:**
- Use USB-A to USB-C cables (not USB-C to USB-C)
- Use non-e-marked USB-C cables
- Update to newer Pi 4 revision

**Pros:**
- Higher current capacity (3A)
- Better voltage regulation via PMIC
- More robust connector than Micro-USB
- Supports USB-C PD power banks

**Cons:**
- USB-C PD negotiation can be complex
- Some cheap cables don't provide full 3A
- Still limited by polyfuse for very high loads

---

## 3. GPIO 5V Pin Power (Advanced Method)

### 3.1 Overview

The Raspberry Pi can be powered by applying **regulated 5V directly to the GPIO header**, bypassing the USB input entirely. This is a common method for embedded and robotics applications.

### 3.2 GPIO Power Pins

```
Raspberry Pi GPIO Header (40-pin)
┌─────────────────────────────────────────────────┐
│                    TOP VIEW                      │
│                 (USB ports facing down)          │
│                                                  │
│  Pin 1 (3.3V)  ●  ●  Pin 2 (5V) ←── POWER IN    │
│  Pin 3 (SDA)   ●  ●  Pin 4 (5V) ←── POWER IN    │
│  Pin 5 (SCL)   ●  ●  Pin 6 (GND) ←── GROUND     │
│  Pin 7 (GPIO4) ●  ●  Pin 8 (TXD)                │
│  Pin 9 (GND)   ●  ●  Pin 10 (RXD) ←── GROUND    │
│  ...                                             │
│  Pin 39 (GND)  ●  ●  Pin 40 (GPIO21)            │
│                                                  │
└─────────────────────────────────────────────────┘

POWER INPUT PINS:
├── Pin 2  (5V)  ─┬─ Connected internally, use either or both
├── Pin 4  (5V)  ─┘
├── Pin 6  (GND) ─┬─ Use any GND pin
├── Pin 9  (GND) ─┤
├── Pin 14 (GND) ─┤
├── Pin 20 (GND) ─┤
├── Pin 25 (GND) ─┤
├── Pin 30 (GND) ─┤
├── Pin 34 (GND) ─┤
└── Pin 39 (GND) ─┘
```

### 3.3 Electrical Specifications for GPIO Power

| Parameter | Pi 3B+ | Pi 4 | Notes |
|-----------|--------|------|-------|
| **Voltage Required** | 5.0V +/- 0.25V | 5.1V +/- 0.25V | MUST be regulated |
| **Minimum Voltage** | 4.75V | 4.85V | Below this: undervoltage |
| **Maximum Voltage** | 5.25V | 5.35V | Above this: component damage |
| **Recommended Current** | 2.5A+ | 3.0A+ | Size for peak load |
| **Peak Current** | ~2.5A | ~3.5A | During boot/heavy compute |
| **Absolute Max Voltage** | 5.5V | 5.5V | May damage SoC |

### 3.4 Power Path via GPIO (Bypassing Polyfuse)

**CRITICAL: GPIO power bypasses all protection circuitry**

```
Standard USB Power Path:
USB → Polyfuse → TVS → 5V Rail → Components
        ↓
    (Protection)

GPIO Power Path:
GPIO Pin 2/4 → 5V Rail → Components (DIRECTLY!)
        ↓
    (NO Protection!)
```

### 3.5 Polyfuse Bypass - Risks and Benefits

**What is the Polyfuse?**
- A resettable fuse (PTC thermistor)
- Hold current: 2.5A (Pi 3B+) / 3A (Pi 4)
- Trip current: ~5A
- Resistance: ~50mΩ (adds voltage drop)

**Benefits of Bypassing (GPIO Power):**
| Benefit | Explanation |
|---------|-------------|
| Higher current capacity | No 2.5-3A limitation |
| Lower voltage drop | No ~0.1-0.2V drop across polyfuse |
| More stable voltage | Eliminates polyfuse resistance variation |
| Better for high-power HATs | Can support HATs drawing >2A |
| Cleaner power delivery | Direct path to 5V rail |

**Risks of Bypassing (GPIO Power):**
| Risk | Consequence | Mitigation |
|------|-------------|------------|
| **No overcurrent protection** | Short circuit can damage SoC, traces | Add external fuse |
| **No reverse polarity protection** | Reversed connection destroys Pi | Add diode/MOSFET protection |
| **No ESD protection** | Static discharge can damage Pi | Handle carefully, use TVS |
| **No hot-plug safety** | Connecting live can cause arcing | Power off before connecting |
| **Overvoltage damage** | >5.5V destroys components | Use quality regulator |

### 3.6 Voltage Regulation Requirements

**CRITICAL: You MUST use a regulated 5V supply. Raw battery voltage will destroy the Pi.**

**Acceptable Voltage Sources:**
- Switching buck converter (recommended)
- Linear regulator (inefficient, gets hot)
- Quality UBEC (Universal Battery Eliminator Circuit)

**Unacceptable Voltage Sources:**
- Raw LiPo battery (7.4V-12.6V will destroy Pi)
- Unregulated power supply
- Voltage divider (cannot handle current, voltage varies with load)
- Zener diode regulation (insufficient current, poor regulation)

**Voltage Regulation Specifications:**

| Parameter | Minimum | Recommended | Maximum |
|-----------|---------|-------------|---------|
| Output Voltage | 4.9V | 5.0-5.1V | 5.25V |
| Voltage Ripple | - | <50mV pp | <100mV pp |
| Load Regulation | - | <2% | <5% |
| Line Regulation | - | <1% | <3% |
| Efficiency | 80% | 90%+ | - |

### 3.7 Current Requirements by Configuration

**Raspberry Pi 3B+ Current Draw:**
| State | Current (Pi only) | With Peripherals |
|-------|-------------------|------------------|
| Idle | 350-400mA | 500-800mA |
| Moderate load | 500-600mA | 800-1200mA |
| Heavy load (stress) | 700-750mA | 1200-1500mA |
| Boot peak | 700-800mA | 1000-1500mA |
| **Recommended Supply** | 1.5A min | **2.5A+ recommended** |

**Raspberry Pi 4 Current Draw:**
| State | Current (Pi only) | With Peripherals |
|-------|-------------------|------------------|
| Idle | 550-600mA | 800-1200mA |
| Moderate load | 800-1000mA | 1200-1800mA |
| Heavy load (4 cores) | 1200-1500mA | 1800-2500mA |
| Boot peak | 1000-1200mA | 1500-2000mA |
| **Recommended Supply** | 2A min | **3A+ recommended** |

**Rule of Thumb:** Size your regulator for **150% of expected peak current** to ensure stable operation and component longevity.

---

## 4. Power over Ethernet (PoE)

### 4.1 PoE Overview

Power over Ethernet delivers power alongside data through standard Ethernet cables. Requires a PoE HAT on the Pi and a PoE-capable switch or injector.

### 4.2 PoE HAT for Raspberry Pi

**Official Raspberry Pi PoE+ HAT:**
- Input: 802.3af (PoE) or 802.3at (PoE+)
- Output: 5V @ 4A (20W max)
- Compatible: Pi 3B+, Pi 4
- Price: ~$20-25

**Specifications:**
| Parameter | 802.3af (PoE) | 802.3at (PoE+) |
|-----------|---------------|----------------|
| Voltage | 44-57V DC | 50-57V DC |
| Max Power | 15.4W | 30W |
| Max at Device | 12.95W | 25.5W |
| Pi HAT Output | 5V @ 2.5A | 5V @ 4A |

### 4.3 PoE Power Path

```
PoE Switch/Injector (48V)
         │
         ▼
    Ethernet Cable
    (Cat5e/Cat6)
         │
         ▼
┌─────────────────────┐
│     PoE+ HAT        │
│  ┌───────────────┐  │
│  │ PoE Converter │  │
│  │  48V → 5V     │  │
│  └───────┬───────┘  │
└──────────┼──────────┘
           │
           ▼
    GPIO 5V Pins (2 & 4)
           │
           ▼
    Raspberry Pi 5V Rail
```

### 4.4 PoE Pros and Cons

**Pros:**
- Single cable for power and data
- Centralized power management
- Remote power cycling capability
- Clean, professional installation
- No local power supply needed
- Full protection via HAT

**Cons:**
- Requires PoE switch/injector (~$30-200)
- HAT adds cost (~$20-25)
- Not suitable for mobile robots (tethered)
- Lower efficiency than direct DC
- Adds heat from DC-DC conversion

**Best Use Cases:**
- Stationary Pi installations
- Network equipment rooms
- Surveillance/camera applications
- Industrial automation
- NOT for mobile robotics

---

## 5. GPIO vs USB Comparison

### 5.1 Detailed Comparison Table

| Feature | USB Power | GPIO Power |
|---------|-----------|------------|
| **Connector** | Micro-USB (3B+) / USB-C (Pi 4) | 2.54mm header pins |
| **Voltage Input** | 5V (regulated by source) | 5V (MUST be regulated) |
| **Polyfuse Protection** | Yes (2.5-3A) | **NO** |
| **Reverse Polarity Protection** | Yes (connector) | **NO** |
| **ESD/TVS Protection** | Yes | **NO** |
| **Maximum Current** | Limited by polyfuse | Limited by supply & traces |
| **Voltage Drop** | ~0.1-0.2V (polyfuse) | Minimal (~0.05V) |
| **Hot Plugging** | Safe | **Risky** |
| **Cable Quality Impact** | High | N/A (direct connection) |
| **Suitable for Beginners** | Yes | No (requires knowledge) |
| **Mobile Robotics** | OK | **Preferred** |
| **High-Power HATs** | Limited | Better |
| **System Reliability** | Good | Excellent (with proper design) |
| **Wiring Complexity** | Simple | More complex |

### 5.2 When to Use Each Method

**Use USB Power When:**
- Learning/prototyping
- Using commercial power supplies/banks
- Hot-plugging is needed
- Maximum safety is priority
- No special power requirements
- Using standard accessories

**Use GPIO Power When:**
- Building mobile robots
- Powering from LiPo/Li-ion batteries
- Need higher current (>2.5A)
- Using high-power HATs
- Want cleaner voltage delivery
- Integrating with existing power systems
- Custom embedded applications

### 5.3 Performance Comparison

| Metric | USB (Pi 4) | GPIO (Pi 4) | Notes |
|--------|------------|-------------|-------|
| Voltage Stability | 4.9-5.0V typical | 5.0-5.1V typical | GPIO more stable |
| Boot Reliability | 98%+ | 99%+ (with good PSU) | GPIO slightly better |
| Max Sustained Load | ~2.8A | >3A | GPIO can exceed polyfuse |
| Undervoltage Events | More common | Rare | GPIO avoids polyfuse drop |
| Heat Generation | Low | Very low | No polyfuse heating |

---

## 6. Wiring Diagrams for GPIO Power from LiPo

### 6.1 2S LiPo (7.4V Nominal) with Buck Converter

**Battery Specifications:**
- Nominal Voltage: 7.4V (2 x 3.7V)
- Full Charge: 8.4V (2 x 4.2V)
- Discharged: 6.0V (2 x 3.0V cutoff)
- Usable Range: 6.4V - 8.4V

**Recommended Buck Converters:**
- Pololu D24V30F5 (3A)
- Adafruit UBEC (3A, 6-16V input)
- MP2307-based modules (3A)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    2S LiPo to Raspberry Pi Wiring                       │
└─────────────────────────────────────────────────────────────────────────┘

    ┌──────────────────────┐
    │     2S LiPo          │
    │   7.4V / 2200mAh     │
    │                      │
    │  (+) RED ────────────┼─────┐
    │  (-) BLACK ──────────┼──┐  │
    │  (Balance) ──────────┼──┼──┼─── (For charging only)
    └──────────────────────┘  │  │
                              │  │
                              │  │  ┌──────────────────┐
                              │  │  │  OPTIONAL:       │
                              │  │  │  Inline Fuse     │
                              │  │  │  5A Fast-Blow    │
                              │  │  └────────┬─────────┘
                              │  │           │
                              │  │           │
    ┌─────────────────────────┼──┼───────────┼─────────────────────────────┐
    │                         │  │           │                             │
    │  ┌──────────────────────┼──┼───────────┼──────────────────────┐      │
    │  │     BUCK CONVERTER   │  │           │                      │      │
    │  │     (Pololu D24V30F5)│  │           │                      │      │
    │  │                      │  │           │                      │      │
    │  │  VIN+ ◄──────────────┼──┼───────────┘                      │      │
    │  │                      │  │                                  │      │
    │  │  GND  ◄──────────────┼──┘                                  │      │
    │  │                      │                                     │      │
    │  │  VOUT+ (5V) ─────────┼───────────────────────────┐         │      │
    │  │                      │                           │         │      │
    │  │  GND ────────────────┼─────────┐                 │         │      │
    │  │                      │         │                 │         │      │
    │  └──────────────────────┘         │                 │         │      │
    │                                   │                 │         │      │
    └───────────────────────────────────┼─────────────────┼─────────┘      │
                                        │                 │                │
                                        │                 │                │
    ┌───────────────────────────────────┼─────────────────┼────────────────┘
    │                                   │                 │
    │   RASPBERRY PI GPIO HEADER        │                 │
    │   ┌─────────────────────────────┐ │                 │
    │   │                             │ │                 │
    │   │   Pin 2 (5V)  ◄─────────────┼─┘                 │
    │   │   Pin 4 (5V)  (leave unconnected or parallel)   │
    │   │   Pin 6 (GND) ◄─────────────────────────────────┘
    │   │                             │
    │   └─────────────────────────────┘
    │
    └──────────────────────────────────────────────────────────────────────

    WIRE GAUGE RECOMMENDATIONS:
    ├── Battery to Buck: 18-20 AWG (handles full current)
    ├── Buck to Pi: 20-22 AWG (5V output current)
    └── Ground: Same gauge as positive

    CONNECTION NOTES:
    ├── Use quality connectors (XT30, JST-XH)
    ├── Solder or crimp connections (no breadboard for power!)
    └── Keep wires short to minimize voltage drop
```

**Runtime Calculation (2S 2200mAh):**
```
Battery Energy = 7.4V × 2.2Ah = 16.3Wh
Converter Efficiency = ~90%
Usable Energy = 16.3 × 0.9 = 14.7Wh
Pi 4 Average Power = ~5W (moderate load)
Runtime = 14.7Wh / 5W = ~2.9 hours
```

---

### 6.2 3S LiPo (11.1V Nominal) with Buck Converter

**Battery Specifications:**
- Nominal Voltage: 11.1V (3 x 3.7V)
- Full Charge: 12.6V (3 x 4.2V)
- Discharged: 9.0V (3 x 3.0V cutoff)
- Usable Range: 9.6V - 12.6V

**Recommended Buck Converters:**
- Pololu D24V50F5 (5A, excellent regulation)
- DFRobot DC-DC Converter
- LM2596-based modules (budget option)
- Adafruit UBEC (3A, up to 16V input)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    3S LiPo to Raspberry Pi Wiring                       │
└─────────────────────────────────────────────────────────────────────────┘

    ┌──────────────────────┐
    │     3S LiPo          │
    │  11.1V / 5000mAh     │
    │     25C Rating       │
    │                      │
    │  (+) RED ────────────┼─────────────────────────────────────┐
    │  (-) BLACK ──────────┼─────────────────────────────┐       │
    │  (Balance) ──────────┼─ (For charging only)        │       │
    └──────────────────────┘                             │       │
                                                         │       │
                      ┌──────────────────────────┐       │       │
                      │    MAIN SWITCH           │       │       │
                      │    (SPST, 10A rated)     │       │       │
                      └────────────┬─────────────┘       │       │
                                   │                     │       │
                      ┌────────────┴─────────────┐       │       │
                      │    FUSE HOLDER           │       │       │
                      │    (5A blade fuse)       │       │       │
                      └────────────┬─────────────┘       │       │
                                   │                     │       │
    ┌──────────────────────────────┼─────────────────────┼───────┼────────┐
    │                              │                     │       │        │
    │  ┌───────────────────────────┼─────────────────────┼───────┼───┐    │
    │  │     BUCK CONVERTER        │                     │       │   │    │
    │  │     (Pololu D24V50F5)     │                     │       │   │    │
    │  │                           │                     │       │   │    │
    │  │  VIN+ ◄───────────────────┼─────────────────────┼───────┘   │    │
    │  │                           │                     │           │    │
    │  │  GND  ◄───────────────────┼─────────────────────┘           │    │
    │  │                           │                                 │    │
    │  │  VOUT+ (5V) ──────────────┼──────────────┐                  │    │
    │  │                           │              │                  │    │
    │  │  GND ─────────────────────┼──────┐       │                  │    │
    │  │                           │      │       │                  │    │
    │  │  ENABLE ──────────────────┼──┐   │       │   (Optional:     │    │
    │  │  (pull high or connect    │  │   │       │    tie to VIN    │    │
    │  │   to VIN for always-on)   │  │   │       │    or use GPIO   │    │
    │  │                           │  │   │       │    for shutdown) │    │
    │  └───────────────────────────┘  │   │       │                  │    │
    │                                 │   │       │                  │    │
    │  OPTIONAL: 100µF cap ───────────┼───┼───────┼─ across 5V/GND   │    │
    │  (smooths voltage)              │   │       │                  │    │
    └─────────────────────────────────┼───┼───────┼──────────────────┘    │
                                      │   │       │                       │
                                      │   │       │                       │
    ┌─────────────────────────────────┼───┼───────┼───────────────────────┘
    │                                 │   │       │
    │   RASPBERRY PI                  │   │       │
    │   ┌─────────────────────────────┼───┼───────┼───────────────┐
    │   │  GPIO HEADER                │   │       │               │
    │   │                             │   │       │               │
    │   │   Pin 1 (3.3V)  ●  ●  Pin 2 (5V) ◄──────┘               │
    │   │   Pin 3 (SDA)   ●  ●  Pin 4 (5V) ─── (optional 2nd 5V) │
    │   │   Pin 5 (SCL)   ●  ●  Pin 6 (GND) ◄─────┘               │
    │   │   Pin 7 (GPIO4) ●  ●  Pin 8 (TXD)                       │
    │   │   Pin 9 (GND)   ●  ●  Pin 10 (RXD) ◄── (alt GND)        │
    │   │   ...                                                   │
    │   │   Optional: GPIO for ENABLE control ◄───┘               │
    │   │                                                         │
    │   └─────────────────────────────────────────────────────────┘
    │
    └──────────────────────────────────────────────────────────────────────

    WIRE SPECIFICATIONS:
    ┌──────────────────────────────────────────────────────────────────────┐
    │  Section              │ Wire Gauge │ Max Length │ Notes             │
    ├───────────────────────┼────────────┼────────────┼───────────────────┤
    │  Battery to Fuse      │ 16-18 AWG  │ 15cm       │ Silicone jacket   │
    │  Fuse to Buck Input   │ 18-20 AWG  │ 10cm       │ Silicone jacket   │
    │  Buck Output to Pi    │ 20-22 AWG  │ 10cm       │ Keep short!       │
    │  Ground (all)         │ Same as +  │ -          │ Star ground ideal │
    └──────────────────────────────────────────────────────────────────────┘
```

**Runtime Calculation (3S 5000mAh):**
```
Battery Energy = 11.1V × 5.0Ah = 55.5Wh
Converter Efficiency = ~90%
Usable Energy = 55.5 × 0.9 = 50Wh
Pi 4 Average Power = ~5W (moderate load)
Runtime = 50Wh / 5W = ~10 hours
```

---

### 6.3 Complete Robot Power System (3S LiPo, Pi + Motors)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│           COMPLETE ROBOT POWER SYSTEM - 3S LiPo Single Battery              │
└─────────────────────────────────────────────────────────────────────────────┘

                    ┌───────────────────────────┐
                    │       3S LiPo             │
                    │    11.1V / 5000mAh        │
                    │       25C Rating          │
                    │                           │
                    │   (+) ────────────────────┼────────────┐
                    │   (-) ────────────────────┼─────────┐  │
                    └───────────────────────────┘         │  │
                                                          │  │
                                                          │  │
                              ┌────────────────────────┐  │  │
                              │      MAIN SWITCH       │  │  │
                              │      (DPST 10A)        │  │  │
                              └───────────┬────────────┘  │  │
                                          │               │  │
                              ┌───────────┴────────────┐  │  │
                              │   DISTRIBUTION POINT   │  │  │
                              │      (Terminal Block)  │◄─┼──┘
                              └───────────┬────────────┘  │
                                          │               │
                    ┌─────────────────────┼───────────────┼─────────────────┐
                    │                     │               │                 │
              ┌─────┴─────┐         ┌─────┴─────┐   ┌─────┴─────┐           │
              │   FUSE    │         │   FUSE    │   │   FUSE    │           │
              │ 3A (Logic)│         │ 10A(Motor)│   │ (Optional)│           │
              └─────┬─────┘         └─────┬─────┘   └───────────┘           │
                    │                     │                                  │
                    │                     │                                  │
    ┌───────────────┴──────────┐    ┌────┴─────────────────────────────┐    │
    │                          │    │                                  │    │
    │  ┌────────────────────┐  │    │  ┌─────────────────────────────┐ │    │
    │  │  BUCK CONVERTER    │  │    │  │       MOTOR DRIVER          │ │    │
    │  │  (Pololu D24V50F5) │  │    │  │       (L298N / TB6612)      │ │    │
    │  │                    │  │    │  │                             │ │    │
    │  │  VIN+ ◄────────────┼──┘    │  │   VIN (12V) ◄───────────────┼─┘    │
    │  │  GND  ◄────────────┼───────┼──┼── GND ◄─────────────────────┼──────┤
    │  │                    │       │  │                             │      │
    │  │  VOUT+ (5V) ───────┼──┐    │  │   OUT1/2 ────► Motor A      │      │
    │  │  GND ──────────────┼──┼────┤  │   OUT3/4 ────► Motor B      │      │
    │  │                    │  │    │  │                             │      │
    │  └────────────────────┘  │    │  │   ENA/ENB ◄── Pi PWM GPIO   │      │
    │                          │    │  │   IN1-4 ◄──── Pi GPIO       │      │
    │                          │    │  │                             │      │
    │                          │    │  │   5V Logic ◄─── (from Buck) │      │
    │                          │    │  │   GND Logic ◄── (common)    │      │
    │                          │    │  └─────────────────────────────┘      │
    └──────────────────────────┼────┘                                       │
                               │                                            │
    ┌──────────────────────────┼────────────────────────────────────────────┘
    │                          │
    │   RASPBERRY PI           │
    │   ┌──────────────────────┼───────────────────────────────────────┐
    │   │  GPIO HEADER         │                                       │
    │   │                      │                                       │
    │   │   Pin 2 (5V)  ◄──────┘   (Power from Buck)                   │
    │   │   Pin 6 (GND) ◄──────────(Common Ground)                     │
    │   │                                                              │
    │   │   Motor Control Pins:                                        │
    │   │   Pin 12 (GPIO18) ──────► ENA (PWM for Motor A speed)        │
    │   │   Pin 16 (GPIO23) ──────► IN1 (Motor A direction)            │
    │   │   Pin 18 (GPIO24) ──────► IN2 (Motor A direction)            │
    │   │   Pin 35 (GPIO19) ──────► ENB (PWM for Motor B speed)        │
    │   │   Pin 11 (GPIO17) ──────► IN3 (Motor B direction)            │
    │   │   Pin 13 (GPIO27) ──────► IN4 (Motor B direction)            │
    │   │                                                              │
    │   │   I2C (for IMU/sensors):                                     │
    │   │   Pin 3 (SDA) ──────────► IMU SDA                            │
    │   │   Pin 5 (SCL) ──────────► IMU SCL                            │
    │   │                                                              │
    │   └──────────────────────────────────────────────────────────────┘
    │
    │   USB PERIPHERALS:
    │   ├── USB Webcam
    │   ├── USB LiDAR
    │   └── (powered from Pi 5V rail via USB)
    │
    └──────────────────────────────────────────────────────────────────────────


    GROUND ROUTING (IMPORTANT):
    ┌──────────────────────────────────────────────────────────────────────────┐
    │                                                                          │
    │   Use STAR GROUND topology to prevent ground loops and motor noise:      │
    │                                                                          │
    │                    Battery GND (-)                                       │
    │                         │                                                │
    │             ┌───────────┼───────────┐                                    │
    │             │           │           │                                    │
    │             ▼           ▼           ▼                                    │
    │        Buck GND    Motor Driver   Pi GND                                 │
    │                       GND       (via Buck)                               │
    │                                                                          │
    │   NOTE: All grounds connect at ONE point near the battery                │
    │                                                                          │
    └──────────────────────────────────────────────────────────────────────────┘
```

---

## 7. Buck Converter Recommendations

### 7.1 Premium Tier (Best Regulation, Highest Reliability)

| Model | Input Range | Output | Current | Efficiency | Price | Notes |
|-------|-------------|--------|---------|------------|-------|-------|
| **Pololu D24V50F5** | 4.5-38V | 5.0V | 5A cont, 6.5A peak | 90-95% | ~$15 | Best overall choice |
| **Pololu D24V30F5** | 4.5-38V | 5.0V | 3.4A cont | 90-95% | ~$12 | Good for Pi 3B+ |
| **Pololu D36V50F5** | 4.5-50V | 5.0V | 5.5A cont | 90-95% | ~$20 | Higher input voltage |
| **Pololu S18V20F5** | 3-30V | 5.0V | 2A cont | 90%+ | ~$15 | Step-up/down |

**Why Pololu?**
- Excellent voltage regulation (±2%)
- Low output ripple
- Enable pin for power control
- Compact form factor
- Well-documented specifications

### 7.2 Mid-Tier (Good Value)

| Model | Input Range | Output | Current | Efficiency | Price | Notes |
|-------|-------------|--------|---------|------------|-------|-------|
| **Adafruit UBEC** | 6-16V | 5.0V | 3A cont | ~85% | ~$9 | Proven for robotics |
| **DFRobot Buck** | 7.5-30V | 5.0V | 3A cont | ~85% | ~$8 | Dual output available |
| **Drok LM2596** | 4-35V | 5.0V adj | 3A cont | ~80% | ~$8 | Adjustable, display option |

### 7.3 Budget Tier (Use with Caution)

| Model | Input Range | Output | Current | Efficiency | Price | Notes |
|-------|-------------|--------|---------|------------|-------|-------|
| **LM2596 Module** | 4-40V | Adjustable | 3A | 75-85% | ~$2-4 | Needs adjustment, verify output |
| **MP1584 Mini** | 4.5-28V | Adjustable | 3A | 92% | ~$1-3 | Very small, verify specs |
| **XL4015 Module** | 8-36V | Adjustable | 5A | 85% | ~$3-5 | Higher current |

**Budget Module Warnings:**
- Output voltage may drift - verify with multimeter
- Adjust BEFORE connecting to Pi
- Quality varies between suppliers
- May have higher ripple
- Efficiency claims often exaggerated

### 7.4 Buck Converter Selection Guide

```
┌─────────────────────────────────────────────────────────────────────┐
│                BUCK CONVERTER SELECTION FLOWCHART                   │
└─────────────────────────────────────────────────────────────────────┘

                    What is your battery voltage?
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
          2S LiPo         3S LiPo         4S+ LiPo
         (6.0-8.4V)      (9.0-12.6V)     (12.0-16.8V+)
              │               │               │
              ▼               ▼               ▼
        Min 6V input    Min 9V input    Min 12V input
              │               │               │
              └───────────────┼───────────────┘
                              │
                              ▼
                    What current do you need?
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
          <2.5A           2.5-4A            >4A
        (Pi 3B+ only)   (Pi 4 standard)  (Pi 4 + HATs)
              │               │               │
              ▼               ▼               ▼
        Pololu D24V30   Pololu D24V50   Pololu D36V50
        or UBEC 3A      or DFRobot 5A   or XL4015 5A
              │               │               │
              └───────────────┼───────────────┘
                              │
                              ▼
                    What is your budget?
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
          Premium         Mid-Range        Budget
          ($15-25)        ($8-15)          (<$5)
              │               │               │
              ▼               ▼               ▼
         Pololu          Adafruit UBEC    LM2596/MP1584
         (Best)          or DFRobot       (Verify output!)
```

---

## 8. Safety Considerations for GPIO Power

### 8.1 Critical Safety Points

**GPIO power bypasses ALL built-in protection. You MUST understand these risks:**

#### 8.1.1 No Reverse Polarity Protection

**Risk:** Connecting power backwards will **instantly destroy** the Raspberry Pi.

**Consequences of Reverse Polarity:**
- Immediate destruction of SoC
- Damage to PMIC (Pi 4)
- Destruction of 3.3V regulator
- Potential fire hazard
- Board is unrepairable

**Mitigation:**
```
Method 1: Schottky Diode (Simple)
┌─────────────────────────────────────────────────┐
│                                                 │
│  (+) Battery ──►|──── Schottky ──► Pi Pin 2    │
│                      (SS34/SB540)               │
│  Voltage Drop: 0.3-0.5V                         │
│  Current: Size for expected load                │
│                                                 │
└─────────────────────────────────────────────────┘

Method 2: P-Channel MOSFET (Better)
┌─────────────────────────────────────────────────┐
│                                                 │
│  (+) Battery ──┬──────────────────► Pi Pin 2   │
│                │    ┌──────┐                    │
│                └────┤ Gate │                    │
│                     │  P-  │                    │
│  (-) Battery ───────┤ FET  │                    │
│                     └──────┘                    │
│  Voltage Drop: <0.1V (RDS(on) × I)             │
│  Recommended: SI2301 or IRF4905                │
│                                                 │
└─────────────────────────────────────────────────┘

Method 3: Polarized Connector (Preventive)
- Use XT30/XT60 connectors (keyed, cannot reverse)
- Use JST connectors with correct polarity
- NEVER use bare wires or non-polarized connectors
```

#### 8.1.2 No Overcurrent Protection

**Risk:** Short circuits or excessive current draw can damage traces, components, or cause fire.

**Consequences:**
- PCB trace burnout
- Component overheating
- Fire hazard
- Damage to connected peripherals

**Mitigation:**
```
Add External Fuse:
┌─────────────────────────────────────────────────┐
│                                                 │
│  (+) Regulator Out ── [FUSE] ──► Pi Pin 2      │
│                                                 │
│  Fuse Selection:                                │
│  ├── Pi 3B+: 3A fast-blow or 4A slow-blow      │
│  ├── Pi 4:   4A fast-blow or 5A slow-blow      │
│  └── Type: Blade fuse, glass fuse, or PTC      │
│                                                 │
│  PTC (Resettable) Alternative:                  │
│  ├── MF-R300 (3A hold) for Pi 3B+              │
│  └── MF-R400 (4A hold) for Pi 4                │
│                                                 │
└─────────────────────────────────────────────────┘
```

#### 8.1.3 No ESD Protection

**Risk:** Static discharge through GPIO pins can damage the Pi.

**Mitigation:**
- Add TVS diode (SMBJ5.0A) across 5V and GND
- Handle with proper ESD precautions
- Use grounded enclosure in dry environments

#### 8.1.4 Overvoltage Risk

**Risk:** Voltage above 5.5V will damage the Pi.

**Causes:**
- Regulator failure
- Load dump transients
- Incorrect regulator setting
- Unregulated battery connection

**Mitigation:**
```
Add Overvoltage Protection:
┌─────────────────────────────────────────────────┐
│                                                 │
│  Option 1: Zener Clamp                          │
│  5V Rail ──┬──────────────► Pi                  │
│            │                                    │
│            └── 5.1V Zener ──► GND               │
│            (Clamps spikes above 5.1V)           │
│                                                 │
│  Option 2: TVS Diode                            │
│  5V Rail ──┬──────────────► Pi                  │
│            │                                    │
│            └── SMBJ5.0A ────► GND               │
│            (Clamps transients, faster than Zener)│
│                                                 │
│  Option 3: Crowbar Circuit (Ultimate)           │
│  - SCR triggers on overvoltage                  │
│  - Blows fuse to disconnect power               │
│  - Protects against sustained overvoltage       │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 8.2 Hot-Plugging Warning

**NEVER connect or disconnect GPIO power while the system is energized.**

**Risks of Hot-Plugging GPIO Power:**
- Voltage transients during connection
- Potential for incorrect pin contact order
- Arcing can damage pins and connectors
- Incomplete contact can cause undervoltage

**Safe Power-Up Sequence:**
1. Connect all wiring with power OFF
2. Verify all connections
3. Turn on main switch
4. Pi boots normally

**Safe Power-Down Sequence:**
1. `sudo shutdown -h now`
2. Wait for green LED to stop blinking
3. Turn off main switch
4. Then disconnect if needed

### 8.3 Safety Checklist for GPIO Power

```
BEFORE FIRST POWER-ON:
□ Verify regulator output is 5.0-5.1V with multimeter
□ Check polarity at GPIO pins (Pin 2 = +5V, Pin 6 = GND)
□ Confirm no shorts between 5V and GND
□ Verify all solder joints are solid
□ Check wire gauge is adequate
□ Confirm fuse is installed and rated correctly
□ Test with dummy load before connecting Pi

DURING OPERATION:
□ Monitor for undervoltage warnings
□ Check temperature of regulator (should be warm, not hot)
□ Verify stable voltage under load

REGULAR MAINTENANCE:
□ Inspect connections for corrosion
□ Check battery health
□ Verify fuse integrity
□ Clean dust from electronics
```

---

## 9. Protection Circuits

### 9.1 Recommended Minimum Protection

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    MINIMUM PROTECTION CIRCUIT                           │
└─────────────────────────────────────────────────────────────────────────┘

                    ┌─────────────────────────────────────────────────┐
                    │                BUCK CONVERTER                    │
                    │                (5V Regulated)                    │
                    └──────────────────────┬──────────────────────────┘
                                           │
                                           │ 5V Output
                                           │
    ┌──────────────────────────────────────┼──────────────────────────────┐
    │                                      │                              │
    │         ┌────────────────────────────┼────────────────────┐         │
    │         │                            │                    │         │
    │         │    ┌───────────┐     ┌─────┴─────┐     ┌───────┴───────┐ │
    │         │    │  FUSE     │     │   TVS     │     │  CAPACITOR    │ │
    │    (+) ─┼────┤  4A       ├─────┤  SMBJ5.0A │     │  100µF/10V    │ │
    │         │    │  Fast-Blow│     │  Bidirect │     │  Electrolytic │ │
    │         │    └─────┬─────┘     └─────┬─────┘     └───────┬───────┘ │
    │         │          │                 │                   │         │
    │         │          │                 │                   │         │
    │         │          │ ┌───────────────┴───────────────────┘         │
    │         │          │ │                                             │
    │         │          └─┼──────────────────────────────► Pi Pin 2     │
    │         │            │                                (5V)         │
    │         │            │                                             │
    │    (-) ─┼────────────┴──────────────────────────────► Pi Pin 6     │
    │         │                                             (GND)        │
    │         │                                                          │
    │         └──────────────────────────────────────────────────────────┘
    │
    │    COMPONENT VALUES:
    │    ├── Fuse: 4A fast-blow (or 5A PTC resettable)
    │    ├── TVS: SMBJ5.0A bidirectional
    │    └── Capacitor: 100µF 10V electrolytic (low ESR preferred)
    │
    └─────────────────────────────────────────────────────────────────────────
```

### 9.2 Enhanced Protection (Reverse Polarity + Overcurrent + Overvoltage)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ENHANCED PROTECTION CIRCUIT                          │
└─────────────────────────────────────────────────────────────────────────┘

    Battery/Regulator Input
           │
           │ (Unprotected 5V)
           │
    ┌──────┴──────────────────────────────────────────────────────────────┐
    │                                                                      │
    │      ┌─────────────────────────────────────────────────────────┐    │
    │      │              REVERSE POLARITY PROTECTION                 │    │
    │      │                                                          │    │
    │      │     (+) ──┬──────────────────────────────────► OUT (+)   │    │
    │      │           │         ┌────────┐                           │    │
    │      │           │         │   P-   │                           │    │
    │      │           └─────────┤  FET   ├───┐                       │    │
    │      │                     │ SI2301 │   │                       │    │
    │      │     (-) ────────────┴────────┴───┴───────────► OUT (-)   │    │
    │      │                                                          │    │
    │      │     When polarity is correct: MOSFET conducts            │    │
    │      │     When reversed: MOSFET blocks, protects circuit       │    │
    │      │                                                          │    │
    │      └──────────────────────────────────────────────────────────┘    │
    │                              │                                       │
    │                              │ Protected from reversal               │
    │                              │                                       │
    │      ┌───────────────────────┴──────────────────────────────────┐    │
    │      │              OVERCURRENT PROTECTION                       │    │
    │      │                                                           │    │
    │      │     (+) ──── [  FUSE 4A  ] ──────────────────► OUT (+)    │    │
    │      │                                                           │    │
    │      │     Alternative: PTC Resettable Fuse (MF-R400)           │    │
    │      │                                                           │    │
    │      └───────────────────────┬──────────────────────────────────┘    │
    │                              │                                       │
    │                              │ Protected from overcurrent            │
    │                              │                                       │
    │      ┌───────────────────────┴──────────────────────────────────┐    │
    │      │              OVERVOLTAGE PROTECTION                       │    │
    │      │                                                           │    │
    │      │     (+) ──┬─────────────────────────────────► OUT (+)     │    │
    │      │           │                                               │    │
    │      │           │    ┌─────────┐                                │    │
    │      │           └────┤  TVS    ├──────┐                         │    │
    │      │                │SMBJ5.0A │      │                         │    │
    │      │                └─────────┘      │                         │    │
    │      │     (-) ───────────────────────┴─────────────► OUT (-)    │    │
    │      │                                                           │    │
    │      │     TVS clamps voltage spikes above ~6V                   │    │
    │      │                                                           │    │
    │      └───────────────────────────────────────────────────────────┘    │
    │                              │                                       │
    │                              ▼                                       │
    │                    TO RASPBERRY PI                                   │
    │                    (Pin 2 = 5V, Pin 6 = GND)                        │
    │                                                                      │
    └──────────────────────────────────────────────────────────────────────┘


    BILL OF MATERIALS:
    ┌──────────────────────────────────────────────────────────────────────┐
    │  Component          │  Part Number       │  Qty  │  Est. Cost       │
    ├─────────────────────┼────────────────────┼───────┼──────────────────┤
    │  P-Channel MOSFET   │  SI2301CDS         │  1    │  $0.30           │
    │  PTC Fuse 4A        │  MF-R400           │  1    │  $0.50           │
    │  TVS Diode          │  SMBJ5.0A          │  1    │  $0.30           │
    │  Capacitor 100µF    │  Electrolytic 10V  │  1    │  $0.10           │
    │  Capacitor 0.1µF    │  Ceramic           │  1    │  $0.05           │
    ├─────────────────────┼────────────────────┼───────┼──────────────────┤
    │  TOTAL              │                    │       │  ~$1.25          │
    └──────────────────────────────────────────────────────────────────────┘
```

### 9.3 PCB-Ready Protection Module Design

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    PROTECTION MODULE SCHEMATIC                          │
└─────────────────────────────────────────────────────────────────────────┘

                            VIN (5V from regulator)
                                    │
                                    │
    ┌───────────────────────────────┴───────────────────────────────────┐
    │                                                                   │
    │   ┌─────┐                                                         │
    │   │     │ Q1: SI2301CDS (P-ch MOSFET, SOT-23)                     │
    │   │ S   ├────────────────────────────────────────┐                │
    │   │     │                                        │                │
    │   │  G  ├───────────────────────┐                │                │
    │   │     │                       │                │                │
    │   │ D   ├─────┐                 │                │                │
    │   │     │     │                 │                │                │
    │   └─────┘     │                 │                │                │
    │               │                 │                │                │
    │         ┌─────┴─────┐    ┌──────┴──────┐   ┌─────┴─────┐          │
    │         │           │    │             │   │           │          │
    │         │   R1      │    │    R2       │   │   F1      │          │
    │         │  10kΩ     │    │   100kΩ     │   │  PTC 4A   │          │
    │         │           │    │  (optional) │   │  MF-R400  │          │
    │         │           │    │             │   │           │          │
    │         └─────┬─────┘    └──────┬──────┘   └─────┬─────┘          │
    │               │                 │                │                │
    │               │                 │                │                │
    │    ───────────┴─────────────────┴────────────────┴──── GND        │
    │                                                  │                │
    │                                                  │                │
    │                               ┌──────────────────┴───────┐        │
    │                               │                          │        │
    │                         ┌─────┴─────┐             ┌──────┴──────┐ │
    │                         │           │             │             │ │
    │                         │   D1      │             │    C1       │ │
    │                         │ SMBJ5.0A  │             │   100µF     │ │
    │                         │   TVS     │             │   10V       │ │
    │                         │           │             │             │ │
    │                         └─────┬─────┘             └──────┬──────┘ │
    │                               │                          │        │
    │                               │                          │        │
    │    ───────────────────────────┴──────────────────────────┴── GND  │
    │                                                                   │
    │                                                                   │
    │                     VOUT ────────────────────► To Pi Pin 2 (5V)   │
    │                     GND  ────────────────────► To Pi Pin 6 (GND)  │
    │                                                                   │
    └───────────────────────────────────────────────────────────────────┘


    OPERATION:
    ├── Normal: Q1 conducts (Vgs < -2V), power flows through
    ├── Reversed: Q1 blocks (Vgs > 0), no current flows
    ├── Overcurrent: F1 (PTC) trips, limits current
    ├── Overvoltage: D1 clamps spikes, C1 filters ripple
    └── Typical Vdrop: <0.15V at 3A
```

---

## 10. Troubleshooting

### 10.1 Common GPIO Power Problems

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Pi won't boot | Insufficient current | Use higher-rated regulator |
| Pi won't boot | Voltage too low (<4.75V) | Check regulator output, reduce load |
| Pi won't boot | Reversed polarity | Check wiring (may need new Pi) |
| Random reboots | Voltage dips under load | Add capacitors, check wiring gauge |
| Lightning bolt icon | Undervoltage detected | Increase supply current, check connections |
| Hot regulator | Overloaded or inefficient | Use higher-rated or more efficient regulator |
| Erratic behavior | EMI/noise | Add filtering capacitors, ferrite beads |
| USB devices disconnecting | Insufficient 5V current | Use powered hub or increase supply |
| SD card corruption | Power interruption | Add UPS/capacitor bank, safe shutdown |

### 10.2 Voltage Verification Procedure

```bash
# On the Raspberry Pi, check for throttling/undervoltage
vcgencmd get_throttled

# Results interpretation:
# 0x0     = No issues
# 0x50000 = Throttled (past)
# 0x50005 = Currently under-voltage
# Bit meanings:
#   0: Under-voltage detected
#   1: Arm frequency capped
#   2: Currently throttled
#   3: Soft temperature limit active
#   16: Under-voltage has occurred
#   17: Arm frequency capping has occurred
#   18: Throttling has occurred
#   19: Soft temperature limit has occurred

# Monitor continuously
watch -n 1 vcgencmd get_throttled

# Check core voltage
vcgencmd measure_volts core
```

### 10.3 Recommended Test Procedure

1. **Before connecting to Pi:**
   ```
   - Measure regulator output with multimeter (should be 5.0-5.1V)
   - Apply load resistor (10Ω/5W = 0.5A) and verify voltage stability
   - Check for voltage ripple with oscilloscope if available
   ```

2. **After connecting to Pi:**
   ```
   - Boot Pi and verify normal operation
   - Run stress test: stress --cpu 4 --timeout 60
   - Monitor voltage: vcgencmd get_throttled
   - Check regulator temperature (should be warm, not hot)
   ```

3. **Under full load:**
   ```
   - Add USB peripherals
   - Run CPU stress + I/O: stress --cpu 4 --io 2 --timeout 120
   - Monitor for throttling or instability
   - Verify no random reboots over extended period
   ```

---

## Quick Reference Card

### GPIO Power Pins
```
Pin 2 (5V)  ← Connect regulated 5V here
Pin 4 (5V)  ← Or here (internally connected to Pin 2)
Pin 6 (GND) ← Ground connection
```

### Voltage Requirements
```
Pi 3B+: 5.0V ± 0.25V, 2.5A recommended
Pi 4:   5.1V ± 0.25V, 3.0A recommended
ABSOLUTE MAX: 5.5V (damage above this)
MINIMUM: 4.75V (undervoltage below this)
```

### Recommended Regulators (Quick Pick)
```
Budget:    LM2596 module (~$3) - verify output with multimeter!
Mid-range: Adafruit UBEC (~$9) - proven, 3A
Premium:   Pololu D24V50F5 (~$15) - best regulation, 5A
```

### Essential Protection
```
1. Fuse: 4A fast-blow (minimum)
2. TVS Diode: SMBJ5.0A (voltage clamp)
3. Capacitor: 100µF electrolytic (filtering)
4. Polarized connector: XT30/XT60 (prevent reversal)
```

---

## Document Information

**Created:** February 2026
**Purpose:** Comprehensive reference for GPIO power methods in mobile robotics applications
**Project:** WayfindR Mobile Robot Platform
**Related Documents:**
- `/home/devel/WayfindR-driver/docs/raspberry_pi_battery_power_guide.md`
- `/home/devel/WayfindR-driver/docs/power-system-guide.md`
