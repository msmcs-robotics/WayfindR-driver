# Jetson Orin Nano Battery Options for Mobile Robots

> Date: 2026-02-03
> Status: Research Complete
> Keywords: jetson orin nano, battery, LiPo, USB-C PD, power supply, mobile robot, voltage regulator

## Summary

Comprehensive guide to powering a Jetson Orin Nano on a mobile robot platform, including power requirements, battery options, voltage regulation, and runtime calculations.

---

## 1. Jetson Orin Nano Power Requirements

### Voltage Input Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Input Voltage Range | 7-20V DC | Via barrel jack (5.5mm OD x 2.5mm ID) |
| Recommended Voltage | 19V | Standard NVIDIA power adapter |
| Maximum Current Draw | 5A | IDDmax specification |

### Power Modes

| Mode | Power Budget | Use Case | AI Performance |
|------|-------------|----------|----------------|
| 7W | 7 watts | Power saving | Reduced |
| 15W | 15 watts | Balanced (default) | Good |
| 25W | 25 watts | Maximum performance | 30-70% better than 15W |

**Important Notes:**
- The 25W is for the **module only** - the carrier board can handle up to **45W total** including peripherals
- Actual power consumption varies with workload (GPU inference, peripherals, etc.)
- Current draw is determined by consumption, not power supply rating

### Power Calculation Reference

For the robot platform:
- Jetson Orin Nano (15W mode): 15W
- USB camera (~0.9A @ 5V): ~4.5W
- LiDAR (~150mA @ 5V): ~0.75W
- Miscellaneous peripherals: ~5W
- **Total estimated system power: 20-25W** (15W mode) or **30-35W** (25W mode)

---

## 2. Battery Options

### Option A: 4S LiPo Battery (Recommended for Robots)

**Why 4S LiPo:**
- Voltage range: 12.8V (discharged) to 16.8V (fully charged) - well within 7-20V input range
- High energy density
- High discharge rates available
- Common in robotics/drones

#### Recommended Products

| Battery | Capacity | Voltage | Weight | Approximate Price |
|---------|----------|---------|--------|------------------|
| **Tattu 10000mAh 4S 25C** | 10000mAh (148Wh) | 14.8V nominal | ~940g | $80-100 |
| **Turnigy High Capacity 10000mAh 4S 12C** | 10000mAh (148Wh) | 14.8V nominal | ~900g | $60-80 |
| **Tattu 5000mAh 4S 35C** | 5000mAh (74Wh) | 14.8V nominal | ~500g | $45-60 |
| **Turnigy 5000mAh 4S 25C** | 5000mAh (74Wh) | 14.8V nominal | ~450g | $35-50 |

**Sources:**
- [Tattu 10000mAh on GensTattu](https://genstattu.com/tattu-10000mah-14-8v-25c-4s1p-lipo-battery-pack-without-plug.html)
- [Turnigy 5000mAh on HobbyKing](https://hobbyking.com/en_us/turnigy-battery-5000mah-4s-25c-lipo-pack-xt-90.html)

**Direct Connection Consideration:**
- A 4S LiPo can connect directly to the Jetson barrel jack (voltage is within spec)
- However, **a buck converter is recommended** for voltage stability (see Section 3)

---

### Option B: USB-C Power Delivery (PD) Power Bank

**Advantages:**
- Consumer-friendly, readily available
- Built-in battery management
- Safe and well-protected

**Limitations:**
- Requires USB-C PD trigger cable
- May not provide full 45W depending on power bank
- Some power banks are finicky about triggering higher voltages

#### Recommended Products

| Power Bank | Capacity | Output | Notes |
|------------|----------|--------|-------|
| **Anker PowerCore+ 26800mAh PD 45W** | 26800mAh (~97Wh) | 45W USB-C PD | Good for 15W mode |
| **Baseus Blade HD 20000mAh 100W** | 20000mAh (~72Wh) | 100W USB-C PD | Overkill but future-proof |
| **Anker Prime 20000mAh** | 20000mAh (~72Wh) | 100W USB-C PD | Premium option |

**Sources:**
- [Anker 45W Power Banks](https://www.anker.com/collections/45w-power-bank)
- [Anker PowerCore+ on Amazon](https://www.amazon.com/Anker-Charger-PowerCore-Portable-Delivery/dp/B07XRJZXKY)

#### Required: USB-C PD Trigger Cable

To use a USB-C PD power bank with the Jetson's barrel jack, you need a **PD trigger cable**:

| Product | Voltage Options | Max Current | Barrel Size |
|---------|-----------------|-------------|-------------|
| **JacobsParts 15V PD Trigger Cable** | 15V fixed | 5A (E-Mark) | 5.5x2.5mm |
| **Adafruit USB-C PD to Barrel (20V)** | 20V fixed | 5A (E-Mark) | 5.5mm |
| **Pimoroni Re-programmable PD Cable** | 5V/9V/12V/15V/20V selectable | 2A | 5.5x2.1mm |

**Important:** For the Jetson Orin Nano, use a **15V or 20V** trigger cable. 15V @ 3A = 45W which is the maximum the dev kit supports.

**Sources:**
- [Adafruit USB-C PD to Barrel Jack](https://www.adafruit.com/product/5452)
- [Pimoroni Re-programmable PD Cable](https://shop.pimoroni.com/en-us/products/re-programmable-usb-type-c-pd-to-2-1-5-5mm-barrel-jack-cable)

---

### Option C: V-Mount Battery (Professional/High Capacity)

**Best for:** Extended runtime, professional applications

| Battery Type | Capacity Range | Voltage | Notes |
|--------------|----------------|---------|-------|
| V-Mount Li-ion | 95Wh - 290Wh | 12.8V - 16.8V | Industry standard, D-Tap output |

- Output from D-Tap: 12.8V (discharged) to 16.8V (fully charged)
- Can provide 10A+ continuous
- Expensive but robust

**Source:** [JetsonHacks V-Mount Battery Guide](https://jetsonhacks.com/2024/05/23/v-mount-battery-to-power-nvidia-jetson-electronics-projects/)

---

### Option D: Waveshare UPS Module (C) for Jetson Orin

**Purpose-built UPS module** designed specifically for Jetson Orin:

| Feature | Specification |
|---------|--------------|
| Battery Type | 3x 21700 Li-ion cells (not included) |
| Connection | Pogo pins (no GPIO required) |
| Charging Current | 1.33A max |
| Communication | I2C (battery monitoring) |
| Price | ~$35-45 |

**Key Features:**
- Automatic switchover when external power is lost
- Simultaneous charging and power output
- Over-charge/discharge protection, over-current protection, short circuit protection
- Battery voltage/current monitoring via I2C
- Equalizing charge feature

**Source:** [Waveshare UPS Power Module (C)](https://www.waveshare.com/ups-power-module-c.htm)

---

## 3. Voltage Regulators / DC-DC Converters

### When Do You Need a Regulator?

| Scenario | Need Regulator? | Reason |
|----------|-----------------|--------|
| 4S LiPo direct to Jetson | Recommended | Voltage stability, protection |
| USB-C PD with trigger cable | No | PD regulates voltage |
| 6S LiPo (22.2V nominal) | **Required** | Exceeds 20V max input |
| 12V lead-acid | Optional | Voltage is stable |

### Recommended Buck Converters

| Product | Input Range | Output | Current | Price | Notes |
|---------|-------------|--------|---------|-------|-------|
| **Pololu D24V50F12** | 6-38V | 12V fixed | 5A | ~$15 | Compact, reliable |
| **DFRobot Buck Boost 5A** | 6-35V | 0-33V adjustable | 5A | ~$12 | Adjustable output |
| **MEAN WELL SD-25A-12** | 9.2-18V | 12V | 2.1A | ~$20 | Industrial quality |
| **DROK Buck Boost 60W** | 5-32V | 1.25-20V | 3A | ~$15 | Popular in robotics |

**Recommendations:**
- For 4S LiPo: **Pololu D24V50F12** (set to 12V) or run direct if you trust the battery
- For higher voltage sources: **DFRobot Buck Boost 5A** (adjustable)

**Key Specifications to Look For:**
- Input voltage range must cover battery's full discharge-charge cycle
- Output current rating: **minimum 5A** (Jetson IDDmax)
- Efficiency: 85%+ to minimize heat
- Protection: Over-current, over-temperature, short-circuit

**Sources:**
- [Pololu D24V50F5 (5V version, 12V also available)](https://www.pololu.com/product/2851)
- [DFRobot DC-DC Buck Boost Converter 5A](https://www.dfrobot.com/product-2539.html)

---

## 4. Runtime Estimates

### Basic Formula

```
Runtime (hours) = Battery Capacity (Wh) / System Power Draw (W) * Efficiency
```

Where efficiency accounts for:
- DC-DC converter losses (~10-15%)
- Battery discharge curve (use 85% of rated capacity for safety)

### Runtime Calculations

#### At 15W Mode (Typical Robot Operation)

Assuming total system draw of ~20W:

| Battery | Capacity (Wh) | Usable (85%) | Runtime @ 20W |
|---------|---------------|--------------|---------------|
| Tattu 10000mAh 4S | 148 Wh | 126 Wh | **6.3 hours** |
| Turnigy 5000mAh 4S | 74 Wh | 63 Wh | **3.1 hours** |
| Anker 26800mAh PD | 97 Wh | 82 Wh | **4.1 hours** |
| 3x 21700 (5000mAh each) | 55 Wh | 47 Wh | **2.3 hours** |

#### At 25W Mode (Maximum Performance)

Assuming total system draw of ~35W:

| Battery | Capacity (Wh) | Usable (85%) | Runtime @ 35W |
|---------|---------------|--------------|---------------|
| Tattu 10000mAh 4S | 148 Wh | 126 Wh | **3.6 hours** |
| Turnigy 5000mAh 4S | 74 Wh | 63 Wh | **1.8 hours** |
| Anker 26800mAh PD | 97 Wh | 82 Wh | **2.3 hours** |

### Runtime Summary Chart

```
Battery Capacity vs Runtime (15W mode, 20W total system):

10000mAh 4S (148Wh):  ████████████████████████████████████  6.3 hrs
Anker 26800mAh (97Wh): ████████████████████████             4.1 hrs
5000mAh 4S (74Wh):     ██████████████████                   3.1 hrs
3x 21700 (55Wh):       █████████████                        2.3 hrs
```

---

## 5. Wiring Diagrams

### Option A: 4S LiPo with Buck Converter

```
┌─────────────┐     ┌─────────────────┐     ┌──────────────────┐
│  4S LiPo    │     │  Buck Converter │     │  Jetson Orin     │
│  14.8V      │────▶│  (e.g. Pololu)  │────▶│  Nano Dev Kit    │
│  10000mAh   │     │  IN: 6-38V      │     │  Barrel Jack     │
│             │     │  OUT: 12V @ 5A  │     │  (5.5x2.5mm)     │
└─────────────┘     └─────────────────┘     └──────────────────┘
     │
     │  XT90 connector
     ▼
   ┌─────────────┐
   │  Low-Voltage│   (Optional but recommended)
   │  Cutoff     │   (Prevents over-discharge)
   │  Module     │
   └─────────────┘
```

### Option B: USB-C PD Power Bank

```
┌─────────────────┐     ┌─────────────────┐     ┌──────────────────┐
│  USB-C PD       │     │  PD Trigger     │     │  Jetson Orin     │
│  Power Bank     │────▶│  Cable (15V)    │────▶│  Nano Dev Kit    │
│  (45W+ output)  │     │                 │     │  Barrel Jack     │
│                 │     │  USB-C to       │     │                  │
│  e.g. Anker     │     │  5.5x2.5mm      │     │                  │
└─────────────────┘     └─────────────────┘     └──────────────────┘
```

### Option C: Direct 4S LiPo (Simplified)

```
┌─────────────┐                              ┌──────────────────┐
│  4S LiPo    │                              │  Jetson Orin     │
│  14.8V      │─────────────────────────────▶│  Nano Dev Kit    │
│  10000mAh   │                              │  Barrel Jack     │
│             │   XT90 to Barrel Jack        │  (5.5x2.5mm)     │
└─────────────┘   adapter cable              └──────────────────┘
       │
       ▼
  Note: Voltage will swing 12.8V-16.8V
  This is within spec (7-20V) but less stable
```

---

## 6. Safety Considerations

### LiPo Battery Safety

1. **Never discharge below 3.2V per cell** (12.8V for 4S)
   - Use a low-voltage cutoff module or battery alarm
   - Configure Jetson to shut down gracefully at low battery

2. **Store at storage voltage** (~3.8V/cell, 15.2V for 4S) when not in use

3. **Use fireproof LiPo bag** for charging and storage

4. **Monitor for puffing** - discard swollen batteries immediately

5. **Ensure adequate C-rating** - 10000mAh @ 25C = 250A max discharge (overkill for Jetson)

### Connector and Wiring Safety

1. **Use appropriate gauge wire:**
   - 5A @ 12V = 60W
   - Recommend 16-18 AWG for power leads

2. **Include inline fuse** (7-10A) for short-circuit protection

3. **Verify polarity** before connecting - center positive for barrel jack

4. **Use quality connectors:** XT60/XT90 for LiPo, not bare wires

---

## 7. Recommended Configuration for Ambot

Based on the research, here is the recommended power setup for the Ambot mobile robot:

### Primary Recommendation: 4S LiPo with Buck Converter

**Components:**
1. **Battery:** Tattu 10000mAh 4S 25C (~$90)
2. **Voltage Regulator:** Pololu D24V50F12 or DFRobot Buck 5A (~$15)
3. **Low-Voltage Alarm:** 4S LiPo buzzer alarm (~$5)
4. **Connectors:** XT60/XT90 to barrel jack adapter (~$10)
5. **Fuse:** 10A inline blade fuse (~$5)

**Total Cost:** ~$125
**Expected Runtime:** 5-6 hours at 15W mode

### Alternative: USB-C PD (Simpler Setup)

**Components:**
1. **Power Bank:** Anker PowerCore+ 26800mAh PD 45W (~$130)
2. **PD Trigger Cable:** JacobsParts 15V 5A to barrel jack (~$15)

**Total Cost:** ~$145
**Expected Runtime:** 3-4 hours at 15W mode

---

## Related

**Project Documents:**
- [docs/scope.md](../scope.md) - Project boundaries
- [docs/roadmap.md](../roadmap.md) - Milestone planning
- [initial-system-research.md](./initial-system-research.md) - Jetson resource planning

**External Resources:**
- [NVIDIA Jetson Orin Nano Datasheet](https://openzeka.com/wp-content/uploads/2023/03/jetson-orin-nano-datasheet-r4-web.pdf)
- [JetsonHacks - NVIDIA Jetsons on Battery Power](https://jetsonhacks.com/2021/07/16/nvidia-jetsons-on-battery-power/)
- [NVIDIA Developer Forums - Powering Jetson Orin Nano from LiPo](https://forums.developer.nvidia.com/t/powering-jetson-orin-nano-from-lipo-battery/294257)
- [Waveshare UPS Power Module (C)](https://www.waveshare.com/ups-power-module-c.htm)
- [Chief Delphi - Powering Jetson Orin Nano](https://www.chiefdelphi.com/t/powering-jetson-orin-nano/459092)

---

## Notes

- Runtime estimates assume typical robot workload; actual results will vary with GPU inference load
- USB-C PD power banks may not trigger correctly with all PD trigger cables - test before committing
- For extended deployments, consider dual-battery setup with hot-swap capability
- Monitor battery temperature during operation, especially in enclosed robot chassis

---

*Last updated: 2026-02-03*
