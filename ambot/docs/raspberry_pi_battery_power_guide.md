# Raspberry Pi Battery Power Guide for Mobile Robots

This guide covers battery and power options for running a Raspberry Pi 3B+/4 headlessly on a mobile robot platform, with USB peripherals like webcams and LiDAR sensors.

## Power Requirements Summary

### Raspberry Pi Power Specifications

| Model | Minimum Current | Recommended Current | Voltage |
|-------|-----------------|---------------------|---------|
| Raspberry Pi 3B+ | 2.5A | 2.5A+ | 5V |
| Raspberry Pi 4B | 3.0A | 3.0A+ | 5.1V |
| Raspberry Pi 5 | 5.0A | 5.0A | 5.0V |

### USB Peripheral Power Consumption

| Device Type | Typical Current Draw |
|-------------|---------------------|
| USB Webcam | 200-500mA |
| USB LiDAR (Tau/similar) | ~250mA |
| Intel RealSense LiDAR | ~700mA |
| USB WiFi Dongle | 40-450mA |
| USB Keyboard/Mouse | 50-150mA each |

**Important:** The Raspberry Pi 4 can supply a maximum of **1.2A total** across all USB ports combined. If your peripherals exceed this, use a powered USB hub.

---

## Recommended USB Power Banks

### Tier 1: Best for Raspberry Pi 4 (3A+ Output)

#### Anker PowerCore III Elite 25600
- **Capacity:** 25,600mAh (~95Wh)
- **Output:** USB-C PD with stable 5V/3A output
- **Pros:** Stable voltage under load, advanced surge/temperature protection
- **Estimated Runtime:** 15-20+ hours for Pi 4 at moderate load
- **Best For:** Extended outdoor use, high-power setups

#### Anker PowerCore 26800
- **Capacity:** 26,800mAh (~99Wh)
- **Output:** Multiple USB-A ports, some models with USB-C PD
- **Pros:** Proven reliable for headless Pi 4B with DAC and other peripherals
- **Estimated Runtime:** 15-20+ hours

### Tier 2: Good for Raspberry Pi 3B+ (2.4A+ Output)

#### Anker PowerCore 10000
- **Capacity:** 10,000mAh (~37Wh)
- **Output:** 2.4A USB-A
- **Pros:** Compact, efficient, stable output
- **Cons:** May be marginal for Pi 4 with peripherals
- **Estimated Runtime:** 6-10 hours for Pi 3B+

#### Generic 20,000mAh Power Banks with USB-C PD
- **Capacity:** 20,000mAh (~74Wh)
- **Output:** 5V/3A via USB-C, 2.4A via USB-A
- **Estimated Runtime:** 12-16 hours for Pi 3B+/4

### Tier 3: Specialized Raspberry Pi Solutions

#### PiSugar 3 Plus (UPS Module)
- **Capacity:** 5,000mAh
- **Output:** 5V/3A (stable)
- **Features:**
  - Attaches directly to Pi via pogo pins (no cables)
  - Built-in RTC for scheduled wake-up
  - UPS functionality - seamless power switching
  - Hardware battery protection (overcharge, over-discharge, overcurrent)
  - Software watchdog and web UI for monitoring
  - GPIO pins remain free
- **Price:** ~$40
- **Best For:** Compact mobile robots, unattended operation
- **Cons:** Limited capacity for long runtimes

#### TalentCell 12V 7000mAh (76.65Wh)
- **Capacity:** 7,000mAh @ 12V (equivalent ~21,000mAh @ 3.7V)
- **Output:** DC 12V/6A, USB-C 45W (5V/5A capable), USB-A 18W
- **Pros:** Multiple voltage outputs, good for powering Pi + other 12V robot components
- **Cons:** Requires step-down converter for 5V output if not using USB-C
- **Best For:** Robots needing both 12V and 5V power from one source

---

## Runtime Calculations

### Understanding Power Bank Capacity

Power banks use lithium cells with a nominal voltage of **3.7V**. When they output 5V, there's an internal conversion with ~70-80% efficiency.

**Formula:**
```
Actual Wh = (mAh × 3.7V) / 1000
Usable Wh = Actual Wh × 0.70 to 0.80 (efficiency factor)
Runtime (hours) = Usable Wh / Power consumption (W)
```

### Raspberry Pi Power Consumption

| State | Pi 3B+ | Pi 4B |
|-------|--------|-------|
| Idle | ~350mA (1.75W) | ~600mA (3W) |
| Moderate Load | ~500mA (2.5W) | ~1000mA (5W) |
| Heavy Load | ~700mA (3.5W) | ~1200mA (6W) |
| With USB Peripherals | Add 300-800mA | Add 300-800mA |

### Example Runtime Calculations

**Scenario:** Pi 4 at moderate load (~5W) with webcam and LiDAR (~2W) = ~7W total

| Power Bank | Capacity | Usable Energy | Est. Runtime |
|------------|----------|---------------|--------------|
| 10,000mAh | 37Wh | ~26Wh | ~3.7 hours |
| 20,000mAh | 74Wh | ~52Wh | ~7.4 hours |
| 26,800mAh | 99Wh | ~70Wh | ~10 hours |

**Note:** Always add a 25-35% margin to account for efficiency losses and ensure reliability.

---

## Common Issues and Solutions

### 1. Undervoltage Warning (Lightning Bolt Icon)

**Cause:** Voltage drops below 4.63V (4.7V threshold for stable operation)

**Solutions:**
- Use a higher-rated power bank (3A+ for Pi 4)
- Replace USB cable with a high-quality, short, thick-gauge cable
- Reduce peripheral load
- Use a powered USB hub for high-draw peripherals

### 2. Power Bank Auto-Shutoff (Sleep Mode)

**Problem:** Many power banks shut off after ~20-30 seconds if they detect low current draw (common when Pi is idle).

**Solutions:**
- Choose power banks designed for always-on operation
- Add a dummy load (150 ohm resistor = ~25mA continuous draw)
- Use UPS-style solutions like PiSugar that don't have this issue
- Look for power banks marketed as "always on" or "low current compatible"

**Known Compatible Power Banks (no auto-shutoff):**
- Silvercrest 6000mAh (Lidl)
- PiSugar modules
- Many 20,000mAh+ banks with USB-C PD

### 3. Cable Quality Issues

**Symptoms:** Undervoltage despite adequate power bank rating

**Solution:** Use cables that are:
- Short (under 1 meter / 3 feet)
- Thick gauge (20 AWG or thicker for power lines)
- Rated for 3A or higher
- From reputable brands (Anker, Amazon Basics, etc.)

**DIY Option:** Make your own power cable with heavier gauge wire.

### 4. SD Card Corruption

**Cause:** Power interruption or undervoltage during write operations

**Solutions:**
- Use UPS solutions (PiSugar, etc.) with graceful shutdown
- Implement shutdown scripts that trigger at low battery
- Use read-only filesystem when possible
- Regular backups

---

## Alternative Power Solutions for Robots

### DC-DC Buck Converters (from Robot Battery)

If your robot has a main battery (12V, 24V, etc.), you can power the Pi through a buck converter:

#### Recommended Buck Converters

| Product | Input | Output | Price |
|---------|-------|--------|-------|
| Adafruit UBEC | 6-16V | 5V/3A | ~$9 |
| DFRobot Dual USB Buck | 8-32V | 5V/3A (x2 ports) | ~$8 |
| Pololu D30V30F5 | 4.5-32V | 5V/3.4A | ~$19 |
| Pololu D36V50F5 | 4.5-50V | 5V/5.5A | ~$25 |

**Advantages:**
- More efficient than carrying separate batteries
- Consistent power from robot's main battery
- No auto-shutoff issues
- Smaller and lighter than power banks

**Considerations:**
- Need to isolate Pi power from motor noise
- May need separate battery if robot uses high-current motors (voltage drops)

### Separate Power Architecture (Recommended for Robots)

**Best Practice:** Use separate power sources for logic (Pi) and motors:

```
Robot Power Architecture:
├── Motor Battery (6-8 AA NiMH or LiPo)
│   └── Motor Controller → Motors
│
└── Logic Battery (USB Power Bank or LiPo + Buck Converter)
    └── 5V/3A → Raspberry Pi → USB Peripherals
```

**Reasons:**
- Motors cause voltage spikes/drops that can crash the Pi
- Keeps Pi running even if motors stall
- Easier to manage power budgets

### RC LiPo Batteries with Buck Converter

For weight-conscious robots:
- **Battery:** 3S LiPo (11.1V) from HobbyKing or similar
- **Converter:** UBEC or Pololu regulator
- **Charger:** Balance charger required (separate purchase)
- **Advantages:** High energy density, lightweight
- **Disadvantages:** Requires LiPo safety knowledge, separate charger

---

## Best Practices for Mobile Robot Power

### 1. Power Budget Planning

Before building, calculate your total power needs:

```
Power Budget Example:
- Raspberry Pi 4 (heavy load): 6W
- USB Webcam: 2W
- LiDAR: 1.5W
- WiFi: 0.5W
- Safety margin (20%): 2W
--------------------------------
Total: 12W

For 4-hour runtime: 12W × 4h = 48Wh needed
With 75% efficiency: 48Wh / 0.75 = 64Wh capacity needed
≈ 17,300mAh power bank minimum
```

### 2. Power Monitoring

- Use `vcgencmd get_throttled` to check for undervoltage events
- Monitor with: `watch -n 1 vcgencmd get_throttled`
- Flags: 0x50005 indicates current and past undervoltage

### 3. Safe Shutdown Implementation

For headless operation, implement safe shutdown:

```bash
# Check for low voltage and shutdown gracefully
# Add to crontab or run as systemd service
#!/bin/bash
THROTTLED=$(vcgencmd get_throttled)
if [[ "$THROTTLED" != "throttled=0x0" ]]; then
    logger "Power issue detected: $THROTTLED"
    # Add shutdown logic here if needed
fi
```

### 4. Startup Configuration

For headless SSH operation:
- Enable SSH before first boot (create empty `ssh` file in boot partition)
- Configure WiFi in `wpa_supplicant.conf`
- Set static IP or use hostname.local (mDNS)
- Consider enabling hardware watchdog

---

## Quick Reference: Choosing the Right Solution

| Use Case | Recommended Solution |
|----------|---------------------|
| Short missions (<2 hours) | 10,000mAh power bank |
| Medium missions (2-6 hours) | 20,000mAh power bank with USB-C PD |
| Long missions (6+ hours) | 26,800mAh power bank or larger |
| Compact robot | PiSugar 3 Plus UPS module |
| Robot with existing 12V battery | UBEC or buck converter |
| High reliability needed | PiSugar with UPS + shutdown scripts |
| Multiple voltages needed | TalentCell multi-voltage pack |

---

## Sources and Further Reading

- [Raspberry Pi Forums - Power Supply Discussions](https://forums.raspberrypi.com/viewtopic.php?t=324502)
- [Power Bank Expert - Raspberry Pi Guide](https://www.powerbankexpert.com/best-raspberry-pi-power-bank/)
- [Raspberry Pi Dramble - Power Consumption Benchmarks](https://pidramble.com/wiki/benchmarks/power-consumption)
- [Seeed Studio - Raspberry Pi Power Supply Guide](https://www.seeedstudio.com/blog/2025/12/01/raspberry-pi-power-supply-guide/)
- [PiSugar Official Documentation](https://www.pisugar.com/)
- [Adafruit UBEC Product Page](https://www.adafruit.com/product/1385)
- [The Pi Hut - Voltage Regulators](https://thepihut.com/collections/voltage-regulators-buck-converters)
- [Articulated Robotics - Mobile Robot Series](https://articulatedrobotics.xyz/mobile-robot-4-raspberry-pi/)
