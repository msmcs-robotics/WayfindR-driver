# FAGM25-370 Brushed DC Gearmotor

Documentation for the FAGM25-370 / 25GA-370 / JGA25-370 motor family used in the Ambot project.

## Overview

The FAGM25-370 is a 25mm brushed DC gearmotor commonly used in robotics applications. It's a **brushed DC motor** (2 wires) that requires a motor driver - **never connect directly to GPIO pins**.

## Specifications

### Voltage Ratings

| Voltage | Typical Applications |
|---------|---------------------|
| **6V** | Lower power, battery-operated |
| **12V** | Most common; robotics, automation |
| **24V** | Higher torque industrial applications |

### Current Specifications (at 12V)

| Parameter | Value |
|-----------|-------|
| No-Load Current | 50mA - 130mA |
| Rated Load Current | 300mA - 800mA |
| Stall Current | 1.2A - 3.0A (varies by gear ratio) |

**Important**: Motors with gear ratios above 164:1 should not be stalled for extended periods.

### RPM by Gear Ratio (at 12V)

| Gear Ratio | Approximate RPM |
|------------|-----------------|
| 1:21 | 399 RPM |
| 1:30 | 300 RPM |
| 1:50 | 200 RPM |
| 1:100 | 100 RPM |
| 1:200 | 30-60 RPM |

### Physical Specifications

- **Motor diameter**: 25mm
- **Shaft diameter**: 4mm (D-cut, 0.5mm depth)
- **Shaft length**: 25mm
- **Motor body length**: 48-50mm
- **Weight**: 84g - 200g (depending on gear ratio)

## Wiring Requirements

### Critical Rules

1. **NEVER connect motor wires directly to GPIO pins**
   - GPIO pins supply max ~16mA
   - These motors draw 1.2A-3A at stall

2. **ALWAYS use a motor driver (H-bridge)**
   - TB6612FNG (recommended)
   - L298N (works, less efficient)
   - DRV8833 (for 6-9V operation only)

3. **Use external power supply**
   - Match motor voltage (6V/12V)
   - Supply should handle 2x stall current
   - Common options: 6x AA battery pack, 2S LiPo (7.4V), 12V power supply

4. **Connect common ground**
   - Raspberry Pi GND
   - Motor driver GND
   - External power supply GND
   - All must be connected together

## Protection Recommendations

1. **Flyback diodes** across motor terminals (for back-EMF protection)
2. **Decoupling capacitors** (100nF ceramic + 100uF electrolytic near driver)
3. **Fuse** the motor circuit at 3-4A
4. **Heatsink** on motor driver if running near stall

## Use Cases

- Robotics and smart cars
- Electronic door locks
- DIY robot toys
- Home/office automation
- Differential drive platforms

## References

- [FONEACC Motor - FAGM25-370 Official](https://www.foneaccmotor.com/gm25-370-p00151p1.html)
- [Open Impulse - JGA25-370 Specifications](https://www.openimpulse.com/blog/products-page/25d-gearmotors/)
