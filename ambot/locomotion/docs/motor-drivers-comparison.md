# Motor Driver Comparison

Comparison of motor drivers for use with FAGM25-370 motors and Raspberry Pi.

## Quick Comparison

| Driver | Voltage | Current | Efficiency | Price | Best For |
|--------|---------|---------|------------|-------|----------|
| **TB6612FNG** | 4.5-13.5V | 1.2A cont | ~95% | $4-8 | **Recommended** |
| L298N | 5-35V | 2A cont | ~75% | $2-5 | High voltage, budget |
| DRV8833 | 2.7-10.8V | 1.2A cont | ~95% | $3-6 | Low voltage only |
| BTS7960 | 6-27V | 43A cont | ~95% | $8-15 | High power |

## TB6612FNG (Recommended)

**Our primary driver choice.**

### Pros
- 3.3V logic compatible (perfect for Raspberry Pi)
- High efficiency (MOSFET-based, ~0.2V drop)
- Handles 12V motors within spec
- 3.2A peak current handles brief stalls
- Compact size
- Dual motor control

### Cons
- 1.2A continuous may be tight for extended stall
- Requires separate logic power

### Verdict
**Best choice** for FAGM25-370 at 12V with Raspberry Pi.

---

## L298N

### Specs
- Motor Voltage: 5-35V
- Continuous Current: 2A per channel
- Peak Current: 3.5A
- Voltage Drop: 2-2.8V (significant loss)

### Pros
- Widely available, cheap ($2-5)
- High voltage range
- Well documented with many tutorials
- Can handle stall currents
- Built-in 5V regulator

### Cons
- **Very inefficient** - loses 2V+ as heat
- Large physical size, requires heatsink
- 5V logic (works with Pi 3.3V but not ideal)
- Outdated BJT technology

### Verdict
Works but **not recommended**. Use only if budget is critical.

---

## DRV8833

### Specs
- Motor Voltage: 2.7-10.8V
- Continuous Current: 1.2A (1.5A with heatsink)
- Peak Current: 2A
- Voltage Drop: ~0.18V (excellent)

### Pros
- Highly efficient
- Built-in protection circuits
- Built-in kickback diodes
- Low-power sleep mode
- 3.3V logic compatible
- Very compact

### Cons
- **Maximum 10.8V - NOT suitable for 12V motors**
- Lower current capacity

### Verdict
**Not recommended** for 12V FAGM25-370. Excellent for 6-9V applications.

---

## BTS7960 / IBT-2

### Specs
- Motor Voltage: 6-27V
- Continuous Current: 43A
- Peak Current: 80A
- 3.3V logic compatible

### Pros
- Massive current headroom
- Wide voltage range
- Built-in heatsink
- Will handle any hobby motor

### Cons
- Single motor per module (need two for differential drive)
- Larger size
- Overkill for small motors

### Verdict
**Excellent choice** if you want guaranteed headroom or plan to upgrade motors.

---

## Recommendation Summary

### For FAGM25-370 at 12V (Our Setup)

1. **TB6612FNG** - Best balance of size, efficiency, and capability
2. **BTS7960** - If you want absolute reliability
3. **L298N** - Budget fallback only

### For 6V Battery-Powered Projects

1. **DRV8833** - Maximum efficiency
2. **TB6612FNG** - If you need more current

### For Large Motors / High Power

1. **BTS7960** - Handles anything
2. **VNH2SP30 Monster Moto** - Arduino shield format
