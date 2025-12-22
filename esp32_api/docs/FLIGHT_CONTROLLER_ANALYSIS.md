# Flight Controller Computational Analysis

Analysis of the dRehmFlight VTOL flight controller from `~/floppi/flight_controller` to determine if it can run on ESP32.

## Original Target: Teensy 4.0

| Specification | Teensy 4.0 | ESP32 |
|---------------|------------|-------|
| CPU | ARM Cortex-M7 | Xtensa LX6 (dual-core) |
| Clock Speed | 600 MHz | 240 MHz per core |
| FPU | FPv5 (double precision) | Single precision |
| RAM | 1 MB | 520 KB |
| Flash | 2 MB | 4 MB |

## Per-Loop Operations Analysis

The flight controller runs at **2000 Hz** (500µs per loop). Here's what happens each iteration:

### 1. IMU Read (~50-100µs)
```
Operations:
- I2C read: 14 bytes from MPU6050 (6 accel + 6 gyro)
- Scale conversion: 6 divisions
- Offset correction: 6 subtractions
- Low-pass filter: 6 multiplications + 6 additions

Estimated cycles: ~2,000-4,000 cycles
At 240MHz: ~10-17µs
```

### 2. Madgwick Filter (~100-200µs)
```
Operations (Madgwick6DOF):
- 1 invSqrt (reciprocal square root)
- ~50 multiplications
- ~30 additions
- ~10 subtractions
- 1 atan2, 1 asin (trigonometric)

Key operations:
- Quaternion math: 4 components, each updated
- Normalization: 2x invSqrt
- Gradient descent step

Estimated cycles: ~8,000-15,000 cycles
At 240MHz: ~33-63µs
```

### 3. PID Controllers x3 (~20-40µs)
```
Per axis (roll, pitch, yaw):
- Error calculation: 1 subtraction
- Integral: 1 multiplication + 1 addition
- Derivative: 1 subtraction + 1 division
- Output: 3 multiplications + 2 additions
- Constraint: 2 comparisons

Total for 3 axes:
Estimated cycles: ~1,500-3,000 cycles
At 240MHz: ~6-13µs
```

### 4. Motor Mixing (~10-20µs)
```
Operations:
- 4 motor outputs = 4x (3 additions/subtractions)
- Constraint: 4x2 comparisons
- Scale to PWM: 4 multiplications + 4 additions

Estimated cycles: ~500-1,000 cycles
At 240MHz: ~2-4µs
```

### 5. PWM Output (~50-100µs for OneShot125)
```
OneShot125 bit-banging:
- 6 digital writes HIGH
- Timing loop: ~250µs max
- 6 digital writes LOW (staggered)

Standard PWM (hardware):
- 6 analogWrite calls
- Near-zero CPU time (hardware handles it)
```

### 6. Radio Input (~10-30µs)
```
SBUS parsing (interrupt-driven):
- Deserialize 25-byte packet
- Extract 16 channels
- Range conversion

Most work done in interrupt, main loop just reads cached values
```

## Total Computational Budget

### Worst Case (OneShot125, debug prints):
| Component | Time (µs) |
|-----------|-----------|
| IMU Read | 100 |
| Madgwick | 200 |
| PID x3 | 40 |
| Mixer | 20 |
| OneShot125 | 100 |
| Radio | 30 |
| Overhead | 50 |
| **Total** | **540µs** |

### Optimized (Standard PWM, no debug):
| Component | Time (µs) |
|-----------|-----------|
| IMU Read | 50 |
| Madgwick | 100 |
| PID x3 | 20 |
| Mixer | 10 |
| PWM (HW) | 5 |
| Radio | 10 |
| Overhead | 25 |
| **Total** | **220µs** |

## Minimum Viable Clock Speed Calculation

### For 2000 Hz loop (500µs period):

```
Required CPU time: ~220-540µs (depending on configuration)
Available time: 500µs
Safety margin: We want at least 20% headroom

Minimum frequency for worst case:
  540µs of work at 240MHz
  Work = 540µs × 240MHz = 129,600 cycles

  To fit in 500µs with 20% headroom (400µs budget):
  Frequency = 129,600 cycles / 400µs = 324 MHz

Minimum frequency for optimized case:
  220µs of work at 240MHz
  Work = 220µs × 240MHz = 52,800 cycles

  To fit in 500µs with 20% headroom (400µs budget):
  Frequency = 52,800 cycles / 400µs = 132 MHz
```

### For 500 Hz loop (2000µs period) - More conservative:

```
Budget: 2000µs × 0.8 = 1600µs available

At 240MHz, worst case needs 540µs
Margin: 1600 - 540 = 1060µs (66% headroom!)

Minimum frequency:
  Work = 540µs × 240MHz = 129,600 cycles
  Frequency = 129,600 / 1600µs = 81 MHz
```

## ESP32 Feasibility Assessment

### Can ESP32 run this flight controller?

**YES, but with considerations:**

| Loop Rate | Feasibility | Notes |
|-----------|-------------|-------|
| 2000 Hz | Marginal | Tight timing, may have jitter |
| 1000 Hz | Good | Comfortable margin |
| 500 Hz | Excellent | Plenty of headroom |
| 400 Hz | Excellent | Recommended minimum for quads |

### Key Findings:

1. **240 MHz is sufficient** for 500-1000 Hz control loops
2. **500 Hz is the minimum** for stable quad flight
3. **ESP32 dual-core advantage**: Web server on Core 1 doesn't affect timing
4. **Use hardware PWM** instead of OneShot125 bit-banging
5. **Avoid debug prints** in the control loop

### Optimizations for ESP32:

1. **Use LEDC PWM** instead of bit-banging (saves ~100µs)
2. **Reduce loop rate** to 500-1000 Hz (still stable flight)
3. **Use single-precision floats** (ESP32 FPU is single-precision only)
4. **Pin control loop to Core 0** (dedicated, no interrupts from WiFi)
5. **Use DMA for I2C** if available

## Physical Stability Requirements

### Why 500 Hz is the minimum for quads:

```
Quad dynamics:
- Natural oscillation frequency: ~10-50 Hz
- Nyquist theorem: Sample at 2x frequency minimum
- Control theory: 5-10x for good control authority
- Therefore: 50 Hz × 10 = 500 Hz minimum

At 500 Hz:
- 2ms response time
- Sufficient for correcting 10-degree attitude errors
- Works for 5" quads with typical motor response

At 1000 Hz:
- 1ms response time
- Better for aggressive maneuvers
- Required for racing quads with high-KV motors

At 250 Hz (too slow):
- 4ms response time
- Oscillations likely
- May crash on sudden disturbances
```

## ESP32 vs Teensy 4.0 Summary

| Aspect | Teensy 4.0 | ESP32 (Dual Core) |
|--------|------------|-------------------|
| Raw Speed | 2.5x faster | Sufficient for 500-1000 Hz |
| FPU | Double precision | Single precision (OK) |
| PWM | FlexPWM (excellent) | LEDC (good) |
| I2C Speed | 1 MHz | 400 kHz typical |
| Dual Core | No | Yes (big advantage!) |
| WiFi | No | Yes (runs on Core 1) |
| Cost | $20-25 | $5-10 |

## Recommendation

**ESP32 CAN run this flight controller** at reduced loop rates:

1. **For stable hover/filming**: 500 Hz is fine
2. **For sport flying**: 1000 Hz recommended
3. **For racing**: Consider Teensy or ESP32-S3 at 240MHz

The dual-core architecture is actually an **advantage** because:
- Core 0: Flight controller at 500-1000 Hz (no interruptions)
- Core 1: Web server, WiFi, telemetry (completely isolated)

This is better isolation than a single-core Teensy where everything shares time.
