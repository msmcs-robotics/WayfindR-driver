# Wheel Encoder Integration Research for WayfindR Robot
**Comprehensive Guide to Accurate Odometry with ROS2**

**Author**: Research Report
**Date**: 2026-01-11
**Version**: 1.0.0
**Target Robot**: WayfindR Differential Drive Platform

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Encoder Technology Comparison](#encoder-technology-comparison)
3. [Encoder Selection Criteria](#encoder-selection-criteria)
4. [Hardware Recommendations](#hardware-recommendations)
5. [Raspberry Pi GPIO Integration](#raspberry-pi-gpio-integration)
6. [ROS2 Odometry Best Practices](#ros2-odometry-best-practices)
7. [ros2_control Integration](#ros2_control-integration)
8. [Integration with cmd_vel_bridge](#integration-with-cmd_vel_bridge)
9. [Sensor Fusion with IMU](#sensor-fusion-with-imu)
10. [Error Sources and Mitigation](#error-sources-and-mitigation)
11. [Calibration and Testing](#calibration-and-testing)
12. [Implementation Plan](#implementation-plan)
13. [References](#references)

---

## Executive Summary

This document provides comprehensive research and implementation guidance for integrating wheel encoders into the WayfindR differential drive robot to replace the current dead reckoning odometry system with accurate encoder-based odometry. The current `cmd_vel_bridge.py` uses open-loop dead reckoning which accumulates significant error over time.

### Key Findings

1. **Encoder Technology**: Magnetic encoders are optimal for small robots due to robustness, low cost, and immunity to environmental contaminants
2. **Recommended Hardware**: Pololu magnetic encoder kits (12 CPR) for micro metal gearmotors provide 2,880-12,000 counts per wheel revolution depending on gear ratio
3. **ROS2 Integration**: Use `ros2_control` hardware_interface for professional-grade integration with Nav2
4. **Sensor Fusion**: Combine wheel encoders with IMU using `robot_localization` package (EKF) for robust odometry
5. **Expected Accuracy**: Sub-1% error with proper calibration versus 10-50% error with dead reckoning

### Current State vs. Target State

| Aspect | Current (Dead Reckoning) | Target (Encoder-based) |
|--------|--------------------------|------------------------|
| Position Accuracy | 10-50% error | <1% error with calibration |
| Drift over Time | High (unbounded) | Low (bounded by calibration) |
| Slip Detection | None | Detectable with IMU fusion |
| Update Rate | 50 Hz (synthetic) | 50 Hz (measured) |
| Hardware Cost | $0 | $20-40 for encoders |
| Complexity | Low | Medium |

---

## Encoder Technology Comparison

### Overview of Encoder Types

Encoders convert rotational motion into digital signals for position and velocity measurement. Three main technologies exist for small robot applications:

### 1. Optical Encoders

**Technology**: Use LED light source and photodetector with slotted disk

**Advantages**:
- Highest resolution (up to 10,000+ CPR)
- Highest accuracy for precision applications
- Well-established technology
- Fast response time

**Disadvantages**:
- **Susceptible to dust, dirt, and oil contamination**
- More fragile (thin glass/plastic disks)
- Higher cost ($30-100+ per encoder)
- Larger physical size
- Sensitive to vibration and shock

**Best For**: High-precision industrial applications, clean environments, stationary robots

### 2. Magnetic Encoders

**Technology**: Use Hall effect sensors or magnetoresistive sensors with magnetic disk

**Advantages**:
- **Highly robust - immune to dust, dirt, oil, and liquids**
- Compact and lightweight
- Low cost ($10-30 per encoder)
- Resistant to shock and vibration
- Simple installation

**Disadvantages**:
- Lower resolution (12-48 CPR typical)
- Can be affected by external magnetic fields
- Temperature sensitivity (-40°C to 125°C typical range)
- Slightly slower response than optical

**Best For**: Small mobile robots, harsh environments, cost-sensitive applications

### 3. Hall Effect Sensors (Simple)

**Technology**: Single or dual Hall effect sensor detecting magnet poles

**Advantages**:
- Very low cost ($2-5 per sensor)
- Extremely simple
- Compact
- No moving parts

**Disadvantages**:
- Very low resolution (1-8 CPR)
- Limited to low-speed applications
- No quadrature output (cannot detect direction without dual sensor)
- Poor accuracy

**Best For**: Basic speed sensing, motor commutation, proof-of-concept

### Technology Comparison Matrix

| Feature | Optical | Magnetic | Hall Effect |
|---------|---------|----------|-------------|
| **Resolution** | 100-10,000 CPR | 12-100 CPR | 1-8 CPR |
| **Cost** | $30-100+ | $10-30 | $2-5 |
| **Dust/Dirt Immunity** | Poor | Excellent | Excellent |
| **Shock Resistance** | Poor | Good | Excellent |
| **Size** | Medium-Large | Small | Very Small |
| **Accuracy** | Excellent | Good | Fair |
| **Robot Suitability** | Low | **High** | Low |

### Recommendation for WayfindR

**Magnetic encoders** are the optimal choice for the WayfindR robot based on:

1. **Environmental robustness**: Indoor environments may have dust, carpet fibers, and occasional spills
2. **Cost-effectiveness**: Budget-friendly for prototype and production
3. **Sufficient resolution**: 12 CPR × 4 (quadrature) × gear_ratio provides excellent resolution
4. **Compact form factor**: Fits micro metal gearmotors without additional mounting
5. **Reliability**: No maintenance required, long operational life

---

## Encoder Selection Criteria

### Key Specifications to Consider

#### 1. Counts Per Revolution (CPR)

**Definition**: Number of pulses generated per complete shaft rotation

**Important Distinctions**:
- **PPR (Pulses Per Revolution)**: Encoder pulses before quadrature multiplication
- **CPR (Counts Per Revolution)**: Total counts after quadrature (typically CPR = PPR × 4)
- **Resolution**: Distance per count

**Calculation**:
```
Motor shaft CPR: 12 CPR (as specified)
Gearbox ratio: N:1 (e.g., 30:1, 75:1, 100:1)
Output shaft CPR: 12 × N × 4 = 48N counts per wheel revolution

Example with 75:1 gearbox:
Output CPR = 12 × 75 × 4 = 3,600 counts per revolution
```

**Resolution Calculation**:
```
Wheel circumference: C = 2πr = 2π × 0.0325m = 0.204m
Counts per revolution: 3,600
Linear resolution: 0.204m / 3,600 = 0.0567mm per count
```

**Minimum Resolution Requirements for Mobile Robots**:
- Indoor navigation: 0.1-1mm per count (adequate)
- Precision manipulation: <0.1mm per count (high)
- Outdoor robots: 1-5mm per count (sufficient)

**WayfindR Resolution**: With 75:1 gearbox and 12 CPR encoder: **0.057mm/count** - excellent for indoor navigation

#### 2. Quadrature Output

**Why Quadrature Matters**:
- **Direction detection**: Two channels (A & B) with 90° phase shift
- **4× resolution**: Count on rising and falling edges of both channels
- **Error detection**: Detect invalid transitions (noise, missed counts)

**Quadrature State Machine**:
```
A: ──┐    ┌────┐    ┌──
     └────┘    └────┘
B: ────┐    ┌────┐
       └────┘    └────

Forward:  AB transitions: 00 → 10 → 11 → 01 → 00
Backward: AB transitions: 00 → 01 → 11 → 10 → 00
```

**Critical**: Always use quadrature encoders for robotics applications

#### 3. Electrical Interface

**Voltage Levels**:
- 3.3V logic: Direct connection to Raspberry Pi GPIO (preferred)
- 5V logic: Requires level shifter for Raspberry Pi
- Open collector: Requires pull-up resistors

**Output Type**:
- **Digital (recommended)**: Clean square wave, noise immunity
- Analog: Requires ADC, more complex processing

**Current Draw**: <20mA per channel typical

#### 4. Mechanical Compatibility

**WayfindR Requirements**:
- Compatible with micro metal gearmotors
- Fits within robot chassis constraints
- Extended motor shaft required for magnetic disk
- Secure mounting (no vibration-induced errors)

#### 5. Update Rate / Bandwidth

**Maximum Shaft Speed**:
```
Max wheel speed: 0.5 m/s (from current config)
Wheel circumference: 0.204 m
Max RPS = 0.5 / 0.204 = 2.45 rev/s = 147 RPM

With 75:1 gearbox:
Motor shaft RPM = 147 × 75 = 11,025 RPM
Maximum count rate = 11,025 / 60 × 3,600 = 661,500 counts/second
```

Most magnetic encoders support >500 kHz count rates - adequate for WayfindR

#### 6. Environmental Rating

**Operating Conditions for Indoor Robot**:
- Temperature: 0°C to 50°C (typical indoor range)
- Humidity: 20% to 80% non-condensing
- Dust: IP40 rating minimum (protected against >1mm objects)

Magnetic encoders typically meet these requirements without additional protection

---

## Hardware Recommendations

### Primary Recommendation: Pololu Magnetic Encoder Kit

#### Product: Pololu Magnetic Encoder Pair Kit for Micro Metal Gearmotors, 12 CPR

**Part Numbers**:
- [Product #3081](https://www.pololu.com/product/3081): Standard bottom-entry connector
- [Product #4761](https://www.pololu.com/product/4761): Side-entry connector (recommended for tight spaces)
- [Product #4760](https://www.pololu.com/product/4760): Top-entry connector

**Specifications**:
- Resolution: 12 CPR (motor shaft) × 4 (quadrature) = 48 edges per motor revolution
- Operating Voltage: 2.7V to 18V (5V recommended)
- Logic Levels: 3.3V/5V compatible
- Output Type: Digital quadrature (channels A & B)
- Pull-up Resistors: 10kΩ integrated
- Physical: 2mm thick magnetic disk (7.65mm OD, 1.0mm ID)
- Form Factor: Fits within 12mm × 10mm motor cross-section
- Technology: Dual Hall effect sensors with hysteresis (prevents spurious signals)

**Output CPR by Gearbox Ratio** (relevant for WayfindR):

| Gearbox Ratio | Output Shaft CPR | Linear Resolution (65mm wheel) |
|---------------|------------------|--------------------------------|
| 30:1 | 1,440 | 0.14mm/count |
| 50:1 | 2,400 | 0.085mm/count |
| 75:1 | 3,600 | 0.057mm/count |
| 100:1 | 4,800 | 0.042mm/count |
| 150:1 | 7,200 | 0.028mm/count |

**Cost**: $8.95 per encoder kit × 2 motors = **$17.90 total**

**Why This Product**:
1. Designed specifically for micro metal gearmotors (common in small robots)
2. Excellent resolution for the price point
3. Proven reliability in robotics community
4. Simple installation (magnetic disk press-fits onto motor shaft)
5. Wide voltage range supports various power architectures
6. Built-in hysteresis prevents false triggering

### Alternative Options

#### Option 2: DFRobot Micro DC Motor with Encoder

**Product**: [FIT0458 Micro DC Motor with Encoder-SJ02](https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ02_SKU__FIT0458)

**Specifications**:
- Integrated solution (motor + gearbox + encoder)
- Resolution: 8 PPR × 4 = 32 quadrature edges (before gearbox)
- Gearbox: 120:1 ratio = 3,840 counts per output revolution
- Voltage: 6V nominal
- Compatible with Arduino/Raspberry Pi
- Includes mounting bracket

**Pros**:
- All-in-one solution
- No encoder installation required
- Cost-effective

**Cons**:
- Fixed gearbox ratio
- Must replace existing motors
- Lower availability than Pololu

**Cost**: ~$15 per motor × 2 = **$30 total**

#### Option 3: DIY Optical Encoder

**Build Your Own** using:
- IR LED emitter/detector modules ($6 for pair)
- 3D printed encoder disk with slots
- Microcontroller for signal conditioning

**Pros**:
- Educational value
- Customizable resolution
- Very low cost

**Cons**:
- Significant time investment
- Reliability concerns
- No quadrature unless using dual sensors
- Susceptible to ambient light

**Not recommended for production systems**

### Wiring Requirements

**Per Encoder (2 required for differential drive)**:

| Signal | Purpose | Wire |
|--------|---------|------|
| VCC | Power supply (3.3-5V) | Red |
| GND | Ground reference | Black |
| OUT_A | Quadrature channel A | Yellow/Blue |
| OUT_B | Quadrature channel B | Green/White |

**Total**: 4 wires per encoder × 2 encoders = 8 wires to Raspberry Pi GPIO

---

## Raspberry Pi GPIO Integration

### GPIO Pin Selection

**Raspberry Pi GPIO Considerations**:
1. All GPIO pins support interrupt capability
2. Use consecutive pins for each encoder for easier wiring
3. Avoid reserved pins (I2C, SPI, UART if needed elsewhere)
4. Use 3.3V power from Pi (Pin 1 or Pin 17)
5. Use Ground pins (Pins 6, 9, 14, 20, 25, 30, 34, 39)

**Recommended Pin Assignment**:

| Encoder | Signal | GPIO Pin | Physical Pin | Description |
|---------|--------|----------|--------------|-------------|
| Left | VCC | 3.3V | Pin 1 | Power |
| Left | GND | Ground | Pin 6 | Ground |
| Left | OUT_A | GPIO 4 | Pin 7 | Channel A (interrupt) |
| Left | OUT_B | GPIO 17 | Pin 11 | Channel B (interrupt) |
| Right | VCC | 3.3V | Pin 17 | Power |
| Right | GND | Ground | Pin 9 | Ground |
| Right | OUT_A | GPIO 27 | Pin 13 | Channel A (interrupt) |
| Right | OUT_B | GPIO 22 | Pin 15 | Channel B (interrupt) |

**Why These Pins**:
- GPIO 4, 17, 27, 22: No special functions, reliable for GPIO
- Consecutive physical pins for clean wiring
- No conflicts with common peripherals (I2C on GPIO 2/3, SPI on GPIO 8-11)

### Wiring Diagram

```
Raspberry Pi GPIO Header (Top View)
┌──────────────────────────────────────┐
│  3.3V  (1) (2)  5V                   │ ← Left Encoder VCC to Pin 1
│  SDA   (3) (4)  5V                   │
│  SCL   (5) (6)  GND                  │ ← Left Encoder GND to Pin 6
│  GPIO4 (7) (8)  GPIO14               │ ← Left Encoder A to Pin 7
│  GND   (9)(10)  GPIO15               │ ← Right Encoder GND to Pin 9
│  GPIO17(11)(12) GPIO18               │ ← Left Encoder B to Pin 11
│  GPIO27(13)(14) GND                  │ ← Right Encoder A to Pin 13
│  GPIO22(15)(16) GPIO23               │ ← Right Encoder B to Pin 15
│  3.3V (17)(18)  GPIO24               │ ← Right Encoder VCC to Pin 17
│       ... (remaining pins)           │
└──────────────────────────────────────┘

Wire Color Coding (suggested):
Left Encoder:  Red (VCC) → Pin 1, Black (GND) → Pin 6
               Blue (A) → Pin 7, Yellow (B) → Pin 11
Right Encoder: Red (VCC) → Pin 17, Black (GND) → Pin 9
               Green (A) → Pin 13, White (B) → Pin 15
```

### Hardware Interface Circuit

**Basic Connection** (with Pololu encoder):
```
Encoder → Raspberry Pi
VCC (2.7-18V) → 3.3V (or 5V with level shifter)
GND → Ground
OUT_A → GPIO 4 (Left) or GPIO 27 (Right)
OUT_B → GPIO 17 (Left) or GPIO 22 (Right)

Notes:
- Pololu encoders have built-in 10kΩ pull-up resistors
- Can operate at 3.3V directly (no level shifter needed)
- If using 5V VCC, use voltage divider or level shifter on outputs
```

**Optional: Level Shifter Circuit** (if using 5V encoders):
```
        5V Encoder Output
              │
              ├───┤ 2.2kΩ ├───┐
              │                │
              │                ├──→ 3.3V Raspberry Pi GPIO
              │                │
              └───┤ 3.3kΩ ├───┴─── GND

Output voltage: 5V × (3.3k / (2.2k + 3.3k)) = 3V (safe for Pi)
```

### Python Encoder Reading Library

**Recommended Library**: [rpi-rotary-encoder-python](https://github.com/nstansby/rpi-rotary-encoder-python)

**Installation**:
```bash
pip3 install RPi.GPIO
git clone https://github.com/nstansby/rpi-rotary-encoder-python.git
cd rpi-rotary-encoder-python
pip3 install .
```

**Basic Usage Example**:
```python
#!/usr/bin/env python3
import time
from encoder import Encoder

# Initialize encoders
left_encoder = Encoder(4, 17)   # GPIO 4 (A), GPIO 17 (B)
right_encoder = Encoder(27, 22) # GPIO 27 (A), GPIO 22 (B)

# Read continuously
while True:
    left_count = left_encoder.read()
    right_count = right_encoder.read()

    print(f"Left: {left_count:6d}  Right: {right_count:6d}")
    time.sleep(0.1)
```

**Key Features**:
- Interrupt-driven (no polling overhead)
- Thread-safe
- Handles quadrature decoding automatically
- Provides position delta (velocity calculation)

### Alternative: Direct GPIO Interrupt Handling

**Using RPi.GPIO Library** (lower-level control):

```python
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Pin definitions
LEFT_A = 4
LEFT_B = 17
RIGHT_A = 27
RIGHT_B = 22

# Encoder state
left_count = 0
right_count = 0
left_last_a = 0
right_last_a = 0

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Add edge detection (interrupts)
    # Use BOTH edges for 4x resolution
    GPIO.add_event_detect(LEFT_A, GPIO.BOTH, callback=left_callback)
    GPIO.add_event_detect(RIGHT_A, GPIO.BOTH, callback=right_callback)

def left_callback(channel):
    global left_count, left_last_a

    # Read both channels
    a = GPIO.input(LEFT_A)
    b = GPIO.input(LEFT_B)

    # Determine direction using quadrature logic
    if a != left_last_a:  # Change on A
        if a != b:  # Forward
            left_count += 1
        else:       # Backward
            left_count -= 1

    left_last_a = a

def right_callback(channel):
    global right_count, right_last_a

    a = GPIO.input(RIGHT_A)
    b = GPIO.input(RIGHT_B)

    if a != right_last_a:
        if a != b:
            right_count += 1
        else:
            right_count -= 1

    right_last_a = a

# Main program
setup_gpio()
print("Encoder reading started. Press Ctrl+C to exit.")

try:
    while True:
        print(f"Left: {left_count:6d}  Right: {right_count:6d}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nExiting...")
    GPIO.cleanup()
```

**Important Notes**:
1. **Interrupt latency**: Keep callback functions fast (<100μs)
2. **Debouncing**: GPIO library includes built-in debouncing (default 0.5ms)
3. **Thread safety**: Use locks if accessing counts from multiple threads
4. **Signal quality**: Poor wiring can cause missed counts - use shielded cables

### Testing Encoder Installation

**Test Procedure**:

1. **Static Test** (no movement):
   ```python
   # Read encoders for 10 seconds
   # Expected: counts should be 0 or minimal drift (<5 counts)
   ```

2. **Manual Rotation Test**:
   ```python
   # Rotate wheel exactly 1 revolution by hand
   # Expected: count ≈ output_shaft_CPR (±2% acceptable)
   # Example: 75:1 gearbox → 3,600 counts expected
   ```

3. **Direction Test**:
   ```python
   # Rotate wheel forward → count increases
   # Rotate wheel backward → count decreases
   # If inverted, swap A and B connections
   ```

4. **High-Speed Test**:
   ```python
   # Run motor at maximum speed
   # Monitor for missed counts (irregular velocity)
   # Check for electromagnetic interference (EMI) from motor
   ```

5. **Noise Test**:
   ```python
   # Motor running at various speeds
   # Plot count rate vs. time → should be smooth
   # Spikes indicate electrical noise (improve shielding/filtering)
   ```

---

## ROS2 Odometry Best Practices

### Odometry Message Structure

**ROS2 Message**: `nav_msgs/msg/Odometry`

**Message Fields**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id               # "odom"

string child_frame_id           # "base_link"

geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    Point position              # x, y, z in meters
    Quaternion orientation      # qx, qy, qz, qw
  float64[36] covariance       # 6×6 covariance matrix

geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    Vector3 linear              # vx, vy, vz in m/s
    Vector3 angular             # ωx, ωy, ωz in rad/s
  float64[36] covariance       # 6×6 covariance matrix
```

### Computing Odometry from Wheel Encoders

**Differential Drive Kinematics** (forward kinematics):

**Given**:
- Left encoder counts: `enc_left`
- Right encoder counts: `enc_right`
- Time delta: `dt` (seconds)

**Step 1: Convert Counts to Distance**
```python
# Encoder parameters (from calibration)
COUNTS_PER_METER_LEFT = counts_per_rev / wheel_circumference
COUNTS_PER_METER_RIGHT = ...  # may differ due to wheel differences

# Distance traveled by each wheel since last update
dist_left = enc_left_delta / COUNTS_PER_METER_LEFT   # meters
dist_right = enc_right_delta / COUNTS_PER_METER_RIGHT # meters
```

**Step 2: Calculate Linear and Angular Displacement**
```python
# Robot center displacement (average of wheels)
dist_center = (dist_left + dist_right) / 2.0

# Change in orientation (difference between wheels)
delta_theta = (dist_right - dist_left) / wheelbase

# Instantaneous velocities
v_linear = dist_center / dt    # m/s
v_angular = delta_theta / dt   # rad/s
```

**Step 3: Update Pose (Integration)**
```python
# Method 1: Simple integration (assumes straight line in dt)
x += dist_center * cos(theta)
y += dist_center * sin(theta)
theta += delta_theta

# Method 2: Arc integration (more accurate for curves)
if abs(delta_theta) < 1e-6:  # Straight line
    x += dist_center * cos(theta)
    y += dist_center * sin(theta)
else:  # Curved path
    radius = dist_center / delta_theta
    x += radius * (sin(theta + delta_theta) - sin(theta))
    y += -radius * (cos(theta + delta_theta) - cos(theta))
theta += delta_theta

# Normalize theta to [-π, π]
theta = atan2(sin(theta), cos(theta))
```

**Step 4: Populate Odometry Message**
```python
odom_msg = Odometry()
odom_msg.header.stamp = self.get_clock().now().to_msg()
odom_msg.header.frame_id = "odom"
odom_msg.child_frame_id = "base_link"

# Position
odom_msg.pose.pose.position.x = x
odom_msg.pose.pose.position.y = y
odom_msg.pose.pose.position.z = 0.0

# Orientation (convert yaw to quaternion)
odom_msg.pose.pose.orientation = yaw_to_quaternion(theta)

# Velocity (in body frame)
odom_msg.twist.twist.linear.x = v_linear
odom_msg.twist.twist.angular.z = v_angular

# Covariance (covered in next section)
odom_msg.pose.covariance = [...]
odom_msg.twist.covariance = [...]
```

### Covariance Tuning

**What is Covariance?**
- Represents uncertainty in odometry measurements
- Used by Nav2 and robot_localization for sensor fusion
- 6×6 matrix (x, y, z, roll, pitch, yaw) but only diagonal values used for 2D robots

**Covariance Matrix Structure** (row-major order):
```
[σx²   0    0    0    0    0  ]
[0    σy²   0    0    0    0  ]
[0     0   σz²   0    0    0  ]  ← Not used in 2D
[0     0    0   σφ²   0    0  ]  ← Roll (not used)
[0     0    0    0   σθ²   0  ]  ← Pitch (not used)
[0     0    0    0    0   σψ² ]  ← Yaw
```

**Recommended Starting Values** (encoder-based odometry):

```python
# Pose covariance (position units: m², orientation: rad²)
pose_covariance = [
    0.001, 0, 0, 0, 0, 0,  # x:   1mm standard deviation
    0, 0.001, 0, 0, 0, 0,  # y:   1mm
    0, 0, 0, 0, 0, 0,      # z:   not used
    0, 0, 0, 0, 0, 0,      # φ:   not used
    0, 0, 0, 0, 0, 0,      # θ:   not used
    0, 0, 0, 0, 0, 0.01    # ψ:   0.1 rad (5.7°)
]

# Twist covariance (velocity units: (m/s)², (rad/s)²)
twist_covariance = [
    0.001, 0, 0, 0, 0, 0,  # vx:  0.03 m/s std dev
    0, 0, 0, 0, 0, 0,      # vy:  not used (diff drive)
    0, 0, 0, 0, 0, 0,      # vz:  not used
    0, 0, 0, 0, 0, 0,      # ωx:  not used
    0, 0, 0, 0, 0, 0,      # ωy:  not used
    0, 0, 0, 0, 0, 0.01    # ωz:  0.1 rad/s std dev
]
```

**Tuning Process**:

1. **Collect Ground Truth Data**:
   - Drive robot in known pattern (e.g., 1m square)
   - Measure actual vs. reported position
   - Repeat 10+ times

2. **Calculate Empirical Variance**:
   ```python
   x_errors = [actual_x[i] - odom_x[i] for i in range(n)]
   variance_x = np.var(x_errors)  # This becomes σx²
   ```

3. **Adjust Covariance**:
   - If Nav2 ignores odometry: covariances too high
   - If Nav2 trusts odometry too much: covariances too low
   - Increase values for slippery surfaces (carpet, wet floor)
   - Decrease values on smooth floors with good wheel traction

4. **Validation**:
   - Use `robot_localization` diagnostics to check innovation (difference between prediction and measurement)
   - Innovation should be small relative to covariance

**Critical Rule**: NEVER set covariance to zero for fused variables. Use small epsilon (1e-6) minimum.

### Publishing Rate

**Nav2 Requirements**:
- Minimum: 10 Hz
- Recommended: 20-50 Hz
- Maximum: 100 Hz (diminishing returns, CPU overhead)

**WayfindR Target**: 50 Hz (matches current dead reckoning rate)

**Implementation**:
```python
class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer at 50 Hz
        self.timer = self.create_timer(1.0 / 50.0, self.publish_odometry)

        # Encoder reader (running in background thread)
        self.encoders = EncoderReader()

    def publish_odometry(self):
        # Get encoder counts (thread-safe)
        left, right = self.encoders.get_counts()

        # Compute odometry
        odom_msg = self.compute_odometry(left, right)

        # Publish
        self.odom_pub.publish(odom_msg)

        # Also publish TF (odom → base_link)
        self.publish_tf(odom_msg)
```

### TF Tree Publishing

**Transform Structure**:
```
map                          ← Published by AMCL or SLAM
 └─ odom                     ← Published by odometry node
     └─ base_link            ← Fixed (identity)
         └─ laser            ← Static TF
```

**Publishing odom → base_link Transform**:
```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def publish_tf(self, odom_msg):
    """Broadcast odom → base_link transform"""
    t = TransformStamped()

    t.header.stamp = odom_msg.header.stamp
    t.header.frame_id = 'odom'
    t.child_frame_id = 'base_link'

    # Translation
    t.transform.translation.x = odom_msg.pose.pose.position.x
    t.transform.translation.y = odom_msg.pose.pose.position.y
    t.transform.translation.z = 0.0

    # Rotation
    t.transform.rotation = odom_msg.pose.pose.orientation

    self.tf_broadcaster.sendTransform(t)
```

**Important**: Only ONE node should publish each transform. If `cmd_vel_bridge` currently publishes odom → base_link, disable it when encoder odometry is active.

---

## ros2_control Integration

### Overview of ros2_control

`ros2_control` is the standard ROS2 framework for robot control, providing:
- Hardware abstraction layer
- Controller plugins (diff_drive_controller, joint_state_controller)
- Real-time capable architecture
- Standardized interfaces

**Architecture**:
```
Nav2 Controller
    ↓ (cmd_vel)
diff_drive_controller
    ↓ (wheel velocities)
Hardware Interface
    ↓ (GPIO / motor driver)
Physical Robot
    ↑ (encoder counts)
Hardware Interface
    ↑ (joint states)
Joint State Publisher
```

### Hardware Interface Implementation

**Create Hardware Interface Class** (`wayfinder_hardware.cpp` or `.py`):

```python
from ros2_control_py_hardware_interface import SystemInterface
from rclpy.duration import Duration

class WayfinderHardware(SystemInterface):
    """
    Hardware interface for WayfindR differential drive robot.
    Interfaces with PI_API for motor control and GPIO for encoders.
    """

    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger('WayfinderHardware')

    def on_init(self, hardware_info):
        """Initialize hardware interface"""
        # Parse parameters from URDF
        self.left_wheel_name = "left_wheel_joint"
        self.right_wheel_name = "right_wheel_joint"

        # State interfaces (what we READ from hardware)
        self.hw_positions = [0.0, 0.0]  # Wheel positions in radians
        self.hw_velocities = [0.0, 0.0] # Wheel velocities in rad/s

        # Command interfaces (what we WRITE to hardware)
        self.hw_commands = [0.0, 0.0]   # Velocity commands in rad/s

        # Initialize encoder reader
        self.encoders = EncoderReader(
            left_pins=(4, 17),
            right_pins=(27, 22)
        )

        # Initialize motor controller (PI_API client)
        self.motor_controller = MotorController(api_url="http://localhost:8000")

        return True

    def read(self, time, period):
        """
        Read encoder values and update state interfaces.
        Called by controller manager at control loop rate (typically 50-100 Hz).
        """
        # Get encoder counts
        left_counts, right_counts = self.encoders.get_counts()

        # Convert counts to radians
        # Position: θ = counts / counts_per_rev * 2π
        self.hw_positions[0] = left_counts / self.counts_per_rev * 2 * pi
        self.hw_positions[1] = right_counts / self.counts_per_rev * 2 * pi

        # Velocity: ω = Δθ / Δt
        dt = period.nanoseconds / 1e9
        self.hw_velocities[0] = (self.hw_positions[0] - self.last_positions[0]) / dt
        self.hw_velocities[1] = (self.hw_positions[1] - self.last_positions[1]) / dt

        self.last_positions = self.hw_positions.copy()

    def write(self, time, period):
        """
        Send velocity commands to motors.
        Called by controller manager after controllers have updated commands.
        """
        # Get commanded wheel velocities (rad/s)
        v_left = self.hw_commands[0]
        v_right = self.hw_commands[1]

        # Convert to motor driver format (throttle, steering)
        throttle, steering = self.wheel_vel_to_motor_cmd(v_left, v_right)

        # Send to PI_API
        self.motor_controller.set_velocity(throttle, steering)

    def export_state_interfaces(self):
        """Define what state data is available to controllers"""
        return [
            StateInterface(self.left_wheel_name, "position", self.hw_positions[0]),
            StateInterface(self.left_wheel_name, "velocity", self.hw_velocities[0]),
            StateInterface(self.right_wheel_name, "position", self.hw_positions[1]),
            StateInterface(self.right_wheel_name, "velocity", self.hw_velocities[1]),
        ]

    def export_command_interfaces(self):
        """Define what commands controllers can send"""
        return [
            CommandInterface(self.left_wheel_name, "velocity", self.hw_commands[0]),
            CommandInterface(self.right_wheel_name, "velocity", self.hw_commands[1]),
        ]
```

### URDF Configuration

**Add ros2_control Hardware Tag to URDF** (`wayfinder_robot.urdf.xacro`):

```xml
<ros2_control name="WayfinderSystem" type="system">
  <hardware>
    <plugin>wayfinder_hardware/WayfinderHardware</plugin>
    <param name="api_url">http://localhost:8000</param>
    <param name="encoder_cpr">3600</param>  <!-- 75:1 gearbox × 12 CPR × 4 -->
  </hardware>

  <!-- Left wheel joint -->
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>  <!-- rad/s -->
      <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- Right wheel joint -->
  <joint name="right_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### diff_drive_controller Configuration

**Controller Config File** (`config/diff_drive_controller.yaml`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    # Wheel joints
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    # Kinematics
    wheel_separation: 0.15  # meters (matches URDF)
    wheel_radius: 0.0325    # meters (matches URDF)

    # Feedback interface (read from encoders)
    # Options: position or velocity
    position_feedback: true  # Use encoder position for odometry

    # Open loop (false = use encoder feedback)
    open_loop: false

    # Odometry
    publish_odom: true
    publish_odom_tf: true
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal: [0.001, 0.001, 0.0, 0.0, 0.0, 0.01]
    twist_covariance_diagonal: [0.001, 0.0, 0.0, 0.0, 0.0, 0.01]

    # Velocity limits
    linear:
      x:
        has_velocity_limits: true
        max_velocity: 0.5         # m/s
        has_acceleration_limits: true
        max_acceleration: 1.0     # m/s²

    angular:
      z:
        has_velocity_limits: true
        max_velocity: 1.0         # rad/s
        has_acceleration_limits: true
        max_acceleration: 2.0     # rad/s²
```

### Launch File Integration

**Launch ros2_control Stack** (`launch/robot_control.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                'config/diff_drive_controller.yaml'
            ],
            output='screen'
        ),

        # Spawn diff_drive_controller
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller',
                 'diff_drive_controller', '--set-state', 'active'],
            output='screen'
        ),

        # Spawn joint_state_broadcaster
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller',
                 'joint_state_broadcaster', '--set-state', 'active'],
            output='screen'
        ),
    ])
```

### Benefits of ros2_control Approach

1. **Standardization**: Industry-standard approach, compatible with all Nav2 tools
2. **Separation of Concerns**: Hardware interface separate from control logic
3. **Reusability**: Swap controllers without changing hardware interface
4. **Real-time Capable**: Designed for deterministic control loops
5. **Simulation Support**: Same controllers work in Gazebo simulation
6. **Ecosystem**: Leverage existing controllers and tools

### Alternative: Custom Odometry Node

If ros2_control is too complex for initial prototyping, create a standalone odometry node:

```python
class EncoderOdometry(Node):
    """Simple encoder-based odometry publisher"""

    def __init__(self):
        super().__init__('encoder_odometry')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Encoder reader
        self.encoders = EncoderReader(...)

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Timer (50 Hz)
        self.timer = self.create_timer(0.02, self.update_odometry)

    def update_odometry(self):
        # Read encoders
        left, right = self.encoders.get_counts_delta()

        # Compute odometry (forward kinematics)
        dist_left = left / COUNTS_PER_METER
        dist_right = right / COUNTS_PER_METER

        dist = (dist_left + dist_right) / 2
        dtheta = (dist_right - dist_left) / WHEELBASE

        # Update pose
        self.x += dist * cos(self.theta)
        self.y += dist * sin(self.theta)
        self.theta += dtheta

        # Publish odometry and TF
        self.publish_odometry()
```

**Pros**: Simple, quick to implement, easy to debug
**Cons**: Not integrated with ros2_control ecosystem, less flexible

---

## Integration with cmd_vel_bridge

### Current Architecture

**cmd_vel_bridge.py** currently:
1. Subscribes to `/cmd_vel` (geometry_msgs/Twist)
2. Converts Twist to motor commands (throttle, steering)
3. Sends HTTP requests to PI_API
4. **Publishes dead reckoning odometry** (open-loop)
5. Publishes TF (odom → base_link)

### Integration Strategy: Option 1 - Separate Encoder Node

**Architecture**:
```
Nav2 → /cmd_vel → cmd_vel_bridge → PI_API → Motors
                                              ↓
                        Encoders → encoder_odometry_node → /odom, /tf
```

**Changes Required**:

1. **Create new encoder_odometry_node**:
   - Reads GPIO encoders
   - Publishes `/odom` and TF
   - Independent of cmd_vel_bridge

2. **Modify cmd_vel_bridge**:
   - Remove odometry publishing code (lines 482-605)
   - Remove TF broadcasting
   - Keep only cmd_vel → motor control functionality
   - Optionally: Subscribe to `/odom` for velocity feedback

**config/cmd_vel_bridge_params.yaml** (updated):
```yaml
cmd_vel_bridge:
  ros__parameters:
    # ... existing parameters ...

    # Disable odometry publishing (handled by encoder node)
    publish_odom: false
    publish_tf: false
```

**Pros**:
- Minimal changes to cmd_vel_bridge
- Clear separation of concerns
- Easy to test independently
- Can run cmd_vel_bridge without encoders (simulation)

**Cons**:
- Two separate nodes (more complexity)
- No direct feedback loop in motor control

### Integration Strategy: Option 2 - Unified Node

**Architecture**:
```
Nav2 → /cmd_vel → cmd_vel_bridge_with_encoders → PI_API → Motors
                                                    ↓
                                                 Encoders
                                                    ↓
                                            /odom, /tf
```

**Changes Required**:

1. **Add encoder reading to cmd_vel_bridge**:
   - Initialize encoder reader in `__init__()`
   - Replace `_update_odometry()` with encoder-based version
   - Keep existing TF and odometry publishing

2. **Add closed-loop velocity control** (optional):
   - Compare commanded velocity vs. actual (from encoders)
   - Adjust motor commands to compensate for slip

**Example Code** (additions to cmd_vel_bridge.py):

```python
class CmdVelBridge(Node):
    def __init__(self):
        # ... existing initialization ...

        # Initialize encoders
        self.encoders = EncoderReader(
            left_pins=(4, 17),
            right_pins=(27, 22),
            counts_per_rev=3600  # 75:1 gearbox × 12 CPR × 4
        )
        self.logger.info('Encoders initialized')

    def _update_odometry(self, dt: float):
        """
        Update robot pose using encoder feedback.
        Replaces dead reckoning with measured odometry.
        """
        # Get encoder deltas since last update
        left_counts, right_counts = self.encoders.get_delta()

        # Convert to distance (meters)
        dist_left = left_counts / self.counts_per_meter_left
        dist_right = right_counts / self.counts_per_meter_right

        # Differential drive kinematics
        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheelbase

        # Update pose (arc integration)
        if abs(delta_theta) < 1e-6:
            self.x += dist_center * math.cos(self.theta)
            self.y += dist_center * math.sin(self.theta)
        else:
            radius = dist_center / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update velocities (actual, not commanded)
        self.current_v_x = dist_center / dt
        self.current_omega_z = delta_theta / dt
```

**Pros**:
- Single node (simpler architecture)
- Easy to add closed-loop control
- Encoder data immediately available for motor adjustments

**Cons**:
- More complex node (harder to maintain)
- Harder to test in simulation (requires encoder mocking)
- Tightly coupled

### Integration Strategy: Option 3 - Full ros2_control

**Architecture**:
```
Nav2 → /cmd_vel → diff_drive_controller → Hardware Interface
                                                 ↓
                                           PI_API + Encoders
                                                 ↓
                                            /odom, /tf
```

**Changes Required**:

1. **Deprecate cmd_vel_bridge** entirely
2. **Create hardware_interface** (as shown in ros2_control section)
3. **Use standard diff_drive_controller**

**Pros**:
- Industry standard approach
- Maximum flexibility
- Best long-term solution
- Gazebo simulation compatible

**Cons**:
- Most complex implementation
- Steep learning curve
- More files and configuration

### Recommended Approach

**For WayfindR Project**:

**Phase 1** (Initial Implementation): **Option 1 - Separate Encoder Node**
- Quick to implement
- Low risk (doesn't break existing system)
- Easy to validate encoder accuracy independently
- Can run both old (dead reckoning) and new (encoder) odometry simultaneously for comparison

**Phase 2** (Advanced Integration): **Option 3 - Full ros2_control**
- Migrate when system is stable
- Enables advanced features (trajectory control, force feedback, etc.)
- Better for production deployment

---

## Sensor Fusion with IMU

### Why Sensor Fusion?

**Encoder Limitations**:
- Wheel slip (carpet, wet floors, sudden accelerations)
- Calibration errors accumulate
- No absolute heading reference (yaw drift)

**IMU Advantages**:
- Measures angular velocity directly (independent of wheels)
- Detects slip events (acceleration mismatch)
- Provides absolute orientation (with magnetometer)

**Combined System**:
- Encoders: Accurate short-term position and velocity
- IMU: Accurate orientation and slip detection
- Fusion: Best of both worlds

### IMU Sensor Recommendations

**Option 1: MPU6050** (Budget)
- 6-axis (3-axis gyro + 3-axis accelerometer)
- I2C interface
- Cost: $2-5
- No magnetometer (yaw drift remains)
- Sufficient for basic fusion

**Option 2: BNO055** (Recommended)
- 9-axis (gyro + accel + magnetometer)
- Built-in sensor fusion (quaternion output)
- I2C interface
- Cost: $20-30
- Absolute orientation (no yaw drift)
- Easier integration

**Option 3: BMI088** (Advanced)
- 6-axis automotive-grade
- High accuracy, low noise
- SPI/I2C interface
- Cost: $15-25

**Recommendation**: **BNO055** for ease of use and absolute heading

### Hardware Connection (BNO055)

**I2C Wiring**:
```
BNO055 → Raspberry Pi
VIN → 3.3V (Pin 1)
GND → Ground (Pin 6)
SDA → GPIO 2 (Pin 3) - I2C Data
SCL → GPIO 3 (Pin 5) - I2C Clock
```

**Enable I2C on Raspberry Pi**:
```bash
sudo raspi-config
# Interface Options → I2C → Enable

# Verify I2C device
sudo i2cdetect -y 1
# Should show device at address 0x28 or 0x29
```

### robot_localization Package

**Installation**:
```bash
sudo apt install ros-humble-robot-localization
```

**Purpose**: Fuses multiple odometry sources using Extended Kalman Filter (EKF)

**Configuration** (`config/ekf_fusion.yaml`):

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0  # Filter update rate (Hz)

    # Frame IDs
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom  # Use 'odom' for odometry, 'map' for full localization

    # Sensor inputs
    odom0: /odom  # Wheel encoder odometry
    odom0_config: [true,  true,  false,  # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
    odom0_queue_size: 10
    odom0_differential: false

    imu0: /imu/data  # IMU data
    imu0_config: [false, false, false,  # x, y, z (don't use position from IMU)
                  false, false, true,   # roll, pitch, yaw (use orientation)
                  false, false, false,  # vx, vy, vz (don't use linear velocity)
                  false, false, true,   # vroll, vpitch, vyaw (use angular velocity)
                  true,  true,  false]  # ax, ay, az (use linear acceleration)
    imu0_queue_size: 10
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true

    # Process noise covariance (how much we trust the motion model)
    process_noise_covariance: [0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                                0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
                                0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
                                0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
                                0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015]

    # Output
    publish_tf: true
    publish_acceleration: false
```

**Launch Integration** (`launch/sensor_fusion.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Encoder odometry node
        Node(
            package='wayfinder_odometry',
            executable='encoder_odometry',
            name='encoder_odometry',
            output='screen'
        ),

        # IMU driver (BNO055)
        Node(
            package='bno055',
            executable='bno055',
            name='imu_node',
            parameters=[{
                'frame_id': 'imu_link',
                'frequency': 100  # Hz
            }],
            output='screen'
        ),

        # EKF fusion node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['config/ekf_fusion.yaml'],
            remappings=[
                ('/odometry/filtered', '/odom_filtered')
            ],
            output='screen'
        ),
    ])
```

**Data Flow**:
```
Wheel Encoders → /odom (raw) ─┐
                               ├→ EKF → /odom_filtered (fused)
IMU Sensor → /imu/data ────────┘
```

**Using Fused Odometry in Nav2**:
```yaml
# In nav2_params.yaml, change odometry topic:
controller_server:
  ros__parameters:
    odom_topic: /odom_filtered  # Use fused instead of raw
```

### Benefits of Sensor Fusion

1. **Slip Detection**: IMU detects wheel slip (acceleration mismatch with velocity)
2. **Improved Heading**: IMU gyroscope + magnetometer provides accurate yaw
3. **Robustness**: System continues working even if one sensor fails
4. **Reduced Drift**: EKF corrects encoder drift using IMU orientation
5. **Better Localization**: More accurate odometry improves AMCL performance

---

## Error Sources and Mitigation

### 1. Quantization Error

**Cause**: Discrete encoder counts vs. continuous motion

**Effect**:
- "Staircase" velocity profile at low speeds
- Position jitter (±0.5 counts)

**Magnitude**:
```
Resolution: 0.057mm per count (WayfindR with 75:1 gearbox)
Quantization error: ±0.0285mm (half count)
```

**Mitigation**:
- Use high-resolution encoders (3,600+ CPR recommended)
- Filter velocity estimates (exponential moving average)
- Use time-stamping for accurate velocity calculation
- Interpolate between counts for sub-count resolution

**Example Filtering**:
```python
# Exponential moving average filter
alpha = 0.2  # Smoothing factor (0 = max smoothing, 1 = no filtering)
filtered_velocity = alpha * measured_velocity + (1 - alpha) * previous_filtered_velocity
```

### 2. Wheel Slip

**Cause**: Loss of traction between wheel and ground

**Conditions**:
- Smooth floors (tile, linoleum)
- Carpet (high friction, but fiber compression)
- Wet surfaces
- Sudden acceleration/deceleration
- Turning at high speed

**Effect**: Encoders report motion, but robot doesn't move as expected

**Magnitude**: 1-10% error typical, up to 50% during severe slip

**Mitigation**:
- Use high-friction wheels (rubber, silicone)
- Limit acceleration (max_acceleration parameter)
- Sensor fusion with IMU (detect slip via acceleration mismatch)
- Increase wheel contact pressure (robot weight distribution)
- Use carpet-friendly wheel treads

**Detection**:
```python
# Compare encoder velocity vs. IMU acceleration
expected_accel = (current_velocity - previous_velocity) / dt
measured_accel = imu_linear_x

if abs(expected_accel - measured_accel) > SLIP_THRESHOLD:
    logger.warning("Wheel slip detected!")
    # Increase odometry covariance or apply slip correction
```

### 3. Calibration Errors

**Cause**: Incorrect physical parameters

**Common Errors**:
- Wheel diameter mismeasurement
- Wheelbase (track width) error
- Encoder CPR miscounting
- Left/right wheel diameter mismatch

**Effect**: Systematic odometry error (consistent over/under reporting)

**Example**:
```
Actual wheel diameter: 65mm
Measured wheel diameter: 64mm (1.5% error)
→ Robot reports 1.015m traveled for every 1.0m actual
→ 1.5% systematic position error
```

**Mitigation**:
- Precise physical measurements (calipers for wheel diameter)
- Calibration procedure (UMBmark test - drive square pattern)
- Separate calibration for left and right wheels
- Periodic re-calibration (wheels wear over time)

**Calibration Process** (see Testing section for details)

### 4. Mechanical Backlash

**Cause**: Play/slop in gearbox and drivetrain

**Effect**:
- Position error when changing direction
- "Dead zone" near zero velocity

**Magnitude**: Typically 1-5° backlash in low-cost gearboxes (0.05-0.25mm at wheel)

**Mitigation**:
- Use high-quality gearboxes (metal gears, precision manufacturing)
- Preload gearboxes (apply slight tension)
- Software compensation (add backlash parameter)
- Avoid rapid direction changes

### 5. Electromagnetic Interference (EMI)

**Cause**: Motor electromagnetic noise coupling into encoder signals

**Effect**:
- False encoder counts
- Missed counts
- Erratic velocity readings

**Mitigation**:
- Shielded cables for encoder wiring
- Twisted-pair wiring (A and B channels twisted together)
- Separate power and signal grounds
- Add capacitors across motor terminals (0.1μF ceramic)
- Keep encoder wires away from motor power wires
- Use differential signaling (if supported by encoder)

**Testing for EMI**:
```python
# Monitor encoder counts while motor is stationary but powered
# Any counts indicate EMI or encoder defect
```

### 6. Temperature Drift

**Cause**: Encoder electrical characteristics change with temperature

**Effect**:
- Small CPR variation (typically <0.1%)
- Negligible for most robotics applications

**Mitigation**:
- Use automotive-grade encoders (if required)
- Allow warm-up period before calibration
- Indoor robots: typically not an issue

### 7. Sample Rate / Aliasing

**Cause**: Encoder count rate exceeds sampling rate

**Effect**: Missed counts, incorrect velocity

**Example**:
```
Max velocity: 0.5 m/s
CPR: 3,600
Count rate: 661,500 counts/second (calculated earlier)
Required sample rate: >1.32 MHz (Nyquist theorem)

BUT: Raspberry Pi GPIO interrupt rate limited to ~10-50 kHz
```

**Mitigation**:
- For WayfindR: Count rate is well within GPIO interrupt capability
- Use hardware quadrature decoder (if count rates exceed software capability)
- Reduce maximum velocity if count rates are too high

### 8. Wheel Diameter Mismatch

**Cause**: Manufacturing tolerances, uneven wear, tire pressure

**Effect**: Robot drives in arc instead of straight line

**Detection**:
```python
# Drive robot "straight" for 5 meters
# Measure lateral deviation
# If deviation > 50mm, wheel mismatch likely
```

**Mitigation**:
- Match wheel pairs carefully
- Calibrate counts_per_meter separately for left and right
- Replace wheels as matched pairs
- Regular inspection for uneven wear

### Error Mitigation Summary Table

| Error Source | Magnitude | Mitigation | Priority |
|--------------|-----------|------------|----------|
| Quantization | ±0.03mm | High CPR, filtering | Low |
| Wheel Slip | 1-50% | IMU fusion, friction | **High** |
| Calibration | 1-5% | Precise measurement | **High** |
| Backlash | 0.05-0.25mm | Quality gearbox | Medium |
| EMI | Variable | Shielding, filtering | Medium |
| Temperature | <0.1% | Automotive parts | Low |
| Aliasing | N/A | Sufficient sample rate | Low (WayfindR) |
| Wheel Mismatch | 0.5-2% | Matched pairs, calibration | Medium |

---

## Calibration and Testing

### Pre-Calibration: Physical Measurements

**Required Measurements**:

1. **Wheel Diameter** (precise method):
   ```
   Materials: Digital calipers, flat surface, weight
   Procedure:
   1. Place robot on flat surface
   2. Measure wheel diameter at 3 points (tire may not be perfectly round)
   3. Average measurements
   4. Repeat with robot weight on wheels (tire compression)
   5. Use loaded diameter for calculations

   Example:
   Unloaded: 65.2mm, 65.0mm, 65.1mm → Average: 65.1mm
   Loaded:   64.8mm, 64.9mm, 64.7mm → Average: 64.8mm
   Use: 64.8mm = 0.0648m
   ```

2. **Wheelbase** (track width):
   ```
   Measurement: Center of left tire to center of right tire
   Method: Measure from wheel mounting points or use calipers

   WayfindR URDF: 0.15m (150mm)
   Verify with physical measurement
   ```

3. **Encoder CPR** (verify specification):
   ```
   Motor shaft: 12 CPR (quadrature) = 48 counts/rev
   Gearbox ratio: 75:1 (example)
   Output shaft: 48 × 75 = 3,600 counts/rev

   Verification:
   - Manually rotate wheel exactly 1 revolution
   - Count encoder pulses
   - Should match calculated CPR (±1%)
   ```

### Calibration Step 1: Counts Per Meter

**Objective**: Determine exact conversion factor from encoder counts to distance

**Procedure**:

1. **Mark starting position** on floor with tape
2. **Drive robot straight** for known distance (recommend 5 meters)
3. **Record encoder counts** for journey
4. **Calculate conversion factor**:
   ```python
   counts_per_meter = total_counts / actual_distance

   Example:
   Left encoder: 17,532 counts
   Right encoder: 17,489 counts
   Actual distance: 5.00 meters

   counts_per_meter_left = 17,532 / 5.0 = 3,506.4 counts/m
   counts_per_meter_right = 17,489 / 5.0 = 3,497.8 counts/m
   ```

5. **Repeat 10 times**, average results
6. **Compare to theoretical**:
   ```python
   wheel_circumference = π × diameter = π × 0.0648m = 0.2036m
   counts_per_rev = 3,600
   theoretical_counts_per_meter = 3,600 / 0.2036 = 17,682 counts/m

   If measured differs by >2%, re-check wheel diameter measurement
   ```

### Calibration Step 2: Wheelbase (UMBmark Test)

**UMBmark (University of Michigan Benchmark)**: Standard robot calibration procedure

**Procedure**:

1. **Mark starting pose** (position and orientation) on floor
2. **Drive square pattern** (4 × 1.5m sides, 90° turns)
3. **Return to start** and measure error
4. **Calculate wheelbase correction**

**Square Pattern** (clockwise):
```
1. Drive forward 1.5m
2. Rotate 90° right (clockwise)
3. Drive forward 1.5m
4. Rotate 90° right
5. Drive forward 1.5m
6. Rotate 90° right
7. Drive forward 1.5m
8. Rotate 90° right
→ Should return to exact starting position and orientation
```

**Error Measurement**:
```python
# After completing square
position_error = sqrt(x_error² + y_error²)  # Linear offset from start
orientation_error = final_heading - initial_heading  # Angular error

Example errors:
X error: +50mm (drifted right)
Y error: +30mm (drifted forward)
Heading error: +5° (rotated too much)

Position error = sqrt(50² + 30²) = 58mm
```

**Wheelbase Correction** (if systematic rotation error):
```python
# If robot consistently over-rotates (heading error positive):
# Wheelbase is overestimated, reduce it
# If robot under-rotates (heading error negative):
# Wheelbase is underestimated, increase it

Correction factor = measured_wheelbase × (1 + heading_error / 360)

Example:
Measured wheelbase: 0.150m
Heading error after 4 rotations (1440°): +5°
Correction: 0.150 × (1 + 5 / 1440) = 0.1505m

New wheelbase: 0.1505m
```

**Repeat test** with corrected wheelbase until position and orientation errors <2%

### Calibration Step 3: Straight-Line Test

**Objective**: Verify left/right wheel balance

**Procedure**:

1. **Drive straight** for 10 meters
2. **Measure lateral deviation** (drift left/right)
3. **If deviation > 50mm**: Wheel diameter mismatch

**Correction**:
```python
# If robot drifts right → left wheel travels farther → left wheel larger
# Adjust counts_per_meter for left wheel slightly higher

deviation_ratio = lateral_deviation / distance
adjustment = 1 + deviation_ratio

# Example: 80mm right drift over 10m
# Adjust left counts_per_meter by 0.8% upward
counts_per_meter_left *= 1.008
```

### Testing Step 1: Static Accuracy Test

**Procedure**:
1. Encoders stationary
2. Monitor counts for 60 seconds
3. **Expected**: 0 counts (±1 count acceptable)
4. **If counts drift**: EMI or encoder defect

### Testing Step 2: Repeatability Test

**Procedure**:
1. Drive to waypoint 5 meters away
2. Return to start
3. Measure position error
4. Repeat 10 times

**Acceptance Criteria**:
- Mean position error: <20mm
- Standard deviation: <10mm

### Testing Step 3: Path Accuracy Test

**Procedure**:
1. Use Nav2 to follow a 10m path with turns
2. Measure final position error vs. ground truth
3. Calculate percentage error

**Acceptance Criteria**:
- Position error: <1% of path length (100mm error for 10m path)

### Testing Step 4: Velocity Accuracy Test

**Procedure**:
1. Command constant velocity (e.g., 0.3 m/s)
2. Measure actual velocity from encoders
3. Compare commanded vs. measured

**Acceptance Criteria**:
- Velocity error: <5% steady-state
- Velocity noise: <0.02 m/s standard deviation

### Testing Step 5: Slip Detection Test

**Procedure** (with IMU fusion):
1. Drive on slippery surface (smooth tile with oil/water)
2. Monitor EKF innovation (difference between encoder and IMU)
3. Verify slip detection triggers

### Calibration Data Storage

**Store calibration parameters** in config file:

```yaml
# config/encoder_calibration.yaml
encoder_odometry:
  ros__parameters:
    # Calibration date
    calibration_date: "2026-01-11"

    # Left wheel
    left_counts_per_meter: 17682.5
    left_wheel_diameter: 0.0648  # meters

    # Right wheel
    right_counts_per_meter: 17691.2
    right_wheel_diameter: 0.0647  # meters

    # Wheelbase (track width)
    wheelbase: 0.1505  # meters (corrected from UMBmark)

    # Encoder specifications
    encoder_cpr_motor_shaft: 48  # quadrature counts per motor revolution
    gearbox_ratio: 75
    encoder_cpr_output_shaft: 3600
```

### Validation Metrics

| Metric | Target | Good | Acceptable | Poor |
|--------|--------|------|------------|------|
| Straight-line error (5m) | <10mm | <25mm | <50mm | >50mm |
| UMBmark position error | <20mm | <50mm | <100mm | >100mm |
| UMBmark heading error | <2° | <5° | <10° | >10° |
| Velocity accuracy | <2% | <5% | <10% | >10% |
| Repeatability (std dev) | <5mm | <10mm | <20mm | >20mm |

---

## Implementation Plan

### Phase 1: Hardware Setup

**Tasks**:

1. **Order Hardware**:
   - [ ] 2× Pololu Magnetic Encoder Kit (#3081 or #4761) - $18
   - [ ] (Optional) BNO055 IMU sensor - $25
   - [ ] Jumper wires (female-female, 20cm) - $5
   - **Total**: ~$48 including shipping

2. **Install Encoders**:
   - [ ] Verify motors have extended back shafts
   - [ ] Press-fit magnetic disks onto motor shafts
   - [ ] Mount encoder boards to motor housing
   - [ ] Secure with provided adhesive or screws
   - [ ] Test mechanical installation (no interference with rotation)

3. **Wire to Raspberry Pi**:
   - [ ] Connect encoder VCC to 3.3V (Pins 1, 17)
   - [ ] Connect encoder GND to Ground (Pins 6, 9)
   - [ ] Connect encoder A channels to GPIO 4, 27 (Pins 7, 13)
   - [ ] Connect encoder B channels to GPIO 17, 22 (Pins 11, 15)
   - [ ] Use heat shrink tubing or cable management for clean installation
   - [ ] Label wires clearly

4. **Test Encoder Signals**:
   - [ ] Run GPIO test script (see Raspberry Pi GPIO Integration section)
   - [ ] Verify counts increase when wheels rotate forward
   - [ ] Verify counts decrease when wheels rotate backward
   - [ ] Test at various speeds (slow, medium, max)
   - [ ] Check for EMI (counts with motor stationary)

### Phase 2: Software Development

**Tasks**:

1. **Create Encoder Reader Library**:
   - [ ] Install RPi.GPIO: `pip3 install RPi.GPIO`
   - [ ] Create `encoder_reader.py` class
   - [ ] Implement interrupt-based quadrature decoding
   - [ ] Add thread-safe count access methods
   - [ ] Test thoroughly with bench test

2. **Create Encoder Odometry Node**:
   - [ ] Create new ROS2 package: `wayfinder_odometry`
   - [ ] Implement `encoder_odometry_node.py`:
     - [ ] Initialize encoder readers
     - [ ] Implement forward kinematics
     - [ ] Publish `/odom` messages at 50 Hz
     - [ ] Broadcast `odom → base_link` TF
   - [ ] Create parameter file `encoder_odometry_params.yaml`
   - [ ] Create launch file `encoder_odometry.launch.py`

3. **Update cmd_vel_bridge**:
   - [ ] Modify `cmd_vel_bridge.py`:
     - [ ] Add parameter `publish_odom: false`
     - [ ] Conditionally disable odometry publishing
     - [ ] Conditionally disable TF broadcasting
   - [ ] Update `cmd_vel_bridge_params.yaml`
   - [ ] Test cmd_vel_bridge without odometry publishing

4. **Create Launch Integration**:
   - [ ] Create `robot_bringup.launch.py`:
     - [ ] Launch cmd_vel_bridge (motor control only)
     - [ ] Launch encoder_odometry (odometry publishing)
     - [ ] Launch robot_state_publisher (URDF → TF)
   - [ ] Test integrated system

### Phase 3: Calibration

**Tasks**:

1. **Physical Measurements**:
   - [ ] Measure wheel diameter (loaded) with calipers
   - [ ] Measure wheelbase with calipers
   - [ ] Verify encoder CPR (manual rotation test)
   - [ ] Document measurements in calibration file

2. **Counts Per Meter Calibration**:
   - [ ] Set up 5-meter straight test course
   - [ ] Run straight-line test (10 repetitions)
   - [ ] Calculate counts_per_meter for each wheel
   - [ ] Update calibration file

3. **UMBmark Test**:
   - [ ] Mark 1.5m square pattern on floor
   - [ ] Run square pattern (5 repetitions)
   - [ ] Measure position and orientation errors
   - [ ] Calculate wheelbase correction factor
   - [ ] Update calibration file
   - [ ] Repeat until errors <2%

4. **Validation**:
   - [ ] Run all testing procedures (Static, Repeatability, Path, Velocity)
   - [ ] Document results
   - [ ] Fine-tune covariance values based on empirical data

### Phase 4: Sensor Fusion (Optional)

**Tasks**:

1. **IMU Hardware Setup**:
   - [ ] Connect BNO055 to I2C (GPIO 2/3)
   - [ ] Enable I2C on Raspberry Pi
   - [ ] Install BNO055 ROS2 driver: `sudo apt install ros-humble-bno055`
   - [ ] Test IMU data publishing

2. **robot_localization Configuration**:
   - [ ] Install: `sudo apt install ros-humble-robot-localization`
   - [ ] Create `ekf_fusion.yaml` configuration
   - [ ] Add IMU link to URDF (fixed transform from base_link)
   - [ ] Create `sensor_fusion.launch.py`

3. **Integration and Tuning**:
   - [ ] Launch full sensor fusion system
   - [ ] Monitor `/odom` (raw) vs. `/odom_filtered` (fused)
   - [ ] Tune process noise covariance
   - [ ] Update Nav2 params to use `/odom_filtered`

4. **Slip Testing**:
   - [ ] Create slippery test surface
   - [ ] Verify slip detection works
   - [ ] Document slip recovery behavior

### Phase 5: ros2_control Migration (Optional)

**Tasks**:

1. **Create Hardware Interface**:
   - [ ] Create C++ or Python hardware interface plugin
   - [ ] Implement `read()` method (encoder reading)
   - [ ] Implement `write()` method (motor commands)
   - [ ] Add to URDF `<ros2_control>` tag

2. **Configure diff_drive_controller**:
   - [ ] Create `diff_drive_controller.yaml`
   - [ ] Set wheel parameters
   - [ ] Configure velocity limits
   - [ ] Enable odometry publishing

3. **Launch and Test**:
   - [ ] Create ros2_control launch file
   - [ ] Test with Nav2
   - [ ] Validate against previous implementation
   - [ ] Deprecate old cmd_vel_bridge

### Implementation Summary

| Phase | Dependencies | Deliverables |
|-------|--------------|--------------|
| 1. Hardware | Parts arrival | Encoders installed and wired |
| 2. Software | Phase 1 complete | Encoder odometry node working |
| 3. Calibration | Phase 2 complete | Calibrated and validated |
| 4. Sensor Fusion | Phase 3 complete, IMU hardware | Fused odometry working |
| 5. ros2_control | Phase 4 complete | Professional integration |

**Phases 1-3** are core functionality for accurate odometry
**Phases 4-5** are optional enhancements

### Success Criteria

**Phase 1-3 (Core Functionality)**:
- [ ] Encoders reliably detect wheel rotation (forward/backward)
- [ ] Odometry accuracy <1% error over 5m straight line
- [ ] UMBmark test <50mm position error, <5° heading error
- [ ] System publishes odometry at 50 Hz consistently
- [ ] Nav2 successfully uses encoder odometry for navigation

**Phase 4 (Sensor Fusion)**:
- [ ] IMU data successfully fuses with encoder odometry
- [ ] Slip events detected (innovation increases)
- [ ] Fused odometry shows improvement over encoder-only

**Phase 5 (ros2_control)**:
- [ ] Hardware interface successfully controls motors
- [ ] diff_drive_controller publishes odometry
- [ ] Nav2 navigates with ros2_control stack

---

## References

### Official ROS2 Documentation

1. [ROS2 Control: Wheeled Mobile Robot Kinematics (Rolling Jan 2026)](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html)
2. [diff_drive_controller Documentation (Humble Jan 2026)](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
3. [ROS2 Control: Example 7 - Full Tutorial with 6DOF Robot](https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html)
4. [Writing a Hardware Component - ROS2 Control](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
5. [nav_msgs/Odometry Message Documentation](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)

### GitHub Repositories

6. [eden-desta/ros2_differential_drive - Wheel odometry for differential drive robots](https://github.com/eden-desta/ros2_differential_drive)
7. [masum919/ros2_control_custom_hardware_interface - Arduino encoder integration](https://github.com/masum919/ros2_control_custom_hardware_interface)
8. [PaulStoffregen/Encoder - Arduino quadrature encoder library](https://github.com/PaulStoffregen/Encoder)
9. [nstansby/rpi-rotary-encoder-python - Raspberry Pi rotary encoder library](https://github.com/nstansby/rpi-rotary-encoder-python)

### Encoder Technology Comparison

10. [Enrique del Sol: Magnetic Encoders VS Optical Encoders](https://enriquedelsol.com/2018/03/14/magnetic-encoders-vs-optical-encoders/)
11. [Progressive Automations: Optical Encoders VS Hall Effect Sensors](https://www.progressiveautomations.com/blogs/news/comparing-technologies-optical-encoders-vs-hall-effect-sensors)
12. [Same Sky: Capacitive, Magnetic, and Optical Encoders Comparison](https://www.sameskydevices.com/blog/capacitive-magnetic-and-optical-encoders-comparing-the-technologies)
13. [TT Electronics: Optical vs Magnetic Encoders](https://www.ttelectronics.com/blog/optical-encoders-vs-magnetic-encoders-why-flexsense-is-the-ideal-solution-for-encoder-applications/)

### Encoder Resolution and Calculations

14. [US Digital: Understanding Encoder Resolution and Its 3 Forms](https://www.usdigital.com/blog/understanding-encoder-resolution-and-its-3-forms/)
15. [Precision Microdrives: Encoder Resolution - PPR and CPR](https://www.precisionmicrodrives.com/encoder-resolution-ppr-and-cpr)
16. [Same Sky: What's the Difference Between PPR, CPR, and LPR?](https://www.sameskydevices.com/blog/what-is-encoder-ppr-cpr-and-lpr)

### Raspberry Pi GPIO and Interrupts

17. [The Pi Hut: How to use a Rotary Encoder with the Raspberry Pi](https://thepihut.com/blogs/raspberry-pi-tutorials/how-to-use-a-rotary-encoder-with-the-raspberry-pi)
18. [RasPi.TV: How to use interrupts with Python on the Raspberry Pi - Part 3](https://raspi.tv/2013/how-to-use-interrupts-with-python-on-the-raspberry-pi-and-rpi-gpio-part-3)
19. [Robotics Back-End: Raspberry Pi GPIO Interrupts Tutorial](https://roboticsbackend.com/raspberry-pi-gpio-interrupts-tutorial/)

### Hardware Products

20. [Pololu: Magnetic Encoder Pair Kit for Micro Metal Gearmotors, 12 CPR](https://www.pololu.com/product/3081)
21. [Pololu: Magnetic Encoder with Side-Entry Connector](https://www.pololu.com/product/4761)
22. [DFRobot: Micro DC Motor with Encoder-SJ02 (FIT0458)](https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ02_SKU__FIT0458)

### Sensor Fusion and robot_localization

23. [Automatic Addison: Sensor Fusion Using the Robot Localization Package - ROS 2](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)
24. [Rover Robotics: Fusing IMU + Encoders data using ROS Robot Localization](https://roverrobotics.com/blogs/guides/fusing-imu-encoders-with-ros-robot-localization)
25. [Nav2: Smoothing Odometry using Robot Localization](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
26. [RobotISim: ROS2 Sensor Fusion - Unlock Flawless Robot Navigation](https://robotisim.com/ros2-sensor-fusion-ekf-amcl/)

### Odometry Calibration and Testing

27. [Automatic Addison: Calculating Wheel Odometry for a Differential Drive Robot](https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/)
28. [O'Reilly: Wheel odometry calibration - Learning Robotics Using Python](https://www.oreilly.com/library/view/learning-robotics-using/9781783287536/ch12s02.html)
29. [MDPI: Calibration and Improvement of an Odometry Model with Dynamic Wheel Integration](https://www.mdpi.com/1424-8220/21/2/337)
30. [CMU: Accurate Odometry and Error Modelling for a Mobile Robot](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/kalman/chong_accurate_odometry_error.pdf)

### Error Sources and Mitigation

31. [Next Electronics: Incremental Encoders Tutorial](https://www.next.gr/tutorials/sensors-and-transducers/incremental-encoders-tutorial)
32. [Dynapar: Encoder Signal Overview & Troubleshooting Common Issues](https://www.dynapar.com/knowledge/encoder_issues/encoder_signal/)
33. [Bot Thoughts: AVC - Encoders, Quantization Error](https://www.bot-thoughts.com/2012/02/avc-encoders-quantization-error.html)

### ROS2 Odometry Best Practices

34. [ROS2 robot_localization: Preparing Your Data for Use](http://docs.ros.org/en/jade/api/robot_localization/html/preparing_sensor_data.html)
35. [Nav2: Setting Up Odometry - Gazebo](https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html)
36. [Articulated Robotics: ros2_control on the real robot](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-real/)

---

## Appendix A: Quick Reference - Encoder Specifications

### WayfindR Robot Specifications (from URDF)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel radius | 0.0325 m | 65mm diameter wheels |
| Wheel separation | 0.15 m | 150mm wheelbase |
| Max linear velocity | 0.5 m/s | Current configuration |
| Max angular velocity | 1.0 rad/s | Current configuration |

### Recommended Encoder: Pololu Magnetic Encoder

| Specification | Value |
|---------------|-------|
| Resolution (motor shaft) | 12 CPR × 4 (quadrature) = 48 counts/rev |
| Operating voltage | 2.7V - 18V (use 3.3V or 5V) |
| Output type | Digital (3.3V/5V compatible) |
| Magnetic disk diameter | 7.65mm OD |
| Mounting | Press-fit onto extended motor shaft |
| Cost | $8.95 per encoder |

### Output Resolution by Gearbox Ratio

| Gearbox | Output CPR | Linear Resolution (65mm wheel) | Recommended For |
|---------|-----------|-------------------------------|-----------------|
| 30:1 | 1,440 | 0.14 mm/count | High speed robots |
| 50:1 | 2,400 | 0.085 mm/count | Balanced performance |
| 75:1 | 3,600 | 0.057 mm/count | **WayfindR (recommended)** |
| 100:1 | 4,800 | 0.042 mm/count | Precision navigation |
| 150:1 | 7,200 | 0.028 mm/count | Very slow, high torque |

### GPIO Pin Assignment Quick Reference

| Encoder | Signal | GPIO | Physical Pin |
|---------|--------|------|--------------|
| Left | VCC | 3.3V | 1 |
| Left | GND | Ground | 6 |
| Left | A | GPIO 4 | 7 |
| Left | B | GPIO 17 | 11 |
| Right | VCC | 3.3V | 17 |
| Right | GND | Ground | 9 |
| Right | A | GPIO 27 | 13 |
| Right | B | GPIO 22 | 15 |

---

## Appendix B: Code Templates

### Template 1: Basic Encoder Reader (Python)

```python
#!/usr/bin/env python3
"""
Simple encoder reader for WayfindR robot.
Reads quadrature encoders connected to Raspberry Pi GPIO.
"""

import RPi.GPIO as GPIO
import threading
import time

class EncoderReader:
    """Quadrature encoder reader with interrupt-based counting"""

    def __init__(self, pin_a, pin_b, counts_per_rev):
        """
        Initialize encoder reader.

        Args:
            pin_a: GPIO pin number for channel A
            pin_b: GPIO pin number for channel B
            counts_per_rev: Total counts per output shaft revolution
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.counts_per_rev = counts_per_rev

        # State
        self.count = 0
        self.last_count = 0
        self.last_a = 0
        self.lock = threading.Lock()

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Add interrupt on channel A (both edges for 4x resolution)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH,
                              callback=self._callback,
                              bouncetime=1)  # 1ms debounce

    def _callback(self, channel):
        """Interrupt callback for quadrature decoding"""
        with self.lock:
            a = GPIO.input(self.pin_a)
            b = GPIO.input(self.pin_b)

            if a != self.last_a:
                if a != b:  # Forward
                    self.count += 1
                else:       # Backward
                    self.count -= 1

            self.last_a = a

    def get_count(self):
        """Get current encoder count (thread-safe)"""
        with self.lock:
            return self.count

    def get_delta(self):
        """Get count change since last call"""
        with self.lock:
            delta = self.count - self.last_count
            self.last_count = self.count
            return delta

    def reset(self):
        """Reset count to zero"""
        with self.lock:
            self.count = 0
            self.last_count = 0

    def cleanup(self):
        """Clean up GPIO resources"""
        GPIO.cleanup([self.pin_a, self.pin_b])


# Usage example
if __name__ == '__main__':
    # Initialize encoders for WayfindR (75:1 gearbox)
    left_encoder = EncoderReader(pin_a=4, pin_b=17, counts_per_rev=3600)
    right_encoder = EncoderReader(pin_a=27, pin_b=22, counts_per_rev=3600)

    print("Reading encoders. Press Ctrl+C to exit.")

    try:
        while True:
            left = left_encoder.get_count()
            right = right_encoder.get_count()
            print(f"Left: {left:6d}  Right: {right:6d}", end='\r')
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")
        left_encoder.cleanup()
        right_encoder.cleanup()
```

### Template 2: ROS2 Encoder Odometry Node (Python)

```python
#!/usr/bin/env python3
"""
ROS2 Encoder Odometry Node for WayfindR Robot
Publishes odometry based on wheel encoder feedback.
"""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from encoder_reader import EncoderReader  # Import from template above


class EncoderOdometryNode(Node):
    """Publish odometry from wheel encoders"""

    def __init__(self):
        super().__init__('encoder_odometry')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.15)  # meters
        self.declare_parameter('wheel_radius', 0.0325)  # meters
        self.declare_parameter('counts_per_rev', 3600)  # encoder resolution
        self.declare_parameter('odom_frequency', 50)  # Hz
        self.declare_parameter('left_gpio_a', 4)
        self.declare_parameter('left_gpio_b', 17)
        self.declare_parameter('right_gpio_a', 27)
        self.declare_parameter('right_gpio_b', 22)

        # Load parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        counts_per_rev = self.get_parameter('counts_per_rev').value
        odom_freq = self.get_parameter('odom_frequency').value

        # Calculate conversion factor
        wheel_circumference = 2 * math.pi * self.wheel_radius
        self.counts_per_meter = counts_per_rev / wheel_circumference

        # Initialize encoders
        self.left_encoder = EncoderReader(
            self.get_parameter('left_gpio_a').value,
            self.get_parameter('left_gpio_b').value,
            counts_per_rev
        )
        self.right_encoder = EncoderReader(
            self.get_parameter('right_gpio_a').value,
            self.get_parameter('right_gpio_b').value,
            counts_per_rev
        )

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(1.0 / odom_freq, self.update_odometry)

        self.get_logger().info('Encoder odometry node initialized')

    def update_odometry(self):
        """Calculate and publish odometry"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Get encoder deltas
        left_counts = self.left_encoder.get_delta()
        right_counts = self.right_encoder.get_delta()

        # Convert to meters
        dist_left = left_counts / self.counts_per_meter
        dist_right = right_counts / self.counts_per_meter

        # Differential drive kinematics
        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheelbase

        # Update pose (arc integration)
        if abs(delta_theta) < 1e-6:  # Straight line
            self.x += dist_center * math.cos(self.theta)
            self.y += dist_center * math.sin(self.theta)
        else:  # Curved path
            radius = dist_center / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities
        v_linear = dist_center / dt if dt > 0 else 0.0
        v_angular = delta_theta / dt if dt > 0 else 0.0

        # Publish odometry
        self.publish_odometry(v_linear, v_angular)
        self.publish_tf()

    def publish_odometry(self, v_linear, v_angular):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        quat = self.yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Velocity
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular

        # Covariance (placeholder - tune empirically)
        odom.pose.covariance[0] = 0.001  # x
        odom.pose.covariance[7] = 0.001  # y
        odom.pose.covariance[35] = 0.01  # yaw

        odom.twist.covariance[0] = 0.001  # vx
        odom.twist.covariance[35] = 0.01  # vyaw

        self.odom_pub.publish(odom)

    def publish_tf(self):
        """Broadcast TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        quat = self.yaw_to_quaternion(self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down encoder odometry node')
        self.left_encoder.cleanup()
        self.right_encoder.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Appendix C: Troubleshooting Guide

### Problem: No encoder counts detected

**Symptoms**:
- Count remains at 0 when wheels rotate
- Or counts drift randomly with no wheel motion

**Possible Causes**:
1. Incorrect wiring
2. Encoder not powered
3. GPIO pins not configured
4. Wrong GPIO pin numbers in code
5. Encoder defective

**Solutions**:
- Verify wiring with multimeter (VCC = 3.3V, GND = 0V)
- Check GPIO pin numbers match code
- Test with oscilloscope or logic analyzer
- Swap encoders to isolate defect

---

### Problem: Counts increase regardless of direction

**Symptoms**:
- Counts always increase (or always decrease)
- No direction detection

**Possible Causes**:
1. Only one encoder channel connected
2. Quadrature decoding not implemented
3. Channels A and B swapped

**Solutions**:
- Verify both A and B channels connected
- Check quadrature decoding logic
- Swap A and B connections if needed

---

### Problem: Erratic encoder counts (EMI)

**Symptoms**:
- Counts jump erratically
- Counts change when motor is stationary but powered
- Velocity readings unstable

**Possible Causes**:
1. Electromagnetic interference from motor
2. Poor shielding
3. Encoder wires too close to motor power wires

**Solutions**:
- Use shielded or twisted-pair cables
- Route encoder wires away from motor wires
- Add 0.1μF capacitor across motor terminals
- Use shorter cable runs

---

### Problem: Odometry drifts significantly

**Symptoms**:
- Robot reports incorrect position over time
- Straight-line test shows >2% error

**Possible Causes**:
1. Incorrect calibration (counts_per_meter)
2. Wheel diameter measurement error
3. Wheel slip
4. Different left/right wheel diameters

**Solutions**:
- Re-measure wheel diameter (loaded)
- Run calibration procedure (5m straight test)
- Check for wheel slip (smooth floors)
- Calibrate left and right wheels separately

---

### Problem: Robot turns in arc instead of straight line

**Symptoms**:
- Commanded straight motion results in curved path
- Consistent drift direction (always left or right)

**Possible Causes**:
1. Wheel diameter mismatch
2. Motor power imbalance
3. Different encoder CPR (manufacturing tolerance)

**Solutions**:
- Calibrate counts_per_meter separately for each wheel
- Check motor power (equal voltage to both motors?)
- Replace wheels as matched pair
- Adjust counts_per_meter by ~0.5-2% for affected wheel

---

### Problem: Odometry jumps or jitters

**Symptoms**:
- Reported position makes sudden jumps
- Velocity readings noisy

**Possible Causes**:
1. Quantization error at low speeds
2. Missed encoder counts (aliasing)
3. Electrical noise

**Solutions**:
- Filter velocity estimates (exponential moving average)
- Increase encoder resolution (higher gear ratio)
- Check for EMI (see above)
- Reduce maximum velocity if count rate too high

---

## Document Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2026-01-11 | Research Report | Initial comprehensive research document |

---

**End of Document**
