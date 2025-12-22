# External Controller Integration Guide

This guide explains how to integrate external control systems (flight controllers, custom PID loops, etc.) with the WayfindR ESP32 platform.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    ESP32 Dual-Core System                       │
├─────────────────────────────┬───────────────────────────────────┤
│         CORE 0              │           CORE 1                  │
│    (Control Loop)           │        (Web Server)               │
│                             │                                   │
│  ┌───────────────────────┐  │    ┌───────────────────────┐      │
│  │  ControllerInterface  │  │    │   AsyncWebServer      │      │
│  │  ┌─────────────────┐  │◄─┼────│   - REST API          │      │
│  │  │ Your Controller │  │  │    │   - WebSocket         │      │
│  │  └─────────────────┘  │  │    │   - Dashboard         │      │
│  └───────────────────────┘  │    └───────────────────────┘      │
│            │                │                                   │
│            ▼                │                                   │
│  ┌───────────────────────┐  │                                   │
│  │   Output Channels     │  │                                   │
│  │   (PWM/Servo/ESC)     │  │                                   │
│  └───────────────────────┘  │                                   │
└─────────────────────────────┴───────────────────────────────────┘
```

**Key Insight**: Your controller runs on Core 0 with guaranteed timing, while WiFi/web runs on Core 1 without affecting control loop jitter.

## Quick Start: 3 Steps to Integration

### Step 1: Implement ControllerInterface

Create a class that implements the abstract interface in `controller_interface.h`:

```cpp
#include "controller_interface.h"

class MyController : public ControllerInterface {
public:
    void init() override {
        // Initialize your sensors, PIDs, etc.
    }

    void update(const ControllerInput& input, float dt) override {
        // Called every loop iteration (e.g., 500Hz)
        // input.throttle, input.roll, input.pitch, input.yaw: -1.0 to 1.0
        // input.armed: whether motors should spin
        // input.failsafe: true if no commands received recently
        // dt: time since last update in seconds

        // Your control logic here...
        // Store results for getOutputs()
    }

    void getOutputs(ControllerOutput& output) override {
        // Fill in motor/servo outputs
        output.num_motors = 4;
        output.num_servos = 0;
        output.motors[0] = _motor1;  // ESC: 1000-2000, PWM: -1000 to 1000
        output.motors[1] = _motor2;
        output.motors[2] = _motor3;
        output.motors[3] = _motor4;
    }

    const char* getName() override { return "MyController"; }
    uint16_t getLoopFrequency() override { return 500; }  // Hz
};
```

### Step 2: Register Your Controller

In `main.cpp` before `startControlLoop()`:

```cpp
#include "controller_interface.h"
#include "my_controller.h"

MyController myController;

void setup() {
    // ... WiFi, outputs initialization ...

    // Register your controller
    setController(&myController);

    // Start control loop (uses your controller)
    startControlLoop();
}
```

### Step 3: Configure Output Channels

In `config.h`, set up channels to match your hardware:

```cpp
// For ESCs (flight controller)
#define CH0_TYPE CHANNEL_ESC
#define CH0_PWM_PIN 25

// For H-bridge motors (ground vehicles)
#define CH0_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH0_PWM_PIN 25
#define CH0_DIR1_PIN 26
#define CH0_DIR2_PIN 27
```

## Interface Reference

### ControllerInput (what you receive)

```cpp
struct ControllerInput {
    float throttle;      // -1.0 to 1.0 (or 0 to 1.0 for aircraft)
    float roll;          // -1.0 to 1.0
    float pitch;         // -1.0 to 1.0
    float yaw;           // -1.0 to 1.0
    float aux[4];        // Auxiliary channels (switches, dials)
    bool armed;          // True if motors should be active
    bool failsafe;       // True if no commands received (watchdog)
    unsigned long timestamp_ms;
};
```

### ControllerOutput (what you produce)

```cpp
struct ControllerOutput {
    int16_t motors[8];   // Motor outputs
    int16_t servos[8];   // Servo outputs
    uint8_t num_motors;  // How many motors are valid
    uint8_t num_servos;  // How many servos are valid
};
```

**Output value ranges**:
- ESC: 1000-2000 (microseconds, standard ESC protocol)
- PWM Motor: -1000 to 1000 (negative = reverse, if H-bridge)
- Servo: 0-180 (degrees) or 1000-2000 (microseconds)

### ControllerTelemetry (optional)

```cpp
struct ControllerTelemetry {
    float roll, pitch, yaw;  // Current attitude (degrees)
    float altitude;          // Altitude (meters)
    float battery_voltage;   // Battery voltage
    bool armed;
    bool in_flight;
    uint8_t flight_mode;
};
```

## Common Integration Patterns

### Pattern 1: Simple PID Controller

For ground vehicles or simple stabilization:

```cpp
class SimplePIDController : public ControllerInterface {
private:
    float _kp = 1.0f, _ki = 0.1f, _kd = 0.05f;
    float _integral = 0.0f, _lastError = 0.0f;
    int16_t _outputs[4] = {0};

public:
    void update(const ControllerInput& input, float dt) override {
        if (!input.armed || input.failsafe) {
            _outputs[0] = _outputs[1] = 0;
            _integral = 0;
            return;
        }

        // Simple differential drive
        float left = input.throttle + input.yaw;
        float right = input.throttle - input.yaw;

        _outputs[0] = (int16_t)(left * 1000);
        _outputs[1] = (int16_t)(right * 1000);
    }

    void getOutputs(ControllerOutput& output) override {
        output.num_motors = 2;
        output.motors[0] = _outputs[0];
        output.motors[1] = _outputs[1];
    }

    uint16_t getLoopFrequency() override { return 100; }  // 100Hz is fine
};
```

### Pattern 2: IMU-Based Stabilization

For flight controllers and self-balancing robots:

```cpp
class StabilizedController : public ControllerInterface {
private:
    IMUInterface* _imu;
    AttitudeEstimator _attitude;
    PIDController _rollPID{2.0f, 0.02f, 0.5f};
    PIDController _pitchPID{2.0f, 0.02f, 0.5f};

public:
    StabilizedController(IMUInterface* imu) : _imu(imu) {}

    void update(const ControllerInput& input, float dt) override {
        // Read and filter IMU
        IMUData imuData;
        _imu->read(imuData);
        _attitude.update(imuData, dt);

        // Get current attitude
        float currentRoll, currentPitch, currentYaw;
        _attitude.getEuler(currentRoll, currentPitch, currentYaw);

        // Calculate setpoints from stick input
        float rollSetpoint = input.roll * 30.0f;   // Max 30 degrees
        float pitchSetpoint = input.pitch * 30.0f;

        // Run PIDs
        float rollCmd = _rollPID.update(rollSetpoint, currentRoll, dt);
        float pitchCmd = _pitchPID.update(pitchSetpoint, currentPitch, dt);

        // Mix to motors (quad X configuration)
        mixMotors(input.throttle, rollCmd, pitchCmd, input.yaw);
    }

    uint16_t getLoopFrequency() override { return 500; }  // 500Hz for flight
};
```

### Pattern 3: Wrapping Existing Code

If you have existing flight controller code (like dRehmFlight):

```cpp
// Adapter pattern: wrap existing code
class DRehmFlightAdapter : public ControllerInterface {
public:
    void init() override {
        // Call original initialization
        setupMPU6050();
        setupMadgwick();
        setupPID();
    }

    void update(const ControllerInput& input, float dt) override {
        // Map our input to their variables
        channel_1_pwm = 1500 + (int)(input.roll * 500);
        channel_2_pwm = 1500 + (int)(input.pitch * 500);
        channel_3_pwm = 1000 + (int)(input.throttle * 1000);
        channel_4_pwm = 1500 + (int)(input.yaw * 500);

        // Call original loop
        getIMUdata();
        Madgwick6DOF(dt);
        controlANGLE();

        // Outputs are in m1_command_PWM, m2_command_PWM, etc.
    }

    void getOutputs(ControllerOutput& output) override {
        output.num_motors = 4;
        output.motors[0] = m1_command_PWM;
        output.motors[1] = m2_command_PWM;
        output.motors[2] = m3_command_PWM;
        output.motors[3] = m4_command_PWM;
    }

    uint16_t getLoopFrequency() override { return 2000; }  // Original 2kHz
};
```

## Loop Frequency Recommendations

| Application | Recommended Hz | Notes |
|-------------|----------------|-------|
| Ground vehicle (car, boat) | 50-100 | More is unnecessary |
| Slow aircraft (plane, blimp) | 100-200 | Moderate dynamics |
| Self-balancing robot | 200-400 | Fast response needed |
| Quadcopter (hover) | 400-500 | Minimum for stability |
| Quadcopter (sport) | 500-1000 | Better for maneuvers |
| Quadcopter (racing) | 1000-2000 | Maximum responsiveness |

**ESP32 at 240MHz can comfortably handle up to 1000Hz** with a full flight controller stack.

## Timing Budget

At 500Hz (2000µs per loop), here's a typical budget:

```
┌────────────────────────────────────────┐
│ IMU Read (I2C 400kHz)      ~80µs       │
│ Madgwick Filter            ~100µs      │
│ PID Controllers x3         ~30µs       │
│ Motor Mixing               ~10µs       │
│ Output to PWM              ~10µs       │
├────────────────────────────────────────┤
│ Total                      ~230µs      │
│ Headroom                   ~1770µs     │
│ Utilization                ~12%        │
└────────────────────────────────────────┘
```

## Debugging Tips

### 1. Monitor Loop Timing

```cpp
// In your update() method:
static unsigned long lastPrint = 0;
if (millis() - lastPrint > 1000) {
    Serial.printf("Loop time: %d µs\n", getLoopTimeUs());
    lastPrint = millis();
}
```

### 2. Check for Jitter

Add timing analysis to ensure consistent loop execution:

```cpp
void update(const ControllerInput& input, float dt) override {
    static float maxDt = 0, minDt = 999;
    if (dt > maxDt) maxDt = dt;
    if (dt < minDt) minDt = dt;

    // Periodic report
    static int count = 0;
    if (++count >= 500) {
        Serial.printf("dt range: %.3f - %.3f ms\n", minDt*1000, maxDt*1000);
        count = 0;
        maxDt = 0;
        minDt = 999;
    }
}
```

### 3. Test Without Motors First

Always develop with `setArmed(false)` and monitor outputs:

```cpp
void update(const ControllerInput& input, float dt) override {
    // Calculate outputs...

    #ifdef DEBUG_OUTPUTS
    static int printCount = 0;
    if (++printCount >= 500) {
        Serial.printf("M0:%d M1:%d M2:%d M3:%d\n",
            _outputs[0], _outputs[1], _outputs[2], _outputs[3]);
        printCount = 0;
    }
    #endif
}
```

## Safety Checklist

Before flying or driving:

- [ ] Test all outputs with motors disconnected
- [ ] Verify arming/disarming works correctly
- [ ] Confirm failsafe triggers on WiFi disconnect
- [ ] Check motor direction matches expected
- [ ] Verify PID response (bench test with props off)
- [ ] Test at low throttle first
- [ ] Have emergency stop ready (close WiFi connection)

## File Structure for Custom Controller

```
esp32_api/
├── include/
│   ├── controller_interface.h   # Base interface (don't modify)
│   ├── my_imu.h                 # Your IMU driver
│   └── my_controller.h          # Your controller header
├── src/
│   ├── controller_interface.cpp # Base implementation
│   ├── my_imu.cpp               # Your IMU driver
│   ├── my_controller.cpp        # Your controller implementation
│   └── main.cpp                 # Entry point (register controller)
└── examples/
    ├── flight_controller_wrapper.h/.cpp
    ├── mpu6050_imu.h
    └── example_main.cpp
```

## Example: Complete Flight Controller

See the `examples/` directory for a complete, working flight controller implementation:

- `flight_controller_wrapper.h/.cpp` - Full flight controller with Madgwick, PID, and quad mixing
- `mpu6050_imu.h` - MPU6050 I2C driver
- `example_main.cpp` - Complete main.cpp showing how to use it

To use:
1. Copy example files to `include/` and `src/`
2. Set `VEHICLE_TYPE_QUAD` in `config.h`
3. Connect MPU6050 to I2C pins
4. Build and upload
5. Access web dashboard for control
