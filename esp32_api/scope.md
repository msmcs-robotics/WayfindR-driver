# ESP32 API - Project Scope

## Overview

The **WayfindR ESP32 API** is a dual-core robot control system designed to provide WiFi-based control for various vehicle types including ground vehicles (cars, boats), aircraft (planes), and quadcopters. It leverages the ESP32's dual-core architecture to separate real-time control tasks from network operations, ensuring reliable and responsive vehicle control.

## Purpose

This folder contains a complete firmware implementation for ESP32 microcontrollers that enables:

1. **WiFi-based remote control** via REST API, WebSocket, and web dashboard
2. **Real-time motor control** with precise timing on a dedicated CPU core
3. **Multi-vehicle support** with configurable mixing for different vehicle types
4. **Flight controller integration** framework for custom control algorithms
5. **LLM-friendly API** with natural language command support for autonomous navigation

The system is designed to work with the WayfindR project's higher-level navigation and AI components, providing the low-level hardware interface and motor control layer.

## Why ESP32 vs Raspberry Pi?

The ESP32 complements rather than replaces the Raspberry Pi in the WayfindR architecture:

| Aspect | ESP32 | Raspberry Pi |
|--------|-------|--------------|
| **Role** | Real-time motor control, hardware PWM | High-level AI, vision, planning |
| **Architecture** | Dual-core 240MHz microcontroller | Quad-core 1.5GHz+ application processor |
| **Real-time** | Guaranteed timing, no OS jitter | Linux OS, unpredictable scheduling |
| **PWM Channels** | 16 hardware PWM channels | Software PWM only (unreliable) |
| **Power** | ~500mW active | 2-5W active |
| **Cost** | $5-10 | $35-75 |
| **WiFi** | Built-in, low-power | Built-in or USB dongle |
| **GPIO** | 34 pins, 5V tolerant options | 40 pins, 3.3V only |
| **Boot Time** | <1 second | 20-40 seconds |
| **Use Case** | Motor drivers, ESCs, servos | Computer vision, navigation, LLM |

### Division of Responsibilities

```
┌─────────────────────────────────────────────────────────┐
│                  Raspberry Pi                           │
│  - Computer vision (camera processing)                  │
│  - LLM integration (Claude, GPT)                        │
│  - Path planning and navigation                         │
│  - Object detection and tracking                        │
│  - High-level decision making                           │
│  - Maps and localization                                │
└────────────────────┬────────────────────────────────────┘
                     │ HTTP/WebSocket
                     │ Commands: /api/move, /api/command
                     │
┌────────────────────▼────────────────────────────────────┐
│                  ESP32                                   │
│  - Real-time motor control (100-1000Hz)                 │
│  - PWM generation for motors/servos/ESCs                │
│  - H-bridge control (direction + speed)                 │
│  - Differential drive mixing                            │
│  - Watchdog safety (auto-stop)                          │
│  - Optional: IMU reading and PID control                │
└─────────────────────────────────────────────────────────┘
                     │
                     ▼
          Motors, Servos, ESCs, Sensors
```

### Key Advantages

1. **Reliability**: ESP32 handles time-critical motor control without Linux OS interruptions
2. **Failsafe**: ESP32 can stop motors immediately if Raspberry Pi crashes or loses connection
3. **Efficiency**: Raspberry Pi focuses on compute-intensive tasks (vision, AI) while ESP32 handles hardware
4. **Modularity**: ESP32 can be swapped or upgraded independently of the Pi
5. **Power Management**: ESP32 can keep motors running even if Pi enters power-saving mode

## Architecture

### Dual-Core Design

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 (240MHz x 2)                       │
├─────────────────────────────┬───────────────────────────────┤
│         CORE 0              │           CORE 1              │
│     Control Loop            │        Web Server             │
│                             │                               │
│  ┌─────────────────────┐    │    ┌─────────────────────┐    │
│  │   100Hz-1000Hz      │    │    │   AsyncWebServer    │    │
│  │   - Read inputs     │    │    │   - REST API        │    │
│  │   - Run controller  │◄───┼────│   - WebSocket       │    │
│  │   - Motor mixing    │    │    │   - Dashboard       │    │
│  │   - Output PWM      │    │    └─────────────────────┘    │
│  │   - Watchdog        │    │                               │
│  └─────────────────────┘    │    ┌─────────────────────┐    │
│            │                │    │   WiFi Manager      │    │
│            ▼                │    │   - Station mode    │    │
│  ┌─────────────────────┐    │    │   - AP fallback     │    │
│  │   Output Channels   │    │    └─────────────────────┘    │
│  │   - PWM Motors      │    │                               │
│  │   - H-Bridge        │    │                               │
│  │   - Servos          │    │                               │
│  │   - ESCs            │    │                               │
│  └─────────────────────┘    │                               │
└─────────────────────────────┴───────────────────────────────┘
```

**Core 0** runs the control loop at a fixed frequency (configurable 100-1000Hz) with the highest priority. This ensures consistent motor timing without interference from WiFi or web requests.

**Core 1** handles all network operations (WiFi, HTTP, WebSocket) and the web dashboard. Network jitter and latency do not affect motor control timing.

Communication between cores uses mutex-protected shared memory for thread-safe data exchange.

## API Endpoints and Functionality

### REST API

All endpoints support both GET and POST methods and return JSON responses.

#### Movement Control

- **`/api/move?throttle=X&steering=Y`** - Primary control endpoint
  - `throttle`: -1.0 to 1.0 (backward to forward)
  - `steering`: -1.0 to 1.0 (left to right)
  - `pitch`: -1.0 to 1.0 (aircraft nose up/down)
  - `roll`: -1.0 to 1.0 (aircraft bank left/right)
  - Short form: `?t=X&s=Y`

- **`/api/stop`** - Emergency stop (all motors to zero)

- **`/api/forward?speed=X`** - Move forward at speed (0-255)

- **`/api/backward?speed=X`** - Move backward at speed (0-255)

- **`/api/rotate?direction=left&speed=X`** - Rotate in place

#### Natural Language Commands (LLM Integration)

- **`/api/command?cmd=X`** - Natural language control
  - `"go forward"`, `"move ahead"` → forward at 70% speed
  - `"go backward"`, `"reverse"`, `"go back"` → backward at 70% speed
  - `"turn left"` → forward-left turn
  - `"turn right"` → forward-right turn
  - `"rotate left"`, `"spin left"` → pivot left in place
  - `"rotate right"`, `"spin right"` → pivot right in place
  - `"stop"`, `"halt"` → emergency stop
  - `"arm"` → enable motors
  - `"disarm"` → disable motors

#### System Control

- **`POST /api/arm`** - Enable motors (if ARM_REQUIRED is true)

- **`POST /api/disarm`** - Disable motors and stop

- **`/api/status`** - Get current system state
  ```json
  {
    "throttle": 0.5,
    "steering": 0.2,
    "pitch": 0.0,
    "roll": 0.0,
    "armed": true,
    "vehicle": "car",
    "uptime": 3600,
    "heap": 200000,
    "channels": [
      {"id": 0, "value": 500},
      {"id": 1, "value": 500},
      {"id": 2, "value": 300},
      {"id": 3, "value": 300}
    ]
  }
  ```

#### Channel Control (Advanced)

- **`/api/channel?ch=X&value=Y`** - Direct channel control
  - `ch`: 0-7 (channel number)
  - `value`: -1000 to 1000 (motors), 1000-2000 (ESCs), 0-180 (servos)

- **`POST /api/channel/enable?ch=X&enable=Y`** - Enable/disable specific channel

- **`/api/channels`** - Get all channel configurations and values

### WebSocket API

Connect to `ws://<ip>/ws` for real-time bidirectional control.

#### Client → Server (Commands)

Send JSON commands for immediate motor control:

```javascript
// Ground vehicle control
ws.send(JSON.stringify({
    throttle: 0.5,    // -1.0 to 1.0
    steering: 0.2     // -1.0 to 1.0
}));

// Aircraft control
ws.send(JSON.stringify({
    throttle: 0.7,    // 0.0 to 1.0 for aircraft
    pitch: 0.1,       // -1.0 to 1.0
    roll: -0.2,       // -1.0 to 1.0
    steering: 0.0     // yaw: -1.0 to 1.0
}));

// Emergency stop
ws.send(JSON.stringify({
    throttle: 0,
    steering: 0
}));
```

#### Server → Client (Telemetry)

Receive real-time telemetry at 10Hz (configurable):

```javascript
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    // data.ch0, ch1, ch2, ch3 = channel values
    // data.vehicle = "car", "boat", "plane", "quadcopter"
    // data.armed = true/false
    // data.uptime = seconds since boot
};
```

### Web Dashboard

Served at `http://<ip>/` - Interactive control panel with:

- **D-Pad controls** - Touch/click directional buttons
- **Speed slider** - Adjust movement speed (10-100%)
- **Keyboard control**:
  - `W/↑` - Forward
  - `S/↓` - Backward
  - `A/←` - Turn left
  - `D/→` - Turn right
  - `Q` - Rotate left in place
  - `E` - Rotate right in place
  - `Space` - Emergency stop
- **Real-time telemetry** - Channel values, vehicle type, uptime
- **Connection status** - Visual indicator with auto-reconnect

## Hardware Integration

### Supported Output Types

| Type | Description | Signal | Use Case |
|------|-------------|--------|----------|
| `CHANNEL_PWM_MOTOR` | Simple PWM output | 0-255 | Single-direction DC motor with external driver |
| `CHANNEL_PWM_MOTOR_DIR` | PWM + direction pins | PWM + 2 GPIO | H-bridge motor driver (L298N, TB6612) |
| `CHANNEL_SERVO` | Standard servo | 1000-2000µs PWM | RC servos (steering, control surfaces) |
| `CHANNEL_ESC` | Electronic Speed Controller | 1000-2000µs PWM | Brushless motors (BLDC ESCs) |
| `CHANNEL_BRUSHLESS` | Alias for ESC | 1000-2000µs PWM | Same as ESC |

### Vehicle Configurations

#### 1. Car (4WD Skid Steer)

**Channels:**
- CH0: Left Front Motor (PWM + DIR)
- CH1: Left Rear Motor (PWM + DIR)
- CH2: Right Front Motor (PWM + DIR)
- CH3: Right Rear Motor (PWM + DIR)

**Mixing:** Differential drive with pivot turning
- Throttle controls forward/backward speed
- Steering creates speed differential between left/right sides
- Pivot turn when throttle near zero

**Example Wiring (L298N H-Bridge):**
```
ESP32 Pin → L298N Left Side
GPIO25 → ENA (PWM)
GPIO26 → IN1 (Direction 1)
GPIO27 → IN2 (Direction 2)

ESP32 Pin → L298N Right Side
GPIO32 → ENA (PWM)
GPIO33 → IN1
GPIO15 → IN2
```

#### 2. Boat (Differential Drive)

**Channels:**
- CH0: Left Motor (PWM + DIR)
- CH1: Right Motor (PWM + DIR)

**Mixing:** Same as car but only 2 motors

#### 3. Plane (Fixed Wing)

**Channels:**
- CH0: Throttle (ESC, 1000-2000µs)
- CH1: Aileron Servo (0-180°)
- CH2: Elevator Servo (0-180°)
- CH3: Rudder Servo (0-180°)

**Control:**
- Throttle: 0-1.0 (ESC speed)
- Roll: -1.0 to 1.0 (aileron deflection)
- Pitch: -1.0 to 1.0 (elevator deflection)
- Steering: -1.0 to 1.0 (rudder deflection)

#### 4. Quadcopter (X Configuration)

**Channels:**
- CH0: Front Left Motor (ESC, CCW rotation)
- CH1: Front Right Motor (ESC, CW rotation)
- CH2: Rear Left Motor (ESC, CW rotation)
- CH3: Rear Right Motor (ESC, CCW rotation)

**WARNING:** Built-in quad mixer is for TESTING ONLY! Real flight requires:
- IMU sensor (MPU6050, BMI088, etc.)
- Attitude estimation (Madgwick/Mahony filter)
- PID control loops
- 400-1000Hz update rate
- See "Flight Controller Integration" below

### Pin Configuration

Configured in `include/config.h`:

```cpp
// Example: Left Front Motor on Channel 0
#define CH0_ENABLED true
#define CH0_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH0_PWM_PIN 25          // PWM speed control
#define CH0_DIR1_PIN 26         // Direction pin 1
#define CH0_DIR2_PIN 27         // Direction pin 2
#define CH0_INVERTED false      // Reverse direction if true
```

### PWM Specifications

- **PWM Frequency**: 5kHz (motors), 50Hz (servos/ESCs)
- **Resolution**: 8-bit (0-255) for motors
- **Servo Range**: 1000-2000µs standard
- **ESC Protocol**: Standard PWM (1000µs = stop, 2000µs = full)

## Flight Controller Integration

The system provides a pluggable architecture for custom flight controllers.

### Controller Interface

Custom controllers implement the `ControllerInterface` abstract class:

```cpp
class ControllerInterface {
public:
    virtual void init() = 0;
    virtual void update(const ControllerInput& input, float dt) = 0;
    virtual void getOutputs(ControllerOutput& output) = 0;
    virtual void getTelemetry(ControllerTelemetry& telemetry);
    virtual uint16_t getLoopFrequency() { return 0; }  // 0 = use default
    virtual const char* getName() { return "Generic"; }
};
```

### Integration Example

```cpp
#include "controller_interface.h"

class MyFlightController : public ControllerInterface {
private:
    IMU imu;
    PIDController rollPID, pitchPID, yawPID;
    int16_t motorOutputs[4];

public:
    void init() override {
        imu.begin();
        rollPID.setGains(1.0, 0.05, 0.1);
        pitchPID.setGains(1.0, 0.05, 0.1);
        yawPID.setGains(0.5, 0.02, 0.05);
    }

    void update(const ControllerInput& input, float dt) override {
        // Read IMU
        imu.update();

        // Run PID loops
        float rollCmd = rollPID.compute(input.roll * 30, imu.roll, dt);
        float pitchCmd = pitchPID.compute(input.pitch * 30, imu.pitch, dt);
        float yawCmd = yawPID.compute(input.yaw, imu.yawRate, dt);

        // Mix to motors (quadcopter X)
        float base = input.throttle * 1000;
        motorOutputs[0] = base - pitchCmd + rollCmd - yawCmd;  // FL
        motorOutputs[1] = base - pitchCmd - rollCmd + yawCmd;  // FR
        motorOutputs[2] = base + pitchCmd + rollCmd + yawCmd;  // RL
        motorOutputs[3] = base + pitchCmd - rollCmd - yawCmd;  // RR
    }

    void getOutputs(ControllerOutput& output) override {
        output.num_motors = 4;
        for (int i = 0; i < 4; i++) {
            output.motors[i] = constrain(motorOutputs[i], 0, 1000);
        }
    }

    uint16_t getLoopFrequency() override { return 500; }  // 500Hz
    const char* getName() override { return "MyFlightController"; }
};

// In main.cpp setup():
MyFlightController fc;
setController(&fc);
startControlLoop();  // Now runs your controller at 500Hz
```

### Computational Analysis

Based on analysis of the dRehmFlight VTOL flight controller (see `docs/FLIGHT_CONTROLLER_ANALYSIS.md`):

**ESP32 at 240MHz can handle:**
- **500Hz loop**: Excellent, 66% headroom
- **1000Hz loop**: Good, comfortable margin
- **2000Hz loop**: Marginal, tight timing

**Typical per-loop operations at 500Hz (2000µs budget):**
- IMU read (I2C): ~50µs
- Madgwick filter: ~100µs
- 3x PID controllers: ~20µs
- Motor mixing: ~10µs
- PWM output (hardware): ~5µs
- **Total: ~185µs (9% utilization, 91% headroom)**

### Comparison: ESP32 vs Teensy 4.0

| Aspect | Teensy 4.0 | ESP32 Dual-Core |
|--------|------------|-----------------|
| Clock Speed | 600MHz | 240MHz per core |
| Max Loop Rate | 2000Hz comfortable | 500-1000Hz recommended |
| WiFi | External module | Built-in |
| Dual Core | No | Yes (isolation) |
| Cost | $20-25 | $5-10 |
| **Verdict** | Better for racing quads | Better for autonomous navigation |

**Key Advantage:** ESP32's dual-core architecture isolates control from WiFi, which single-core Teensy cannot do.

## Current State of Implementation

### ✅ Completed Features

**Core System:**
- [x] Dual-core architecture with Core 0/1 separation
- [x] FreeRTOS task management with priorities
- [x] Mutex-protected inter-core communication
- [x] Configurable control loop frequency (100-1000Hz)
- [x] Real-time loop timing and performance monitoring

**Hardware Control:**
- [x] 8-channel output system
- [x] PWM motor control (unidirectional)
- [x] PWM + direction motor control (H-bridge)
- [x] Servo control (0-180°, 1000-2000µs)
- [x] ESC control (1000-2000µs PWM)
- [x] Per-channel enable/disable
- [x] Per-channel inversion
- [x] Hardware PWM via ESP32 LEDC peripheral

**Network & API:**
- [x] WiFi station mode with SSID/password
- [x] WiFi AP fallback mode
- [x] AsyncWebServer (non-blocking)
- [x] REST API (all endpoints implemented)
- [x] WebSocket real-time control
- [x] CORS headers for web integration
- [x] Embedded web dashboard (fallback)
- [x] SPIFFS filesystem support for custom dashboard

**Vehicle Support:**
- [x] Car mixer (4WD skid steer)
- [x] Boat mixer (2-motor differential)
- [x] Plane mixer (throttle + 3 servos)
- [x] Quad mixer (basic X configuration - TESTING ONLY)
- [x] Differential drive with pivot turning
- [x] Exponential curve support
- [x] Configurable turn sensitivity

**Safety:**
- [x] Watchdog timeout (500ms default)
- [x] Failsafe on connection loss
- [x] WebSocket disconnect auto-stop
- [x] Arming system (optional)
- [x] Individual channel disable
- [x] Emergency stop endpoint

**Controller Framework:**
- [x] Abstract `ControllerInterface` class
- [x] Default mixer controller
- [x] Pluggable controller architecture
- [x] `ControllerInput` struct (throttle, roll, pitch, yaw, aux, armed, failsafe)
- [x] `ControllerOutput` struct (motors, servos)
- [x] `ControllerTelemetry` struct
- [x] Thread-safe input/output exchange
- [x] Controller registration system

**Documentation:**
- [x] Comprehensive README.md
- [x] Integration guide for custom controllers
- [x] Flight controller computational analysis
- [x] API documentation
- [x] Example code

### 🚧 Partially Implemented

**Flight Controller:**
- [x] Interface defined
- [x] Example wrapper code (`examples/flight_controller_wrapper.h`)
- [x] Example IMU interface
- [x] Example PID controller
- [x] Example attitude estimator
- [ ] Complete working example with real IMU (MPU6050 example header only)
- [ ] Tested flight controller implementation
- [ ] Calibration utilities

**Web Dashboard:**
- [x] Basic embedded dashboard
- [x] D-pad controls
- [x] Keyboard support
- [x] Speed control
- [x] Telemetry display
- [ ] Mobile-optimized UI
- [ ] Graphical telemetry (attitude indicator, battery, signal)
- [ ] PID tuning interface
- [ ] Channel configuration UI
- [ ] Calibration wizard

### ❌ Not Implemented / Future Features

**Advanced Features:**
- [ ] Radio receiver input (SBUS, PPM, CRSF)
- [ ] GPS integration
- [ ] Barometer/altimeter support
- [ ] Magnetometer support
- [ ] Battery voltage monitoring
- [ ] Current sensor integration
- [ ] SD card logging
- [ ] Bluetooth control
- [ ] OTA (Over-The-Air) firmware updates
- [ ] Multiple flight modes (angle, rate, horizon)
- [ ] Autonomous waypoint navigation
- [ ] Return-to-home (RTH)
- [ ] Geofencing
- [ ] Black box flight recorder

**Advanced Control:**
- [ ] Adaptive PID tuning
- [ ] Filter tuning (lowpass, notch)
- [ ] Motor thrust curve calibration
- [ ] ESC protocol support (OneShot, DShot, MultiShot)
- [ ] Dynamic notch filtering
- [ ] Feed-forward control
- [ ] Cascaded PID (inner rate loop + outer angle loop)

**User Experience:**
- [ ] Setup wizard
- [ ] Pre-configured vehicle templates
- [ ] USB configuration interface
- [ ] CLI (command-line interface)
- [ ] Blackbox viewer
- [ ] Mission planner integration

## Dependencies and Requirements

### Hardware Requirements

**Minimum:**
- ESP32 DevKit (any variant with WiFi)
- 5V power supply (500mA minimum)
- Motor drivers (L298N, TB6612, ESCs, etc.)
- Motors/servos/ESCs as needed

**Recommended:**
- ESP32 DevKitC (official)
- Dedicated 5V/2A power supply for ESP32
- Separate battery for motors (7.4V-12V for brushed, 11.1V-14.8V for brushless)
- MPU6050 or BMI088 IMU (for flight controller)
- Voltage regulator/BEC for clean 5V supply

**Supported ESP32 Variants:**
- ESP32 (original, tested)
- ESP32-S3 (configured in platformio.ini)
- ESP32-C3 (should work, untested)

### Software Requirements

**PlatformIO Libraries:**
- `espressif32` platform (ESP32 Arduino core)
- `me-no-dev/AsyncTCP@^1.1.1` - Async TCP for web server
- `me-no-dev/ESPAsyncWebServer@^1.2.3` - Async web server
- `bblanchon/ArduinoJson@^7.0.0` - JSON serialization
- `madhephaestus/ESP32Servo@^1.1.1` - Servo/ESC control

**Development Environment:**
- PlatformIO (recommended) or Arduino IDE
- USB drivers for ESP32 (CP2102/CH340 depending on board)
- Serial monitor (115200 baud)

**Optional:**
- SPIFFS upload tool (for custom web dashboard)
- IMU library (if using flight controller)

### Build and Upload

```bash
# Install PlatformIO
pip install platformio

# Build firmware
cd esp32_api
pio run

# Upload firmware to ESP32
pio run -t upload

# Upload web dashboard (optional)
pio run -t uploadfs

# Monitor serial output
pio device monitor
```

### Configuration

**Required Configuration in `include/config.h`:**

1. **WiFi Credentials:**
   ```cpp
   #define WIFI_SSID "YourNetwork"
   #define WIFI_PASSWORD "YourPassword"
   ```

2. **Vehicle Type:** (uncomment ONE)
   ```cpp
   #define VEHICLE_TYPE_CAR        // 4WD skid steer
   // #define VEHICLE_TYPE_BOAT    // 2 motors
   // #define VEHICLE_TYPE_PLANE   // 1 ESC + 3 servos
   // #define VEHICLE_TYPE_QUAD    // 4 ESCs + flight controller
   ```

3. **Channel Pins:** (configure for your wiring)
   ```cpp
   #define CH0_PWM_PIN 25
   #define CH0_DIR1_PIN 26
   #define CH0_DIR2_PIN 27
   // ... etc
   ```

**Optional Configuration:**
- `CONTROL_LOOP_FREQ` - Control loop frequency (100-1000Hz)
- `WATCHDOG_TIMEOUT_MS` - Failsafe timeout (default 500ms)
- `ARM_REQUIRED` - Require arming before motors spin
- `TELEMETRY_RATE_HZ` - WebSocket telemetry rate (default 10Hz)

### Power Requirements

**ESP32:**
- Voltage: 5V via USB or VIN, 3.3V on 3V3 pin
- Current: 80mA idle, 160mA WiFi active, 240mA peak
- **Do not power ESP32 from motor battery directly** - use a regulator/BEC

**Motors:**
- Provide adequate current capacity (5A+ per motor for medium-size DC motors)
- Use separate battery from ESP32 logic supply
- Connect grounds together (ESP32 GND to motor battery GND)

### Networking

**Default IP Assignment:**
- Station mode: DHCP from router
- AP mode: `192.168.4.1`

**Ports:**
- HTTP: 80
- WebSocket: 80 (ws:// protocol)

**Firewall:**
- No special configuration needed for local network
- For internet access, port forward 80 or use VPN/tunnel

## Integration with WayfindR Project

The ESP32 API is designed to integrate seamlessly with the WayfindR autonomous navigation system:

### Communication Flow

```
┌──────────────────────────────────────────────────────────┐
│  Raspberry Pi (WayfindR Main System)                     │
│  - Claude LLM integration                                │
│  - Computer vision                                       │
│  - Path planning                                         │
├──────────────────────────────────────────────────────────┤
│  Python navigation script sends HTTP/WebSocket commands  │
│  to ESP32:                                               │
│                                                          │
│  GET http://esp32.local/api/command?cmd=go%20forward    │
│  GET http://esp32.local/api/move?throttle=0.5&steering=0│
│  WebSocket: {"throttle": 0.5, "steering": 0.2}          │
└────────────────────┬─────────────────────────────────────┘
                     │
                     │ WiFi Network
                     │
┌────────────────────▼─────────────────────────────────────┐
│  ESP32 (Motor Control)                                   │
│  - Receives commands via REST API or WebSocket           │
│  - Converts to motor outputs (100-1000Hz)                │
│  - Sends telemetry back to Pi                            │
│  - Automatic failsafe if Pi disconnects                  │
└──────────────────────────────────────────────────────────┘
                     │
                     ▼
              Motors/Servos/ESCs
```

### Python Example (Raspberry Pi Side)

```python
import requests
import json
from websocket import create_connection

# REST API control
ESP32_IP = "192.168.1.100"

# Simple commands
requests.get(f"http://{ESP32_IP}/api/command?cmd=go forward")
requests.get(f"http://{ESP32_IP}/api/stop")

# Precise control
requests.get(f"http://{ESP32_IP}/api/move?throttle=0.5&steering=-0.3")

# WebSocket for real-time control
ws = create_connection(f"ws://{ESP32_IP}/ws")
ws.send(json.dumps({"throttle": 0.5, "steering": 0.2}))
telemetry = json.loads(ws.recv())
print(f"Motor outputs: {telemetry}")
ws.close()
```

### LLM Integration

The `/api/command` endpoint is specifically designed for LLM output:

```
User: "Move forward and turn slightly right"
LLM: GET /api/command?cmd=go%20forward
     (wait 2 seconds)
     GET /api/command?cmd=turn%20right

User: "Stop the vehicle"
LLM: GET /api/command?cmd=stop
```

The natural language parser handles variations in phrasing and maps to appropriate motor commands.

## Future Development Roadmap

### Short-term (Next 3-6 months)
1. Complete MPU6050 IMU integration with working example
2. Implement complete flight controller example (500Hz PID loops)
3. Add battery voltage monitoring
4. Improve web dashboard UI (mobile-responsive, better telemetry)
5. Add OTA firmware updates
6. Implement SBUS radio receiver input

### Medium-term (6-12 months)
1. GPS integration for autonomous navigation
2. Return-to-home functionality
3. Waypoint mission support
4. Enhanced telemetry (battery percentage, signal strength, GPS fix)
5. PID tuning interface in web dashboard
6. Black box logging to SD card
7. Support for more vehicle types (tank treads, mecanum wheels)

### Long-term (12+ months)
1. Advanced flight modes (acro, horizon, GPS hold)
2. Obstacle avoidance integration
3. Multi-robot swarm coordination
4. ROS2 integration
5. Custom ESC protocols (DShot, MultiShot)
6. Advanced filtering (dynamic notch, Kalman)
7. Machine learning-based control optimization

---

**Project Status:** Production-ready for ground vehicles and basic aircraft. Flight controller framework complete but requires user-provided IMU integration and PID tuning for autonomous quadcopter flight.

**License:** Check main WayfindR project repository for license information.

**Maintained by:** WayfindR Project Contributors

**Last Updated:** 2026-01-11
