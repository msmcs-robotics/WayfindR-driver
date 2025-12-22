# WayfindR ESP32 Robot Control API

Dual-core ESP32 control system for robots, vehicles, boats, planes, and quadcopters.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32 (240MHz x 2)                   │
├─────────────────────────────┬───────────────────────────────┤
│         CORE 0              │           CORE 1              │
│     Control Loop            │        Web Server             │
│                             │                               │
│  ┌─────────────────────┐    │    ┌─────────────────────┐    │
│  │   100Hz Loop        │    │    │   AsyncWebServer    │    │
│  │   - Read inputs     │    │    │   - REST API        │    │
│  │   - Run mixer       │◄───┼────│   - WebSocket       │    │
│  │   - Output PWM      │    │    │   - Dashboard       │    │
│  │   - Watchdog        │    │    └─────────────────────┘    │
│  └─────────────────────┘    │                               │
│            │                │    ┌─────────────────────┐    │
│            ▼                │    │   WiFi Manager      │    │
│  ┌─────────────────────┐    │    │   - Station mode    │    │
│  │   Output Channels   │    │    │   - AP fallback     │    │
│  │   - PWM Motors      │    │    └─────────────────────┘    │
│  │   - H-Bridge        │    │                               │
│  │   - Servos          │    │                               │
│  │   - ESCs            │    │                               │
│  └─────────────────────┘    │                               │
└─────────────────────────────┴───────────────────────────────┘
```

### Why Dual-Core?

- **Core 0 (Control)**: Runs at fixed 100Hz (configurable up to 400Hz) for precise motor timing
- **Core 1 (Web)**: Handles WiFi, HTTP, and WebSocket without affecting control timing
- **Thread-Safe**: Mutex-protected shared state between cores
- **Flight Controller Ready**: Core 0 can run custom flight controller code at required frequencies

### Core Capabilities

| Core | Frequency | Purpose |
|------|-----------|---------|
| Core 0 | 100-400 Hz | Control loop, motor output, mixing |
| Core 1 | ~1000 Hz | Web server, WiFi, user interface |

Both cores run at **240 MHz**, providing ample processing power for:
- 400Hz PID loops for flight controllers
- Real-time motor mixing
- Responsive web interface

## Features

- **Configurable Channels**: 1-8 output channels (motors, servos, ESCs)
- **Vehicle Presets**: Car (4WD skid steer), Boat, Plane, Quadcopter
- **REST API**: Full HTTP control for LLM integration
- **WebSocket**: Real-time bidirectional control
- **Web Dashboard**: Touch/keyboard control
- **Safety Watchdog**: Auto-stop on connection loss
- **Arming System**: Optional arming before motors spin

## Quick Start

### 1. Configure Vehicle Type

Edit `include/config.h`:

```cpp
// Uncomment ONE vehicle type:
#define VEHICLE_TYPE_CAR        // 4-wheel skid steer
// #define VEHICLE_TYPE_BOAT    // 2 motors (left/right)
// #define VEHICLE_TYPE_PLANE   // 1 motor + servos
// #define VEHICLE_TYPE_QUAD    // 4 ESCs (needs flight controller)
```

### 2. Configure WiFi

```cpp
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"
```

### 3. Configure Pins

Each channel can be configured individually:

```cpp
// Left Front Motor (Channel 0)
#define CH0_ENABLED true
#define CH0_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH0_PWM_PIN 25
#define CH0_DIR1_PIN 26
#define CH0_DIR2_PIN 27
#define CH0_INVERTED false
```

### 4. Build and Upload

```bash
# Build
pio run

# Upload firmware
pio run -t upload

# Upload web dashboard (optional)
pio run -t uploadfs

# Monitor serial
pio device monitor
```

## Vehicle Types

### Car (VEHICLE_TYPE_CAR)

4-wheel skid steer with differential drive mixing.

```
Channels:
  CH0: Left Front Motor
  CH1: Left Rear Motor
  CH2: Right Front Motor
  CH3: Right Rear Motor

Control:
  throttle: -1.0 to 1.0 (backward to forward)
  steering: -1.0 to 1.0 (left to right)
```

### Boat (VEHICLE_TYPE_BOAT)

2-motor differential drive.

```
Channels:
  CH0: Left Motor
  CH1: Right Motor
```

### Plane (VEHICLE_TYPE_PLANE)

Single motor + 3 control surfaces.

```
Channels:
  CH0: Throttle (ESC)
  CH1: Aileron (Servo)
  CH2: Elevator (Servo)
  CH3: Rudder (Servo)

Control:
  throttle: 0 to 1.0 (ESC)
  roll: -1.0 to 1.0 (ailerons)
  pitch: -1.0 to 1.0 (elevator)
  steering: -1.0 to 1.0 (rudder)
```

### Quadcopter (VEHICLE_TYPE_QUAD)

4 ESCs with basic X-configuration mixing.

```
Channels:
  CH0: Front Left (CCW)
  CH1: Front Right (CW)
  CH2: Rear Left (CW)
  CH3: Rear Right (CCW)
```

**WARNING**: The built-in quad mixer is for testing only!
For real flight, implement a proper flight controller on Core 0.

## Channel Types

| Type | Description | Output Range |
|------|-------------|--------------|
| CHANNEL_PWM_MOTOR | Simple PWM (0-255) | 0-255 |
| CHANNEL_PWM_MOTOR_DIR | PWM + direction pins (H-bridge) | -255 to 255 |
| CHANNEL_SERVO | Servo (0-180 degrees) | 0-180 |
| CHANNEL_ESC | ESC (1000-2000 microseconds) | 1000-2000 |
| CHANNEL_BRUSHLESS | Same as ESC | 1000-2000 |

## API Reference

### Movement Control

```bash
# Move with throttle/steering
GET /api/move?throttle=0.5&steering=0.2
GET /api/move?t=0.5&s=0.2  # Short form

# Aircraft control (plane/quad)
GET /api/move?throttle=0.5&pitch=0.1&roll=-0.1&steering=0

# Stop
GET /api/stop

# Forward/backward
GET /api/forward?speed=200
GET /api/backward?speed=200

# Rotate in place
GET /api/rotate?direction=left&speed=150
```

### Natural Language (for LLM)

```bash
GET /api/command?cmd=go%20forward
GET /api/command?cmd=turn%20right
GET /api/command?cmd=rotate%20left
GET /api/command?cmd=stop
```

### Arming

```bash
POST /api/arm
POST /api/disarm
```

### Channel Control

```bash
# Direct channel control
GET /api/channel?ch=0&value=500

# Enable/disable channel
POST /api/channel/enable?ch=2&enable=false

# Get all channels
GET /api/channels
```

### Status

```bash
GET /api/status
```

Response:
```json
{
  "throttle": 0.5,
  "steering": 0.2,
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

## WebSocket

Connect to `ws://<ip>/ws` for real-time control.

### Send Commands

```javascript
const ws = new WebSocket('ws://192.168.1.100/ws');

// Car/boat control
ws.send(JSON.stringify({
    throttle: 0.5,
    steering: 0.2
}));

// Plane/quad control
ws.send(JSON.stringify({
    throttle: 0.5,
    pitch: 0.1,
    roll: -0.1,
    steering: 0  // yaw
}));

// Stop
ws.send(JSON.stringify({
    throttle: 0,
    steering: 0
}));
```

### Receive Telemetry

```javascript
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    console.log('CH0:', data.ch0);
    console.log('CH1:', data.ch1);
    console.log('Vehicle:', data.vehicle);
    console.log('Armed:', data.armed);
};
```

## Custom Flight Controller

To run your own flight controller code on Core 0:

### 1. Enable in config.h

```cpp
#define EXTERNAL_FLIGHT_CONTROLLER true
```

### 2. Implement userControlLoop()

Create a new file (e.g., `flight_controller.cpp`):

```cpp
#include "control_loop.h"
#include "output_channels.h"

// PID controllers, IMU interface, etc.

void userControlLoop(ControlInput input, unsigned long deltaTime) {
    // Read IMU
    // Run PID loops
    // Calculate motor outputs

    // Set channel outputs (-1000 to 1000 for motors, 1000-2000us for ESC)
    outputChannels.setChannel(0, motor1Output);
    outputChannels.setChannel(1, motor2Output);
    outputChannels.setChannel(2, motor3Output);
    outputChannels.setChannel(3, motor4Output);
}
```

### 3. Control Loop Frequency

Adjust for your needs:

```cpp
#define CONTROL_LOOP_FREQ 400  // 400Hz for flight controller
```

## Wiring Examples

### 4WD Car with L298N

```
ESP32          L298N (Left Side)
------         ------------------
GPIO25 ------> ENA (PWM)
GPIO26 ------> IN1
GPIO27 ------> IN2
GPIO14 ------> ENB (PWM)
GPIO12 ------> IN3
GPIO13 ------> IN4

ESP32          L298N (Right Side)
------         ------------------
GPIO32 ------> ENA (PWM)
GPIO33 ------> IN1
GPIO15 ------> IN2
GPIO4  ------> ENB (PWM)
GPIO16 ------> IN3
GPIO17 ------> IN4
```

### Plane with ESC + Servos

```
ESP32          Components
------         ----------
GPIO25 ------> ESC Signal (Throttle)
GPIO26 ------> Aileron Servo
GPIO27 ------> Elevator Servo
GPIO14 ------> Rudder Servo
```

## Project Structure

```
esp32_api/
├── platformio.ini          # PlatformIO config
├── include/
│   ├── config.h            # Vehicle/channel configuration
│   ├── output_channels.h   # Channel management
│   ├── control_loop.h      # Core 0 control loop
│   ├── mixer.h             # Vehicle-specific mixing
│   └── web_server.h        # Web server interface
├── src/
│   ├── main.cpp            # Entry point, Core 1 loop
│   ├── output_channels.cpp # PWM/servo/ESC output
│   ├── control_loop.cpp    # Core 0 task
│   ├── mixer.cpp           # Differential drive, plane, quad mixing
│   └── web_server.cpp      # REST API, WebSocket, dashboard
├── data/
│   └── index.html          # Optional SPIFFS dashboard
└── README.md
```

## Troubleshooting

### Motors not moving

1. Check `ARM_REQUIRED` - may need to call `/api/arm` first
2. Verify pin configuration matches wiring
3. Check serial output for channel initialization
4. Ensure motor driver has power

### Control feels laggy

1. Use WebSocket instead of REST API
2. Increase `TELEMETRY_RATE_HZ` for faster feedback
3. Reduce `CONTROL_LOOP_FREQ` if overshooting

### WiFi connection fails

1. Double-check SSID and password
2. Connect to AP fallback: `WayfindR-Robot`
3. Check serial for connection status

### Watchdog keeps triggering

1. Increase `WATCHDOG_TIMEOUT_MS`
2. Ensure continuous command stream from dashboard
3. Check WebSocket connection stability

## Safety Features

- **Watchdog**: Stops motors if no commands for 500ms
- **Arming**: Optional requirement before motors spin
- **Disconnect Stop**: Motors stop on WebSocket disconnect
- **Channel Disable**: Individual channels can be disabled at runtime
- **Failsafe**: Configurable behavior on signal loss
