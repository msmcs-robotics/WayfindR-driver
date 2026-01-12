# PI_API Scope Documentation

## Overview

The PI_API folder contains a comprehensive FastAPI-based web application for controlling the WayfindR robot. It provides both a RESTful API for programmatic control (including LLM integration) and an interactive web dashboard for manual operation via keyboard/mouse controls. The system is designed to run on a Raspberry Pi and interfaces with L298N motor drivers to control a skid-steer robot platform.

## Purpose

This API serves as the primary control interface for the WayfindR robot, providing:

1. **Manual Control** - Real-time keyboard/joystick control through web dashboard
2. **Programmatic Control** - REST API endpoints for automated/scripted control
3. **LLM Integration** - Natural language command interface for AI assistants
4. **Telemetry Monitoring** - Real-time sensor data and state broadcasting
5. **Pattern Execution** - Pre-programmed movement patterns for testing and demos
6. **Navigation** - Waypoint-based navigation with ROS2 integration capabilities

---

## API Endpoints

### Control Endpoints (`/api/control`)

Direct robot movement control for throttle, steering, and rotation.

#### POST `/api/control/move`
Move robot with precise throttle and steering control.
- **Parameters:**
  - `throttle` (float): -1.0 (backward) to 1.0 (forward)
  - `steering` (float): -1.0 (left) to 1.0 (right)
  - `duration` (float, optional): Movement duration in seconds
- **Response:** `CommandResponse` with success status and movement data
- **Example:** `{"throttle": 0.5, "steering": 0.0, "duration": 2.0}`

#### POST `/api/control/forward`
Move robot forward at specified speed.
- **Parameters:**
  - `speed` (float): 0.0 to 1.0 (default: 0.5)
  - `duration` (float, optional): Movement duration in seconds
- **Response:** `CommandResponse` with success status

#### POST `/api/control/backward`
Move robot backward at specified speed.
- **Parameters:**
  - `speed` (float): 0.0 to 1.0 (default: 0.5)
  - `duration` (float, optional): Movement duration in seconds
- **Response:** `CommandResponse` with success status

#### POST `/api/control/rotate`
Rotate robot in place.
- **Parameters:**
  - `direction` (str): "left" or "right"
  - `speed` (float): 0.0 to 1.0 (default: 0.5)
  - `angle` (float, optional): Target angle in degrees (continuous if None)
- **Response:** `CommandResponse` with rotation details

#### POST `/api/control/turn_left`
Turn left (counter-clockwise rotation).
- **Parameters:**
  - `angle` (float, optional): Degrees to turn
  - `speed` (float): Rotation speed (default: 0.5)
- **Response:** `CommandResponse` with turn status

#### POST `/api/control/turn_right`
Turn right (clockwise rotation).
- **Parameters:**
  - `angle` (float, optional): Degrees to turn
  - `speed` (float): Rotation speed (default: 0.5)
- **Response:** `CommandResponse` with turn status

#### POST `/api/control/stop`
Stop robot movement.
- **Parameters:**
  - `mode` (str): "normal", "emergency", or "brake" (default: "normal")
- **Response:** `CommandResponse` with stop confirmation

#### GET `/api/control/state`
Get current robot state including position, velocity, motors, and sensors.
- **Response:** Complete `RobotState` dictionary

#### POST `/api/control/command`
Natural language command interface for LLM integration.
- **Parameters:**
  - `command` (str): Natural language command
- **Supported Commands:**
  - "go forward", "move forward"
  - "go backward", "reverse", "back"
  - "turn left 90 degrees"
  - "turn right"
  - "stop", "halt"
  - "emergency stop"
  - "move forward for 2 seconds"
- **Response:** `CommandResponse` with parsed command result

### Navigation Endpoints (`/api/navigation`)

Waypoint-based navigation with ROS2 Nav2 integration support.

#### POST `/api/navigation/goto`
Navigate to specific coordinates.
- **Parameters:**
  - `x` (float): Target X position in meters
  - `y` (float): Target Y position in meters
  - `yaw` (float, optional): Target orientation in degrees
- **Response:** `CommandResponse` with navigation status
- **Note:** Requires navigation service to be active

#### POST `/api/navigation/waypoint/goto`
Navigate to a named waypoint.
- **Parameters:**
  - `waypoint_name` (str): Name of target waypoint
  - `speed` (float): Navigation speed (default: 0.5)
- **Response:** `CommandResponse` with waypoint navigation status

#### GET `/api/navigation/waypoints`
List all defined waypoints.
- **Response:** Array of waypoint objects with name, coordinates, and description

#### POST `/api/navigation/waypoints`
Add a new waypoint.
- **Parameters:**
  - `name` (str): Waypoint identifier
  - `x` (float): X coordinate in meters
  - `y` (float): Y coordinate in meters
  - `yaw` (float): Orientation in degrees (default: 0.0)
  - `description` (str, optional): Waypoint description
- **Response:** `CommandResponse` with waypoint creation status

#### DELETE `/api/navigation/waypoints/{name}`
Delete a waypoint by name.
- **Response:** `CommandResponse` with deletion confirmation

#### POST `/api/navigation/waypoints/save_current`
Save current robot position as a waypoint.
- **Parameters:**
  - `name` (str): Waypoint name
  - `description` (str): Optional description
- **Response:** `CommandResponse` with saved position data

#### GET `/api/navigation/routes`
List all defined routes (sequences of waypoints).
- **Response:** Array of route objects with name and waypoint list

#### POST `/api/navigation/routes`
Create a new route.
- **Parameters:**
  - `name` (str): Route identifier
  - `waypoints` (array): List of waypoint names in sequence
- **Response:** `CommandResponse` with route creation status

#### POST `/api/navigation/routes/{name}/execute`
Execute a named route by following waypoints in sequence.
- **Response:** `CommandResponse` with route execution status

#### POST `/api/navigation/cancel`
Cancel current navigation operation.
- **Response:** `CommandResponse` with cancellation confirmation

#### GET `/api/navigation/status`
Get current navigation status.
- **Response:** Navigation state with current goal, distance, and position

### Telemetry Endpoints (`/api/telemetry`)

Real-time sensor data and robot state monitoring.

#### GET `/api/telemetry/current`
Get comprehensive current telemetry data.
- **Response:** Complete telemetry including:
  - Timestamp and datetime
  - Robot mode and movement state
  - Position (x, y, z, yaw, pitch, roll)
  - Velocity (linear and angular)
  - Drive system (throttle, steering, motor states)
  - Sensors (battery, IMU, temperature, obstacles)
  - Connection status and uptime

#### GET `/api/telemetry/position`
Get current position and orientation only.
- **Response:** Position data (x, y, z, yaw, pitch, roll) with timestamp

#### GET `/api/telemetry/velocity`
Get current velocity.
- **Response:** Velocity data (linear_x, linear_y, angular_z) with timestamp

#### GET `/api/telemetry/battery`
Get battery status.
- **Response:** Voltage and percentage with timestamp

#### GET `/api/telemetry/motors`
Get motor status for all four wheels.
- **Response:** Individual motor data (speed, current, temperature, enabled state)

#### GET `/api/telemetry/imu`
Get IMU sensor data.
- **Response:** Accelerometer, gyroscope, and magnetometer readings

#### GET `/api/telemetry/obstacles`
Get obstacle detection data.
- **Response:** Obstacle distances with timestamp

#### GET `/api/telemetry/history`
Get historical telemetry data.
- **Parameters:**
  - `limit` (int): Number of records (max 1000, default: 100)
  - `offset` (int): Starting offset (default: 0)
- **Response:** Array of historical telemetry records

#### DELETE `/api/telemetry/history`
Clear telemetry history.
- **Response:** Count of cleared records

#### GET `/api/telemetry/summary`
Get dashboard-friendly status summary.
- **Response:** Condensed status with formatted strings for display

### Pattern Endpoints (`/api/patterns`)

Pre-programmed movement patterns for testing and demonstrations.

#### POST `/api/patterns/execute`
Execute a movement pattern.
- **Parameters:**
  - `pattern` (enum): circle, square, figure_eight, zigzag, spiral, line, scan
  - `size` (float): Pattern size in meters (0.1-10.0, default: 1.0)
  - `speed` (float): Movement speed (0.0-1.0, default: 0.5)
  - `repetitions` (int): Repeat count (1-100, default: 1)
  - `clockwise` (bool): Direction for circular patterns (default: true)
- **Response:** `CommandResponse` with pattern execution status

#### POST `/api/patterns/circle`
Execute a circle pattern.
- **Parameters:**
  - `radius` (float): Circle radius in meters (default: 1.0)
  - `speed` (float): Movement speed (default: 0.5)
  - `clockwise` (bool): Direction (default: true)
- **Response:** `CommandResponse`

#### POST `/api/patterns/square`
Execute a square pattern.
- **Parameters:**
  - `side_length` (float): Side length in meters (default: 1.0)
  - `speed` (float): Movement speed (default: 0.5)
- **Response:** `CommandResponse`

#### POST `/api/patterns/figure_eight`
Execute a figure-eight pattern.
- **Parameters:**
  - `size` (float): Pattern size in meters (default: 1.0)
  - `speed` (float): Movement speed (default: 0.5)
- **Response:** `CommandResponse`

#### POST `/api/patterns/zigzag`
Execute a zigzag pattern.
- **Parameters:**
  - `width` (float): Pattern width in meters (default: 2.0)
  - `legs` (int): Number of zigzag segments (default: 4)
  - `speed` (float): Movement speed (default: 0.5)
- **Response:** `CommandResponse`

#### POST `/api/patterns/spiral`
Execute an expanding or contracting spiral.
- **Parameters:**
  - `max_radius` (float): Maximum radius in meters (default: 2.0)
  - `speed` (float): Movement speed (default: 0.5)
  - `outward` (bool): Expand outward vs contract inward (default: true)
- **Response:** `CommandResponse`

#### POST `/api/patterns/scan`
Execute a lawn-mower scan pattern for area coverage.
- **Parameters:**
  - `width` (float): Scan width in meters (default: 3.0)
  - `height` (float): Scan height in meters (default: 2.0)
  - `speed` (float): Movement speed (default: 0.3)
- **Response:** `CommandResponse`

#### POST `/api/patterns/stop`
Stop current pattern execution.
- **Response:** `CommandResponse` with stop confirmation

#### GET `/api/patterns/status`
Get current pattern execution status.
- **Response:** Running state and pattern info

#### GET `/api/patterns/list`
List all available patterns with descriptions.
- **Response:** Array of pattern definitions with parameters

### General Endpoints

#### GET `/api/health`
Health check endpoint.
- **Response:** Status, timestamp, connection state, uptime

#### POST `/api/emergency_stop`
Emergency stop - immediately halt all motors.
- **Response:** Emergency stop confirmation

#### GET `/`
Main dashboard page with interactive controls.
- **Response:** HTML dashboard interface

#### GET `/status`
Status/telemetry page.
- **Response:** HTML status page

#### WebSocket `/ws`
Real-time bidirectional communication.
- **Receives:**
  - `{"type": "keydown", "key": "w"}` - Key press events
  - `{"type": "keyup", "key": "w"}` - Key release events
  - `{"type": "move", "throttle": 0.5, "steering": 0.0}` - Direct movement
  - `{"type": "stop"}` - Stop command
  - `{"type": "get_state"}` - State request
- **Sends:**
  - `{"type": "telemetry", "data": {...}, "timestamp": ...}` - Telemetry updates (10 Hz)
  - `{"type": "ack", "command": "...", "status": "ok"}` - Command acknowledgments
  - `{"type": "state", "data": {...}}` - State responses
  - `{"type": "message", "data": "..."}` - General messages

---

## Communication Protocols

### REST API
- **Protocol:** HTTP/HTTPS
- **Format:** JSON
- **Port:** 8000 (default, configurable)
- **Host:** 0.0.0.0 (listens on all interfaces)
- **Framework:** FastAPI with Uvicorn ASGI server

### WebSocket
- **Protocol:** WebSocket (ws:// or wss://)
- **Endpoint:** `/ws`
- **Update Rate:** 10 Hz telemetry broadcast
- **Format:** JSON messages with type field
- **Features:**
  - Real-time bidirectional communication
  - Automatic connection management
  - Broadcast to multiple clients
  - Graceful disconnection handling

### GPIO Communication
- **Interface:** Raspberry Pi GPIO (BCM mode)
- **Motor Control:** L298N dual H-bridge motor drivers
- **PWM Frequency:** 1000 Hz
- **Pin Configuration:**
  - Left Front: PWM=GPIO12, DIR=GPIO16
  - Left Rear: PWM=GPIO13, DIR=GPIO19
  - Right Front: PWM=GPIO18, DIR=GPIO23
  - Right Rear: PWM=GPIO21, DIR=GPIO24

### Drive System Protocol
- **Type:** Differential/Skid Steering
- **Control Inputs:**
  - Throttle: -1.0 (full backward) to +1.0 (full forward)
  - Steering: -1.0 (full left) to +1.0 (full right)
- **Motor Coordination:** All wheels on same side operate in unison
- **Safety Features:**
  - Deadzone: 0.05 (5% threshold)
  - Watchdog timeout: 0.5 seconds
  - Emergency stop with braking pulse

---

## Integration with Robot Control and ROS2

### Current State

The system is designed with ROS2 integration in mind but currently operates **independently** as a standalone control system. The navigation service includes hooks and structure for ROS2 Nav2 integration but does not require it to function.

### ROS2 Integration Design

**Intended Integration Points:**

1. **Localization** (AMCL/SLAM)
   - Position updates from `/amcl_pose` topic
   - Map-based localization feedback
   - `navigation_service.py` has `update_position()` method ready for pose callbacks

2. **Navigation** (Nav2)
   - Goal poses sent to Nav2 action server
   - Path planning through ROS2 navigation stack
   - Waypoint following with Nav2's behavior trees

3. **Sensor Integration**
   - LiDAR data from `/scan` topic
   - IMU data from `/imu` topic
   - Odometry from `/odom` topic

4. **Motor Commands**
   - Could publish to `/cmd_vel` for ROS2 control
   - Current implementation: Direct GPIO control bypasses ROS2
   - Allows operation with or without ROS2 stack

### Hybrid Architecture

The system uses a **hybrid control architecture**:

- **Direct GPIO Control** (Current): FastAPI → RobotController → MotorDriver → GPIO → L298N
- **ROS2 Integration** (Future): FastAPI → Navigation Service → ROS2 Nav2 → ROS2 Motor Node → GPIO

This allows the robot to:
- Operate standalone without ROS2 for simple control tasks
- Integrate with ROS2 when advanced features (SLAM, Nav2) are needed
- Switch between modes seamlessly

### Simulation Mode

When running without GPIO hardware (development/testing):
- Automatically enters simulation mode
- Logs commands without GPIO operations
- Full API remains functional for testing
- Useful for development on non-Pi systems

---

## Current State of Implementation

### Fully Implemented

1. **Core Control System** ✓
   - Robot controller with differential drive logic
   - Motor driver with GPIO and simulation modes
   - Safety watchdog for connection loss
   - Emergency stop functionality

2. **REST API** ✓
   - All control endpoints functional
   - Pattern execution system complete
   - Telemetry endpoints with history
   - Natural language command parsing

3. **WebSocket Communication** ✓
   - Real-time bidirectional messaging
   - Connection manager for multiple clients
   - 10 Hz telemetry broadcasting
   - Keyboard event handling

4. **Web Dashboard** ✓
   - Interactive control interface
   - Real-time telemetry display
   - Motor status visualization
   - Pattern execution controls
   - Keyboard control support (WASD, arrows, Q/E, Space)

5. **Data Models** ✓
   - Complete state management (RobotState)
   - Command validation (Pydantic models)
   - Movement states and robot modes
   - Sensor data structures

6. **Movement Patterns** ✓
   - Circle, square, figure-eight
   - Zigzag, spiral, scan patterns
   - Configurable size, speed, repetitions

### Partially Implemented

1. **Navigation System** ⚠
   - Waypoint data structures: ✓
   - Route planning: ✓
   - Navigation service skeleton: ✓
   - ROS2 Nav2 integration: ✗ (hooks present, not connected)
   - Dead reckoning navigation: Limited (open-loop timing-based)
   - Position tracking: Simulated (no actual localization)

2. **Telemetry Collection** ⚠
   - State tracking: ✓
   - Motor states: ✓
   - Position/velocity: Simulated (needs sensor integration)
   - Battery monitoring: Placeholder (needs ADC/sensor)
   - IMU integration: Placeholder (needs actual IMU)
   - Obstacle detection: Placeholder (needs LiDAR/ultrasonic)

### Not Yet Implemented

1. **ROS2 Integration** ✗
   - Nav2 action client
   - AMCL pose subscriber
   - cmd_vel publisher
   - Sensor topic subscribers

2. **Sensor Hardware Integration** ✗
   - IMU (BNO055, MPU6050, etc.)
   - Battery voltage monitoring (ADC)
   - Wheel encoders
   - LiDAR/ultrasonic sensors

3. **Advanced Features** ✗
   - Obstacle avoidance
   - Closed-loop position control
   - Path recording and replay
   - Multi-robot coordination

### Testing Status

- **Simulation Mode:** Fully functional
- **GPIO Control:** Implemented, requires hardware testing
- **API Endpoints:** Functional, tested in simulation
- **Dashboard:** Functional UI, requires robot testing
- **Navigation:** Basic structure, needs localization

---

## Dependencies and Requirements

### Python Requirements (requirements.txt)

**Core Web Framework:**
- `fastapi>=0.104.0` - Modern async web framework
- `uvicorn[standard]>=0.24.0` - ASGI server with WebSocket support
- `jinja2>=3.1.2` - Template engine for HTML pages

**Data Validation:**
- `pydantic>=2.5.0` - Request/response models

**WebSocket:**
- `websockets>=12.0` - WebSocket protocol support
- `python-multipart>=0.0.6` - Form data parsing

**Configuration:**
- `pyyaml>=6.0.1` - YAML file support for waypoints

**Optional - Raspberry Pi GPIO:**
- `RPi.GPIO>=0.7.1` - Raspberry Pi GPIO control (comment out for dev)

**Optional - ROS2:**
- ROS2 Humble (system installation, not pip)
- ROS2 Python libraries (system-level)

### System Requirements

**Hardware:**
- Raspberry Pi (3B+ or 4 recommended)
- MicroSD card (16GB+ recommended)
- L298N motor driver boards (2x for 4-wheel skid steer)
- DC motors (4x for skid steer configuration)
- Power supply (appropriate for motors and Pi)
- Optional: IMU, LiDAR, encoders, battery monitor

**Operating System:**
- Raspberry Pi OS (Bullseye or Bookworm)
- Ubuntu 22.04 (for ROS2 Humble)
- Python 3.8 or higher

**Network:**
- Wi-Fi or Ethernet for API access
- Local network for dashboard access
- Optional: External access via port forwarding

### GPIO Pin Requirements

**PWM-Capable Pins (4 required):**
- GPIO12, GPIO13, GPIO18, GPIO21 (default configuration)

**Digital Output Pins (4 required):**
- GPIO16, GPIO19, GPIO23, GPIO24 (direction control)

**Total:** 8 GPIO pins

### Software Dependencies

**Runtime:**
- Python 3.8+
- pip package manager
- systemd (for service deployment)

**Development:**
- Git
- Text editor or IDE
- Web browser (for dashboard)

**Optional:**
- ROS2 Humble
- Nav2 navigation stack
- SLAM Toolbox or Cartographer
- rviz2 (visualization)

### Installation

```bash
# Install system dependencies
sudo apt update
sudo apt install python3-pip python3-venv

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python requirements
cd PI_API
pip install -r requirements.txt

# For Raspberry Pi, uncomment RPi.GPIO in requirements.txt
```

### Running the Server

```bash
# Development mode (auto-reload)
python main.py

# Or with uvicorn directly
uvicorn main:app --host 0.0.0.0 --port 8000 --reload

# Production mode
uvicorn main:app --host 0.0.0.0 --port 8000 --workers 1
```

### Accessing the System

- **Dashboard:** http://robot-ip:8000
- **API Docs:** http://robot-ip:8000/docs (Swagger UI)
- **WebSocket:** ws://robot-ip:8000/ws
- **Health Check:** http://robot-ip:8000/api/health

---

## Architecture Overview

### Project Structure

```
PI_API/
├── main.py                    # FastAPI application entry point
├── requirements.txt           # Python dependencies
├── README.md                  # User documentation
├── scope.md                   # This file - technical scope
│
├── models/                    # Data models
│   ├── __init__.py
│   ├── robot_state.py         # State management dataclasses
│   └── commands.py            # Pydantic request/response models
│
├── services/                  # Core business logic
│   ├── __init__.py
│   ├── robot_controller.py    # High-level robot control
│   ├── motor_driver.py        # Low-level GPIO motor control
│   ├── connection_manager.py  # WebSocket connection handling
│   └── navigation_service.py  # Waypoint navigation (ROS2 hooks)
│
├── routers/                   # API endpoint routers
│   ├── __init__.py
│   ├── control.py             # Movement control endpoints
│   ├── navigation.py          # Waypoint/route endpoints
│   ├── patterns.py            # Pattern execution endpoints
│   └── telemetry.py           # Telemetry/sensor endpoints
│
├── templates/                 # Jinja2 HTML templates
│   ├── index.html             # Main control dashboard
│   └── status.html            # Status monitoring page
│
└── static/                    # Static web assets
    ├── css/
    │   └── style.css          # Dashboard styles
    └── js/
        └── main.js            # Dashboard JavaScript/WebSocket
```

### Key Design Patterns

1. **Service Layer Pattern** - Business logic separated in services/
2. **Router Pattern** - API endpoints organized by domain
3. **Dependency Injection** - Robot controller and manager stored in app.state
4. **Observer Pattern** - WebSocket broadcasting for telemetry
5. **State Machine** - Robot mode and movement state management
6. **Command Pattern** - Pydantic command models for validation

### Control Flow

```
User Input → Web Dashboard → WebSocket → Robot Controller
                 ↓                           ↓
            REST API Client → FastAPI Router → Motor Driver → GPIO → Motors
                                                ↓
                                           State Update → WebSocket Broadcast
```

---

## Future Enhancement Opportunities

1. **Complete ROS2 Integration** - Connect Nav2, AMCL, and sensor topics
2. **Sensor Integration** - Add IMU, encoders, battery monitoring, obstacle detection
3. **Closed-Loop Control** - Use encoder feedback for precise movements
4. **Obstacle Avoidance** - Integrate LiDAR for dynamic path adjustment
5. **Authentication** - Add API key or JWT authentication
6. **Data Persistence** - Save waypoints and telemetry to database
7. **Multi-Robot Support** - Coordinate multiple robots through API
8. **Video Streaming** - Add camera feed to dashboard
9. **Mobile App** - Native mobile control interface
10. **Voice Control** - Integration with voice assistants

---

## Notes

- The system is designed to be **hardware-agnostic** during development (simulation mode)
- GPIO pin assignments are **configurable** in motor_driver.py
- **Watchdog timer** prevents runaway if connection is lost
- **Emergency stop** can be triggered via API, dashboard, or Space key
- All **coordinate systems** use meters for distance and radians for angles
- **Pattern movements** use open-loop timing (will improve with encoders)
- The system supports **multiple simultaneous WebSocket connections**
- **Telemetry history** limited to 1000 records to prevent memory issues

---

**Last Updated:** 2026-01-11
**Version:** 1.0.0
**Author:** WayfindR Team
**License:** See project LICENSE file
