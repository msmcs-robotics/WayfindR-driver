# WayfindR Robot Control API

FastAPI-based web application for controlling a skid-steer robot.

## Features

- **REST API** for programmatic/LLM control
- **Web Dashboard** with keyboard controls
- **WebSocket** for real-time bidirectional communication
- **Pattern Movements** (circle, square, figure-eight, etc.)
- **Waypoint Navigation** integration
- **Telemetry** streaming

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run the server
python main.py

# Or with uvicorn directly
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

Access the dashboard at `http://localhost:8000`

## Keyboard Controls

| Key | Action |
|-----|--------|
| W / Arrow Up | Forward |
| S / Arrow Down | Backward |
| A / Arrow Left | Turn Left |
| D / Arrow Right | Turn Right |
| Q | Rotate Left (in place) |
| E | Rotate Right (in place) |
| Space | Emergency Stop |

**Combinations work!** Press W+D to curve forward-right.

## API Endpoints

### Control
- `POST /api/control/move` - Move with throttle and steering
- `POST /api/control/forward` - Move forward
- `POST /api/control/backward` - Move backward
- `POST /api/control/rotate` - Rotate in place
- `POST /api/control/stop` - Stop movement
- `POST /api/control/command` - Natural language commands (for LLM)

### Navigation
- `GET /api/navigation/waypoints` - List waypoints
- `POST /api/navigation/waypoints` - Add waypoint
- `POST /api/navigation/waypoint/goto` - Navigate to waypoint
- `POST /api/navigation/goto` - Navigate to coordinates
- `POST /api/navigation/cancel` - Cancel navigation

### Patterns
- `GET /api/patterns/list` - List available patterns
- `POST /api/patterns/execute` - Execute a pattern
- `POST /api/patterns/circle` - Circle pattern
- `POST /api/patterns/square` - Square pattern
- `POST /api/patterns/stop` - Stop current pattern

### Telemetry
- `GET /api/telemetry/current` - Current telemetry
- `GET /api/telemetry/summary` - Status summary
- `GET /api/telemetry/position` - Position only
- `GET /api/telemetry/motors` - Motor status

## WebSocket

Connect to `ws://host:8000/ws` for real-time control:

```javascript
const ws = new WebSocket('ws://localhost:8000/ws');

// Send movement command
ws.send(JSON.stringify({
    type: 'move',
    throttle: 0.5,
    steering: 0.0
}));

// Send keyboard event
ws.send(JSON.stringify({
    type: 'keydown',
    key: 'w'
}));

// Stop
ws.send(JSON.stringify({ type: 'stop' }));
```

Receives telemetry updates at 10 Hz.

## Configuration

### GPIO Pins (Raspberry Pi)

Default pin configuration for L298N motor driver:

| Motor | PWM Pin | Direction Pin |
|-------|---------|---------------|
| Left Front | 12 | 16 |
| Left Rear | 13 | 19 |
| Right Front | 18 | 23 |
| Right Rear | 21 | 24 |

Edit `services/motor_driver.py` to change pins.

### Simulation Mode

When running without RPi.GPIO (development/testing), the motor driver automatically enters simulation mode. Commands are logged but no GPIO operations occur.

## Project Structure

```
PI_API/
├── main.py                 # FastAPI application entry point
├── requirements.txt        # Python dependencies
├── models/
│   ├── robot_state.py      # Robot state dataclasses
│   └── commands.py         # Pydantic command models
├── services/
│   ├── robot_controller.py # High-level robot control
│   ├── motor_driver.py     # GPIO motor control
│   ├── connection_manager.py # WebSocket management
│   └── navigation_service.py # ROS2 Nav2 integration
├── routers/
│   ├── control.py          # Movement control endpoints
│   ├── navigation.py       # Waypoint navigation endpoints
│   ├── patterns.py         # Pattern movement endpoints
│   └── telemetry.py        # Telemetry endpoints
├── templates/
│   ├── index.html          # Main dashboard
│   └── status.html         # Status page
└── static/
    ├── css/style.css       # Dashboard styles
    └── js/main.js          # Dashboard JavaScript
```

## LLM Integration

The `/api/control/command` endpoint accepts natural language:

```bash
curl -X POST "http://localhost:8000/api/control/command?command=go%20forward%20for%202%20seconds"
```

Supported phrases:
- "go forward", "move forward"
- "go backward", "reverse"
- "turn left 90 degrees"
- "turn right"
- "stop", "halt"
- "emergency stop"
