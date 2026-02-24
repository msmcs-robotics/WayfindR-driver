# AMBOT Web Control

Browser-based dashboard for the AMBOT robot. Control motors, view camera
feed, visualize LiDAR data, chat with the LLM, and monitor system health.

## Quick Start

```bash
# Install dependencies
pip install -r web_control/requirements.txt

# Run in simulation mode (no hardware needed)
cd ambot/
python3 web_control/run.py --simulate

# Open http://localhost:5000 in your browser
```

## Usage

```bash
# Simulation mode (dev machine)
python3 web_control/run.py --simulate

# Real hardware (RPi)
python3 web_control/run.py

# Custom port
python3 web_control/run.py --port 8080

# Debug mode
python3 web_control/run.py --simulate --debug
```

## Dashboard Panels

| Panel | Description |
|-------|-------------|
| Motor Control | Direction pad, WASD keyboard, speed slider |
| Camera Feed | MJPEG stream with face detection count |
| LiDAR View | Real-time polar plot with safety zone rings |
| LLM Chat | Question answering via Jetson RAG API |
| Diagnostics | CPU/RAM bars, sensor status lights |
| Telemetry | Uptime, motor cmd count, scan rate, FPS |

## Keyboard Controls

| Key | Action |
|-----|--------|
| W / Arrow Up | Forward |
| S / Arrow Down | Backward |
| A / Arrow Left | Turn left |
| D / Arrow Right | Turn right |
| Q | Rotate left (in place) |
| E | Rotate right (in place) |
| Space | Emergency stop |
| W+A, W+D, etc. | Combine for curves |

## Architecture

```
web_control/
├── app.py              # Flask factory + SocketIO
├── run.py              # Entry point (--simulate, --port)
├── config.py           # All settings
├── hardware.py         # HardwareManager singleton
├── routes/
│   ├── pages.py        # Dashboard + MJPEG feed
│   ├── api_motors.py   # Motor REST API
│   ├── api_chat.py     # LLM chat proxy
│   └── api_diagnostics.py  # System stats
├── realtime/
│   ├── motor_events.py # SocketIO motor control
│   ├── lidar_stream.py # LiDAR scan broadcast
│   └── telemetry.py    # System telemetry broadcast
├── templates/          # Jinja2 HTML
├── static/css/         # Dark theme
└── static/js/          # Vanilla JS modules
```

## Requirements

- Python 3.10+
- flask >= 3.0
- flask-socketio >= 5.3
- simple-websocket >= 1.0
- requests (for RAG API proxy)

Hardware drivers (optional — simulation mode works without):
- opencv-python (camera)
- pyserial (LiDAR)
- RPi.GPIO (motors)
