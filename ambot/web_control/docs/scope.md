# AMBOT Web Control — Scope

## Purpose

Browser-based dashboard for controlling and monitoring the AMBOT robot.
Provides real-time motor control, camera feed, LiDAR visualization, LLM
chat, and system diagnostics from any device on the same network.

## In Scope

- **Motor Control**: Direction pad (mouse/touch), WASD keyboard, speed slider
- **Camera Feed**: MJPEG stream with face detection overlay
- **LiDAR View**: Real-time polar plot on HTML5 canvas
- **LLM Chat**: Proxy to Jetson RAG API for question answering
- **Diagnostics**: CPU/RAM bars, sensor status lights, uptime
- **Telemetry**: Real-time motor state, scan rate, camera FPS
- **Safety**: Motor watchdog, emergency stop (button + spacebar), blur-stop
- **Simulation Mode**: Full dashboard works without hardware (`--simulate`)
- **Offline Operation**: Local socket.io.min.js, no CDN dependencies

## Out of Scope (for now)

- User authentication / access control
- SLAM / mapping interface
- Movement patterns (circle, square, zigzag — future)
- Waypoint navigation UI
- Multi-robot support
- HTTPS / TLS
- Mobile app

## Technology Stack

| Component | Choice | Why |
|-----------|--------|-----|
| Backend | Flask + Flask-SocketIO | User preference; lightweight |
| Real-time | SocketIO (threading mode) | Low-latency motor cmds; no eventlet needed |
| Camera | MJPEG via `/video_feed` | Works in any browser |
| LiDAR | JSON via REST + SocketIO | ~467 points rendered client-side |
| CSS | Custom dark theme | Ported from PI_API |
| JS | Vanilla (no framework) | Minimal footprint for RPi |

## Hardware Targets

| Target | Mode | What Works | What Doesn't |
|--------|------|-----------|--------------|
| **Raspberry Pi 3B** | Hardware | Motors, LiDAR, Camera, IMU | LLM Chat (proxies to Jetson) |
| **Jetson Orin Nano** | Simulate | LLM Chat (RAG-enabled), Diagnostics | Motors, LiDAR, Camera (simulated) |
| **Dev machine (WSL2)** | Simulate | UI testing, all panels (mock data) | All hardware |

**Future**: When all hardware migrates to the Jetson, the dashboard runs in full
hardware mode on the Jetson with motors, LiDAR, camera, AND LLM chat all working.

## Accessing the Dashboard

The campus network blocks all ports except SSH (22) to the Jetson. Use SSH port forwarding:

```bash
ssh -L 5000:localhost:5000 jetson
# Then open: http://localhost:5000
```
