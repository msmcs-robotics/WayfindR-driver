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

- **Raspberry Pi 4**: Primary (motors, LiDAR, camera, IMU)
- **Jetson Orin Nano**: LLM/RAG backend
- **Dev machine (WSL2)**: Simulation mode
