# AMBOT Web Control — Roadmap

## Phase 1: MVP Motor Control (Session 16) ✅

- [x] Flask app factory + SocketIO init
- [x] Entry point with `--simulate` flag
- [x] HardwareManager with graceful degradation
- [x] Direction pad (mouse + touch)
- [x] Keyboard control (WASD + arrows + Q/E rotate)
- [x] Speed slider
- [x] Motor watchdog safety timer
- [x] Emergency stop (button + spacebar + REST)
- [x] Motor status bars (L/R with value display)
- [x] Dark theme CSS (ported from PI_API)

## Phase 2: Camera + LiDAR (Session 16) ✅

- [x] MJPEG streaming endpoint (`/video_feed`)
- [x] Face detection count overlay
- [x] LiDAR polar plot (Canvas renderer with safety rings)
- [x] LiDAR scan polling + closest obstacle display
- [x] Camera/LiDAR panels in dashboard

## Phase 3: Chat + Diagnostics + Telemetry (Session 16) ✅

- [x] LLM chat panel (POST to Jetson RAG API)
- [x] LLM status check
- [x] CPU/RAM diagnostic bars
- [x] Sensor status lights (LiDAR, camera, IMU)
- [x] Telemetry panel (uptime, cmd count, scan rate, FPS)
- [x] SocketIO telemetry + LiDAR broadcast streams

## Phase 4: Documentation (Session 16) ✅

- [x] scope.md
- [x] roadmap.md
- [x] todo.md
- [x] README.md

---

## Future Enhancements

### UI Improvements
- [ ] Responsive layout tuning for tablets
- [ ] Touch gesture support (swipe to steer)
- [ ] Dark/light theme toggle
- [ ] Fullscreen mode for individual panels

### Motor Control
- [ ] Movement patterns (circle, square, zigzag, spiral)
- [ ] Acceleration ramping / speed curves
- [ ] Motor current monitoring (if hardware supports)

### Camera
- [ ] Face bounding box overlay on canvas (not just count)
- [ ] Snapshot button (save frame as PNG)
- [ ] Resolution / quality controls
- [ ] Object detection overlay (YOLO on Jetson)

### LiDAR
- [ ] SocketIO streaming instead of REST polling
- [ ] Obstacle coloring (red for close, green for far)
- [ ] Occupancy grid / mini-map
- [ ] Integration with SLAM when available

### LLM Chat
- [ ] Streaming responses (SSE or SocketIO)
- [ ] Chat history persistence
- [ ] Model selection dropdown
- [ ] Context-aware queries (include sensor data)

### System
- [ ] Authentication (basic auth or token)
- [ ] Configuration page (edit settings via UI)
- [ ] Log viewer panel
- [ ] OTA update mechanism
