# AMBOT Web Control — Todo

Last updated: Session 16 (2026-02-24)

## Completed (Session 16)

- [x] Project structure created
- [x] Flask app + SocketIO + blueprints
- [x] HardwareManager (motors, LiDAR, camera, IMU, telemetry)
- [x] Motor control: direction pad, keyboard, speed slider
- [x] Camera: MJPEG feed, face detection
- [x] LiDAR: Canvas polar plot, safety rings
- [x] Chat: RAG API proxy, status check
- [x] Diagnostics: CPU/RAM, sensor lights
- [x] Telemetry: SocketIO broadcast + REST polling
- [x] Emergency stop (button, spacebar, REST, SocketIO)
- [x] Motor watchdog (1s timeout)
- [x] Dark theme CSS (ported from PI_API)
- [x] Documentation (scope, roadmap, todo, README)

## Next Session Priorities

- [ ] Deploy to RPi and test with real hardware
- [ ] Verify motor control via web dashboard
- [ ] Test camera MJPEG stream on RPi
- [ ] Test LiDAR polar plot with real scan data
- [ ] Tune motor speed limits for safety
- [ ] Test from laptop browser over WiFi

## Backlog

- [ ] Add movement pattern execution (circle, square, zigzag)
- [ ] Face bounding box overlay on camera panel
- [ ] Switch LiDAR panel from REST polling to SocketIO stream
- [ ] Add authentication
- [ ] Add snapshot/download button for camera
- [ ] Log viewer panel
- [ ] Configuration page
