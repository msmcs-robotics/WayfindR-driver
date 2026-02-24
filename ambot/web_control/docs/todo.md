# AMBOT Web Control — Todo

Last updated: Session 18 (2026-02-24)

## Completed (Session 16-18)

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
- [x] Fixed chat API endpoint (`/api/query` → `/api/ask`) -- Session 18
- [x] Added `requests` to requirements.txt (missing dependency) -- Session 18
- [x] Deployed to Jetson in simulation mode with RAG chat working -- Session 18
- [x] Added deploy.sh support: `web_control` sync, web-setup, web-start, web-stop, web-status -- Session 18

## Next Session Priorities

### Jetson (chat-only mode)
- [ ] Ingest scraped ERAU content into RAG knowledge base, then re-test chat
- [ ] SSH port forward for browser access: `ssh -L 5000:localhost:5000 jetson`
- [ ] Build pytest-based frontend tests (use Python requests/BeautifulSoup, NOT browser)
- [ ] Separate HTML, JS, CSS into their own files (no inline JS/CSS in HTML templates)

### Raspberry Pi (full hardware mode)
- [ ] Deploy to RPi and test with real hardware
- [ ] Verify motor control via web dashboard
- [ ] Test camera MJPEG stream on RPi
- [ ] Test LiDAR polar plot with real scan data
- [ ] Tune motor speed limits for safety
- [ ] Test from laptop browser over WiFi

## Accessing the Dashboard

**Network restriction**: The campus network only allows SSH (port 22) to the Jetson.
Use SSH port forwarding to access the web dashboard:

```bash
# Forward Jetson port 5000 to your local machine
ssh -L 5000:localhost:5000 jetson

# Then open in browser on your local machine:
# http://localhost:5000
```

## Starting/Stopping the Dashboard

```bash
# Via deploy.sh (from dev machine)
./deploy.sh jetson web_control                # Sync web_control to Jetson
./deploy.sh jetson --test=web-setup           # Install pip dependencies
./deploy.sh jetson --test=web-start           # Start in simulation mode (background)
./deploy.sh jetson --test=web-stop            # Stop the dashboard
./deploy.sh jetson --test=web-status          # Check if running

# Manually on Jetson
RAG_API_URL=http://localhost:8000 python3 web_control/run.py --simulate
```

**Note**: When running on Jetson, set `RAG_API_URL=http://localhost:8000` (the RAG API
runs locally via Docker). The default config points to the Jetson's external IP which
works when the dashboard runs on RPi.

## Backlog

- [ ] Add movement pattern execution (circle, square, zigzag)
- [ ] Face bounding box overlay on camera panel
- [ ] Switch LiDAR panel from REST polling to SocketIO stream
- [ ] Add authentication
- [ ] Add snapshot/download button for camera
- [ ] Log viewer panel
- [ ] Configuration page
- [ ] Pytest-based frontend test suite (endpoints, HTML structure, chat proxy)
- [ ] Separate all inline JS/CSS from HTML templates into standalone files

## Known Issues

### Jetson Browser Crashes
Firefox and Chromium crash on launch from the Jetson desktop (non-root user).
Needs investigation — may be GPU driver conflict, memory issue, or Snap/Flatpak
packaging problem. Workaround: SSH port forward and use browser on dev machine.
