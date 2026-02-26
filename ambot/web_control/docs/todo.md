# AMBOT Web Control — Todo

Last updated: Session 20 (2026-02-26)

## Completed (Session 16-20)

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
- [x] Ingested 52 ERAU docs (49 scraped + 3 existing) into RAG, 104 chunks -- Session 19
- [x] Verified RAG chat answers with grounded ERAU content and source citations -- Session 19
- [x] SSH port forward verified working (localhost:5123 → jetson:5000) -- Session 19
- [x] Pytest frontend test suite: 62 tests (7 classes), all passing in simulation mode -- Session 19
- [x] JS/CSS already separated: style.css (615 lines), 5 JS modules (654 lines), only 1 line inline -- Session 19
- [x] **Streaming chat responses** via SocketIO — token-by-token from Ollama through RAG API → web dashboard -- Session 20
- [x] **Loading animation with pipeline stages** — "Searching knowledge base..." → "Generating response..." with spinner -- Session 20
- [x] **RAG API streaming endpoint** (`/api/ask/stream`) — NDJSON with sources + token events -- Session 20
- [x] **SocketIO chat stream handler** (`realtime/chat_stream.py`) — proxies RAG stream to browser -- Session 20
- [x] **Firefox ESR installed on Jetson** — bypasses broken Snap, set as default browser -- Session 20
- [x] **deploy.sh fix**: `web_control` now accepted as CLI component argument -- Session 20

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
# Forward Jetson port 5000 to local port 5123 (5000 may be occupied locally)
ssh -f -N -L 5123:localhost:5000 jetson

# Then open in browser on your local machine:
# http://localhost:5123

# To kill the tunnel later:
pkill -f 'ssh.*5123.*jetson'
```

**Alternative** (if port 5000 is free locally):
```bash
ssh -L 5000:localhost:5000 jetson
# Then open: http://localhost:5000
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

## Running Tests

```bash
# From the ambot/ directory (on Jetson or dev machine)
python3 -m pytest web_control/tests/ -v

# Quick run (just structure tests)
python3 -m pytest web_control/tests/test_frontend.py::TestDashboardPage -v

# API tests only
python3 -m pytest web_control/tests/test_frontend.py::TestChatAPI -v
```

## Backlog

- [ ] Add movement pattern execution (circle, square, zigzag)
- [ ] Face bounding box overlay on camera panel
- [ ] Switch LiDAR panel from REST polling to SocketIO stream
- [ ] Add authentication
- [ ] Add snapshot/download button for camera
- [ ] Log viewer panel
- [ ] Configuration page
- [ ] Extract final 1 line of inline JS to app.js (trivial)

## Known Issues

### Jetson Browser Crashes
Firefox and Chromium crash on launch from the Jetson desktop (non-root user).
Root cause: `snap-confine` capability failure on Tegra kernel (no AppArmor support).
See `docs/known-issues.md` for full investigation and potential fixes.
Workaround: SSH port forward and use browser on dev machine.
