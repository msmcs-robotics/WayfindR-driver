# AMBOT — Demo Commands for Presentation Day

> Quick reference for running all demos. All commands run from the `ambot/` folder.

---

## Pre-Flight Checklist

Run this FIRST to verify everything is ready:

```bash
# From your dev machine (WSL2)
cd ~/WayfindR-driver/ambot

# 1. Verify Jetson is reachable
ssh jetson "echo 'Jetson ONLINE'"

# 2. Sync latest code
./deploy.sh jetson

# 3. Verify all services
./deploy.sh jetson --test=chat-health
./deploy.sh jetson --test=rag-health

# 4. Quick dependency check on Jetson
ssh jetson "python3 -c '
import cv2, tkinter, serial, numpy, matplotlib, PIL
print(\"All GUI dependencies OK\")
print(f\"  OpenCV {cv2.__version__}, numpy {numpy.__version__}\")
print(f\"  matplotlib {matplotlib.__version__}, Pillow {PIL.__version__}\")
'"

# 5. Check sensors
ssh jetson "ls /dev/video0 /dev/ttyUSB0 2>/dev/null && echo 'Camera + LiDAR connected'"
```

---

## Demo 1: LLM Chat (RAG-Enabled)

**What it shows**: Chat with the robot's AI. Ask about ERAU faculty, departments, programs. The LLM retrieves answers from the knowledge base.

```bash
# Option A: Start from deploy script (starts app + SSH tunnel)
./deploy.sh jetson --test=chat-start
# Then open: http://localhost:5050

# Option B: Just the tunnel (if app already running)
./deploy.sh jetson --test=chat-tunnel
# Then open: http://localhost:5050

# Option C: On Jetson browser directly
# Open Firefox on the Jetson desktop → http://localhost:5050
```

**Stop when done:**
```bash
./deploy.sh jetson --test=chat-stop
```

**Good demo questions:**
- "What is AMBOT?"
- "Tell me about Dr. Richard Stansbury"
- "Who researches unmanned aircraft?"
- "What departments are in the College of Engineering?"
- "What labs does the EECS department have?"
- "Does Professor Bethelmy have a PhD?"
- "Take me to the RASL lab" (MCP tour guide demo)

---

## Demo 2: Face Detection Camera

**What it shows**: Live camera feed with real-time face detection boxes and direction vectors.

```bash
# On the Jetson (requires monitor connected to Jetson)
ssh jetson "cd ~/ambot && DISPLAY=:0 python3 tests/gui_face_tracker.py"

# With motor intention display (no actual motors needed)
ssh jetson "cd ~/ambot && DISPLAY=:0 python3 tests/gui_face_tracker.py --max-speed 0"

# Headless mode (saves frames, no display needed)
ssh jetson "cd ~/ambot && python3 -c '
import cv2, time
cascade = cv2.CascadeClassifier(\"/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml\")
cap = cv2.VideoCapture(0)
for i in range(5):
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
        cv2.imwrite(f\"/tmp/face_demo_{i}.jpg\", frame)
        print(f\"Frame {i}: {len(faces)} faces detected\")
    time.sleep(0.5)
cap.release()
print(\"Frames saved to /tmp/face_demo_*.jpg\")
'"
```

**Exit**: Press `q` in the GUI window, or `Ctrl+C` in terminal.

---

## Demo 3: LiDAR Navigation Visualization

**What it shows**: Real-time 360° LiDAR polar plot showing obstacles, safety zones, and wandering behavior decisions.

```bash
# On the Jetson (requires monitor connected to Jetson)
ssh jetson "cd ~/ambot && DISPLAY=:0 python3 tests/gui_lidar_nav.py"

# Headless mode (saves scan images)
ssh jetson "cd ~/ambot && python3 tests/gui_lidar_nav.py --headless"
```

**Controls** (in GUI):
- `c` — Calibrate front direction (place object in front first)
- `q` — Quit

**Exit**: Press `q` in the GUI window, or `Ctrl+C` in terminal.

---

## Demo 4: Motor Control via LLM (when batteries available)

**What it shows**: Talk to the robot in natural language, it moves the motors.

```bash
# On the Jetson
ssh jetson "cd ~/ambot && sudo python3 -m mcp_ability.chat"

# Example commands:
#   You: Move forward
#   AMBOT: Moving forward at 50% speed.
#   You: Turn right for 2 seconds
#   AMBOT: Turning right for 2 seconds.
#   You: Stop
#   AMBOT: Motors stopped.
```

**Requires**: L298N wired to Jetson GPIO, fresh batteries.

---

## Demo 5: Web Dashboard (Full Robot Control Panel)

**What it shows**: Browser-based dashboard with motor controls, camera feed, LiDAR plot, chat, and diagnostics.

```bash
# Start dashboard on Jetson
./deploy.sh jetson --test=web-start

# Set up tunnel
ssh -f -N -L 5000:localhost:5000 jetson

# Open: http://localhost:5000
# Or on Jetson browser: http://localhost:5000
```

---

## Service Management Quick Reference

```bash
# ── Chat App ──────────────────────────────────
./deploy.sh jetson --test=chat-start     # Start + tunnel
./deploy.sh jetson --test=chat-stop      # Stop + kill tunnel
./deploy.sh jetson --test=chat-status    # Is it running?
./deploy.sh jetson --test=chat-health    # API health check
./deploy.sh jetson --test=chat-logs      # View recent logs
./deploy.sh jetson --test=chat-tunnel    # Just the SSH tunnel

# ── Web Dashboard ─────────────────────────────
./deploy.sh jetson --test=web-start      # Start dashboard
./deploy.sh jetson --test=web-stop       # Stop dashboard
./deploy.sh jetson --test=web-status     # Is it running?

# ── RAG System ────────────────────────────────
./deploy.sh jetson --test=rag-health     # Check RAG API
./deploy.sh jetson --test=rag-status     # Docker containers
./deploy.sh jetson --test=rag-docs       # List ingested docs
./deploy.sh jetson --test=rag-ingest     # Re-ingest knowledge base

# ── Full Sync ─────────────────────────────────
./deploy.sh jetson                       # Sync all code to Jetson
```

---

## Troubleshooting

**Chat app network error in browser:**
- Verify app is running: `./deploy.sh jetson --test=chat-status`
- Check correct port: http://localhost:5050 (not 5000 or 8080)
- Restart: `./deploy.sh jetson --test=chat-stop && ./deploy.sh jetson --test=chat-start`

**GUI demos won't show window:**
- Need a monitor connected to the Jetson
- Or use X forwarding: `ssh -X jetson` (slow for video)
- Verify display: `ssh jetson "echo \$DISPLAY"` should show `:0`
- Use `DISPLAY=:0` prefix if running via SSH

**LiDAR not responding:**
- Check connection: `ssh jetson "ls /dev/ttyUSB0"`
- Check permissions: `ssh jetson "groups"` (need `dialout`)
- Restart LiDAR: unplug and replug USB

**Camera not detected:**
- Check connection: `ssh jetson "ls /dev/video0"`
- Test capture: `ssh jetson "python3 -c 'import cv2; c=cv2.VideoCapture(0); print(c.isOpened()); c.release()'"`

**Docker containers down after reboot:**
- They should auto-restart (`unless-stopped` policy)
- Manual restart: `ssh jetson "cd ~/ambot/bootylicious/rag && docker compose up -d"`

**Motors not spinning (GPIO confirmed, batteries fresh):**
- Check GPIO: `ssh jetson "sudo python3 ~/ambot/locomotion/jetson_motors/test_motors.py --test basic"`
- Check wiring: ENA/ENB jumpers ON, IN1-IN4 on correct header pins
- See: `locomotion/docs/findings/jetson-motor-debug.md`

---

## Hardware Connections

| Device | Connection | Jetson Port |
|--------|-----------|-------------|
| Camera (EMEET S600) | USB | /dev/video0 |
| LiDAR (LD19) | USB-UART (CP210x) | /dev/ttyUSB0 |
| L298N Motor Driver | GPIO Header | Pins 7,13,29,31,32,33 |
| Ground | GPIO Pin 6 | Shared with L298N GND |

---

## Network Info

| Device | IP | SSH |
|--------|-----|-----|
| Jetson Orin Nano | 10.33.155.83 | `ssh jetson` |
| RPi (original) | 10.33.224.1 | `ssh rpi` |
