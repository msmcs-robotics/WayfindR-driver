# Live Monitoring & Wandering Demo Architecture

> Research note: 2026-02-04

## Goal

Create a continuously running system that:
1. Monitors LiDAR data in real-time
2. Monitors camera feed with face detection
3. Can run the wandering demo when motors are connected
4. Provides a live dashboard/display for demos

## Architecture Options

### Option A: Monolithic Script (Simple)

Single Python script that runs all monitoring in threads.

```
┌─────────────────────────────────────────────┐
│          live_monitor.py                     │
│                                              │
│  ┌──────────────┐  ┌──────────────┐         │
│  │ LiDAR Thread │  │ Camera Thread│         │
│  │ - Read scans │  │ - Capture    │         │
│  │ - Detect obs │  │ - Detect face│         │
│  └──────┬───────┘  └──────┬───────┘         │
│         │                  │                 │
│         ▼                  ▼                 │
│  ┌───────────────────────────────┐          │
│  │      Event Queue              │          │
│  │  (faces, obstacles, etc.)     │          │
│  └──────────────┬────────────────┘          │
│                 │                            │
│                 ▼                            │
│  ┌───────────────────────────────┐          │
│  │      Display / Logger         │          │
│  │  - Console output             │          │
│  │  - GUI (if DISPLAY)           │          │
│  │  - Web dashboard (future)     │          │
│  └───────────────────────────────┘          │
└─────────────────────────────────────────────┘
```

**Pros**: Simple, single process, easy to start/stop
**Cons**: Less modular, harder to restart individual components

### Option B: Systemd Services (Robust)

Separate services for each component, managed by systemd.

```
┌─────────────────────────────────────────────────────────┐
│                    systemd                               │
│                                                          │
│  ┌──────────────────┐  ┌──────────────────┐             │
│  │ ambot-lidar.svc  │  │ ambot-camera.svc │             │
│  │ (LiDAR monitor)  │  │ (Camera monitor) │             │
│  └────────┬─────────┘  └────────┬─────────┘             │
│           │                     │                        │
│           ▼                     ▼                        │
│  ┌────────────────────────────────────────┐             │
│  │           Shared State (Redis/File)    │             │
│  │  - Latest LiDAR scan                   │             │
│  │  - Face detection events               │             │
│  │  - System status                       │             │
│  └────────────────────────────────────────┘             │
│                        │                                 │
│                        ▼                                 │
│  ┌────────────────────────────────────────┐             │
│  │  ambot-dashboard.svc (or web server)   │             │
│  └────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────┘
```

**Pros**: Robust, auto-restart, proper logging, can run at boot
**Cons**: More complex setup, requires systemd knowledge

### Option C: Web Dashboard (Visual)

Flask/FastAPI server with WebSocket for real-time updates.

```
┌─────────────────────────────────────────────────────────┐
│                  ambot-dashboard                         │
│                                                          │
│  ┌──────────────────────────────────────────────────┐   │
│  │               Flask/FastAPI Server                │   │
│  │                                                   │   │
│  │  /api/lidar    → Latest scan data (JSON)        │   │
│  │  /api/camera   → Latest frame / face count      │   │
│  │  /api/status   → System status                  │   │
│  │  /ws           → WebSocket for real-time        │   │
│  └───────────────────────────────────────────────────┘   │
│                        ↕                                 │
│  ┌──────────────────────────────────────────────────┐   │
│  │              HTML/JS Dashboard                    │   │
│  │                                                   │   │
│  │  ┌─────────────┐  ┌─────────────┐               │   │
│  │  │ LiDAR View  │  │ Camera View │               │   │
│  │  │ (polar plot)│  │ (feed+faces)│               │   │
│  │  └─────────────┘  └─────────────┘               │   │
│  │                                                   │   │
│  │  ┌─────────────────────────────────┐            │   │
│  │  │ Status / Controls               │            │   │
│  │  │ - Start/Stop wandering         │            │   │
│  │  │ - Select behavior              │            │   │
│  │  └─────────────────────────────────┘            │   │
│  └───────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

**Pros**: Visual, accessible from any device on network
**Cons**: More complex, requires web dev, uses more resources

## Recommended Approach: Option A + Basic Web

Start with Option A (monolithic) but add a simple web endpoint for status.

### Implementation Plan

#### Phase 1: Console Monitor (Now)
```bash
python3 live_monitor.py --console
```
- LiDAR thread: reads scans, detects obstacles
- Camera thread: captures frames, detects faces
- Console output: periodic status, events

#### Phase 2: GUI Monitor (When display available)
```bash
python3 live_monitor.py --gui
```
- Tkinter or pygame window
- LiDAR polar plot (updating)
- Camera view with face boxes
- Status panel

#### Phase 3: Web Dashboard (Future)
```bash
python3 live_monitor.py --web --port 8080
```
- Flask server
- REST API for status
- Simple HTML dashboard
- WebSocket for updates

## Data Flow

```
LiDAR (LD19)              Camera (USB)
     │                         │
     ▼                         ▼
┌─────────────┐         ┌─────────────┐
│ lidar_ld19  │         │   cv2       │
│  .iter_*()  │         │  .read()    │
└──────┬──────┘         └──────┬──────┘
       │                       │
       ▼                       ▼
┌─────────────┐         ┌─────────────┐
│ Obstacle    │         │ Face        │
│ Detector    │         │ Detector    │
└──────┬──────┘         └──────┬──────┘
       │                       │
       └───────────┬───────────┘
                   │
                   ▼
           ┌─────────────┐
           │ Event Bus   │
           │ (queue)     │
           └──────┬──────┘
                  │
          ┌───────┼───────┐
          ▼       ▼       ▼
       Console   GUI    Web
```

## Events to Track

| Event | Source | Data |
|-------|--------|------|
| `obstacle_detected` | LiDAR | sector, distance, safety_level |
| `face_detected` | Camera | count, bounding_boxes |
| `face_lost` | Camera | - |
| `emergency_stop` | Safety | reason, sector |
| `behavior_change` | Wandering | new_behavior |
| `motor_command` | Locomotion | left_speed, right_speed |

## File Structure

```
ambot/
├── live_monitor.py       # Main entry point
├── monitor/              # Monitor module
│   ├── __init__.py
│   ├── lidar_monitor.py  # LiDAR monitoring thread
│   ├── camera_monitor.py # Camera monitoring thread
│   ├── event_bus.py      # Event queue/pub-sub
│   ├── console_view.py   # Console output
│   ├── gui_view.py       # Tkinter/pygame view
│   └── web_view.py       # Flask dashboard
```

## Console Output Format

```
┌─────────────────────────────────────────────────────────┐
│ AMBOT LIVE MONITOR                     [Running 0:05:32]│
├─────────────────────────────────────────────────────────┤
│ LiDAR: ✓ Connected    Scans: 1523    Rate: 10.2 Hz     │
│ Camera: ✓ Connected   Frames: 4567   Faces: 1          │
│ Motors: ○ Not wired   Behavior: safe_wanderer          │
├─────────────────────────────────────────────────────────┤
│ SECTORS                                                 │
│   Front: 0.45m [SLOW]  Left: 1.23m [CLEAR]             │
│   Back:  2.10m [CLEAR] Right: 0.89m [WARN]             │
├─────────────────────────────────────────────────────────┤
│ EVENTS (last 5)                                         │
│   05:32:10  face_detected  count=1                      │
│   05:32:08  obstacle_slow  sector=front dist=0.45m      │
│   05:31:55  face_lost                                   │
│   05:31:40  sector_clear   all sectors clear            │
│   05:31:22  face_detected  count=2                      │
└─────────────────────────────────────────────────────────┘
```

## Next Steps

1. Create `live_monitor.py` with console mode
2. Test with LiDAR + camera simultaneously
3. Add GUI mode when display testing
4. Add web dashboard when needed for demos

## Resource Considerations

RPi 3 has only 906MB RAM. Need to be careful:
- Use threading, not multiprocessing (shared memory)
- Keep camera resolution at 640x480
- Don't store too many frames/scans in memory
- Use generators for LiDAR data

## Systemd Service (Future)

```ini
# /etc/systemd/system/ambot-monitor.service
[Unit]
Description=AMBOT Live Monitor
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/ambot
ExecStart=/usr/bin/python3 live_monitor.py --console --log
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Install:
```bash
sudo cp ambot-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ambot-monitor
sudo systemctl start ambot-monitor
```
