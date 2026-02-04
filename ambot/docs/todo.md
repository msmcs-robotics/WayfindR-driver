# Ambot - Todo & Roadmap

> Last updated: 2026-02-03

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                           AMBOT SYSTEM                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────┐      ┌─────────────────┐                       │
│  │   PATHFINDER    │      │   BOOTYLICIOUS  │                       │
│  │ (Sensor Fusion) │ ───► │  (LLM + RAG)    │                       │
│  │                 │      │                 │                       │
│  │ • LiDAR (LD19)  │      │ • Ollama/HF     │                       │
│  │ • USB Camera    │      │ • RAG System    │                       │
│  │ • MPU6050 (opt) │      │ • Conversation  │                       │
│  │ • Facial Detect │      │ • MCP Server*   │                       │
│  └────────┬────────┘      └─────────────────┘                       │
│           │                        ▲                                │
│           │ obstacles/events       │ context/triggers               │
│           ▼                        │                                │
│  ┌─────────────────┐               │                                │
│  │   LOCOMOTION    │───────────────┘                                │
│  │ (Motor Control) │                                                │
│  │                 │                                                │
│  │ • L298N/TB6612  │                                                │
│  │ • DRV8833       │                                                │
│  │ • Diff. Drive   │                                                │
│  └─────────────────┘                                                │
│                                                                     │
│  * Future: MCP server for LLM-controlled locomotion (8-12B models)  │
└─────────────────────────────────────────────────────────────────────┘
```

### Component Responsibilities

| Component | Purpose | Sensors/Outputs | Optional Sensors |
|-----------|---------|-----------------|------------------|
| **Pathfinder** | Sensor fusion & obstacle avoidance | LiDAR LD19 (primary) | Camera, MPU6050, IMU |
| **Locomotion** | Motor control & movement execution | GPIO/PWM to motors | Encoders |
| **Bootylicious** | Conversation & decision-making | LLM + RAG | - |

### Design Principles

1. **Graceful Degradation**: Optional sensors (MPU6050, camera) enhance but don't block functionality
2. **Sensor Fusion in Pathfinder**: All perception sensors live here, locomotion just executes
3. **Event-Driven LLM Triggers**: Camera facial detection can trigger LLM responses
4. **Platform Agnostic**: Components run on RPi or Jetson as available
5. **Idempotent Scripts**: All deploy/bootstrap scripts safe to run repeatedly

---

## In Progress

_Tasks actively being worked on_

- [ ] Wire L298N motor driver to RPi GPIO

## Blocked

_Tasks waiting on something (include reason)_

- [ ] Jetson system inventory — **Blocked by**: Fresh Ubuntu 22.04 / JetPack 6.1 installation in progress
- [ ] Motor hardware test — **Blocked by**: Need to wire L298N to RPi GPIO

## Up Next

_Priority queue for immediate work_

### Raspberry Pi (pi@10.33.224.1)
- [ ] Wire L298N motor driver to RPi GPIO (see [locomotion/docs/l298n-driver-wiring-guide.md](../locomotion/docs/l298n-driver-wiring-guide.md))
- [ ] Test motors: `./deploy.sh rpi locomotion --test=motors`
- [ ] Tune pathfinder safety zones for robot size
- [ ] Test GUI diagnostics with display attached

### Jetson (when available)
- [ ] Complete Jetson first boot (set up user: ambot)
- [ ] Set `JETSON_HOST` in `deploy.sh`
- [ ] Deploy: `./deploy.sh jetson --bootstrap`
- [ ] Test Ollama LLM deployment
- [ ] Test RAG system with Docker

### Integration
- [ ] Combine pathfinder + locomotion for basic wandering demo
- [ ] Test obstacle avoidance with actual robot movement

## Backlog

_Lower priority, do when time permits_

### Sensor Integration
- [ ] MPU6050 driver with graceful absence handling
- [ ] Sensor fusion combining LiDAR + IMU data
- [ ] Camera facial recognition event system

### Inter-Component Communication
- [ ] Define protocol (REST API vs MQTT)
- [ ] Pathfinder → Locomotion obstacle commands
- [ ] Pathfinder → Bootylicious event triggers (face detected, etc.)

### Future Vision (see roadmap.md Milestone 5)
- [ ] MCP server (FastMCP) for LLM-to-locomotion control
- [ ] Scale to 8-12B parameter LLMs on Jetson
- [ ] Voice interaction (STT/TTS via Android device)

## Recently Completed

_Session 2 - 2026-02-03_

- [x] **LD19 LiDAR driver** - Created `pathfinder/lidar_ld19.py` (230400 baud, working!)
- [x] **LD19 protocol research** - Documented in `docs/findings/ld19-lidar-protocol.md`
- [x] **LiDAR test script** - `tests/test_ld19_lidar.py` (all tests passing: ~497 pts/scan)
- [x] **Deploy script enhanced** - Modular component deployment with `./deploy.sh`
- [x] **Docker added to bootstrap** - Docker Compose v2.35.0 (ARM64)
- [x] **Roadmap updated** - Added Milestone 5 (MCP, 8-12B LLMs, sensor fusion)
- [x] **Session summary** - `docs/archive/2026-02-03-session-2-summary.md`

_Session 1 - 2026-02-03_

- [x] Set up RPi SSH key access
- [x] Rsync ambot to RPi ~/ambot/
- [x] RPi system inventory (Debian 13, 906MB RAM, Cortex-A53)
- [x] Create modular motor driver (TB6612FNG, L298N, DRV8833)
- [x] Create motor driver documentation
- [x] Create power system documentation
- [x] Create test scripts (GPIO, camera, LiDAR)
- [x] Install OpenCV and dependencies on RPi
- [x] LLM deployment research for Jetson
- [x] JetPack 6.1 setup documentation
- [x] Create RPi bootstrap scripts (idempotent)
- [x] Camera capture test passing (640x480)
- [x] GUI diagnostic tools (gui_camera.py, gui_lidar.py)

---

## Folder Structure

```
ambot/
├── bootylicious/          # LLM + RAG system (conversation brain)
│   ├── deploy.sh          # Master deployment script
│   ├── scripts/           # Setup scripts (Ollama, Docker, HuggingFace)
│   ├── rag/               # RAG system (Docker)
│   └── tests/             # System tests
├── locomotion/            # Motor control component (ONLY movement)
│   ├── deploy.sh          # Deployment and diagnostics
│   ├── rpi_motors/        # Modular RPi motor drivers
│   │   ├── config.py      # Pin configurations per driver
│   │   ├── drivers.py     # TB6612FNG, L298N, DRV8833 implementations
│   │   └── factory.py     # Robot factory for easy setup
│   ├── yahboomg1/         # YAHBOOM G1 motor driver (Jetson)
│   └── docs/              # Motor driver wiring guides
├── pathfinder/            # Sensor fusion & obstacle avoidance
│   ├── config.py          # LiDAR type selection (LD19/C1M1), safety zones
│   ├── lidar.py           # RPLidar C1M1 driver
│   ├── lidar_ld19.py      # YOUYEETOO LD19 driver (NEW - working!)
│   ├── obstacle_detector.py
│   └── scripts/           # udev rules, etc.
├── scripts/               # Bootstrap & deployment scripts
│   ├── rpi-bootstrap.sh   # Master RPi bootstrap
│   ├── rpi-bootstrap-system.sh   # System packages + Docker
│   └── rpi-bootstrap-python.sh   # Python libraries
├── deploy.sh              # Master deployment script (NEW)
├── tests/                 # Hardware test & diagnostic scripts
│   ├── run_all_tests.sh   # Run all tests
│   ├── test_gpio.py       # GPIO tests
│   ├── test_usb_camera.py # Camera tests
│   ├── test_ld19_lidar.py # LD19 LiDAR tests (NEW - all passing!)
│   ├── test_usb_lidar.py  # Generic LiDAR tests
│   ├── gui_camera.py      # Camera GUI with face detection
│   ├── gui_lidar.py       # LiDAR polar visualization
│   └── results/           # Test output
└── docs/                  # Project documentation
    ├── todo.md            # This file
    ├── roadmap.md         # Project roadmap & milestones
    ├── scope.md           # Project scope
    ├── findings/          # Research findings
    │   ├── ld19-lidar-protocol.md   # LD19 protocol (NEW)
    │   └── jetson-llm-deployment-research.md
    └── archive/           # Session summaries
```

## Hardware Status (RPi)

| Device | Status | Notes |
|--------|--------|-------|
| EMEET SmartCam S600 | **Working** | /dev/video0, 640x480 capture verified |
| LiDAR (LD19) | **Working** | /dev/ttyUSB0, 230400 baud, ~497 pts/scan |
| GPIO | **Ready** | RPi.GPIO 0.7.2, gpiozero, lgpio all working |
| Motors | Not wired | L298N driver ready in code |

## Hardware Status (Jetson)

| Device | Status | Notes |
|--------|--------|-------|
| System | Pending | Ubuntu 22.04 (JetPack 6.1) being installed |

## Quick Commands

```bash
# Check device status
./deploy.sh --status

# Deploy to RPi
./deploy.sh rpi                     # All components
./deploy.sh rpi pathfinder          # Just LiDAR system
./deploy.sh rpi tests --test=lidar  # Deploy + run LiDAR test

# Run bootstrap (includes Docker)
./deploy.sh rpi --bootstrap

# SSH to RPi
ssh pi@10.33.224.1
```

## Notes

- **LiDAR is LD19** (YOUYEETOO/LDRobot), NOT RPLidar C1M1
  - Uses 230400 baud (not 460800)
  - One-way protocol (auto-streams, no commands)
  - See `docs/findings/ld19-lidar-protocol.md`
- **Platform-agnostic**: Components can run on Jetson or Raspberry Pi
- **RPi 3 limitation**: Only 906MB RAM - be mindful of memory usage
- **Docker ready**: Bootstrap includes Docker Compose v2.35.0 for RAG system

---

*Update every session: start by reading, end by updating.*
