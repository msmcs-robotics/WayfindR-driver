# AMBOT - Autonomous Mobile Robot Platform

An autonomous conversational robot with three independent components that can be deployed to Jetson Orin Nano or Raspberry Pi.

## Components

| Component | Folder | Purpose | Current Status |
|-----------|--------|---------|----------------|
| **pathfinder** | `pathfinder/` | LiDAR sensing & obstacle avoidance | ✅ Working on RPi |
| **locomotion** | `locomotion/` | Motor control (differential drive) | ✅ Code ready, needs wiring |
| **bootylicious** | `bootylicious/` | LLM + RAG (conversation brain) | ⏳ Pending Jetson setup |

## Quick Start

### 1. Deploy to Raspberry Pi (from development machine)

```bash
# Check device status
./deploy.sh --status

# Deploy all components
./deploy.sh rpi

# Deploy specific component
./deploy.sh rpi pathfinder    # LiDAR system
./deploy.sh rpi locomotion    # Motor control
./deploy.sh rpi tests         # Test scripts
```

### 2. Install Dependencies (on Raspberry Pi)

```bash
# SSH to RPi
ssh pi@10.33.224.1

# Check what's already installed
sudo ./install.sh --check

# Full install (pathfinder + locomotion)
sudo ./install.sh

# Include GUI packages (matplotlib, tkinter, opencv-data)
sudo ./install.sh --gui

# Include Docker for bootylicious
sudo ./install.sh --docker

# Install everything
sudo ./install.sh --all
```

### 3. Run Tests

```bash
# On RPi - run integration tests
python3 tests/test_wandering_integration.py

# LiDAR test
python3 tests/test_ld19_lidar.py

# Camera test
python3 tests/test_usb_camera.py

# GPIO test
python3 tests/test_gpio.py
```

### 4. Run Wandering Demo

```bash
# Simulation mode (no motors needed)
python3 wandering_demo.py --simulate

# Different behaviors
python3 wandering_demo.py --simulate --behavior max_clearance
python3 wandering_demo.py --simulate --behavior wall_follower_right
python3 wandering_demo.py --simulate --behavior random_wander

# Real mode (requires motors wired)
python3 wandering_demo.py
```

### 5. GUI Diagnostics (on RPi with display)

```bash
# Live camera feed with face detection
python3 tests/gui_camera.py

# Live LiDAR polar plot
python3 tests/gui_lidar.py
```

### 6. Headless Testing (via SSH)

```bash
# Camera test - capture 3 images
python3 tests/gui_camera.py --headless --captures 3

# LiDAR test - save 3 scan images
python3 tests/gui_lidar.py --headless --scans 3
```

## Folder Structure

```
ambot/
├── bootylicious/          # LLM + RAG system (Jetson)
│   ├── deploy.sh
│   ├── scripts/           # Ollama, Docker, HuggingFace setup
│   └── rag/               # RAG Docker Compose
├── locomotion/            # Motor control (RPi/Jetson)
│   ├── rpi_motors/        # Modular motor drivers
│   │   ├── config.py      # Pin configurations (TB6612, L298N, DRV8833)
│   │   ├── drivers.py     # Driver implementations
│   │   └── factory.py     # create_robot() factory
│   └── docs/              # Wiring guides
├── pathfinder/            # Sensor fusion (RPi)
│   ├── config.py          # LiDAR config (LD19/C1M1)
│   ├── lidar_ld19.py      # LD19 driver (working!)
│   ├── behaviors.py       # Wandering behaviors
│   └── obstacle_detector.py
├── scripts/               # Bootstrap scripts
│   ├── rpi-bootstrap.sh
│   ├── rpi-bootstrap-system.sh
│   └── rpi-bootstrap-python.sh
├── tests/                 # Test & diagnostic scripts
│   ├── test_*.py          # Automated tests
│   ├── gui_*.py           # Visual diagnostics
│   └── results/           # Test output (gitignored)
├── docs/                  # Documentation
│   ├── todo.md            # Current tasks
│   ├── roadmap.md         # Project milestones
│   ├── scope.md           # Project boundaries
│   ├── findings/          # Research notes
│   └── archive/           # Session summaries
├── deploy.sh              # Master deployment script
├── install.sh             # Dependency installer (idempotent)
├── wandering_demo.py      # Pathfinder + Locomotion integration
└── connections.md         # Device credentials (keep private!)
```

## Hardware

### Raspberry Pi 3 (pi@10.33.224.1)

| Device | Port | Status | Notes |
|--------|------|--------|-------|
| LiDAR (LD19) | /dev/ttyUSB0 | ✅ Working | 230400 baud, ~450 pts/scan |
| Camera (EMEET S600) | /dev/video0 | ✅ Working | 640x480, face detection OK |
| Motors (L298N) | GPIO | ⏳ Not wired | See `locomotion/docs/l298n-driver-wiring-guide.md` |

### Jetson Orin Nano (pending setup)

| Device | Status | Notes |
|--------|--------|-------|
| System | ⏳ Installing | Ubuntu 22.04 / JetPack 6.1 |

## Install Script Options

```bash
sudo ./install.sh [OPTIONS]

Options:
  --check        Check what's installed (no changes)
  --pathfinder   Only pathfinder dependencies
  --locomotion   Only locomotion dependencies
  --gui          Include GUI packages (matplotlib, tkinter)
  --docker       Include Docker for RAG system
  --all          Install everything
  -v, --verbose  Verbose output
  -h, --help     Show help
```

### What It Installs

**System Packages (apt):**
- python3, python3-pip, python3-dev
- python3-serial (pyserial)
- python3-rpi.gpio, python3-gpiozero
- i2c-tools, python3-smbus
- opencv packages (with --gui)
- Docker (with --docker)

**Python Packages (pip):**
- pyserial, numpy
- gpiozero
- matplotlib, opencv-python-headless (with --gui)

**System Configuration:**
- Udev rules for serial devices
- User groups: dialout, gpio, i2c, docker

## Deploy Script Options

```bash
./deploy.sh [TARGET] [COMPONENT] [OPTIONS]

Targets:
  rpi              Raspberry Pi
  jetson           Jetson Orin Nano (when configured)

Components:
  pathfinder       LiDAR system
  locomotion       Motor control
  tests            Test scripts
  (none)           All components

Options:
  --status         Check device connectivity
  --bootstrap      Run bootstrap scripts after deploy
  --test=NAME      Run specific test after deploy
```

## Behaviors Available

| Behavior | Description | Use Case |
|----------|-------------|----------|
| `safe_wanderer` | MaxClearance + dynamic obstacle safety | Default, safe for demos |
| `max_clearance` | Move toward longest distance | Open spaces |
| `wall_follower_right` | Follow wall on right side | Room perimeter |
| `wall_follower_left` | Follow wall on left side | Room perimeter |
| `random_wander` | Random exploration | General exploration |
| `avoid_and_go` | Simple reactive avoidance | Basic navigation |

## Architecture

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

## Design Principles

1. **Graceful Degradation**: Optional sensors (MPU6050, camera) enhance but don't block functionality
2. **Sensor Fusion in Pathfinder**: All perception sensors live here, locomotion just executes
3. **Event-Driven LLM Triggers**: Camera facial detection can trigger LLM responses
4. **Platform Agnostic**: Components run on RPi or Jetson as available
5. **Idempotent Scripts**: All deploy/install scripts safe to run repeatedly

## Development

See [docs/todo.md](docs/todo.md) for current tasks and [docs/roadmap.md](docs/roadmap.md) for project milestones.

### Session Summaries
- [2026-02-04 Session 3](docs/archive/2026-02-04-session-3-summary.md) - Integration, GUI updates
- [2026-02-03 Session 2](docs/archive/2026-02-03-session-2-summary.md) - LD19 driver
- [2026-02-03 Session 1](docs/archive/2026-02-03-session-summary.md) - Initial setup

## Known Limitations

- **RPi 3**: Only 906MB RAM - be mindful of memory usage
- **LD19 LiDAR**: One-way protocol, auto-streams (no commands)
- **No SLAM**: Current system is reactive wandering only
