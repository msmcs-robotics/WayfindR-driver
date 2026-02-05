# Ambot - Todo & Roadmap

> Last updated: 2026-02-05 (Session 7)

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

## Next Session Priorities

_Start here when resuming work_

1. **Test GUI diagnostics on RPi desktop** - Venv approach now active, ImageTk fixed
2. **Wire L298N motor driver** - See [locomotion/tmp_wiring.md](../locomotion/tmp_wiring.md) for pin table
3. **Test motors** - `./deploy.sh rpi --test=motors` once wired
4. **Create wandering_demo_llm.py** - LLM-integrated wandering (requires Jetson or API)

## In Progress

_Tasks actively being worked on_

- [ ] Wire L298N motor driver to RPi GPIO
  - **IMPORTANT: Connect common ground first** (L298N GND → RPi Pin 6, motor PSU GND → L298N GND)
  - See [locomotion/tmp_wiring.md](../locomotion/tmp_wiring.md) for full pin table
- [ ] Create `wandering_demo_llm.py` - LLM-integrated wandering with conversation

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
- [x] Combine pathfinder + locomotion for basic wandering demo — **Done** (`wandering_demo.py`)
- [ ] Test obstacle avoidance with actual robot movement (needs motors wired)

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

_Session 7 - 2026-02-05_

- [x] **Switched to .venv approach** - Clean Python virtual environment instead of system-wide pip hacks
  - Created `.venv` with `--system-site-packages` (accesses apt packages like RPi.GPIO, tkinter)
  - `run_tests.sh` auto-activates venv
  - `deploy.sh` activates venv for individual test commands, excludes `.venv` from rsync
  - `install.sh` creates venv and installs pip packages into it
  - `.gitignore` updated (both root and ambot)
- [x] **Fixed PIL ImageTk import error** - Installed `python3-pil.imagetk` (separate Debian apt package)
- [x] **Updated wiring documentation** - `locomotion/tmp_wiring.md` with proper markdown tables
  - Separated Motor A, Motor B, Power/Ground sections
  - Added notes about ENA/ENB jumpers, common ground, motor power
- [x] **All 12 tests passing on RPi** - Verified with venv activated

_Session 6 - 2026-02-05_

- [x] **Diagnosed SSH vs desktop terminal Python issue** - Packages in `~/.local/` not found from desktop
  - Root cause: `pip3 install --break-system-packages` (as user) installs to `~/.local/lib/python3.13/site-packages/`
  - RPi desktop (labwc/Wayland + lxterminal) may not load user site-packages
  - Fix: Switched to .venv approach (Session 7)
- [x] **Environment diagnostic tool** - `scripts/env_diagnostic.py`
  - Compares SSH vs desktop terminal environments
  - Shows package locations (user-local vs system vs system-local)
  - `--fix` flag installs packages system-wide
  - `--json` for automated comparison
- [x] **All 12 tests passing on RPi** (up from 10) - New environment precheck test added
  - Environment precheck now validates packages at runtime
  - All hardware tests still passing (camera, LiDAR, GPIO)
- [x] **Updated install.sh** - Venv-based package installation
  - Creates `.venv` with `--system-site-packages`
  - Installs pip packages into venv (no `--break-system-packages` needed)
  - Added `check_environment()` diagnostic to verification step
- [x] **Updated deploy.sh** - Added `env` and `env-fix` test types, venv activation
- [x] **Updated run_tests.sh** - Added environment precheck and venv activation

_Session 5 - 2026-02-05_

- [x] **All 10 tests passing on RPi** - Full hardware test suite verified
  - Camera: EMEET SmartCam S600 capturing + face detection (2 faces detected)
  - LiDAR: LD19 scanning ~481 pts/scan, 113mm nearest, 6902mm furthest
  - GPIO: All libraries available
  - Integration: All 5 sub-tests passing
- [x] **Fixed live_monitor.py JSON crash** - numpy int32 not JSON serializable
  - Added NumpySafeEncoder for OpenCV numpy types
  - Converted face center coords to native Python int
- [x] **Fixed run_tests.sh** - `set -e` + `((counter++))` killed script at first test
  - Removed `set -e` (wrong for test runner), fixed counter arithmetic
- [x] **Unified deploy.sh `--test=TYPE` interface** - One flag for all test types
  - Suites: `all`, `quick`, `hardware`, `integration`, `verify`
  - Individual: `gpio`, `camera`, `lidar`, `motors`
  - Diagnostics: `check` (verify installed packages)
  - Backwards compatible: `--verify`, `--full-test`, `--run` still work

_Session 4 - 2026-02-04_

- [x] **Live Monitor** - `live_monitor.py` terminal-based sensor monitoring (no GUI needed)
  - LiDAR status: connection, scan rate, nearest/furthest points, sectors
  - Camera status: connection, frame rate, face detection with centers
  - System status: CPU usage, RAM usage, GPU usage (Jetson), network status
  - Supports `--json` for programmatic output, `--log` for event logging
- [x] **GUI Camera with face centers** - Added center point crosshairs on detected faces
- [x] **GUI LiDAR with edge highlighting** - Red marker for nearest, blue for furthest
- [x] **Two wandering demo architecture planned** (see Design Decisions below)
- [x] **Comprehensive syntax verification** - `tests/verify_all_imports.py` checks all modules
  - 36 Python files: all syntax valid
  - Fixed missing `cleanup_gpio` export in rpi_motors
  - Graceful handling of GPIO-dependent modules on dev machines
- [x] **Network troubleshooting script** - `scripts/network-refresh.sh` for RPi connectivity issues
  - DHCP refresh, service restart, status diagnostics
  - Run ON the RPi when SSH connectivity is lost
- [x] **WSL2 SSH helper** - `scripts/wsl-ssh-helper.sh` bypasses WSL2 NAT limitations
  - Uses Windows SSH from WSL to reach LAN devices
  - Includes scan, test, and rsync functionality

_Session 3 - 2026-02-04_

- [x] **Wandering demo integration** - `wandering_demo.py` connects pathfinder + locomotion
- [x] **RobotAdapter** - Converts pathfinder float speeds (-1.0 to 1.0) → locomotion int speeds (-100 to 100)
- [x] **Integration test suite** - `tests/test_wandering_integration.py` (all 5 tests passing)
- [x] **LiDAR GUI updated** - Uses LD19 driver, added `--scans` limit for headless mode
- [x] **Camera GUI updated** - Added `--captures` limit for headless testing
- [x] **Comprehensive install script** - `install.sh` (idempotent, sudo-based, component selection)
- [x] **Face detection verified** - Camera GUI detected 1 face in test capture!
- [x] **LiDAR visualization** - Saved 3 polar plot images (~450 points/scan)

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
│   ├── rpi-bootstrap-python.sh   # Python libraries
│   ├── env_diagnostic.py  # Environment diagnostic (SSH vs desktop, package locations)
│   ├── network-refresh.sh # RPi network troubleshooting (run ON RPi)
│   └── wsl-ssh-helper.sh  # WSL2 SSH helper (run FROM dev machine)
├── deploy.sh              # Master deployment script
├── install.sh             # Comprehensive install script (sudo, idempotent)
├── run_tests.sh           # Comprehensive test runner (run ON target device)
├── live_monitor.py        # Terminal-based sensor + system monitoring
├── wandering_demo.py      # Basic wandering: Camera + LiDAR + Locomotion
├── tests/                 # Hardware test & diagnostic scripts
│   ├── run_all_tests.sh   # Run all tests
│   ├── test_gpio.py       # GPIO tests
│   ├── test_usb_camera.py # Camera tests
│   ├── test_ld19_lidar.py # LD19 LiDAR tests (all passing!)
│   ├── test_wandering_integration.py  # Pathfinder+Locomotion test
│   ├── gui_camera.py      # Camera GUI: basic feed (--no-faces) or face detection (--faces)
│   ├── gui_lidar.py       # LiDAR GUI: polar plot + nearest/furthest edges
│   ├── verify_all_imports.py  # Comprehensive syntax/import verification (NEW)
│   └── results/           # Test output (JSON, PNG, JPG)
└── docs/                  # Project documentation
    ├── todo.md            # This file
    ├── roadmap.md         # Project roadmap & milestones
    ├── scope.md           # Project scope
    ├── findings/          # Research findings
    │   ├── ld19-lidar-protocol.md
    │   ├── live-monitoring-architecture.md  # Live monitor design doc
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

# Deploy to RPi (from dev machine)
./deploy.sh rpi                     # All components
./deploy.sh rpi pathfinder          # Just LiDAR system
./deploy.sh rpi --test=all          # Deploy + run ALL tests
./deploy.sh rpi --test=quick        # Deploy + quick verification
./deploy.sh rpi --test=hardware     # Deploy + test connected hardware
./deploy.sh rpi --test=lidar        # Deploy + single LiDAR test
./deploy.sh rpi --test=check        # Deploy + check installed packages
./deploy.sh rpi --test=env          # Deploy + environment diagnostic
./deploy.sh rpi --test=env-fix      # Deploy + fix packages system-wide

# Install dependencies on RPi (run ON the RPi)
sudo ./install.sh                   # Full install (creates .venv + installs packages)
sudo ./install.sh --check           # Check what's installed
sudo ./install.sh --gui             # Include GUI packages (tkinter, ImageTk, matplotlib)

# Environment diagnostic (run ON RPi)
source .venv/bin/activate           # Activate venv first (or scripts do it automatically)
python3 scripts/env_diagnostic.py            # Full diagnostic
python3 scripts/env_diagnostic.py --json     # JSON for comparison

# Comprehensive test runner (run ON RPi)
./run_tests.sh                     # Standard tests (verification + integration)
./run_tests.sh --all               # ALL tests including hardware
./run_tests.sh --quick             # Quick syntax/import verification
./run_tests.sh --hardware          # Hardware tests (GPIO, camera, LiDAR)
./run_tests.sh --json              # JSON output for automation

# Verify all imports/syntax (manual)
python3 tests/verify_all_imports.py          # Quick check
python3 tests/verify_all_imports.py --verbose # Detailed output
python3 tests/verify_all_imports.py --json   # JSON for automation

# Live monitoring (terminal-based, works over SSH!)
python3 live_monitor.py             # Monitor all sensors + system
python3 live_monitor.py --lidar-only
python3 live_monitor.py --camera-only
python3 live_monitor.py --json      # JSON output for scripts

# Run wandering demo (simulation mode - no motors needed)
python3 wandering_demo.py --simulate
python3 wandering_demo.py --simulate --behavior wall_follower_right

# GUI diagnostics (run ON RPi with display)
python3 tests/gui_camera.py                # Live camera (basic feed)
python3 tests/gui_camera.py --faces        # Live camera + face detection + bounding boxes
python3 tests/gui_lidar.py                 # Live LiDAR polar plot + nearest/furthest

# Headless testing (via SSH)
python3 tests/gui_camera.py --headless --captures 3 --no-faces  # Basic capture
python3 tests/gui_camera.py --headless --captures 3 --faces     # Face detection
python3 tests/gui_lidar.py --headless --scans 3

# SSH to RPi
ssh pi@10.33.224.1

# WSL2 SSH Helper (when SSH fails from WSL due to NAT)
./scripts/wsl-ssh-helper.sh              # SSH using Windows SSH
./scripts/wsl-ssh-helper.sh --check      # Check if known IP is reachable
./scripts/wsl-ssh-helper.sh --test       # Test connectivity
./scripts/wsl-ssh-helper.sh --rsync      # Sync ambot folder to RPi
./scripts/wsl-ssh-helper.sh --ip 10.33.171.167  # Use specific IP
```

## Design Decisions

### Two Wandering Demo Versions

The system will have two wandering demo variants for different development stages:

| Demo | File | Components | Purpose |
|------|------|------------|---------|
| **Basic** | `wandering_demo.py` | Camera + LiDAR + Locomotion | Sensor testing, obstacle avoidance, no AI |
| **LLM-Integrated** | `wandering_demo_llm.py` | All + Bootylicious (LLM/RAG) | Full robot with conversation capability |

**Rationale**: Separating these allows:
1. Clear iteration stages during development
2. Testing sensors/motors without Jetson/LLM dependencies
3. Basic demo can run on RPi alone
4. LLM demo requires Jetson (or powerful RPi 5) for inference

### Platform Portability

Scripts should work on both Raspberry Pi and Jetson with minimal changes:

| Script | RPi | Jetson | Notes |
|--------|-----|--------|-------|
| `live_monitor.py` | ✅ | ✅ | GPU stats only show on Jetson |
| `wandering_demo.py` | ✅ | ✅ | Same behavior, different GPIO lib |
| `wandering_demo_llm.py` | ⚠️ | ✅ | Requires LLM (Jetson preferred) |
| `install.sh` | ✅ | - | RPi-specific packages |
| `install-jetson.sh` (future) | - | ✅ | Jetson-specific packages |

**Key differences handled by platform detection:**
- GPIO: `RPi.GPIO` (RPi) vs `Jetson.GPIO` (Jetson)
- GPU monitoring: `/sys/devices/17000000.ga10b/load` (Jetson only)
- LLM: Ollama/nanoLLM on Jetson, optional fallback to API on RPi

## Notes

- **Python packages use .venv** inside `ambot/` folder
  - Venv created with `--system-site-packages` (sees apt packages: RPi.GPIO, tkinter, PIL.ImageTk)
  - Pip packages installed into `.venv/` (pyserial, numpy, matplotlib, opencv-python-headless)
  - Scripts auto-activate venv (`run_tests.sh`, `deploy.sh` individual commands)
  - `.venv/` excluded from rsync and git
  - Use `./deploy.sh rpi --test=env` to diagnose environment
- **LiDAR is LD19** (YOUYEETOO/LDRobot), NOT RPLidar C1M1
  - Uses 230400 baud (not 460800)
  - One-way protocol (auto-streams, no commands)
  - See `docs/findings/ld19-lidar-protocol.md`
- **Platform-agnostic**: Components can run on Jetson or Raspberry Pi
- **RPi 3 limitation**: Only 906MB RAM - be mindful of memory usage
- **Docker ready**: Bootstrap includes Docker Compose v2.35.0 for RAG system

---

*Update every session: start by reading, end by updating.*
