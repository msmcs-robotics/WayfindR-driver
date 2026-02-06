# Ambot - Roadmap

> Last updated: 2026-02-06

## Overview

This roadmap tracks project-level features and milestones. For immediate tasks, see `todo.md`.

**Note**: No time estimates. Focus on WHAT needs to be done, not WHEN.

**Project Components:**
| Component | Folder | Purpose |
|-----------|--------|---------|
| **bootylicious** | `ambot/bootylicious/` | LLM + RAG system (conversation brain) |
| **locomotion** | `ambot/locomotion/` | Motor control (differential drive) |
| **pathfinder** | `ambot/pathfinder/` | LiDAR wandering & obstacle avoidance (NO SLAM) |

*Each component can be deployed to Jetson or Raspberry Pi as needed.*

**Goal Horizons:**
- **Short-term**: System inventory and basic subsystem setup
- **Mid-term (First Stable Release)**: All three components independently functional and testable
- **Long-term**: Integrated robot with conversation + navigation working together

---

## Milestone 1: System Inventory & Environment Setup

### Jetson Orin Nano Setup
- [ ] Resolve "no space left on device" issue (user handling, then verify)
- [ ] Audit disk usage: Docker images, unused packages, logs, cache
- [ ] SSH into Jetson and inventory installed packages/toolkits
- [ ] Document GPU, RAM, storage, and OS version
- [ ] Document installed NVIDIA toolkits and versions
- [ ] Debug and fix Docker installation issues
- [ ] Verify Docker Compose works with a test container
- [ ] Document all findings in `docs/findings/`

### Raspberry Pi Setup
- [x] Obtain Pi IP address, username, and password -- Completed 2026-02-03
- [x] SSH into Pi and inventory system specs -- Completed 2026-02-03
- [x] Document OS version, available resources -- Debian 13, 906MB RAM
- [x] Verify LiDAR sensor connectivity -- LD19 working, 230400 baud
- [x] Document all findings in `docs/findings/` -- Completed

### Development Environment
- [x] Set up SSH key-based access to both devices -- RPi done, Jetson pending
- [x] Document SSH connection procedures -- In `connections.md` and `docs/todo.md`
- [x] Verify network connectivity between devices -- RPi: 10.33.224.1

---

## Milestone 2: Jetson Nano - LLM Subsystem

### LLM Inference
- [ ] Install/verify nanoLLM framework on Jetson
- [ ] Test loading TinyLlama-1.1B via nanoLLM
- [ ] Test loading Phi-2 via nanoLLM
- [ ] Benchmark GPU memory usage and inference speed
- [ ] Select primary model based on benchmarks
- [ ] Create basic conversation loop (text in -> text out)

### RAG System
- [ ] Adapt rag-bootstrap Docker Compose for Jetson (ARM64)
  - Dependencies: Docker working on Jetson
- [ ] Deploy PostgreSQL + pgvector container
- [ ] Deploy Redis container
- [ ] Deploy FastAPI RAG API container
- [ ] Test document ingestion pipeline
- [ ] Test search (semantic, keyword, hybrid)
- [ ] Integrate RAG search results into LLM prompt context
- [ ] Benchmark resource usage (CPU/RAM) of RAG stack alongside LLM

### Visual System (Person Detection)
- [ ] Connect USB camera to Jetson
- [ ] Test basic camera feed capture
- [ ] Implement person/face detection (OpenCV or similar)
- [ ] Create trigger logic: detected person -> initiate conversation
- [ ] Test detection reliability and false positive rate

### Text Display Output
- [ ] Research display options compatible with Jetson
- [ ] Connect and test display
- [ ] Implement text output rendering for conversation responses

---

## Milestone 3: Locomotion & Pathfinder Components

### Locomotion (`ambot/locomotion/`)
- [x] Create motor control module for YAHBOOM G1 (TB6612FNG) -- Completed 2026-01-29
- [x] Implement Motor and DifferentialDrive classes -- Completed 2026-01-29
- [x] Create deploy.sh with diagnostics -- Completed 2026-01-29
- [x] Platform auto-detection (Jetson.GPIO vs RPi.GPIO) -- Completed 2026-01-29
- [ ] Test on actual Jetson hardware with motors connected
- [ ] Test on Raspberry Pi hardware
- [ ] Implement PID speed control (future enhancement)

### Pathfinder (`ambot/pathfinder/`)
> **Note**: Pathfinder is for simple demo wandering, NOT SLAM. The robot should wander naturally around a room avoiding obstacles using reactive algorithms. This is a precursor to SLAM — when ready, we'll add ROS2.

- [x] Create LiDAR module for RPLidar C1M1 -- Completed 2026-01-29
- [x] Implement sector-based obstacle detection -- Completed 2026-01-29
- [x] Create deploy.sh with diagnostics -- Completed 2026-01-29
- [x] Create udev rules setup script -- Completed 2026-01-29
- [x] Implement basic behaviors (MaxClearance, WallFollower, RandomWander, AvoidAndGo) -- Completed 2026-01-29
- [x] Create BehaviorRunner for behavior loop execution -- Completed 2026-01-29
- [x] Add dynamic obstacle detection (DynamicObstacleMonitor, SafetyWrapper) -- Completed 2026-01-29
- [x] Add create_safe_wanderer() for safe operation around people -- Completed 2026-01-29
- [x] Test with actual LiDAR sensor -- LD19 working (not C1M1 as originally thought)
- [x] Create visualization tool for debugging -- `tests/gui_lidar.py` with polar plot
- [x] Integrate with locomotion for wandering demos -- `wandering_demo_1.py`, `wandering_demo_2.py`
- [x] L298N motors wired and tested -- Forward/reverse/turn/arc at 30% -- 2026-02-06
- [x] Camera face tracking demo -- WANDERING ↔ TRACKING state machine -- 2026-02-06
- [x] **NaturalWanderBehavior** -- Top-N clearance cycling for natural wandering -- 2026-02-06
  - Bins raw scan into 36 angular buckets (10° each) for 360° clearance profile
  - Picks top 10 clearance targets, each ≥30° apart, cycles through them
  - Avoids ping-ponging by not always going to the single longest distance
  - Time-based target switching (5s per target) + safety zone fallback
  - Now the default behavior for `safe_wanderer` and both demos
- [x] **PWM cleanup fix** — Set `_pwm = None` after `stop()` to prevent TypeError warnings -- 2026-02-06
- [x] **Motor test wrapper** — `tests/test_motors.py` with L298N defaults + deploy.sh integration -- 2026-02-06
- [x] **Wandering behavior visualization** — `tests/gui_wandering.py` shows NaturalWander decisions on live LiDAR -- 2026-02-06
- [x] **Localization research** — Documented in `docs/findings/localization-pre-slam.md` -- 2026-02-06
  - Reactive wandering does NOT need position tracking (the environment IS the memory)
  - Recommended next: MPU6050 gyro heading (Level 1) for closed-loop turns
  - Do NOT attempt accelerometer position (drifts 17m in 1 minute)
- [x] **Fixed NaturalWanderBehavior target cycling** — Separated "advance" from "rebuild queue" logic -- 2026-02-06
- [x] **MPU6050 IMU driver** (`pathfinder/imu.py`) — Gyro heading, calibration, complementary filter -- 2026-02-06
  - Wiring guide: `docs/findings/mpu6050-wiring.md`
  - Hardware test: `tests/test_imu.py`
- [x] **demos_common/ module** — Shared sensor wrappers (LiDAR, camera, IMU, behaviors) -- 2026-02-06
  - Both wandering demos refactored to import from `demos_common`
  - Reduced code duplication significantly
- [x] **install.sh MPU6050 deps** — Added smbus2 pip package, verified on RPi -- 2026-02-06
- [x] **Quick-reference pin tables** — Added to all wiring guides (L298N, TB6612FNG, DRV8833) -- 2026-02-06
- [ ] Fix left motor wiring (right works, left doesn't spin — ENA Pin 33 / terminal screws)
- [ ] Tune safety zone distances for robot size
- [ ] Test wandering behaviors in real environment
- [ ] Test face tracking with real hardware

#### Wandering Behavior Philosophy (Pre-SLAM)

The robot should look like it's **naturally wandering** — not bouncing between two open areas. The approach:

1. **Fine-grained clearance mapping**: Use raw LiDAR data (~467 points/scan) to build a 360° clearance profile at 10° resolution
2. **Multi-target selection**: Pick top N (10) clearance directions, enforcing minimum angular separation (30°) so the robot has diverse directions to explore
3. **Sequential cycling**: Visit targets in order (longest → 2nd → 3rd → ...) rather than always chasing the absolute max
4. **Reactive safety**: Safety zones (STOP/SLOW/WARN) and DynamicObstacleMonitor still override everything

This creates the illusion of purposeful exploration without needing SLAM, odometry, or waypoints. When SLAM is eventually added (via ROS2), these behaviors can be replaced with proper path planning.

#### Demo Progression

| Demo | File | Behavior | Camera |
|------|------|----------|--------|
| **1** | `wandering_demo_1.py` | NaturalWander + SafetyWrapper | No |
| **2** | `wandering_demo_2.py` | NaturalWander + face tracking interrupt | Yes |
| **3** | `wandering_demo_3.py` (future) | LLM-driven behavior selection | Yes |

---

## Milestone 4: Integration & Communication

- [ ] Define communication protocol between Jetson and Pi (REST API / MQTT / serial)
- [ ] Implement Jetson -> Pi movement commands
- [ ] Implement Pi -> Jetson sensor status updates
- [ ] Test end-to-end: person detected -> conversation + navigation
- [ ] Integration testing with both systems running

---

## Milestone 5: Future Vision (Long-Term)

> These are aspirational goals for the full robot system. Not blockers for initial release.

### Advanced LLM Capabilities
- [ ] Scale to 8-12B parameter LLM on Jetson Orin Nano
  - Research indicates 8-12B models can run on Jetson with optimizations
  - Larger models provide better conversation quality and fewer hallucinations
  - Target: Run entirely on-device (no cloud dependency)
- [ ] Implement MCP server (Python FastMCP) for LLM-to-locomotion control
  - Enable LLM to control robot movements through MCP tools
  - Example: "move forward", "turn left", "stop" as MCP functions
  - Rudimentary control without ROS2 complexity
- [ ] LLM-driven behavior selection
  - LLM can choose between different pathfinder behaviors
  - Context-aware movement (e.g., "follow the person", "go to charging station")

### Sensor Fusion (see `docs/findings/localization-pre-slam.md`)
- [x] **MPU6050 IMU driver** — gyro heading for closed-loop turns (Level 1 localization) -- Completed 2026-02-06
  - `pathfinder/imu.py`: I2C reads at ~100Hz, startup calibration, complementary filter for pitch/roll
  - Graceful degradation: `setup_imu()` returns None if sensor not found
  - Hardware test: `tests/test_imu.py`, deploy: `./deploy.sh rpi --test=imu`
- [ ] Modify NaturalWanderBehavior to use heading-error-based turns
- [ ] Camera-based facial detection events triggering LLM
  - Detect specific faces → personalized greetings
  - Detect crowd → different behavior than single person
- [ ] LiDAR scan matching / ICP (Level 2 — only when position tracking needed)
- [ ] Combined LiDAR + camera + IMU for robust navigation
  - Still NO SLAM - just better reactive navigation

### Voice Interaction
- [ ] Speech-to-text input (possibly via Android device)
- [ ] Text-to-speech output (possibly via Android device)
- [ ] Continuous microphone listening for voice commands
- [ ] Wake word detection ("Hey Ambot")

### ROS2 Migration (When Ready for SLAM)
- [ ] ROS2 integration for SLAM and advanced navigation
  - Description: Once basic Python system is proven, introduce ROS2 for SLAM mapping
  - Dependencies: Basic Python LiDAR system working first
  - Reference: WayfindR-driver `ros2_comprehensive_attempt/` has ROS2 Humble stack

### Other Enhancements
- [ ] Port LiDAR/movement system from Python to C
- [ ] Multiple display modes (conversation, status, map)
- [ ] Leverage optimized LLMs from external project
- [ ] Multi-person conversation handling
- [ ] Persistent conversation memory across sessions

---

## Completed

> Features moved here when done, for historical reference.

### Project Setup
- [x] Create ambot project structure -- Completed 2026-01-27
- [x] Draft scope, roadmap, and documentation -- Completed 2026-01-27
- [x] Reorganize into three components (bootylicious, locomotion, pathfinder) -- Completed 2026-01-29

### Bootylicious (LLM/RAG)
- [x] Create Docker Compose for RAG system (PostgreSQL + pgvector + Redis + FastAPI) -- Completed 2026-01-29
- [x] Add dual LLM backend support (Ollama + HuggingFace) -- Completed 2026-01-29
- [x] Create deploy.sh with diagnostics -- Completed 2026-01-29
- [x] Create rsync-to-jetson.sh for deployment -- Completed 2026-01-29
- [x] Create system tests (test-system.sh) -- Completed 2026-01-29

### Locomotion (Motor Control)
- [x] Create YAHBOOM G1 motor driver module -- Completed 2026-01-29
- [x] Implement Motor and DifferentialDrive classes -- Completed 2026-01-29
- [x] Create deploy.sh with GPIO diagnostics -- Completed 2026-01-29

### Pathfinder (LiDAR Wandering)
- [x] Create RPLidar C1M1 module -- Completed 2026-01-29
- [x] Implement sector-based obstacle detection -- Completed 2026-01-29
- [x] Create deploy.sh with device diagnostics -- Completed 2026-01-29
- [x] Implement wandering behaviors (MaxClearance, WallFollower, RandomWander, AvoidAndGo, NaturalWander) -- Completed 2026-01-29, updated 2026-02-06
- [x] Create BehaviorRunner for autonomous behavior execution -- Completed 2026-01-29
- [x] Research and document wandering algorithms -- Completed 2026-01-29
- [x] Add dynamic obstacle detection for people walking by (DynamicObstacleMonitor) -- Completed 2026-01-29
- [x] Add SafetyWrapper and create_safe_wanderer() for safe demos -- Completed 2026-01-29
- [x] Create LD19 LiDAR driver -- Completed 2026-02-03 (LD19, not C1M1!)
- [x] Create LiDAR visualization GUI -- Completed 2026-02-04
- [x] Create wandering behavior visualization GUI (`tests/gui_wandering.py`) -- Completed 2026-02-06
- [x] PWM cleanup fix (set `_pwm = None` after `stop()`) -- Completed 2026-02-06
- [x] Motor test convenience wrapper (`tests/test_motors.py`) -- Completed 2026-02-06
- [x] Localization research document (`docs/findings/localization-pre-slam.md`) -- Completed 2026-02-06

### Integration & Deployment
- [x] Create master deploy.sh script -- Completed 2026-02-03
- [x] Create comprehensive install.sh script (idempotent) -- Completed 2026-02-04
- [x] Create wandering demos (pathfinder + locomotion) -- Completed 2026-02-04, renamed 2026-02-06
- [x] Create integration test suite -- Completed 2026-02-04
- [x] Create README.md for ambot folder -- Completed 2026-02-04

### Automation & Tooling -- Completed 2026-02-04, Updated 2026-02-05
- [x] **Python virtual environment (.venv)** -- Added 2026-02-05
  - `.venv` created with `--system-site-packages` (accesses apt packages)
  - All scripts auto-activate venv before running Python
  - Excluded from rsync and git
  - Solves SSH vs desktop terminal package visibility issues
- [x] **Comprehensive test runner** (`run_tests.sh`)
  - Runs all tests automatically without manual SSH/curl commands
  - Test suites: `--quick`, `--all`, `--hardware`, `--integration`
  - JSON output for automation: `--json`
  - Auto-detects platform and connected hardware
  - Environment precheck validates package availability at runtime
  - Auto-activates `.venv` at startup
- [x] **Deploy script enhancements** (`deploy.sh`)
  - Unified `--test=TYPE`: suites (all/quick/hardware/integration/verify), individual (gpio/camera/lidar/motors), diagnostics (check/env/env-fix)
  - One command for deploy + test: `./deploy.sh rpi --test=all`
  - Environment diagnostic: `./deploy.sh rpi --test=env`
  - Excludes `.venv/` from rsync sync
  - Activates venv for individual test commands via SSH
- [x] **Install script** (`install.sh`)
  - Creates `.venv` with `--system-site-packages`
  - Installs pip packages into venv (no `--break-system-packages` needed)
  - Installs apt system packages (RPi.GPIO, tkinter, PIL.ImageTk)
  - Idempotent, component-selectable (`--pathfinder`, `--locomotion`, `--gui`)
- [x] **Environment diagnostic** (`scripts/env_diagnostic.py`) -- Added 2026-02-05
  - Diagnoses SSH vs desktop terminal Python environment differences
  - Shows package locations (user-local vs system-wide)
  - `--json` for automated comparison between environments
- [x] **Syntax/import verification** (`tests/verify_all_imports.py`)
  - Checks all Python files for syntax errors
  - Tests all module imports with GPIO graceful degradation
  - JSON output for CI/CD integration
- [x] **WSL SSH helper** (`scripts/wsl-ssh-helper.sh`)
  - Bypasses WSL2 NAT networking issues
  - Commands: `--check`, `--test`, `--rsync`
- [x] **Network troubleshooting** (`scripts/network-refresh.sh`)
  - DHCP refresh, service restart, status diagnostics
  - Run ON the RPi when SSH connectivity is lost

---

## Notes

- **Platform-agnostic**: Each component can run on Jetson or Raspberry Pi as needed
- **Three independent components**: bootylicious (brain), locomotion (motors), pathfinder (sensing)
- Developer teams can work independently on their respective components
- Python-first approach; C port is a future optimization milestone
- **Development philosophy: get basic system working first, then iterate** (no ROS2 initially, add it later for SLAM)
- External projects will feed into Ambot: LLM optimization project, audio/voice project
- First stable release = all three components working independently (not necessarily integrated)
- ROS2 is a planned upgrade path, not excluded - just not a dependency for the initial working system
- **Automation philosophy**: Avoid manual SSH/curl commands - use `deploy.sh` and `run_tests.sh` scripts
  - Deploy + test from dev machine: `./deploy.sh rpi --test=all`
  - Quick verification: `./deploy.sh rpi --test=quick`
  - Run locally on RPi: `./run_tests.sh --all`

---

*Update as features complete. Check boxes when done. Add new features as they're identified.*
