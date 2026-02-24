# Ambot - Roadmap

> Last updated: 2026-02-19

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
- [x] ~~Resolve "no space left on device" issue~~ — Not applicable, 86 GB free -- 2026-02-12
- [x] SSH into Jetson and inventory installed packages/toolkits -- 2026-02-12
- [x] Document GPU, RAM, storage, and OS version -- JetPack R36.4.4, 7.4 GiB, CUDA 12.6 -- 2026-02-12
- [x] Document installed NVIDIA toolkits and versions -- Driver 540.4.0, Container Toolkit 1.16.2 -- 2026-02-12
- [x] Docker working (28.2.2), Compose (v2.36.2), NVIDIA GPU runtime verified -- 2026-02-12
- [x] Document all findings in `bootylicious/docs/findings/jetson-system-inventory.md` -- 2026-02-12
- [x] Install Ollama 0.6.2 and pull tinyllama model (637 MB) -- 2026-02-12
- [x] Start and test RAG Docker stack (PostgreSQL + Redis + FastAPI all healthy) -- 2026-02-12
- [x] Ollama configured for Docker access (`OLLAMA_HOST=0.0.0.0`, bridge gateway URL) -- 2026-02-12
- [x] Ingest test documents into RAG knowledge base (3 docs, 3 chunks — search + ask verified) -- 2026-02-12
- [x] Pulled llama3.2:3b model (2.0 GB, better quality than tinyllama) -- 2026-02-12

### Raspberry Pi Setup
- [x] Obtain Pi IP address, username, and password -- Completed 2026-02-03
- [x] SSH into Pi and inventory system specs -- Completed 2026-02-03
- [x] Document OS version, available resources -- Debian 13, 906MB RAM
- [x] Verify LiDAR sensor connectivity -- LD19 working, 230400 baud
- [x] Document all findings in `docs/findings/` -- Completed

### Development Environment
- [x] Set up SSH key-based access to both devices -- RPi done, Jetson done (id_git) -- 2026-02-12
- [x] Document SSH connection procedures -- `connections.md`, SSH config (`ssh jetson` / `ssh rpi`)
- [x] Verify network connectivity between devices -- RPi: 10.33.224.1, Jetson: 10.33.255.82
- [x] NOPASSWD sudo configured on Jetson -- 2026-02-12

---

## Milestone 2: Jetson Nano - LLM Subsystem

### LLM Inference
- [x] ~~nanoLLM framework~~ — Using Ollama instead (simpler, model management built-in) -- 2026-02-12
- [x] Tested TinyLlama 1.1B — works but hallucinated heavily, ignored context -- 2026-02-12
- [x] Tested llama3.2:3b — accurately answers from retrieved context, selected as primary -- 2026-02-17
- [x] Benchmarked resource usage: 5.5 GiB used (Docker 148 MiB + Ollama ~2 GB), 1.9 GiB headroom -- 2026-02-17
- [x] Added `num_ctx` (4096) and `num_predict` (512) to Ollama config and API calls -- 2026-02-19
  - Default `num_predict=128` was truncating RAG answers with citations
  - Documented thinking model gotcha (qwen3/deepseek-r1 consume num_predict in hidden `<think>` block)
- [x] Created LLM configuration guide: `docs/findings/llm-configuration-guide.md` -- 2026-02-19
- [x] Created edge LLM research doc: `docs/findings/edge-llm-research.md` -- 2026-02-19
  - Model selection, quantization, context management, deployment patterns
  - Findings from `~/exudeai/` and web research
- [x] Created model comparison test script: `scripts/test-llm-models.sh` -- 2026-02-19
- [x] Tested additional models on Jetson: phi3:mini, gemma2:2b, smollm2:1.7b -- 2026-02-19
  - gemma2:2b fastest (27s, 2.4 GiB), phi3:mini slowest (50s, 3.7 GiB)
  - smollm2:1.7b hallucinated — not reliable for RAG
  - llama3.2:3b confirmed as best choice for RAG quality
- [x] Applied Ollama memory optimizations (flash attention, q8 KV cache, single model/parallel) -- 2026-02-19
- [ ] Create basic conversation loop (text in -> text out)

### RAG System
- [x] Adapt rag-bootstrap Docker Compose for Jetson (ARM64) -- 2026-02-12
- [x] Deploy PostgreSQL + pgvector container (512 MB limit) -- 2026-02-12
- [x] Deploy Redis container (256 MB limit) -- 2026-02-12
- [x] Deploy FastAPI RAG API container (sentence-transformers + Ollama backend) -- 2026-02-12
- [x] Test document ingestion pipeline (3 docs, 3 chunks) -- 2026-02-12
- [x] Test search (semantic, keyword, hybrid) -- 2026-02-12
- [x] Test RAG ask pipeline (search + LLM generation with sources) -- 2026-02-12
- [x] Benchmarked resource usage: Docker 148 MiB total, 1.9 GiB headroom -- 2026-02-17

### RAG Optimization (adapted from rag-atc-testing)
> See `bootylicious/docs/findings/rag-optimization-review.md` for full analysis

#### Phase 1: Quick Wins
- [x] Switch default LLM to llama3.2:3b -- 2026-02-17
- [x] Reduce DB connection pool (20→5, overflow 10→5) + pool_pre_ping + pool_recycle -- 2026-02-17
- [x] Switch Redis eviction policy (LRU→LFU) -- 2026-02-17
- [x] Increase hybrid search fetch multiplier (3x→5x) -- 2026-02-17

#### Phase 2: Resilience
- [x] Add junk chunk filtering (>25% digits, >10% dot leaders) -- 2026-02-19
- [x] Add text normalization for embeddings (unicode, NFKC, 800-char cap) -- 2026-02-19
- [x] Add embedding retry with progressive truncation (8 attempts, truncate to 600 chars at halfway) -- 2026-02-19
- [x] Add inter-request cooldown (1.5s) for Ollama embedding calls -- 2026-02-19
- [x] Add batch commit + resume for ingestion pipeline (25 chunks per batch) -- 2026-02-19
- [x] Deploy Phase 2 to Jetson and verify (Docker rebuild, health check, RAG ask test) -- 2026-02-19

#### Phase 3: Search Quality
- [ ] Evaluate nomic-embed-text (768-dim, 8192-token context) vs current MiniLM
- [x] Add database indexes (IVFFlat vector + GIN full-text, lists=10) -- 2026-02-24
- [x] Implement dual keyword search (English stemmed AND + Simple exact OR, 20% overlap boost) -- 2026-02-24
- [x] Add adaptive semantic weight (acronym→0.2, mixed→0.3, short→0.5, long→0.7) -- 2026-02-24
- [ ] Create domain-specific acronym expansion table

### GPU & Hardware Acceleration
> See `bootylicious/docs/findings/jetson-gpu-acceleration.md` for full analysis

- [x] GPU spec documented: Orin (Compute 8.7, Ampere), 8 SMs, 7620 MiB unified memory -- 2026-02-17
- [x] Ollama GPU acceleration verified: CUDA0 buffers (model 1918 MiB, KV 448 MiB, compute 256 MiB) -- 2026-02-17
- [x] CUDA 12.6 fully functional: compiled + ran test kernel on GPU -- 2026-02-17
- [x] cuDNN 9.3.0 + TensorRT 10.3.0 pre-installed via JetPack -- 2026-02-17
- [x] Created `scripts/setup-cuda.sh` — detection, verification, PATH fix, test compilation -- 2026-02-17
- [x] Fixed nvcc PATH via `/etc/profile.d/cuda-path.sh` -- 2026-02-17
- [x] Docker NVIDIA runtime configured as default -- 2026-02-12
- [ ] Evaluate GPU-accelerated embeddings (requires L4T PyTorch image, +4-7 GB Docker image)

**CUDA Version Policy**: JetPack R36.4.4 = CUDA 12.6. DO NOT install additional CUDA versions.

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
- [x] **Face tracking GUI** (`tests/gui_face_tracker.py`) — Camera + direction vectors + motor intention -- 2026-02-10
- [x] **LiDAR navigation GUI** (`tests/gui_lidar_nav.py`) — Front calibration + movement intent -- 2026-02-10
- [x] **IMU calibration tool** (`tests/test_imu_calibrate.py`) — Axis orientation detection + bias -- 2026-02-10
- [x] **CommandSmoother** (`tests/gui_lidar_nav.py`) — 1.5s direction hold + EMA speed smoothing -- 2026-02-19
  - Prevents motor direction oscillation on every LiDAR scan update
  - Intent classification (FWD/LEFT/RIGHT/STOP/REV), emergency stops bypass hold
- [x] **SLAM/localization research** — 3 research docs in `docs/findings/` -- 2026-02-19
  - BreezySLAM: only viable standalone Python SLAM (risk: Python 3.13 C extension compat)
  - Gyro-constrained ICP: 4.5% drift vs 27.4% pure ICP (6x improvement with MPU6050)
  - ICP timing: ~32ms for 467pts on Pi 3B (31Hz, within 10Hz budget)
  - Particle filter: memory OK (<65MB), phased approach recommended
  - Existing ROS2/SLAM docs found in repo (`ros2_comprehensive_attempt/`, etc.)
- [ ] Fix left motor (right works, left doesn't spin — ENA Pin 33 wiring or power issue)
- [ ] Tune safety zone distances for robot size
- [ ] Test wandering behaviors in real environment (CommandSmoother + NaturalWander)
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

## Milestone 4: Single-Platform Integration

> **Updated 2026-02-12**: No inter-device communication. All components run on a single platform (Jetson or RPi). Integration is in-process, not over the network.

- [ ] Integrate pathfinder + locomotion + bootylicious on single platform
- [ ] Test end-to-end: person detected -> conversation + navigation (all on one device)
- [ ] Demo 3 (`wandering_demo_3.py`): LLM-driven behavior selection
- [ ] Benchmark full-stack resource usage on Jetson (LLM + RAG + sensors + motors)

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
> Also see: `lightweight-slam-research.md`, `research-icp-scan-matching.md`, `research-particle-filter-localization.md`

- [x] **MPU6050 IMU driver** — gyro heading for closed-loop turns (Level 1 localization) -- Completed 2026-02-06
  - `pathfinder/imu.py`: I2C reads at ~100Hz, startup calibration, complementary filter for pitch/roll
  - Graceful degradation: `setup_imu()` returns None if sensor not found
  - Hardware test: `tests/test_imu.py`, deploy: `./deploy.sh rpi --test=imu`
- [x] **SLAM/localization research** — Comprehensive research completed -- 2026-02-19
  - BreezySLAM: only viable standalone Python SLAM (RMHC algo, C extensions with ARM NEON)
  - Gyro-constrained ICP: 4.5% drift vs 27.4% pure ICP (6x improvement — strong case for wiring MPU6050)
  - ICP timing: ~32ms for 467pts on Pi 3B (31Hz, within 10Hz budget)
  - Particle filter: memory OK (<65MB on 906MB RPi), phased approach recommended
  - Existing ROS2/SLAM docs in repo: `ros2_comprehensive_attempt/`, `ros2_cartography_attempt/`
- [ ] Wire MPU6050 hardware and run calibration (driver + tools already exist)
- [ ] Modify NaturalWanderBehavior to use heading-error-based turns (with IMU)
- [ ] Camera-based facial detection events triggering LLM
  - Detect specific faces → personalized greetings
  - Detect crowd → different behavior than single person
- [ ] LiDAR scan matching / ICP (Level 2 — gyro-constrained, keyframe-based)
  - Implementation plan in `research-icp-scan-matching.md`
  - Only run ICP when robot moved ≥8cm or rotated ≥5° from last keyframe
- [ ] Try BreezySLAM (risk: Python 3.13 C extension compatibility)
- [ ] Combined LiDAR + camera + IMU for robust navigation
  - Still NO SLAM initially - just better reactive navigation

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

### LiDAR Denoising & Native Code Port (C/Rust)
> **Key insight**: Video game development techniques for particle rendering, point cloud denoising, and spatial clustering translate directly to LiDAR processing. Research needed.

#### Why C/Rust for LiDAR Processing
- Python LiDAR processing (~467 pts/scan) is functional but slow for real-time SLAM
- C is significantly faster for tight loops over scan data (ICP, filtering, clustering)
- Both RPi (aarch64) and Jetson (aarch64) support C and Rust natively
- Opportunity to reduce scan points (don't need all 360°) and still get good navigation
- BreezySLAM already uses C extensions for performance — same principle applies to our code

#### Video Game Development Techniques for LiDAR
> **RESEARCH NEEDED**: Web research on how game dev particle/point-cloud techniques apply to LiDAR

- **Density-based clustering (DBSCAN-like)**: Group nearby LiDAR points into objects; discard isolated noise points that are too small to be real obstacles
- **Spatial hashing / grid binning**: Same technique used in game engines for collision detection on thousands of particles — bin points into cells for O(1) neighbor lookup
- **Level-of-detail (LOD)**: Reduce point density at long range (far points matter less), keep full resolution close up — exactly like game engine LOD for distant objects
- **Temporal smoothing / motion blur**: Average scan data across multiple frames (like frame accumulation in rendering) to reduce single-scan noise
- **Octree / KD-tree spatial indexing**: Game engines use these for efficient nearest-neighbor queries on large point sets — directly applicable to ICP scan matching
- **GPU-accelerated point processing**: Jetson's GPU could process point clouds in parallel (same as game engine particle systems)
- **Frustum culling**: Only process points in the robot's direction of travel (like camera frustum in games) — skip rear-facing data when moving forward

#### Object Detection via Point Density
- Cluster LiDAR points by proximity to identify discrete objects
- Filter out clusters below a minimum point count (noise, dust, thin wires)
- Classify objects by cluster size/shape (wall = long thin cluster, person = medium blob, chair leg = small cluster)
- This replaces naive "nearest point in sector" with actual object awareness

#### C/Rust Implementation Considerations
- **Language choice**: C for maximum compatibility with both platforms; Rust for safety but potentially more complex cross-compilation
- **Challenge — Python interop**: If LiDAR processing is in C but face detection (OpenCV) stays in Python, need clean IPC (shared memory, Unix sockets, or ctypes/cffi)
- **Challenge — GPIO from C**: Both RPi and Jetson have C GPIO libraries (pigpio, Jetson.GPIO C API) but need testing
- **Challenge — OpenCV in C**: Face detection via OpenCV's C++ API is well-supported but adds build complexity
- **Phased approach**: Start with C extension for ICP/denoising only (called from Python via ctypes), then expand if beneficial
- **Alternative**: Keep Python orchestration, use C only for hot inner loops (scan matching, filtering) — best of both worlds

#### Research Tasks (Future Session)
- [ ] **Web research**: Video game particle system techniques that apply to 2D LiDAR point clouds
- [ ] **Web research**: DBSCAN and density-based clustering for LiDAR denoising on embedded ARM
- [ ] **Web research**: C vs Rust for real-time robotics on aarch64 (RPi + Jetson compatibility)
- [ ] **Web research**: GPU-accelerated point cloud processing on Jetson Orin Nano (CUDA kernels for LiDAR)
- [ ] **Benchmark**: Compare Python vs C for ICP scan matching on RPi (current: ~32ms in Python)
- [ ] **Prototype**: C extension for LiDAR point filtering/clustering, callable from Python
- [ ] **Evaluate**: Whether to port face detection to C++ OpenCV or keep Python + C hybrid

### Other Enhancements
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
- [x] **Python virtual environment (venv)** -- Added 2026-02-05
  - `venv/` created with `--system-site-packages` (accesses apt packages)
  - All scripts auto-activate venv before running Python
  - Excluded from rsync and git
  - Solves SSH vs desktop terminal package visibility issues
- [x] **Comprehensive test runner** (`run_tests.sh`)
  - Runs all tests automatically without manual SSH/curl commands
  - Test suites: `--quick`, `--all`, `--hardware`, `--integration`
  - JSON output for automation: `--json`
  - Auto-detects platform and connected hardware
  - Environment precheck validates package availability at runtime
  - Auto-activates `venv` at startup
- [x] **Deploy script enhancements** (`deploy.sh`)
  - Unified `--test=TYPE`: suites (all/quick/hardware/integration/verify), individual (gpio/camera/lidar/motors), diagnostics (check/env/env-fix)
  - One command for deploy + test: `./deploy.sh rpi --test=all`
  - Environment diagnostic: `./deploy.sh rpi --test=env`
  - Excludes `venv/` from rsync sync
  - Activates venv for individual test commands via SSH
- [x] **Install script** (`install.sh`)
  - Creates `venv/` with `--system-site-packages`
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

- **Single-platform design**: All components run on the same device (Jetson or RPi) — no inter-device communication
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
