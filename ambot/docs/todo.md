# Ambot - Todo & Roadmap

> Last updated: 2026-02-24 (Session 19)

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

### Chat Panel UX Improvements (User Requested — Session 19)
1. **Streaming chat responses** — Stream LLM output token-by-token via SocketIO (not block response)
   - Requires RAG API streaming support (check `/api/ask` or add streaming endpoint)
   - Frontend: append tokens to chat message div as they arrive
2. **Loading animation with status stages** — Show pipeline progress while generating:
   - Stage 1: "Searching knowledge base..." (RAG retrieval)
   - Stage 2: "Loading model..." (Ollama model load into VRAM)
   - Stage 3: "Generating response..." (LLM inference)
   - Spinner/pulse animation in chat panel
3. **Fix Jetson browser crashes** — Try fixes from `docs/known-issues.md`
   - First check: `which firefox && dpkg -l firefox` vs `snap list firefox` (APT or Snap?)
   - Try `sudo apt install --reinstall snapd`, or Firefox ESR from Mozilla PPA

### Web Control Dashboard (RPi Hardware Mode)
1. **Deploy web_control to RPi** - `pip install -r web_control/requirements.txt`, run `python3 web_control/run.py`
2. **Test motor control from browser** - Direction pad, WASD keys, speed slider via WiFi
3. **Test camera MJPEG stream** - Verify `/video_feed` serves real camera frames
4. **Test LiDAR polar plot** - Verify real scan data renders correctly in canvas
5. **Test from laptop browser** - Access `http://10.33.224.1:5000` over WiFi

### Raspberry Pi (Pathfinder + Locomotion)
6. **Fix left motor** - Right motor works, left doesn't spin. Check ENA (Pin 33) wiring, terminal screws, power supply
2. **Wire and test MPU6050 IMU** - Wire GY-521 (VCC→Pin1, GND→Pin9, SCL→Pin5, SDA→Pin3), run `test_imu_calibrate.py`
3. **Test face-to-motor tracking** - Run `gui_face_tracker.py --motors` on RPi desktop, verify motors orient toward faces
4. **Calibrate LiDAR front orientation** - Run `gui_lidar_nav.py`, place object in front, press 'c' to calibrate
5. **Test real-world wandering** - Once both motors work, run `wandering_demo_1.py` with actual movement
6. **Test CommandSmoother** - Verify 1.5s direction hold + EMA smoothing on real robot with LiDAR + motors

### Jetson (Bootylicious)
7. ~~Deploy RAG resilience to Jetson~~ - **Done** (2026-02-19)
8. ~~Ingest real EECS docs~~ - **Done** (2026-02-24): Online scraper built + content cleaner, 49 pages scraped from ERAU CoE
9. ~~Evaluate nomic-embed-text~~ - **Done** (2026-02-24): Kept MiniLM (nomic causes model-swap latency)
10. ~~Copy cleaned scraper output to knowledge/ and ingest~~ - **Done** (Session 19): 52 docs, 104 chunks ingested, RAG answers verified

### Future: SLAM / Localization (when ready)
> See research docs in `docs/findings/` — lightweight-slam-research.md, research-icp-scan-matching.md, research-particle-filter-localization.md

10. **Wire MPU6050 first** - Gyro-constrained ICP achieves 4.5% drift vs 27.4% pure ICP (6x improvement)
11. **Try BreezySLAM** - Only viable standalone Python SLAM library (no ROS2). Risk: Python 3.13 compatibility with C extensions
12. **ICP scan matching** - ~32ms for 467pts on Pi 3B (31Hz). Keyframe matching critical (skip < 8cm motion)
13. **ROS2 migration** - Existing docs in `ros2_comprehensive_attempt/`, `ros2_cartography_attempt/`, etc.

## In Progress

_Tasks actively being worked on_

- [x] Face tracker GUI improved and verified on RPi desktop
- [x] LiDAR nav GUI cleaned up and verified on RPi desktop
- [x] deploy.sh refactored to use run_tests.sh wrapper (no more SSH venv sourcing)
- [x] Face-to-motor bridge added (`--motors` flag on gui_face_tracker.py)
- [x] CommandSmoother added to gui_lidar_nav.py (1.5s direction hold + EMA smoothing)
- [x] SLAM/localization research completed (3 research docs in `docs/findings/`)
- [x] RAG embedding resilience coded and deployed (retry, normalization, cooldown, junk filtering)
- [ ] Fix left motor (ENA Pin 33 wiring / power supply)
- [ ] Wire and calibrate MPU6050 IMU on RPi (`python3 tests/test_imu_calibrate.py`)
- [ ] Test real-world wandering with both motors (`python3 wandering_demo_1.py`)

## Blocked

_Tasks waiting on something (include reason)_

- [x] ~~Jetson system inventory~~ — **RESOLVED**: Jetson online at 10.33.255.82, inventory complete (Session 11)
- [x] ~~RAG knowledge base content~~ — **RESOLVED**: Online scraper built, 49 ERAU CoE pages scraped + cleaned (Session 18)

## Up Next

_Priority queue for immediate work_

### Raspberry Pi (pi@10.33.224.1)
- [ ] Tune pathfinder safety zones for robot size
- [ ] Test GUI diagnostics with display attached
- [ ] Verify motor direction (adjust offsets in config.py if reversed)

### Jetson (georgejetson@10.33.255.82)
- [x] Jetson online: JetPack R36.4.4, Ubuntu 22.04.5, CUDA 12.6, 7.4 GiB RAM, 86 GB free
- [x] Set `JETSON_HOST` and `JETSON_USER` in `deploy.sh` and `rsync-to-jetson.sh`
- [x] Rsync ambot folder to Jetson (`~/ambot/`)
- [x] pip installed, Docker group configured, NVIDIA runtime set as default
- [x] `install-jetson.sh` created (idempotent, nohup-safe)
- [x] Ollama 0.6.2 installed, tinyllama model verified working
- [x] Docker GPU access: `nvidia-smi` works inside containers (`default-runtime: nvidia`)
- [x] RAG Docker stack running: PostgreSQL + Redis + FastAPI (all healthy)
- [x] RAG API healthy: `curl http://localhost:8000/api/health` → all services green
- [x] Ollama configured to listen on all interfaces (`OLLAMA_HOST=0.0.0.0`)
- [x] RAG ingestion tested: 3 docs, search + ask pipeline working end-to-end
- [x] Pulled llama3.2:3b model (2.0 GB, better quality than tinyllama 1.1B)
- [x] Tested llama3.2:3b — accurately answers from context (tinyllama hallucinated)
- [x] Switched RAG API default to llama3.2:3b, updated .env.example
- [x] Benchmarked: 5.5 GiB used, Docker 148 MiB, 1.9 GiB headroom
- [x] Comprehensive RAG optimization review (from rag-atc-testing patterns)
- [x] Applied RAG Phase 1 quick wins (DB pool 5/5, Redis LFU, 5x fetch multiplier)
- [x] GPU acceleration verified: Ollama uses CUDA0 (model 1918 MiB, KV 448 MiB, compute 256 MiB)
- [x] Created `scripts/setup-cuda.sh` — CUDA detection, verification, PATH fix, test compilation
- [x] Fixed nvcc PATH via `/etc/profile.d/cuda-path.sh`
- [x] CUDA test passed: compiled + ran kernel on Orin GPU (Compute 8.7, 8 SMs)
- [x] Added `num_ctx=4096` and `num_predict=512` to Ollama config + API calls (default 128 truncated RAG answers)
- [x] Created LLM configuration guide (`docs/findings/llm-configuration-guide.md`)
- [x] Created edge LLM research doc (`docs/findings/edge-llm-research.md`) from exudeai + web research
- [x] Created model comparison test script (`scripts/test-llm-models.sh`)
- [x] Model comparison results reviewed (llama3.2:3b confirmed best for RAG)
- [x] RAG resilience coded (junk filtering, text normalization, retries, cooldown)
- [x] Deploy RAG resilience to Jetson (Docker rebuild, tested, verified) -- 2026-02-19
- [x] Built online scraper for ERAU CoE website (49 pages, stdlib-only, strict scope enforcement)
- [x] Built content cleaner/curator (removes footer, nav links, enrollment tables — 27% junk removed)
- [x] Moved cleaned scraper output to `bootylicious/rag/knowledge/` and ingested on Jetson (52 docs, 104 chunks)
- [x] Web dashboard deployed to Jetson (simulation mode), chat working end-to-end with RAG
- [x] Pytest frontend test suite (62 tests, all passing in simulation mode)
- [x] SSH port forwarding verified (localhost:5123 → jetson:5000)
- [x] Browser crash root cause identified (snap-confine capability issue on Tegra kernel)
- [x] deploy.sh: added `rag-ingest`, `rag-ingest-bg` commands for background ingestion

### Integration
- [x] Combine pathfinder + locomotion for basic wandering demo — **Done** (`wandering_demo_1.py`)
- [x] L298N motors wired and tested — **Done** (Session 8) — right motor works, left needs wiring fix
- [x] Camera face tracking demo — **Done** (`wandering_demo_2.py`)
- [ ] Test obstacle avoidance with actual robot movement (needs both motors working)

## Backlog

_Lower priority, do when time permits_

### Sensor Integration
- [x] MPU6050 IMU driver — **Done** (Session 8 cont.) — `pathfinder/imu.py`, gyro heading + complementary filter
- [ ] Modify NaturalWanderBehavior to use heading-error-based turns (IMU integrated, needs testing)
- [ ] Sensor fusion combining LiDAR + IMU data
- [ ] Camera facial recognition event system

### Single-Platform Integration
- [ ] Integrate all components on one device (no inter-device comms)
- [ ] Demo 3: LLM-driven behavior selection (pathfinder + locomotion + bootylicious)
- [ ] Pathfinder → Bootylicious event triggers (face detected, etc.)

### Future Vision (see roadmap.md Milestone 5)
- [ ] MCP server (FastMCP) for LLM-to-locomotion control
- [ ] Scale to 8-12B parameter LLMs on Jetson
- [ ] Voice interaction (STT/TTS via Android device)

## Recently Completed

_Session 19 - 2026-02-24_

- [x] **RAG ingestion with ERAU content** — 52 documents (49 scraped + 3 existing), 104 chunks
  - Moved cleaned scraper output from `online_scraper/output/` to `bootylicious/rag/knowledge/`
  - Synced to Jetson and ran ingestion — RAG now answers ERAU-specific questions with source citations
  - Verified: engineering programs, robotics research, EECS labs — all grounded in real content
- [x] **Pytest frontend test suite** (`web_control/tests/test_frontend.py`)
  - 62 tests across 7 classes: dashboard page, health, motors, chat, diagnostics, video, errors
  - Uses Flask test_client + BeautifulSoup (no browser needed)
  - All tests pass in simulation mode
  - Mocked RAG API for deterministic chat tests
- [x] **SSH port forwarding** verified and documented
  - `ssh -f -N -L 5123:localhost:5000 jetson` — campus network only allows SSH
  - Documented in `web_control/docs/todo.md`, `docs/known-issues.md`
- [x] **Browser crash investigation** — root cause: `snap-confine` capability failure
  - JetPack Tegra kernel lacks AppArmor support
  - `snap-confine` requires `cap_dac_override` capability not available
  - Documented 4 potential fixes in `docs/known-issues.md`
- [x] **deploy.sh enhancements** — `rag-ingest`, `rag-ingest-bg` for background ingestion
- [x] **JS/CSS separation audit** — already 99% separated (615 lines CSS, 654 lines JS in standalone files)

_Session 18 - 2026-02-24_

- [x] **nomic-embed-text evaluation** — Benchmarked on Jetson, decided to keep MiniLM
  - nomic-embed-text: 768-dim, 120ms warm, 25-40s cold, 568 MiB VRAM
  - MiniLM: 384-dim, ~50ms (CPU), always available, no model swapping
  - With OLLAMA_MAX_LOADED_MODELS=1, swapping would add 60-90s per query
- [x] **Domain acronym expansion table** (35+ terms) in `search.py`
  - Expands EECS/robotics/AI acronyms before keyword search
  - "LLM" → searches for "LLM large language model" etc.
- [x] **RAG deployment infrastructure** (deploy.sh + rag-ctl.sh + rag-test.py)
  - `./deploy.sh jetson bootylicious --rebuild` — sync + Docker rebuild
  - `./deploy.sh jetson --test=rag` — sync + rebuild + 10-test suite
  - `rag-ctl.sh` — single entry point for all Jetson RAG ops (one SSH call)
  - `rag-test.py` — 10 automated tests (health, search, ask, models)
  - Eliminates ad-hoc SSH command spam

_Session 18 - 2026-02-24_

- [x] **Online Scraper** (`online_scraper/scraper.py`)
  - BFS web crawler for ERAU College of Engineering (strict scope enforcement)
  - Python stdlib only (html.parser, urllib), polite 1.5s delay
  - 49 pages scraped at depth 2, all within `daytonabeach.erau.edu/college-engineering/*`
  - Skip patterns: login/SSO, PDFs, images, media files
  - Saves one .md file per page with source URL and timestamp
- [x] **Content Cleaner** (`online_scraper/cleaner.py`)
  - Post-scrape curation: removes footer, nav links, enrollment tables, contact blocks
  - 27% junk content removed (100K chars out of 371K total)
  - Dry-run, stats, verbose modes; idempotent
- [x] **RAG Knowledge Workflow** documented (`docs/rag-knowledge-workflow.md`)
- [x] **Scraper scope** documented (`online_scraper/scope.md`)
- [x] Committed ingest.sh (was uncommitted from prior session)
- [x] Evaluated nomic-embed-text — Kept MiniLM (nomic causes 25-40s model-swap latency)
- [x] Domain acronym expansion (35+ terms in search.py, wired into keyword search)

_Session 17 - 2026-02-24_

- [x] **Web Control Dashboard** (`web_control/`)
  - Full Flask + Flask-SocketIO web dashboard for browser-based robot control
  - 6-panel layout: Motor Control, Camera Feed, LiDAR View, LLM Chat, Diagnostics, Telemetry
  - Motor control: Direction pad (mouse/touch), WASD keyboard, speed slider
  - Camera: MJPEG streaming endpoint, face detection count
  - LiDAR: Canvas polar plot with safety zone rings, mock room scan in sim mode
  - LLM Chat: POST proxy to Jetson RAG API with status check
  - Diagnostics: CPU/RAM bars, sensor status lights (LiDAR, camera, IMU)
  - Telemetry: Uptime, motor cmd count, scan rate, camera FPS (SocketIO broadcast)
  - Safety: Motor watchdog (1s), emergency stop (button + spacebar + REST + SocketIO), blur-stop
  - HardwareManager with graceful degradation (simulation mode works without any hardware)
  - Dark theme CSS ported from PI_API, offline socket.io.min.js for RPi
  - All 10 REST endpoints verified working in simulation mode
  - Docs: scope.md, roadmap.md, todo.md, README.md

_Session 16 - 2026-02-24_

- [x] **RAG Phase 3: Search Quality** (database indexes, dual keyword, adaptive weight)
  - Added IVFFlat vector index + GIN full-text index to chunks table
  - Dual keyword search: English stemmed (AND) + Simple exact (OR), 20% overlap boost
  - Adaptive semantic weight based on query type (acronym→0.2, mixed→0.3, short→0.5, long→0.7)
  - Deployed to Jetson, Docker rebuild, verified with multiple query types
  - Index creation added to `init_db()` for persistence across rebuilds
- [x] **Basic conversation loop** (`bootylicious/chat.py`)
  - CLI tool: text in → RAG search → LLM answer → text out
  - Custom system prompts (`--system`), source toggle (`--no-sources`)
  - Configurable API URL (`--url`) for remote or local use
  - Health check on startup, graceful error handling
  - Uses only stdlib (urllib) — no extra dependencies needed

_Session 15 - 2026-02-19_

- [x] **CommandSmoother for motor direction stability** (`tests/gui_lidar_nav.py`)
  - Every LiDAR scan update was flipping motor direction — robot couldn't hold a heading
  - Added `CommandSmoother` class: classifies commands into intents (FWD/LEFT/RIGHT/STOP/REV)
  - Direction held for 1.5s minimum before allowing changes (emergency stops bypass)
  - EMA speed smoothing (alpha=0.3) for gradual speed transitions
  - Changed default behavior from `max_clearance` to `natural_wander` (5s target hold)
  - Intent displayed in motor info text
- [x] **SLAM/localization research** (3 research docs in `docs/findings/`)
  - `lightweight-slam-research.md` — BreezySLAM is only viable standalone Python SLAM (no ROS2). C extensions, RMHC algorithm. Risk: Python 3.13 compatibility.
  - `research-icp-scan-matching.md` — Gyro-constrained ICP = 4.5% drift vs 27.4% pure ICP (6x improvement). ~32ms for 467pts on Pi 3B. Keyframe matching critical.
  - `research-particle-filter-localization.md` — Memory not a bottleneck (SLAM working set <65MB). Phased approach: ICP → occupancy grid → BreezySLAM.
  - Found extensive existing ROS2/SLAM docs in repo: `ros2_cartography_attempt/`, `ros2_comprehensive_attempt/`, `ros2_localization_attempt/`
- [x] **RAG embedding resilience** (`bootylicious/rag/app/embeddings.py`)
  - Text normalization: strip null bytes, NFKC unicode, collapse whitespace, cap at 800 chars
  - Ollama retry with exponential backoff (up to 8 attempts) + progressive truncation (600 chars after 4 failures)
  - Inter-request cooldown (1.5s) between Ollama embedding calls to prevent overload
  - Updated `.env.example` with new config values
- [x] **RAG ingestion improvements** (`bootylicious/rag/app/ingestion.py`)
  - Junk chunk filtering: discard chunks with >25% digits or >10% dot-leader patterns (TOC/index content)
  - Batch commit every 25 chunks for resume-on-interruption support
  - Content-hash-based skip for already-ingested files (resume support in `ingest_directory`)

_Session 14 - 2026-02-19_

- [x] **Added Ollama generation parameters** (`num_ctx`, `num_predict`) to config + API calls
  - Default `num_predict=128` was truncating RAG answers with source citations
  - Added `LLM_NUM_CTX=4096` and `LLM_NUM_PREDICT=512` to config.py, llm.py, .env.example
  - Deployed to Jetson, rebuilt API container, verified working
- [x] **Created LLM configuration guide** (`docs/findings/llm-configuration-guide.md`)
  - Ollama API parameter reference (num_ctx, num_predict, temperature, etc.)
  - Thinking model gotcha (qwen3/deepseek-r1 consume num_predict in hidden `<think>` block)
  - Model selection table for Jetson Orin Nano (6 candidates with sizes)
  - Quantization guide (Q4_K_M recommended default)
  - Memory budget calculations
- [x] **Created edge LLM research doc** (`docs/findings/edge-llm-research.md`)
  - Comprehensive findings from `~/exudeai/` repository exploration
  - SLM selection, quantization techniques, context window management
  - "Lost in the Middle" attention problem and mitigation
  - Three-layer cognitive architecture mapping to AMBOT components
  - Embedding model comparison (MiniLM vs nomic-embed-text)
- [x] **Created model comparison test script** (`scripts/test-llm-models.sh`)
  - Pulls and benchmarks: llama3.2:3b, phi3:mini, gemma2:2b, smollm2:1.7b
  - Tests both direct speed and RAG-quality (with retrieved context)
  - Reports timing, token count, and answer quality
- [x] **Changed config.py default model** from `tinyllama` to `llama3.2:3b`
- [x] **Model comparison results** — Pulled and tested phi3:mini, gemma2:2b, smollm2:1.7b on Jetson
  - gemma2:2b fastest (27s, 2.4 GiB VRAM), phi3:mini slowest (50s, 3.7 GiB VRAM)
  - smollm2:1.7b hallucinated when context missing — not recommended for RAG
  - llama3.2:3b remains best choice: proven RAG quality, moderate speed/VRAM
  - RAG script context-escaping issue needs fix; direct API test confirmed llama3.2 works correctly

_Session 13 - 2026-02-17_

- [x] **Tested llama3.2:3b on Jetson** - Massive quality improvement over tinyllama
  - tinyllama: hallucinated "Advanced Machine-to-Machine Communication Toolkit" (ignored context)
  - llama3.2:3b: correctly identified Bootylicious, Locomotion, Pathfinder from retrieved docs
  - Resource usage: 5.5 GiB total, Docker 148 MiB, ~1.9 GiB headroom
- [x] **Switched RAG default to llama3.2:3b** - Updated .env on Jetson + .env.example
- [x] **Comprehensive RAG optimization review** - Analyzed ~/exudeai/rag-atc-testing/ patterns
  - Documented 18 optimization techniques across 4 priority phases
  - See `bootylicious/docs/findings/rag-optimization-review.md`
  - Key patterns: junk filtering, text normalization, embedding retry, dual keyword search, adaptive weights
- [x] **Applied RAG Phase 1 quick wins** - DB pool reduced (5/5), Redis LFU eviction, 5x search fetch multiplier
- [x] **GPU acceleration verified** - Ollama uses CUDA0 for LLM inference (model 1918 MiB, KV 448 MiB, compute 256 MiB)
  - Embeddings run on CPU (acceptable: 22MB model, ~50ms vs LLM 10-20s)
  - CUDA 12.6 fully functional, cuDNN 9.3.0, TensorRT 10.3.0
  - Created `bootylicious/docs/findings/jetson-gpu-acceleration.md`
- [x] **Created CUDA setup script** (`scripts/setup-cuda.sh`)
  - Detects JetPack version → determines correct CUDA version
  - Verifies installation (nvcc, libraries, cuDNN, TensorRT)
  - Fixes PATH (`/etc/profile.d/cuda-path.sh`)
  - Compiles + runs CUDA test kernel on GPU
  - Checks Ollama GPU usage and Docker NVIDIA runtime
  - CUDA version policy: one version per JetPack, never install extras
- [x] **Updated roadmap with RAG optimization phases** - 3-phase plan (quick wins → resilience → search quality)
- [x] **Added face-to-motor bridge** (`tests/gui_face_tracker.py --motors`)
  - `--motors` flag enables live motor output from face tracking steering
  - `--driver` selects motor driver (L298N/TB6612FNG/DRV8833, default L298N)
  - `--max-speed` caps PWM duty cycle (default 40% — safe for bench testing)
  - No face = motors stop (no aimless spinning)
  - Press 'm' during GUI to toggle motors on/off
  - Clean GPIO cleanup on exit (Ctrl+C safe)
  - Tested headless on RPi: motors init, 17.4fps, clean shutdown
  - Tested GUI on RPi desktop with motors at 30%
  - Without `--motors`: fully backward compatible (no GPIO touched)
- [x] **Fixed RPi desktop login** - Stray `fi` in `~/.profile` (line 29) and `~/.bashrc` (line 115)
  - Left over from Session 10 venv auto-activation revert
  - X session sources `.profile` on login, hit syntax error, session crashed
  - Fix: removed stray `fi`, restarted lightdm
- [x] **Created `docs/known-issues.md`** - Troubleshooting doc for recurring issues
- [x] **Created `scripts/kill-hardware.sh`** - Emergency stop for motors + kill hardware processes
  - `--motors`: GPIO reset only, `--procs`: kill processes only, default: both
  - Fallback raw GPIO pin reset for all motor pins (11,13,15,16,18,32,33)
- [x] **Motor safety mechanisms** added to `gui_face_tracker.py`:
  - Motors stopped immediately on startup (in case still spinning from a crash)
  - Signal handlers (SIGTERM, SIGINT) stop motors on any termination
  - atexit handler as last-resort cleanup
  - Watchdog: motors auto-stop if no drive command for 2 seconds
- [x] **Verified RPi SSH access** - pi@10.33.224.1, password: erau, SSH working

_Session 12 - 2026-02-12_

- [x] **Fixed jetson-monitor.sh buffering** - All watch functions now buffer output before displaying (no more jagged incremental rendering)
- [x] **Pulled llama3.2:3b model on Jetson** - Upgraded from tinyllama 1.1B to llama 3.2 3B for better RAG quality
- [x] **Added Mermaid JS wiring diagram to MPU6050 docs** - Two diagrams: GY-521↔RPi connections + full GPIO header pin allocation
- [x] **Updated to single-platform design** - Removed Jetson↔RPi communication from scope/roadmap/todo; all components run on one device
- [x] **Updated scope.md** - Single-platform design, resolved comms question, updated LLM models
- [x] **Updated roadmap.md** - Milestone 4 rewritten for single-platform integration, marked RAG ingest + llama3.2 as done
- [x] **Updated todo.md** - New session priorities, single-platform integration backlog, llama3.2 progress

_Session 11 - 2026-02-12_

- [x] **RAG Docker stack running on Jetson** - PostgreSQL + pgvector, Redis, FastAPI (all healthy)
- [x] **Ollama connectivity fixed** - Bound to 0.0.0.0, Docker bridge gateway IP (172.18.0.1)
- [x] **RAG pipeline validated end-to-end** - Ingest 3 docs → search → ask with tinyllama
- [x] **Created jetson-monitor.sh** - Docker, logs, SSH, system, RAG health monitoring
- [x] **Jetson auto-detection** - Monitor script detects Tegra kernel/device-tree for local mode
- [x] **Docker permission documentation** - `docker-permission-fix.md` findings
- [x] **Created install-jetson.sh** - Idempotent Jetson setup script
- [x] **Created RAG knowledge base docs** - ambot-overview.md, jetson-setup.md, lidar-navigation.md

_Session 10 - 2026-02-11_

- [x] **Renamed `.venv` → `venv`** - Made virtual environment folder visible (not hidden)
  - Updated all scripts: `deploy.sh`, `run_tests.sh`, `install.sh`, `rsync-to-jetson.sh`
  - Updated `.gitignore`, docs (todo.md, roadmap.md)
  - Renamed on RPi, fixed internal venv paths (activate, pyvenv.cfg)
  - Fixed `verify_all_imports.py` to exclude `venv/` directory from file scan
- [x] **Fixed OpenCV GUI support on RPi** - Switched from pip `opencv-python-headless` (no GUI) to apt `python3-opencv` (Qt5 backend)
  - Installed `opencv-data` for Haar cascade files
  - `cv2.imshow()` now works on RPi desktop
- [x] **Verified GUI demos on RPi desktop** - Both `gui_face_tracker.py` and `gui_lidar_nav.py` run on RPi desktop with display
- [x] **Reverted `.bashrc`/`.profile` venv auto-activation** - Decided against auto-sourcing venv in shell configs. Scripts handle venv activation themselves.
- [x] **Design decisions documented**:
  - Venv activation: handled by deploy scripts and `run_tests.sh`, not shell login configs
  - Face tracking: horizontal centering priority (vertical needs servos later)
  - Wandering: basic pre-SLAM logic without odometry, no SLAM until ROS2
  - LiDAR nav: needs smoothing to reduce visual clutter and heading jitter
- [x] **Refactored deploy.sh** - No more SSH venv sourcing for individual tests
  - All individual tests now use `./run_tests.sh --test=NAME` on RPi
  - Added `--test=NAME` support to `run_tests.sh` with dispatch function
  - Added individual test functions for each test type (gpio, camera, lidar, imu, motors, etc.)
  - Venv activation happens once in `run_tests.sh`, not per SSH command
- [x] **Improved face tracker GUI** (`tests/gui_face_tracker.py`)
  - Performance: detect every 3rd frame at 50% resolution (17.8fps on RPi 3B, up from ~2.4fps)
  - Display: big green crosshair (full width/height), bounding boxes on ALL faces
  - Tracking: selects face closest to horizontal center (not largest area)
  - Tracked face: yellow box + "TRACK" label + yellow arrow from center
  - Other faces: green box + darker green arrow
  - Removed: dead zone overlay, help overlay, offset indicator, frame.copy() overlays
- [x] **Cleaned up LiDAR nav GUI** (`tests/gui_lidar_nav.py`)
  - Dark theme (black background)
  - EMA-smoothed heading arrow (alpha=0.15) with circular angle wraparound handling
  - Single gold heading arrow (removed separate max clearance + intent arrows)
  - Removed: nearest obstacle marker, controls text on plot, legend clutter
  - Minimal one-line info text at top
- [x] **52 files, 34 modules, 3/3 tests passing** on RPi after all changes

_Session 9 - 2026-02-10_

- [x] **Motor test** - Ran basic + individual motor tests. Motors heard trying to spin but didn't move (likely underpowered/battery issue). Skipping motors for now.
- [x] **Created `tests/gui_face_tracker.py`** - Camera face-tracking GUI with direction vectors
  - Live camera feed with face detection (Haar cascade)
  - Direction vector (arrow) from frame center to nearest face
  - Horizontal offset indicator bar at top
  - Motor intention panel (bottom): L/R speed bars, steering direction, steering arrow
  - Dead zone visualization (adjustable with `[`/`]` keys)
  - Gain control (`+`/`-` keys), headless mode for SSH testing
  - Tested on RPi: 2.4 fps face detection, overlays render correctly
- [x] **Created `tests/gui_lidar_nav.py`** - LiDAR navigation GUI with front calibration
  - Live polar plot with scan points colored by safety zone
  - **Front marker** (red triangle + line at 0 degrees = physical front)
  - **Calibration mode** (press 'c'): finds nearest cluster → sets as front offset
  - Max clearance direction (cyan star + line = where robot would go)
  - Movement intention arrow (yellow, shorter, shows navigation intent)
  - Nearest obstacle marker (red circle)
  - Calibration saved to `tests/results/lidar_calibration.json`
  - Tested headless on RPi: 451 pts/scan, max clearance at 165deg, nearest at 58deg
- [x] **Created `tests/test_imu_calibrate.py`** - IMU calibration & axis orientation tool
  - Step 1: Gyro bias calibration (keep still for 3s, averages noise)
  - Step 2: Gravity axis detection (finds which accel axis reads ~1g)
  - Step 3: Heading axis detection (rotate robot, finds dominant gyro axis)
  - Saves calibration to `tests/results/imu_calibration.json`
  - `--stream N` mode for raw data inspection, `--quick` for bias+gravity only
  - SSH-friendly (no GUI needed)
- [x] **Updated verify_all_imports.py** - Added gui_face_tracker, gui_lidar_nav, test_imu_calibrate
- [x] **52 Python files, 34 modules, 0 failures** on RPi

_Session 8 (continued, final) - 2026-02-06_

- [x] **Updated install.sh for MPU6050** - Added `smbus2` pip package to `install_pathfinder_python()` and `verify_installation()`. System deps (`python3-smbus`, `i2c-tools`) already present.
- [x] **Added quick-reference pin tables to all wiring guides** - Simple `| Pin | Function | Physical Pin | BCM GPIO |` table at top of L298N, TB6612FNG, DRV8833, TB6612FNG-RPi-Pinout guides
- [x] **Verified install.sh on RPi** - All deps installed (smbus2 confirmed), 3/3 quick tests passing, 49 files, 31 modules

_Session 8 (continued) - 2026-02-06_

- [x] **Fixed NaturalWanderBehavior target cycling bug** - `_refresh_targets()` was rebuilding the entire queue every 5s, always restarting from the longest clearance. Fixed: separated "advance to next target" from "rebuild queue" logic.
- [x] **Created MPU6050 IMU driver** - `pathfinder/imu.py` — full I2C driver for GY-521 breakout
  - Gyro heading integration (Z-axis), startup calibration (bias averaging), complementary filter for pitch/roll
  - Temperature sensor, graceful degradation (`connect()` returns False if missing)
  - Wiring docs: `docs/findings/mpu6050-wiring.md`
- [x] **Created `demos_common/` module** - Shared sensor wrappers for all demos
  - `robot.py` — RobotAdapter (float-to-int motor speed bridge)
  - `sensors.py` — `create_lidar()`, `CameraFaceThread`, `FaceData`, `setup_imu()`
  - `behaviors.py` — `create_behavior()` factory
- [x] **Refactored wandering demos to use `demos_common`** - Both demos now import shared code
  - `wandering_demo_1.py`: 498 → 299 lines (removed duplicate classes)
  - `wandering_demo_2.py`: 498 → 398 lines (removed sys.path hack + duplicate imports from demo 1)
  - Both demos now support `--no-imu` flag and IMU integration
- [x] **Created `tests/test_imu.py`** - MPU6050 hardware test (I2C bus, WHO_AM_I, raw reads, calibration, heading integration)
- [x] **Updated deploy.sh** - Added `--test=imu` for MPU6050 testing
- [x] **Updated pathfinder/__init__.py** - Exports `IMU` class
- [x] **Updated verify_all_imports.py** - Added `pathfinder.imu`, `demos_common.*` modules

_Session 8 - 2026-02-06_

- [x] **Motors wired and tested** - L298N motor driver connected, basic motor test passing
  - Forward, reverse, turn left/right, arc left/right all working at 30% speed
  - **Individual motor test**: RIGHT motor works (fwd+rev), LEFT motor doesn't spin
  - Suspected: ENA (Pin 33) wiring, terminal screws, or underpowered supply
- [x] **Fixed PWM cleanup TypeError warnings** - RPi.GPIO's `PWM.__del__` fired after `GPIO.cleanup()` freed lgpio chip
  - Fix: Set `self._pwm = None` after `stop()` in `drivers.py` so `__del__` never fires on freed handle
  - Added `if self._pwm:` guards on all PWM calls in TB6612FNG, L298N, DRV8833
- [x] **Created `tests/test_motors.py`** - Convenience motor test wrapper
  - Defaults to L298N driver at 30% speed (no args needed)
  - Added deploy.sh sub-tests: `motors`, `motors-basic`, `motors-individual`, `motors-pinout`
- [x] **Created `tests/gui_wandering.py`** - Wandering behavior visualization
  - Shows NaturalWanderBehavior decisions overlaid on live LiDAR data
  - Displays: scan points colored by safety zone, clearance bin wedges, target queue (magenta), current target (cyan star+arrow), safety zone circles, motor command panel
  - Supports headless mode (save snapshots) and interactive GUI ([Q]uit, [P]ause, [N]ext, [R]eset, [S]ave)
- [x] **Localization research** - Documented in `docs/findings/localization-pre-slam.md`
  - Key finding: reactive wandering does NOT need position tracking
  - Recommended: MPU6050 gyro heading only (Level 1), NOT accelerometer position (drifts 17m in 1 min)
  - Localization levels: 0 (current) → 1 (gyro heading) → 2 (ICP scan matching) → 3 (fusion) → 4 (SLAM)
- [x] **Renamed wandering demo** - `wandering_demo.py` → `wandering_demo_1.py` (LiDAR-only)
  - Updated all imports in test_wandering_integration.py, verify_all_imports.py
- [x] **Created wandering_demo_2.py** - Camera face tracking + LiDAR + motor control
  - State machine: WANDERING ↔ TRACKING
  - Camera thread runs at ~10fps, detects faces via Haar cascade
  - P-controller pivots robot to center face horizontally (dead zone ±40px)
  - LiDAR safety override: emergency stop regardless of state
  - Face timeout: resumes wandering after 2s without face
  - Supports --simulate, --no-camera, --dead-zone, --face-timeout
- [x] **12/12 tests passing on RPi** (43 Python files, 26 modules)
- [x] **Fixed LiDAR GUI NameError** - `collector` undefined when LD19 driver used in interactive GUI mode
  - Root cause: LD19 path set `use_ld19=True` but never created `collector`, GUI `update()` referenced it
  - Fix: Added LD19 background scan thread + `get_gui_points()` abstraction for both modes
- [x] **Renamed demos** to numbered convention: `wandering_demo_1.py`, `wandering_demo_2.py`
- [x] **Created NaturalWanderBehavior** - Natural wandering using top-N clearance cycling
  - Bins ~467 raw scan points into 36 angular buckets (10° each)
  - Picks top 10 clearance peaks, ≥30° apart, cycles through them sequentially
  - Avoids ping-ponging (the old MaxClearance problem)
  - Now the default behavior for `safe_wanderer` in both demos
  - Added `raw_points` field to DetectionResult for fine-grained behavior access

_Session 7 - 2026-02-05_

- [x] **Switched to venv approach** - Clean Python virtual environment instead of system-wide pip hacks
  - Created `venv` with `--system-site-packages` (accesses apt packages like RPi.GPIO, tkinter)
  - `run_tests.sh` auto-activates venv
  - `deploy.sh` activates venv for individual test commands, excludes `venv` from rsync
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
  - Fix: Switched to venv approach (Session 7)
- [x] **Environment diagnostic tool** - `scripts/env_diagnostic.py`
  - Compares SSH vs desktop terminal environments
  - Shows package locations (user-local vs system vs system-local)
  - `--fix` flag installs packages system-wide
  - `--json` for automated comparison
- [x] **All 12 tests passing on RPi** (up from 10) - New environment precheck test added
  - Environment precheck now validates packages at runtime
  - All hardware tests still passing (camera, LiDAR, GPIO)
- [x] **Updated install.sh** - Venv-based package installation
  - Creates `venv` with `--system-site-packages`
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
│   ├── chat.py            # Interactive conversation loop (CLI, RAG-powered)
│   ├── deploy.sh          # Master deployment script
│   ├── scripts/           # Setup scripts (Ollama, Docker, rsync-to-jetson)
│   ├── rag/               # RAG system (Docker Compose + FastAPI)
│   ├── docs/              # Bootylicious-specific documentation
│   │   ├── findings/      # Research & setup findings (Docker, Jetson inventory)
│   │   ├── features/      # Feature documentation
│   │   └── archive/       # Session summaries
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
│   ├── imu.py             # MPU6050 IMU driver (gyro heading + complementary filter)
│   ├── obstacle_detector.py
│   └── scripts/           # udev rules, etc.
├── demos_common/          # Shared components for wandering demos
│   ├── robot.py           # RobotAdapter (float-to-int speed bridge)
│   ├── sensors.py         # create_lidar(), CameraFaceThread, FaceData, setup_imu()
│   └── behaviors.py       # create_behavior() factory
├── scripts/               # Bootstrap & deployment scripts
│   ├── rpi-bootstrap.sh   # Master RPi bootstrap
│   ├── rpi-bootstrap-system.sh   # System packages + Docker
│   ├── rpi-bootstrap-python.sh   # Python libraries
│   ├── env_diagnostic.py  # Environment diagnostic (SSH vs desktop, package locations)
│   ├── kill-hardware.sh   # Emergency motor stop + kill hardware processes
│   ├── setup-cuda.sh      # CUDA detection, verification, PATH fix (Jetson)
│   ├── test-llm-models.sh # LLM model comparison test (pull + benchmark)
│   ├── jetson-monitor.sh  # Jetson monitoring (Docker, SSH, system, RAG health)
│   ├── network-refresh.sh # RPi network troubleshooting (run ON RPi)
│   └── wsl-ssh-helper.sh  # WSL2 SSH helper (run FROM dev machine)
├── deploy.sh              # Master deployment script
├── install.sh             # Comprehensive RPi install script (sudo, idempotent)
├── install-jetson.sh      # Comprehensive Jetson install script (sudo, idempotent)
├── run_tests.sh           # Comprehensive test runner (run ON target device)
├── live_monitor.py        # Terminal-based sensor + system monitoring
├── wandering_demo_1.py   # Demo 1: LiDAR obstacle avoidance + motors
├── wandering_demo_2.py   # Demo 2: Camera face tracking + LiDAR + motors
├── tests/                 # Hardware test & diagnostic scripts
│   ├── run_all_tests.sh   # Run all tests
│   ├── test_gpio.py       # GPIO tests
│   ├── test_usb_camera.py # Camera tests
│   ├── test_ld19_lidar.py # LD19 LiDAR tests (all passing!)
│   ├── test_wandering_integration.py  # Pathfinder+Locomotion test
│   ├── test_motors.py     # Motor test wrapper (defaults to L298N @ 30%)
│   ├── test_imu.py        # MPU6050 IMU hardware test
│   ├── gui_camera.py      # Camera GUI: basic feed (--no-faces) or face detection (--faces)
│   ├── gui_lidar.py       # LiDAR GUI: polar plot + nearest/furthest edges
│   ├── gui_wandering.py   # Wandering behavior visualization (LiDAR + targets + safety zones)
│   ├── gui_face_tracker.py # Face tracking GUI (camera + direction vectors + motor intention)
│   ├── gui_lidar_nav.py   # LiDAR navigation GUI (front calibration + movement intention)
│   ├── test_imu_calibrate.py # IMU calibration (axis orientation, bias, heading axis detection)
│   ├── verify_all_imports.py  # Comprehensive syntax/import verification
│   └── results/           # Test output (JSON, PNG, JPG)
└── docs/                  # Project documentation
    ├── todo.md            # This file
    ├── roadmap.md         # Project roadmap & milestones
    ├── scope.md           # Project scope
    ├── findings/          # Research findings
    │   ├── ld19-lidar-protocol.md
    │   ├── live-monitoring-architecture.md  # Live monitor design doc
    │   ├── jetson-llm-deployment-research.md
    │   ├── localization-pre-slam.md  # IMU/localization analysis (Level 0-4)
    │   ├── mpu6050-wiring.md  # MPU6050 GY-521 wiring guide for RPi 3B
    │   ├── llm-configuration-guide.md  # Ollama/HF API parameters, model selection
    │   ├── edge-llm-research.md  # Edge LLM findings from exudeai + web research
    │   ├── lightweight-slam-research.md  # BreezySLAM + standalone SLAM options
    │   ├── research-icp-scan-matching.md  # ICP scan matching + gyro constraint
    │   └── research-particle-filter-localization.md  # Particle filter for RPi
    └── archive/           # Session summaries
```

## Hardware Status (RPi)

| Device | Status | Notes |
|--------|--------|-------|
| EMEET SmartCam S600 | **Working** | /dev/video0, 640x480 capture verified |
| LiDAR (LD19) | **Working** | /dev/ttyUSB0, 230400 baud, ~497 pts/scan |
| GPIO | **Ready** | RPi.GPIO 0.7.2, gpiozero, lgpio all working |
| Motors (L298N) | **Wired, Underpowered** | Both motors try to spin but don't move — need stronger power supply |
| MPU6050 (GY-521) | **Driver + Calibration Ready** | `pathfinder/imu.py` + `tests/test_imu_calibrate.py`, needs hardware wiring |

## Hardware Status (Jetson)

| Device | Status | Notes |
|--------|--------|-------|
| System | **Working** | Ubuntu 22.04.5, JetPack R36.4.4, CUDA 12.6, 7.4 GiB RAM |
| GPU | **Working** | Orin (Compute 8.7, Ampere), 8 SMs, CUDA 12.6, cuDNN 9.3.0 |
| Docker | **Working** | 28.2.2 + NVIDIA runtime default, Compose v2.36.2 |
| Ollama | **Working** | 0.6.2, llama3.2:3b (primary) + tinyllama, GPU-accelerated |
| RAG Stack | **Working** | PostgreSQL + pgvector, Redis, FastAPI API |

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
sudo ./install.sh                   # Full install (creates venv + installs packages)
sudo ./install.sh --check           # Check what's installed
sudo ./install.sh --gui             # Include GUI packages (tkinter, ImageTk, matplotlib)

# Environment diagnostic (run ON RPi)
source venv/bin/activate            # Activate venv first (or scripts do it automatically)
python3 scripts/env_diagnostic.py            # Full diagnostic
python3 scripts/env_diagnostic.py --json     # JSON for comparison

# Emergency hardware stop (run ON RPi)
bash scripts/kill-hardware.sh              # Kill processes + stop motors
bash scripts/kill-hardware.sh --motors     # Stop motors only (GPIO reset)
bash scripts/kill-hardware.sh --procs      # Kill hardware processes only

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

# Demo 1: LiDAR obstacle avoidance wandering (simulation - no motors needed)
python3 wandering_demo_1.py --simulate
python3 wandering_demo_1.py --simulate --behavior wall_follower_right

# Demo 2: Camera face tracking + LiDAR wandering
python3 wandering_demo_2.py --simulate
python3 wandering_demo_2.py --simulate --no-camera  # LiDAR only
python3 wandering_demo_2.py --dead-zone 50 --face-timeout 3.0

# Motor testing
python3 tests/test_motors.py               # Quick test (L298N, 30%, basic sequence)
python3 tests/test_motors.py --check       # Platform check only (no motor movement)
python3 tests/test_motors.py --individual  # Test each motor separately
python3 tests/test_motors.py --pinout      # Show wiring pinout
./deploy.sh rpi --test=motors              # Deploy + run motor check

# IMU testing (MPU6050)
python3 tests/test_imu.py                  # Full test suite (I2C, WHO_AM_I, reads, calibration)
python3 tests/test_imu.py --duration 10    # Continuous sensor readout for 10s
./deploy.sh rpi --test=imu                 # Deploy + run IMU test
i2cdetect -y 1                             # Verify MPU6050 at 0x68 (run on RPi)

# GUI diagnostics (run ON RPi with display)
python3 tests/gui_camera.py                # Live camera (basic feed)
python3 tests/gui_camera.py --faces        # Live camera + face detection + bounding boxes
python3 tests/gui_face_tracker.py          # Face tracking + direction vectors + motor intention
python3 tests/gui_face_tracker.py --motors               # + actual motor output (L298N, 40%)
python3 tests/gui_face_tracker.py --motors --max-speed 30 # Cap motor PWM at 30%
python3 tests/gui_face_tracker.py --motors --driver TB6612FNG  # Use different driver
python3 tests/gui_lidar.py                 # Live LiDAR polar plot + nearest/furthest
python3 tests/gui_lidar_nav.py             # LiDAR navigation (front calibration + movement intent)
python3 tests/gui_wandering.py             # Wandering behavior visualization (LiDAR + targets)

# Headless testing (via SSH)
python3 tests/gui_camera.py --headless --captures 3 --no-faces  # Basic capture
python3 tests/gui_camera.py --headless --captures 3 --faces     # Face detection
python3 tests/gui_face_tracker.py --headless -n 3               # Face tracker snapshots
python3 tests/gui_lidar.py --headless --scans 3
python3 tests/gui_lidar_nav.py --headless -n 3                  # LiDAR nav snapshots
python3 tests/gui_wandering.py --headless --scans 3  # Save wandering viz snapshots

# IMU calibration (run ON RPi)
python3 tests/test_imu_calibrate.py                  # Full calibration (bias + gravity + heading axis)
python3 tests/test_imu_calibrate.py --quick           # Quick: bias + gravity only
python3 tests/test_imu_calibrate.py --stream 10       # Stream raw data for 10s
python3 tests/test_imu_calibrate.py --load             # Show saved calibration

# Jetson monitoring (from WSL)
./scripts/jetson-monitor.sh docker          # Watch Docker containers + stats
./scripts/jetson-monitor.sh rag             # RAG health dashboard (API + Ollama + docs)
./scripts/jetson-monitor.sh logs api        # Follow RAG API container logs
./scripts/jetson-monitor.sh logs            # Follow all RAG container logs
./scripts/jetson-monitor.sh ssh             # Watch SSH/remote connections
./scripts/jetson-monitor.sh system          # CPU/mem/GPU/disk usage
./scripts/jetson-monitor.sh all             # Combined overview
./scripts/jetson-monitor.sh docker -n 5     # Custom refresh interval (5s)

# Jetson install/setup (run ON Jetson or via SSH)
sudo ./install-jetson.sh                    # Full install (all components)
sudo ./install-jetson.sh --check            # Check what's installed
sudo ./install-jetson.sh --inventory        # System inventory
sudo ./install-jetson.sh --docker           # Docker setup only
sudo ./install-jetson.sh --ollama           # Ollama install only
sudo ./install-jetson.sh --rag              # Start RAG Docker stack
sudo ./install-jetson.sh --cuda            # CUDA verification & PATH fix

# CUDA verification (Jetson)
bash scripts/setup-cuda.sh                  # Full check + fix + test
bash scripts/setup-cuda.sh --check          # Check only (no changes)
bash scripts/setup-cuda.sh --fix-path       # Fix nvcc PATH only
bash scripts/setup-cuda.sh --test           # Compile + run CUDA test

# RAG API (Jetson)
curl http://10.33.255.82:8000/api/health    # Health check from WSL
curl http://localhost:8000/api/health       # Health check from Jetson
curl -X POST http://10.33.255.82:8000/api/ask -H 'Content-Type: application/json' -d '{"question": "test"}'

# SSH to RPi / Jetson
ssh rpi                                     # RPi (via SSH config)
ssh jetson                                  # Jetson (via SSH config)
ssh pi@10.33.224.1

# WSL2 SSH Helper (when SSH fails from WSL due to NAT)
./scripts/wsl-ssh-helper.sh              # SSH using Windows SSH
./scripts/wsl-ssh-helper.sh --check      # Check if known IP is reachable
./scripts/wsl-ssh-helper.sh --test       # Test connectivity
./scripts/wsl-ssh-helper.sh --rsync      # Sync ambot folder to RPi
./scripts/wsl-ssh-helper.sh --ip 10.33.171.167  # Use specific IP
```

## Design Decisions

### Wandering Behavior Philosophy (Pre-SLAM)

The robot should look like it's **naturally wandering** — not bouncing between two open areas. Key principles:

1. **Fine-grained clearance mapping**: Raw LiDAR data (~467 pts/scan) binned into 36 angular buckets (10° each)
2. **Multi-target selection**: Top 10 clearance peaks, ≥30° apart, cycled sequentially
3. **Not always the max**: Cycles longest → 2nd → 3rd... so robot explores diverse directions
4. **Reactive safety**: STOP/SLOW/WARN zones + DynamicObstacleMonitor always override
5. **No SLAM, no odometry**: Pure reactive navigation — precursor to future SLAM via ROS2

`NaturalWanderBehavior` is the default for both demos (via `safe_wanderer`). Face tracking in Demo 2 is just an interrupt to the wandering progression.

### Three Wandering Demo Versions

| Demo | File | Components | Purpose |
|------|------|------------|---------|
| **Demo 1** | `wandering_demo_1.py` | LiDAR + Locomotion | Natural wandering with obstacle avoidance |
| **Demo 2** | `wandering_demo_2.py` | Camera + LiDAR + Locomotion | Wander + pivot to face detected faces |
| **Demo 3 (LLM)** | `wandering_demo_3.py` | All + Bootylicious (LLM/RAG) | Full robot with conversation capability |

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
| `wandering_demo_1.py` | ✅ | ✅ | Same behavior, different GPIO lib |
| `wandering_demo_2.py` | ✅ | ✅ | Camera + LiDAR, same GPIO abstraction |
| `wandering_demo_3.py` | ⚠️ | ✅ | Requires LLM (Jetson preferred) |
| `install.sh` | ✅ | - | RPi-specific packages |
| `install-jetson.sh` | - | ✅ | Jetson-specific: pip, Docker, Ollama, RAG |

**Key differences handled by platform detection:**
- GPIO: `RPi.GPIO` (RPi) vs `Jetson.GPIO` (Jetson)
- GPU monitoring: `/sys/devices/17000000.ga10b/load` (Jetson only)
- LLM: Ollama/nanoLLM on Jetson, optional fallback to API on RPi

## Notes

- **Python packages use `venv/`** inside `ambot/` folder
  - Venv created with `--system-site-packages` (sees apt packages: RPi.GPIO, tkinter, PIL.ImageTk)
  - Pip packages installed into `venv/` (pyserial, numpy, matplotlib, smbus2)
  - OpenCV: apt `python3-opencv` (Qt5 GUI) + `opencv-data` (cascade files) — NOT pip headless
  - Scripts auto-activate venv (`run_tests.sh`, `deploy.sh` individual commands)
  - Venv NOT auto-sourced in `.bashrc`/`.profile` — explicit activation only
  - `venv/` excluded from rsync and git
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
