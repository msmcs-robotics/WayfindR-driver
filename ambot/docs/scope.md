# Ambot - Scope

> Last updated: 2026-02-12
> Status: Active

---

## Overview

Ambot is an autonomous conversational robot platform with three independent components deployed to a **single platform** (either Jetson Orin Nano or Raspberry Pi — no inter-device communication):

- **bootylicious** - LLM inference, RAG knowledge retrieval, visual person detection
- **locomotion** - Motor control for differential drive robots (YAHBOOM G1 / TB6612FNG)
- **pathfinder** - LiDAR-based wandering and obstacle avoidance (LD19) - NO SLAM, just demo wandering

The project enables multiple developer teams to work independently on their respective subsystems while targeting a unified robot. Each component has its own deployment scripts and can run on either platform. All components run on the same device — there is no Jetson-to-RPi communication protocol.

## Objectives

- Deploy a small conversational LLM on the Jetson Orin Nano for face-to-face interaction
- Run a RAG system (based on rag-bootstrap) on the Jetson for context-aware responses
- Implement USB camera-based person/face detection on the Jetson to trigger conversations
- Implement LiDAR object avoidance and basic movement on the Raspberry Pi (Python first, C later)
- Enable two independent developer teams to work in parallel on their respective subsystems

## Requirements

### Functional Requirements
*What the system must do*

- [ ] Detect when a person is nearby using USB camera (Jetson)
- [ ] Initiate conversation when a person is detected
- [ ] Run a small LLM locally on the Jetson for conversational responses
- [ ] Provide RAG-augmented responses using ingested documents
- [ ] Display text responses on a small display (no speaker required initially)
- [ ] Avoid obstacles using LiDAR data (Raspberry Pi)
- [ ] Execute basic algorithmic movement patterns (Raspberry Pi)
- [ ] Allow independent development and deployment of each subsystem

### Technical Requirements
*Technical constraints, compatibility, performance needs*

- [ ] Both systems run Ubuntu-based OS
- [x] Jetson runs Ollama for LLM inference (GPU-accelerated via CUDA 12.6)
- [x] RAG system containerized via Docker (PostgreSQL + pgvector + Redis + FastAPI)
- [ ] LiDAR system runs without ROS2 (lightweight Python-based)
- [ ] SSH access to both systems for remote development
- [ ] Python-first development on Raspberry Pi (C port is future work)

### Resource Requirements
*Hardware, software, dependencies, services*

- [x] Jetson Orin Nano (user: georgejetson, IP: 10.33.255.82) — JetPack R36.4.4, CUDA 12.6, 7.4 GiB
- [x] Raspberry Pi 3B (user: pi, IP: 10.33.224.1) — Debian 13, 906 MB RAM
- [x] USB camera (EMEET SmartCam S600) on RPi — face detection working
- [x] LiDAR sensor (LD19) on RPi — 230400 baud, ~467 pts/scan
- [x] Docker 28.2.2 + Compose v2.36.2 on Jetson — NVIDIA GPU runtime configured
- [ ] Small display for text output (model TBD)

## Constraints

| Constraint | Reason | Flexible? |
|------------|--------|-----------|
| No ROS2 dependency initially | Start simple, iterate; ROS2 planned for future SLAM operations | Yes |
| No ML training on either device | Only inference; training/optimization is external project | No |
| No speaker/microphone initially | First iteration uses text display only | Yes |
| Python-first on Pi | Rapid development, C port later | Yes |
| Small LLM only (1-3B params, Q4_K_M quantized) | Jetson Orin Nano: 7.4 GiB shared memory | No |
| RAG system via Docker | Isolation and reproducibility (based on rag-bootstrap) | No |
| No LLM optimization work in this project | Covered by external project | No |

## Assumptions

- [VERIFIED] Jetson Orin Nano runs Ollama LLM + RAG embedding model concurrently (~4 GiB used, ~3.4 GiB headroom)
- [VERIFIED] RAG system (PostgreSQL + Redis + embeddings) runs on CPU while Ollama LLM uses GPU
- [VERIFIED] NVIDIA toolkits installed: CUDA 12.6, cuDNN 9.3.0, TensorRT 10.3.0, Driver 540.4.0
- [VERIFIED] Docker 28.2.2 working with NVIDIA Container Toolkit 1.16.2 (default GPU runtime)
- [ASSUMED] RPLidar or compatible LiDAR will be used on the Pi (based on WayfindR-driver)
- [UPDATED] Single-platform design — all components run on one device (no inter-device communication needed)
- [VERIFIED] Jetson SSH: `ssh jetson` (georgejetson@10.33.255.82)

## Boundaries

### In Scope
- **bootylicious**: LLM inference (nanoLLM or HuggingFace), RAG system (pgvector + Redis + FastAPI), person detection
- **locomotion**: Motor control for YAHBOOM G1 (TB6612FNG driver), differential drive support
- **pathfinder**: LiDAR wandering behaviors (MaxClearance, WallFollower, RandomWander), sector-based obstacle detection, safety zones - simple demo wandering WITHOUT SLAM
- USB camera person/face detection
- Text display output for conversation responses
- SSH-based development and deployment tooling
- Platform-agnostic components (deploy to Jetson or Pi as needed)
- System capability research and documentation
- Future ROS2 integration path for SLAM and advanced navigation

### Out of Scope (Exclusions)
- LLM training, fine-tuning, or optimization (external project)
- ROS2 dependency in initial iterations (planned for future SLAM integration)
- Speech-to-text / text-to-speech (future consideration, possibly via Android device)
- Continuous microphone listening (future roadmap item)
- C/C++ port of LiDAR system (future, after Python is working)
- Multi-robot fleet management
- Cloud-based LLM inference
- SLAM mapping (future work - current focus is simple wandering algorithms)

## Technical Decisions

| Decision | Choice | Rationale | Date |
|----------|--------|-----------|------|
| LLM Framework | Ollama 0.6.2 (GPU-accelerated via llama.cpp) | Simpler model management, memory efficient, proven on Jetson | 2026-02-12 |
| LLM Model | llama3.2:3b Q4_K_M (primary) | Best RAG quality at 3B; gemma2:2b, phi3:mini, smollm2 tested and rejected | 2026-02-19 |
| RAG Stack | rag-bootstrap (pgvector + Redis + FastAPI) | Proven stack, containerized, MCP-compatible | 2026-01-27 |
| Embedding Model | all-MiniLM-L6-v2 (384-dim) | Lightweight (~80MB), good quality | 2026-01-27 |
| LiDAR Approach (initial) | Custom Python (no ROS2) | Start simple, get basic system working first | 2026-01-27 |
| LiDAR Approach (future) | ROS2 for SLAM/advanced nav | Leverage WayfindR-driver ROS2 stack when ready to iterate | 2026-01-27 |
| Wandering Algorithms | MaxClearance, WallFollower, RandomWander | Simple demo behaviors without SLAM | 2026-01-29 |
| Jetson OS | JetPack R36.4.4 | Pre-installed on Jetson Orin Nano | 2026-02-12 |
| Container Runtime | Docker 28.2.2 (pre-installed) | Works with NVIDIA Container Toolkit 1.16.2 for GPU access | 2026-02-12 |
| Output Method | Text display (no speaker) | Simplest first iteration | 2026-01-27 |

## Integration Points

- **Single-platform deployment**: All components run on the same device (no inter-device communication)
- **rag-bootstrap**: Template for RAG system deployment on Jetson
- **WayfindR-driver**: Reference codebase for LiDAR code and motor control patterns
- **External LLM optimization project**: Will provide optimized models in the future
- **External audio/voice project**: May provide STT/TTS capabilities later

## Open Questions

- [x] What are the actual available resources on the Jetson Orin Nano? — **7.4 GiB RAM, 113GB disk (86GB free), Orin GPU, CUDA 12.6**
- [x] What NVIDIA toolkits are already installed on the Jetson? — **Driver 540.4.0, CUDA 12.6, NVIDIA Container Toolkit 1.16.2**
- [x] ~~Jetson currently has "no space left on device" error~~ — **RESOLVED: 86GB free, no space issue**
- [x] ~~What Docker issues were encountered on the Jetson?~~ — **Docker 28.2.2 working, just needed user group membership**
- [x] What is the Raspberry Pi IP address, username, and password? — **pi@10.33.224.1, password: erau**
- [x] What specific LiDAR sensor will be used on the Pi? — **LD19 (YOUYEETOO/LDRobot), 230400 baud**
- [x] Can the Jetson run nanoLLM + RAG Docker stack concurrently without resource contention? — **Yes, ~3.6GB total, ~3.8GB headroom**
- [ ] What display will be used for text output on the Jetson?
- [x] ~~How will Jetson and Pi communicate with each other?~~ — **N/A: Single-platform design, no inter-device comms**
- [ ] Should an Android device be used for STT/TTS instead of on-device processing?

## Critical Notes

- The Jetson has `user::georgejetson` / `password::Ambot` - do not commit credentials to public repos
- The RPi has `user::pi` / `password::erau` - do not commit credentials to public repos
- Docker on Jetson is working (28.2.2) — user needs logout/login after `usermod -aG docker`
- RAG stack verified: ~3.6 GB total (LLM + RAG + OS), ~3.8 GB headroom on 7.4 GiB system

---

## Revision History

| Date | Changes | By |
|------|---------|-----|
| 2026-01-27 | Initial draft from project kickoff context | LLM |
| 2026-01-29 | Reorganized into three components: bootylicious, locomotion, pathfinder | LLM |
| 2026-02-12 | Updated to single-platform design (no Jetson↔RPi comms); added Jetson details | LLM |

---

*This document evolves as the project develops. Requirements, constraints, and boundaries can be added, modified, or removed as understanding improves. Major scope changes should be discussed with the user.*
