# Ambot - Roadmap

> Last updated: 2026-01-29

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
- [ ] Obtain Pi IP address, username, and password
- [ ] SSH into Pi and inventory system specs
- [ ] Document OS version, available resources
- [ ] Verify LiDAR sensor connectivity
- [ ] Document all findings in `docs/findings/`

### Development Environment
- [ ] Set up SSH key-based access to both devices
- [ ] Document SSH connection procedures
- [ ] Verify network connectivity between devices

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
> **Note**: Pathfinder is for simple demo wandering, NOT SLAM. The robot should wander around a room avoiding obstacles using reactive algorithms.

- [x] Create LiDAR module for RPLidar C1M1 -- Completed 2026-01-29
- [x] Implement sector-based obstacle detection -- Completed 2026-01-29
- [x] Create deploy.sh with diagnostics -- Completed 2026-01-29
- [x] Create udev rules setup script -- Completed 2026-01-29
- [x] Implement wandering behaviors (MaxClearance, WallFollower, RandomWander, AvoidAndGo) -- Completed 2026-01-29
- [x] Create BehaviorRunner for behavior loop execution -- Completed 2026-01-29
- [x] Add dynamic obstacle detection (DynamicObstacleMonitor, SafetyWrapper) -- Completed 2026-01-29
- [x] Add create_safe_wanderer() for safe operation around people -- Completed 2026-01-29
- [ ] Test with actual RPLidar C1M1 sensor
- [ ] Tune safety zone distances for robot size
- [ ] Test wandering behaviors in real environment
- [ ] Create visualization tool for debugging
- [ ] Integrate with locomotion for full wandering demo

---

## Milestone 4: Integration & Communication

- [ ] Define communication protocol between Jetson and Pi (REST API / MQTT / serial)
- [ ] Implement Jetson -> Pi movement commands
- [ ] Implement Pi -> Jetson sensor status updates
- [ ] Test end-to-end: person detected -> conversation + navigation
- [ ] Integration testing with both systems running

---

## Nice to Have (Lower Priority / Future Roadmap)

- [ ] ROS2 integration for SLAM and advanced navigation
  - Description: Once basic Python LiDAR/movement is working, introduce ROS2 for SLAM mapping, localization (AMCL), and path planning
  - Dependencies: Basic Python LiDAR system working first
  - Reference: WayfindR-driver `ros2_comprehensive_attempt/` has a complete ROS2 Humble stack ready to adapt
- [ ] Speech-to-text input (possibly via Android device)
- [ ] Text-to-speech output (possibly via Android device)
- [ ] Continuous microphone listening for voice commands
- [ ] Port LiDAR/movement system from Python to C
- [ ] Multiple display modes (conversation, status, map)
- [ ] Leverage optimized LLMs from external project
- [ ] Multi-person conversation handling
- [ ] Persistent conversation memory

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
- [x] Implement wandering behaviors (MaxClearance, WallFollower, RandomWander, AvoidAndGo) -- Completed 2026-01-29
- [x] Create BehaviorRunner for autonomous behavior execution -- Completed 2026-01-29
- [x] Research and document wandering algorithms -- Completed 2026-01-29
- [x] Add dynamic obstacle detection for people walking by (DynamicObstacleMonitor) -- Completed 2026-01-29
- [x] Add SafetyWrapper and create_safe_wanderer() for safe demos -- Completed 2026-01-29

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

---

*Update as features complete. Check boxes when done. Add new features as they're identified.*
