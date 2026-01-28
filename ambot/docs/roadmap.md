# Ambot - Roadmap

> Last updated: 2026-01-27

## Overview

This roadmap tracks project-level features and milestones. For immediate tasks, see `todo.md`.

**Note**: No time estimates. Focus on WHAT needs to be done, not WHEN.

**Goal Horizons:**
- **Short-term**: System inventory and basic subsystem setup
- **Mid-term (First Stable Release)**: Both subsystems independently functional and testable
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

## Milestone 3: Raspberry Pi - LiDAR & Movement Subsystem

### LiDAR Integration (Python)
- [ ] Port relevant LiDAR code from WayfindR-driver (without ROS2 dependencies)
  - Reference: `WayfindR-driver/ros_tank_xiaor/lidar_youyeetoo_visual_plotter.py`
  - Reference: `WayfindR-driver/PI_API/` for motor control patterns
- [ ] Implement raw LiDAR data reading via serial
- [ ] Implement basic obstacle detection from scan data
- [ ] Create real-time visualization tool for debugging
- [ ] Test with actual LiDAR sensor

### Object Avoidance
- [ ] Implement simple obstacle avoidance algorithm (no ML)
  - Approach: sector-based closest-obstacle detection
- [ ] Define safety zones (stop distance, slow distance)
- [ ] Test avoidance with static obstacles
- [ ] Test avoidance with dynamic obstacles

### Basic Movement
- [ ] Implement motor control interface (reference PI_API motor_driver.py)
- [ ] Implement basic movement commands (forward, backward, turn)
- [ ] Implement algorithmic movement patterns
- [ ] Integrate obstacle avoidance with movement
- [ ] Test movement reliability

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

---

## Notes

- The Jetson is the "brain" (LLM + RAG + vision), the Pi is the "body" (movement + sensing)
- Two developer teams should be able to work independently on their subsystem
- Python-first approach on Pi; C port is a future optimization milestone
- **Development philosophy: get basic system working first, then iterate** (no ROS2 initially, add it later for SLAM)
- External projects will feed into Ambot: LLM optimization project, audio/voice project
- First stable release = both subsystems working independently (not necessarily integrated)
- ROS2 is a planned upgrade path, not excluded - just not a dependency for the initial working system

---

*Update as features complete. Check boxes when done. Add new features as they're identified.*
