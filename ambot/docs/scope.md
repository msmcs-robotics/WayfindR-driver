# Ambot - Scope

> Last updated: 2026-01-27
> Status: Draft

---

## Overview

Ambot is an autonomous conversational robot platform split across two embedded Linux systems: a Jetson Orin Nano handling LLM inference, RAG knowledge retrieval, and visual person detection, and a Raspberry Pi handling LiDAR-based object avoidance and algorithmic movement. The project enables two developer teams to work independently on their respective subsystems while targeting a unified robot.

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
- [ ] Jetson runs nanoLLM framework for LLM inference (GPU-accelerated)
- [ ] RAG system containerized via Docker (PostgreSQL + pgvector + Redis + FastAPI)
- [ ] LiDAR system runs without ROS2 (lightweight Python-based)
- [ ] SSH access to both systems for remote development
- [ ] Python-first development on Raspberry Pi (C port is future work)

### Resource Requirements
*Hardware, software, dependencies, services*

- [ ] Jetson Orin Nano (user: ambot, IP: 10.33.183.100)
- [ ] Raspberry Pi with Ubuntu (IP/credentials TBD)
- [ ] USB camera for Jetson (not yet connected)
- [ ] LiDAR sensor (RPLidar or compatible) for Raspberry Pi
- [ ] Docker and Docker Compose on Jetson
- [ ] Small display for text output (model TBD)

## Constraints

| Constraint | Reason | Flexible? |
|------------|--------|-----------|
| No ROS2 dependency initially | Start simple, iterate; ROS2 planned for future SLAM operations | Yes |
| No ML training on either device | Only inference; training/optimization is external project | No |
| No speaker/microphone initially | First iteration uses text display only | Yes |
| Python-first on Pi | Rapid development, C port later | Yes |
| Small LLM only (1-3B params) | Jetson Orin Nano resource constraints | No |
| RAG system via Docker | Isolation and reproducibility (based on rag-bootstrap) | No |
| No LLM optimization work in this project | Covered by external project | No |

## Assumptions

- [ASSUMED] Jetson Orin Nano has sufficient GPU memory for nanoLLM + RAG embedding model concurrently
- [ASSUMED] RAG system (PostgreSQL + Redis) can run on CPU while LLM uses GPU
- [ASSUMED] NVIDIA toolkits are partially installed on the Jetson already
- [ASSUMED] Docker is installed on the Jetson (but may have configuration issues)
- [ASSUMED] RPLidar or compatible LiDAR will be used on the Pi (based on WayfindR-driver)
- [ASSUMED] Both devices are on the same local network for inter-device communication
- [VERIFIED] Jetson SSH: `ssh ambot@10.33.183.100`

## Boundaries

### In Scope
- LLM inference using nanoLLM on Jetson (loading and prompting small models)
- RAG system deployment (based on rag-bootstrap: PostgreSQL + pgvector + Redis + FastAPI)
- USB camera person/face detection on Jetson
- Text display output for conversation responses
- LiDAR object avoidance on Raspberry Pi (Python, no ROS2 initially)
- Basic algorithmic movement on Raspberry Pi (no ML)
- SSH-based development and deployment tooling
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
- SLAM mapping (unnecessary for basic object avoidance)

## Technical Decisions

| Decision | Choice | Rationale | Date |
|----------|--------|-----------|------|
| LLM Framework | nanoLLM (NVIDIA Jetson AI Lab) | Optimized for Jetson hardware, GPU-accelerated | 2026-01-27 |
| LLM Models | TinyLlama-1.1B, Phi-2 (candidates) | Small enough for Jetson Orin Nano | 2026-01-27 |
| RAG Stack | rag-bootstrap (pgvector + Redis + FastAPI) | Proven stack, containerized, MCP-compatible | 2026-01-27 |
| Embedding Model | all-MiniLM-L6-v2 (384-dim) | Lightweight (~80MB), good quality | 2026-01-27 |
| LiDAR Approach (initial) | Custom Python (no ROS2) | Start simple, get basic system working first | 2026-01-27 |
| LiDAR Approach (future) | ROS2 for SLAM/advanced nav | Leverage WayfindR-driver ROS2 stack when ready to iterate | 2026-01-27 |
| Output Method | Text display (no speaker) | Simplest first iteration | 2026-01-27 |

## Integration Points

- **Jetson <-> Raspberry Pi**: Future communication protocol TBD (likely REST API or MQTT)
- **rag-bootstrap**: Template for RAG system deployment on Jetson
- **WayfindR-driver**: Reference codebase for LiDAR code and motor control patterns
- **External LLM optimization project**: Will provide optimized models in the future
- **External audio/voice project**: May provide STT/TTS capabilities later

## Open Questions

- [ ] What are the actual available resources on the Jetson Orin Nano? (GPU memory, RAM, storage)
- [ ] What NVIDIA toolkits are already installed on the Jetson?
- [ ] Jetson currently has "no space left on device" error - needs disk cleanup before proceeding
- [ ] What Docker issues were encountered on the Jetson?
- [ ] What is the Raspberry Pi IP address, username, and password?
- [ ] What specific LiDAR sensor will be used on the Pi?
- [ ] What display will be used for text output on the Jetson?
- [ ] How will Jetson and Pi communicate with each other?
- [ ] Can the Jetson run nanoLLM + RAG Docker stack concurrently without resource contention?
- [ ] Should an Android device be used for STT/TTS instead of on-device processing?

## Critical Notes

- The Jetson has `user::ambot` / `password::AmbotRules` - do not commit credentials to public repos
- Docker on the Jetson may have existing configuration issues that need debugging
- Some NVIDIA packages may already be installed but need to be inventoried
- The rag-bootstrap system needs ~2.5-3GB RAM minimum; verify Jetson has headroom after nanoLLM

---

## Revision History

| Date | Changes | By |
|------|---------|-----|
| 2026-01-27 | Initial draft from project kickoff context | LLM |

---

*This document evolves as the project develops. Requirements, constraints, and boundaries can be added, modified, or removed as understanding improves. Major scope changes should be discussed with the user.*
