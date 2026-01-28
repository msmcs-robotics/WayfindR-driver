# Ambot - Todo

> Last updated: 2026-01-27

## In Progress

_Tasks actively being worked on_

- [ ] Project bootstrapping and initial documentation

## Blocked

_Tasks waiting on something (include reason)_

- [ ] Jetson system inventory — **Blocked by**: "No space left on device" error on Jetson; user is making modifications
- [ ] Jetson Docker/RAG deployment — **Blocked by**: Disk space issue must be resolved first
- [ ] Raspberry Pi setup — **Blocked by**: Need IP address, username, and password

## Up Next

_Priority queue for immediate work_

- [ ] Resolve "no space left on device" on Jetson (user handling, then verify)
- [ ] SSH into Jetson and document system specs (GPU, RAM, storage, OS)
- [ ] Inventory installed NVIDIA toolkits and packages on Jetson
- [ ] Audit disk usage on Jetson (Docker images, unused packages, logs)
- [ ] Debug Docker installation issues on Jetson
- [ ] Research Jetson Orin Nano capabilities (max LLM size, concurrent workloads)
- [ ] Port LiDAR reading code from WayfindR-driver without ROS2 dependencies

## Backlog

_Lower priority, do when time permits_

- [ ] Research display options for Jetson text output
- [ ] Research Android STT/TTS as alternative to on-device audio
- [ ] Define Jetson <-> Pi communication protocol
- [ ] Adapt rag-bootstrap Docker Compose for ARM64 (Jetson)
- [ ] Set up USB camera on Jetson and test basic capture

## Recently Completed

_For context; clear periodically_

- [x] Create ambot project structure — 2026-01-27
- [x] Draft scope, roadmap, and documentation — 2026-01-27

---

## Notes

- Two teams: Jetson team (LLM/RAG/vision) and Pi team (LiDAR/movement)
- Python-first on Pi, Docker-based on Jetson
- No ROS2 on the Pi - keep it lightweight

---

*Update every session: start by reading, end by updating.*
