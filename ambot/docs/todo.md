# Ambot - Todo

> Last updated: 2026-01-29

## In Progress

_Tasks actively being worked on_

- [ ] Flash Ubuntu 22.04 to Jetson SD card and complete first boot setup

## Blocked

_Tasks waiting on something (include reason)_

- [ ] Jetson system inventory — **Blocked by**: Fresh Ubuntu 22.04 installation in progress
- [ ] Hardware testing (motors, LiDAR) — **Blocked by**: Need target device (Jetson or Pi) available

## Up Next

_Priority queue for immediate work_

- [ ] Complete Jetson first boot (set up user: ambot)
- [ ] SSH into Jetson and verify system specs (GPU, RAM, storage)
- [ ] Copy SSH keys to Jetson: `ssh-copy-id ambot@<ip>`
- [ ] Rsync bootylicious to Jetson: `./scripts/rsync-to-jetson.sh`
- [ ] Run setup on Jetson: `./deploy.sh setup`
- [ ] Start RAG system: `./deploy.sh start --build`
- [ ] Run diagnostics: `./deploy.sh diagnose`
- [ ] Run system tests: `./tests/test-system.sh`
- [ ] Add sample EECS documentation to knowledge/ folder
- [ ] Test document ingestion and search
- [ ] Test locomotion on target device: `cd locomotion && ./deploy.sh diagnose`
- [ ] Test pathfinder with RPLidar: `cd pathfinder && ./deploy.sh diagnose`

## Backlog

_Lower priority, do when time permits_

- [ ] Research display options for text output
- [ ] Research Android STT/TTS as alternative to on-device audio
- [ ] Define inter-component communication protocol (REST API / MQTT)
- [ ] Set up USB camera and test basic capture
- [ ] Test HuggingFace LLM backend vs Ollama performance
- [ ] Optimize chunk size and RAG parameters for EECS content
- [ ] Tune pathfinder safety zones for robot dimensions

## Recently Completed

_For context; clear periodically_

- [x] Create ambot project structure — 2026-01-27
- [x] Draft scope, roadmap, and documentation — 2026-01-27
- [x] Create Jetson Ubuntu 22.04 flash guide — 2026-01-29
- [x] Create Jetson setup scripts (Docker, Ollama, HuggingFace) — 2026-01-29
- [x] Adapt rag-bootstrap for Jetson with Docker Compose — 2026-01-29
- [x] Add HuggingFace LLM backend support — 2026-01-29
- [x] Create rsync deployment script — 2026-01-29
- [x] Document RAG system setup findings — 2026-01-29
- [x] Rename jetson_nano → bootylicious — 2026-01-29
- [x] Create comprehensive deploy.sh with diagnostics — 2026-01-29
- [x] Add idempotent system tests — 2026-01-29
- [x] Add SSH tunnel/network error handling — 2026-01-29
- [x] Reorganize into three components (bootylicious, locomotion, pathfinder) — 2026-01-29
- [x] Create locomotion component with YAHBOOM G1 motor driver — 2026-01-29
- [x] Create pathfinder component with RPLidar C1M1 support — 2026-01-29
- [x] Create deploy.sh for locomotion and pathfinder — 2026-01-29

---

## Folder Structure

```
ambot/
├── bootylicious/          # LLM + RAG system (conversation brain)
│   ├── deploy.sh          # Master deployment script
│   ├── scripts/           # Setup scripts
│   ├── rag/               # RAG system (Docker)
│   └── tests/             # System tests
├── locomotion/            # Motor control component
│   ├── deploy.sh          # Deployment and diagnostics
│   └── yahboomg1/         # YAHBOOM G1 motor driver
├── pathfinder/            # LiDAR obstacle avoidance
│   ├── deploy.sh          # Deployment and diagnostics
│   ├── lidar.py           # RPLidar C1M1 driver
│   └── obstacle_detector.py
└── docs/                  # Project documentation
```

## Notes

- **Platform-agnostic**: Components can run on Jetson or Raspberry Pi as needed
- **bootylicious/** syncs to `~/bootylicious` on target device
- All deploy scripts are idempotent (safe to run multiple times)
- Fresh Ubuntu 22.04 (JetPack 6.x) being installed on Jetson

---

*Update every session: start by reading, end by updating.*
