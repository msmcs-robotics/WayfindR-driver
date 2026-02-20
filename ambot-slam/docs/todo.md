# AMBOT-SLAM - Todo

> Last updated: 2026-02-11

## 🔴 In Progress

_Tasks actively being worked on_

(none — project just initialized)

## 🟡 Blocked

_Tasks waiting on something (include reason)_

- [ ] IMU hardware testing — **Blocked by**: MPU6050 not yet physically wired on current robot
- [ ] Motor testing — **Blocked by**: Left motor wiring issue (ENA Pin 33)
- [ ] ROS2 installation on target — **Blocked by**: RPi still running Debian 13 for ambot; will re-image when ready

## 🟢 Up Next

_Priority queue for immediate work_

- [ ] Comprehensive code review of `ambot/` and `ros2_*_attempt/` folders (next session)
  - Review all source files, configs, launch files, and findings
  - Document reusable components in `docs/findings/`
  - Refine scope and roadmap based on review
- [ ] Research LD19 LiDAR ROS2 driver options
- [ ] Research C1M1 LiDAR ROS2 driver status (rplidar_ros2)

## 📋 Backlog

_Lower priority, do when time permits_

- [ ] Create URDF robot description
- [ ] Decide on RPi re-imaging strategy (new SD card vs overwrite)
- [ ] Research wheel encoder options
- [ ] Set up RViz on desktop for remote visualization

### LiDAR Denoising Research (Future)
> See `roadmap.md` "LiDAR Denoising & Native Code" section for full context

- [ ] **Web research**: Video game particle system techniques → LiDAR point cloud denoising
- [ ] **Web research**: DBSCAN density-based clustering on embedded ARM (RPi/Jetson)
- [ ] **Web research**: C vs Rust for real-time robotics on aarch64
- [ ] **Prototype**: C extension for scan pre-filtering before SLAM Toolbox ingestion
- [ ] **Benchmark**: Python vs C for ICP and scan processing on RPi
- [ ] **Evaluate**: GPU-accelerated point cloud processing on Jetson (CUDA)

## ✅ Recently Completed

_For context; clear periodically_

- [x] Project structure created — 2026-02-11
- [x] Scope, roadmap, todo, README initialized — 2026-02-11
- [x] Initial review of ambot/ codebase and ROS2 research folders — 2026-02-11

---

## Notes

_Context or decisions affecting current tasks_

- All ROS2 research from `ros2_*_attempt/` folders has been surveyed; detailed review planned for next session
- Both LD19 and C1M1 LiDARs will be supported
- RPi re-flash to Ubuntu 22.04 is deferred — ambot project is still active on current Debian 13 setup
- Existing `ambot/` hardware drivers can be wrapped as ROS2 nodes rather than rewritten

---

*Update every session: start by reading, end by updating.*
