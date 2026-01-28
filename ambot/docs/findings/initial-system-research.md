# Initial System Research - Ambot Platform Capabilities

> Date: 2026-01-27
> Status: In Progress
> Keywords: jetson orin nano, raspberry pi, nanoLLM, rag-bootstrap, LiDAR, resource planning

## Summary

Initial research on hardware capabilities, software stack feasibility, and resource planning for the Ambot dual-system robot platform.

---

## Topics

### Jetson Orin Nano - LLM Feasibility

**Problem / Question:**
Can the Jetson Orin Nano run a small LLM (nanoLLM) AND a RAG system (PostgreSQL + pgvector + Redis + FastAPI) concurrently?

**Findings:**
- **Jetson Orin Nano specs** (from NVIDIA documentation):
  - GPU: NVIDIA Ampere architecture with 1024 CUDA cores
  - RAM: 8GB unified LPDDR5 (shared between CPU and GPU)
  - Storage: microSD or NVMe SSD
  - AI Performance: Up to 40 TOPS (INT8)
  - Power: 7W-15W configurable

- **nanoLLM resource usage** (estimated):
  - TinyLlama-1.1B (Q4 quantized): ~1-2GB GPU memory
  - Phi-2 (2.7B, Q4 quantized): ~2-3GB GPU memory
  - nanoLLM uses TensorRT for optimized Jetson inference

- **RAG system resource usage** (from rag-bootstrap analysis):
  - PostgreSQL: ~200MB RAM baseline
  - Redis: ~50MB RAM baseline
  - FastAPI + sentence-transformers (all-MiniLM-L6-v2): ~2-3GB RAM (model ~80MB, runtime overhead)
  - Total RAG: ~2.5-3GB RAM

- **Concurrent feasibility analysis** (8GB total):
  - OS + system: ~1GB
  - nanoLLM (TinyLlama Q4): ~1.5GB GPU memory
  - RAG stack: ~2.5-3GB CPU memory (embedding model can use CPU)
  - Remaining: ~2.5-3GB headroom
  - **Verdict: Feasible with TinyLlama-1.1B. Tight with Phi-2. GPU handles LLM, CPU handles RAG.**

**Recommendation:**
- Start with TinyLlama-1.1B for maximum headroom
- Run RAG embedding model (all-MiniLM-L6-v2) on CPU, not GPU
- Monitor memory usage closely during testing
- Consider using NVMe SSD for swap if needed
- Phi-2 is possible but will leave very little headroom

<details>
<summary>Details (click to expand)</summary>

Key resource split strategy:
- **GPU**: Reserved for nanoLLM inference (TensorRT optimized)
- **CPU**: RAG stack (PostgreSQL, Redis, FastAPI, embedding generation)
- **Memory**: Shared 8GB - must be carefully managed

The rag-bootstrap system uses sentence-transformers by default which loads the model into CPU memory. This is ideal for the Jetson since the GPU is reserved for the LLM. If Ollama is used as the embedding backend instead, it would compete for GPU with nanoLLM - avoid this configuration.

nanoLLM uses NVIDIA's Jetson AI Lab optimizations including TensorRT-LLM and can run quantized models efficiently. The key is quantization: Q4 models use roughly half the memory of FP16.

Reference: https://www.jetson-ai-lab.com/archive/tutorial_nano-llm.html

</details>

---

### Raspberry Pi - LiDAR Without ROS2

**Problem / Question:**
Can we run a LiDAR-based object avoidance system on the Pi without ROS2?

**Findings:**
- **WayfindR-driver contains relevant non-ROS2 code:**
  - `ros_tank_xiaor/lidar_youyeetoo_visual_plotter.py` - Direct serial LiDAR reading with visualization
  - `PI_API/services/motor_driver.py` - GPIO motor control (L298N driver)
  - `PI_API/services/robot_controller.py` - High-level movement control
  - `PI_API/services/navigation_service.py` - Waypoint navigation (dead reckoning)

- **Approach: Lightweight Python stack**
  - Read LiDAR data directly via serial (pyserial)
  - Simple sector-based obstacle detection (divide 360° scan into zones)
  - Reactive avoidance algorithm (no mapping needed)
  - Motor control via GPIO (RPi.GPIO or gpiozero)

- **Resource requirements** (minimal):
  - CPU: Low (serial reading + basic math)
  - RAM: <100MB
  - No GPU needed
  - No ROS2 overhead

**Recommendation:**
- Port the serial LiDAR reading code from WayfindR-driver
- Implement simple obstacle avoidance (not SLAM, not pathfinding)
- Use PI_API motor driver patterns for hardware control
- Python first, C port later for real-time performance if needed

---

### Audio I/O Options

**Problem / Question:**
Should we implement STT/TTS on the Jetson or use an external Android device?

**Findings:**
- **On-device (Jetson) options:**
  - Whisper (STT): Would need ~1-2GB additional RAM
  - Piper/eSpeak (TTS): Lightweight, <100MB
  - Concern: Additional RAM pressure on already constrained 8GB system

- **Android device option:**
  - Android has excellent built-in STT/TTS (Google Speech Services)
  - Offloads compute from Jetson entirely
  - Would communicate via WiFi/Bluetooth to Jetson
  - Adds hardware complexity but saves Jetson resources

- **Current decision: Text display first**
  - No audio I/O in first iteration
  - Small display shows conversation text for reading
  - Audio is future roadmap item

**Recommendation:**
- Defer audio entirely for now
- Text display is simplest first iteration
- When ready for audio, Android device is likely better than on-device given resource constraints
- Research this further when approaching that milestone

---

## Related

**Project Documents:**
- [docs/scope.md](../scope.md) - Project boundaries including audio exclusion
- [docs/roadmap.md](../roadmap.md) - Milestone planning

**External Resources:**
- [Jetson AI Lab - nanoLLM Tutorial](https://www.jetson-ai-lab.com/archive/tutorial_nano-llm.html) - Official nanoLLM guide
- [TinyLlama on HuggingFace](https://huggingface.co/TinyLlama) - Primary LLM candidate

**Code Files:**
- `~/exudeai/rag-bootstrap/` - RAG system to adapt for Jetson
- `~/WayfindR-driver/ros_tank_xiaor/lidar_youyeetoo_visual_plotter.py` - LiDAR serial reading reference
- `~/WayfindR-driver/PI_API/services/motor_driver.py` - Motor control reference

---

## Notes

- The 8GB shared memory on the Jetson Orin Nano is the primary constraint
- Strategy: GPU for LLM, CPU for everything else
- Need to actually SSH into the Jetson to get ground-truth specs and installed software
- Pi system specs unknown until we get access

---

*This file will be updated after SSH access to both systems is established and actual resource measurements are taken.*
