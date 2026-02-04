# Jetson Orin Nano LLM Performance Benchmarks

## Research Summary
This document compiles LLM performance benchmarks for the NVIDIA Jetson Orin Nano, gathered from official NVIDIA sources, community benchmarks, and GitHub repositories.

---

## 1. Tokens Per Second for Different Models

### NVIDIA Official Benchmarks (Jetson Orin Nano Super 8GB with JetPack 6.2)

LLM generation performance measured with **INT4 quantization using MLC API**:

| Model | Original (tokens/s) | Super Mode (tokens/s) | Performance Boost |
|-------|---------------------|----------------------|-------------------|
| Llama 3.1 8B | 14.0 | 19.14 | 1.37x |
| Llama 3.2 3B | 27.7 | 43.07 | 1.55x |
| Qwen2.5 7B | 14.2 | 21.75 | 1.53x |
| Gemma 2 2B | 21.5 | 34.97 | 1.63x |
| Gemma 2 9B | 7.2 | 9.21 | 1.28x |
| Phi 3.5 3B | 24.7 | 38.1 | 1.54x |
| SmolLM2 | 41.0 | 64.5 | 1.57x |

**Source:** [NVIDIA Jetson Orin Nano Super Developer Kit Blog](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/)

### Community Benchmarks (Ollama on Jetson Orin Nano Super, 25W mode)

| Model | Size | Tokens/Second | Total Duration | Eval Count |
|-------|------|---------------|----------------|------------|
| Llama 3.2 | 1B | ~62 | ~26s | ~1600 |
| Llama 3.2 | 3B | ~30 | ~53s | ~1600 |
| TinyLlama | 1.1B | ~58 | ~28s | ~1600 |
| TinyDolphin | 1.1B | ~60 | ~27s | ~1600 |
| Qwen2.5 | 0.5B | ~50 | ~32s | ~1600 |
| Qwen2 | 0.5B | ~45 | ~34.5s | ~1535 |
| Qwen | 0.5B | ~48 | ~33s | ~1580 |
| SmolLM | 1.7B | ~45 | ~36s | ~1600 |
| Gemma2 | 2B | ~38 | ~42s | ~1600 |
| Gemma | 2B | ~38 | ~42s | ~1550 |
| Qwen2.5-Coder | 0.5B | ~48 | ~33s | ~1580 |
| Qwen2.5-Coder | 1.5B | ~34 | ~47s | ~1600 |
| Qwen | 1.8B | ~35 | ~45s | ~1575 |
| Qwen2 | 1.5B | ~32 | ~50s | ~1600 |
| Orca-Mini | 3B | ~28 | ~57s | ~1600 |
| Phi3 | 3.5B | ~25 | ~65s | ~1600 |
| Qwen | 4B | ~16 | ~100s | ~1600 |
| Qwen2 | 7B | ~8 | ~180s | ~1400 |

**Source:** [Jeremy Morgan - Jetson Orin Nano Speed Test](https://www.jeremymorgan.com/blog/tech/nvidia-jetson-orin-nano-speed-test/)

### Vision Language Models (VLM) Performance

| Model | Original (tokens/s) | Super Mode (tokens/s) | Boost |
|-------|---------------------|----------------------|-------|
| VILA 1.5 3B | 0.7 | 1.06 | 1.51x |
| VILA 1.5 8B | 0.574 | 0.83 | 1.45x |
| InternVL2.5 4B | 2.5 | 5.1 | 2.04x |
| PaliGemma2 3B | 13.7 | 21.6 | 1.58x |
| SmolVLM 2B | 8.1 | 12.9 | 1.59x |

---

## 2. Memory Usage During Inference

### Memory Constraints

| Aspect | Details |
|--------|---------|
| Total Unified Memory | 8GB (shared between CPU/GPU) |
| Memory Bandwidth (Original) | 68 GB/s |
| Memory Bandwidth (Super Mode) | 102 GB/s |
| Memory Type | LPDDR5 |

### Memory Usage by Quantization

| Quantization Level | Memory Reduction | Accuracy Impact |
|-------------------|-----------------|-----------------|
| INT4 (4-bit) | 60-70% reduction | 2-5% accuracy loss |
| INT8 (8-bit) | ~50% reduction | Marginal loss |
| FP16 | Baseline | Full accuracy |

### Model Size Recommendations

| Device Memory | Recommended Model Size | Example Models |
|---------------|----------------------|----------------|
| 8GB | Up to 8B parameters (quantized) | Llama 3.1 8B, Qwen2.5 7B |
| 8GB | Optimal: 1B-3B parameters | Llama 3.2 3B, Phi-3, Gemma 2B |
| 4GB | Up to 3B parameters | Gemma 2B, SmolLM |

### Memory Tuning for vLLM
- Default settings may cause OOM errors
- Requires limiting GPU memory utilization
- Reduce maximum batched tokens
- Lower concurrency settings

**Source:** [Deployment on Edge: vLLM on Jetson](https://learnopencv.com/deployment-on-edge-vllm-on-jetson/)

---

## 3. GPU vs CPU Utilization

### Hardware Specifications

| Specification | Orin Nano 4GB | Orin Nano 8GB | Orin Nano Super 8GB |
|--------------|---------------|---------------|---------------------|
| CUDA Cores | 512 | 1024 | 1024 |
| Tensor Cores | 16 | 32 | 32 |
| GPU Clock (Original) | 625 MHz | 625 MHz | 635 MHz |
| GPU Clock (Super) | - | - | 1020 MHz |
| CPU Cores | 6 ARM Cortex-A78AE | 6 ARM Cortex-A78AE | 6 ARM Cortex-A78AE |
| CPU Clock (Super) | - | - | 1.7 GHz |
| AI Performance | 20 TOPS | 40 TOPS | 67 TOPS (sparse) |

### GPU Utilization During LLM Inference

| Model Size | Typical GPU Utilization |
|-----------|------------------------|
| 1B-3B quantized | Stable, ~28-55 tokens/sec |
| 7B-8B quantized | Higher utilization, memory-bound |
| >8B | May exceed memory limits |

### Streaming Multiprocessor (SM) Utilization
- Deep learning models can achieve approximately **100% SM active utilization** at peak
- INT8 precision models exhibit lower SM utilization
- ResNet50: SM active ~20% of runtime
- FCN_ResNet50: SM utilization predominantly ~25%

### Power Consumption by Mode

| Power Mode | Tokens/Second | Use Case |
|-----------|---------------|----------|
| ~5W (Eco) | ~5 tokens/s | Battery-powered, background tasks |
| ~15W (Default) | ~13 tokens/s | Document processing, async generation |
| ~25W (Super) | ~20+ tokens/s | Real-time chat, interactive applications |

**Sources:**
- [NVIDIA Jetson Power Documentation](https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/)
- [JetPack 6.2 Super Mode Blog](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)

---

## 4. Latency for Different Model Sizes

### Token Generation Latency

| Model Size | Approximate Latency per Token |
|-----------|------------------------------|
| 0.5B | ~20-25ms |
| 1B | ~16-20ms |
| 2B | ~26-30ms |
| 3B | ~23-35ms |
| 7B-8B | ~45-70ms |
| 9B+ | ~100-140ms |

### Time to First Token (TTFT)
TTFT measures the latency from request submission to first output token:
- Includes scheduling delay and prompt processing time
- Scales with prompt length (quadratic relationship)
- Sub-second latency verified for 1B-3B models under continuous load

### Power Mode Impact on Latency

| Power Mode | Relative Performance |
|-----------|---------------------|
| 5W | Slowest (~5 tokens/s) |
| 15W | Medium (~13 tokens/s) |
| 25W | Fastest (~20+ tokens/s) |

**Source:** [Jeremy Morgan Power Mode Tutorial](https://www.jeremymorgan.com/tutorials/generative-ai/how-to-25w-jetson-orin-nano/)

---

## 5. Comparison: Jetson Orin Nano vs Other Edge Devices

### Jetson Family Comparison

| Device | Memory | AI Performance | LLM Capability | Price |
|--------|--------|---------------|----------------|-------|
| Jetson Orin Nano 4GB | 4GB | 20 TOPS | Up to 3B models | ~$199 |
| Jetson Orin Nano 8GB | 8GB | 40 TOPS | Up to 8B models | ~$249 |
| Jetson Orin Nano Super 8GB | 8GB | 67 TOPS | Up to 8B models (faster) | $249 |
| Jetson Orin NX 8GB | 8GB | 70 TOPS | Up to 8B models | ~$399 |
| Jetson Orin NX 16GB | 16GB | 100 TOPS | Up to 13B models | ~$599 |
| Jetson AGX Orin 32GB | 32GB | 200 TOPS | Up to 30B models | ~$999 |
| Jetson AGX Orin 64GB | 64GB | 275 TOPS | Up to 70B quantized | ~$1999 |
| Jetson AGX Thor | 128GB | 800+ TOPS | 100B+ models | TBD |

### Cross-Platform Comparison

| Device | AI TOPS | Power | LLM Performance | Best For |
|--------|---------|-------|-----------------|----------|
| **Jetson Orin Nano 8GB** | 40 (67 Super) | 15-25W | ~20-40 tok/s (3B) | Edge AI, robotics |
| Raspberry Pi 5 + Coral TPU | ~4 TOPS TPU | 5-10W | Very limited | Simple vision tasks |
| Coral Dev Board | 4 TOPS | 2-4W | Not suitable | TFLite inference only |
| Apple M2/M3 (comparison) | ~15+ TOPS | 10-30W | ~100+ tok/s | Desktop/laptop |

### Key Differentiators

| Aspect | Jetson Orin Nano | Raspberry Pi 5 | Coral TPU |
|--------|-----------------|----------------|-----------|
| LLM Support | Excellent (up to 8B) | Limited (1-3B CPU) | None |
| Framework Support | TensorFlow, PyTorch, ONNX, TensorRT | Limited GPU accel | TFLite only |
| Power Efficiency | 15-25W for AI | 5-10W total | 2-4W |
| Thermal | 42-45C (stable) | Up to 80C (no fan) | Low |
| Price | $249 (Super) | ~$80 | ~$150 |

### Performance Comparison (Object Detection)

| Device | FPS (YOLO-style) | Notes |
|--------|-----------------|-------|
| Jetson Orin NX | 41.8 FPS | ~2x Pi 5 + Coral |
| Raspberry Pi 5 + Coral | 21.5 FPS | Requires USB accelerator |
| Jetson Nano (original) | ~28 FPS | Previous generation |

**Sources:**
- [Georgia Southern University Benchmark Study](https://scholars.georgiasouthern.edu/en/publications/benchmarking-edge-ai-platforms-performance-analysis-of-nvidia-jet/)
- [Edge AI Comparison - ThinkRobotics](https://thinkrobotics.com/blogs/learn/edge-ai-accelerators-jetson-vs-coral-tpu-a-detailed-comparison-for-developers)

---

## 6. Real-World Robot/Edge AI Deployment Examples

### 1. ReMEmbR - Long-Horizon Robot Memory
- **Project:** Combines LLMs, VLMs, and RAG for robot reasoning
- **Capability:** Robots can reason over observations from hours to days
- **Integration:** WhisperTRT for speech recognition on Jetson Orin
- **Source:** [NVIDIA ReMEmbR Blog](https://developer.nvidia.com/blog/using-generative-ai-to-enable-robots-to-reason-and-act-with-remembr/)

### 2. HuggingFace LeRobot
- **Project:** End-to-end imitation learning for robotics
- **Platform:** Runs on Jetson Orin Nano Super
- **Capability:** Predicts actions from visual inputs and prior trajectories
- **Source:** [NVIDIA Edge AI Blog](https://developer.nvidia.com/blog/getting-started-with-edge-ai-on-nvidia-jetson-llms-vlms-and-foundation-models-for-robotics/)

### 3. Jetson Voice Assistant
- **Project:** Real-time AI voice assistant
- **Hardware:** Jetson Orin Nano 8GB
- **Models:** Mistral 7B (Q2_K quantized), Tacotron 2 DDC TTS
- **Source:** [GitHub - jedld/jetson-voice-assistant](https://github.com/jedld/jetson-voice-assistant)

### 4. Home Assistant + LLM Integration
- **Project:** Local voice control for smart home
- **Components:** Riva ASR + Llama2 on Jetson AGX Orin
- **Benefit:** Privacy-preserving, low-latency command processing
- **Source:** [Seeed Studio Tutorial](https://www.seeedstudio.com/blog/2024/03/08/control-home-assistant-with-your-voice-integrate-riva-llama2-to-a-smart-home-hub-powered-by-nvidia-jetson/)

### 5. ROS2 Voice-Controlled Robots
- **Projects:** JetBot Voice Tools, ROSMASTER R2/M1
- **Features:**
  - Voice command processing via RIVA ASR
  - NanoLLM for natural language understanding
  - Lidar-assisted navigation
  - Object detection and person following
- **Source:** [GitHub - ros2_jetbot_voice](https://github.com/Jen-Hung-Ho/ros2_jetbot_voice)

### 6. Open Voice OS on Jetson
- **Project:** Offline AI assistant
- **Stack:** LLM + TTS + STT on K3s
- **Platform:** Jetson Orin Nano
- **Source:** [NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/open-voice-os-on-jetson-orin-nano-offline-ai-assistant-with-llm-tts-stt-on-k3s/330132)

### 7. Nova Carter AMR Platform
- **Platform:** Complete robotics development platform
- **Hardware:** Nova Orin reference architecture
- **Features:** 3D Lidar, accurate metric mapping
- **Application:** Autonomous mobile robots (AMRs)

---

## 7. Inference Framework Comparison

### Framework Performance on Jetson

| Framework | Strengths | Weaknesses | Best For |
|-----------|-----------|------------|----------|
| **MLC** | NVIDIA official, optimized | May have slower prefill | Official benchmarks |
| **llama.cpp** | Memory efficient, portable | Lower concurrency | Single-user, stability |
| **vLLM** | High throughput, concurrency | Requires memory tuning | Multi-user serving |
| **Ollama** | Easy setup, Docker support | Slightly slower than NanoLLM | Quick deployment |
| **NanoLLM** | Jetson-optimized, low latency | Jetson-specific | Edge robotics |
| **TensorRT-LLM** | Maximum optimization | Complex setup | Production deployment |

### Benchmark Configuration Notes
- NVIDIA official LLM benchmarks use **vLLM with ISL/OSL 2048/128**
- LLM generation benchmarks use **INT4 quantization via MLC API**
- Power mode should be set to **MAX (25W)** for peak performance
- Use `nvpmodel -m 0` to ensure maximum power mode

---

## 8. GitHub Repositories for Testing

### Official NVIDIA Resources
1. **[NVIDIA-AI-IOT/jetson_benchmarks](https://github.com/NVIDIA-AI-IOT/jetson_benchmarks)** - Official benchmark scripts
2. **[dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers)** - ML containers including MLC, vLLM, Ollama
3. **[dusty-nv/jetson-inference](https://github.com/dusty-nv/jetson-inference)** - Deep learning deployment guide

### Community Resources
4. **[ajeetraina/jetson-orin-nano-super-guide](https://github.com/ajeetraina/jetson-orin-nano-super-guide)** - Setup and optimization guide
5. **[implyinfer/jetson-orin-nano-field-kit](https://github.com/implyinfer/jetson-orin-nano-field-kit)** - Pre-configured LLM stack

### Monitoring Tools
6. **[jetson-stats (jtop)](https://pypi.org/project/jetson-stats/)** - Monitor GPU, memory, power on Jetson

---

## 9. Key Takeaways

### Performance Summary
- **Optimal model size:** 1B-3B parameters for real-time interaction (~30-60 tokens/s)
- **Maximum viable model:** 8B parameters with INT4 quantization (~15-20 tokens/s)
- **Super Mode boost:** 1.3x - 1.7x performance improvement across models

### Recommendations by Use Case

| Use Case | Recommended Model | Expected Performance |
|----------|------------------|---------------------|
| Real-time chat | Llama 3.2 3B | ~43 tokens/s |
| Code assistance | Qwen2.5-Coder 1.5B | ~34 tokens/s |
| Vision + Language | Qwen2.5-VL-3B | ~1-5 tokens/s |
| Voice assistant | Phi 3.5 3B + Whisper | ~38 tokens/s |
| Document processing | Qwen2.5 7B | ~22 tokens/s |

### Important Caveats
1. **Benchmark variance:** Community tests often show 3-4x lower performance than published benchmarks
2. **Memory constraints:** 8GB unified memory limits concurrent workloads
3. **Thermal management:** Sustained performance requires adequate cooling
4. **Power mode:** Always verify power mode with `nvpmodel -q` before benchmarking

---

## Sources

### Official NVIDIA Documentation
- [NVIDIA Jetson Benchmarks](https://developer.nvidia.com/embedded/jetson-benchmarks)
- [Jetson AI Lab](https://www.jetson-ai-lab.com/)
- [NanoLLM Documentation](https://dusty-nv.github.io/NanoLLM/)
- [JetPack 6.2 Super Mode Blog](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)
- [Jetson Orin Nano Super Blog](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/)

### Community Benchmarks
- [Jeremy Morgan - Jetson Speed Test](https://www.jeremymorgan.com/blog/tech/nvidia-jetson-orin-nano-speed-test/)
- [DFRobot - AGX Orin LLaMA Test](https://www.dfrobot.com/blog-13496.html)
- [Collabnix - TensorRT-LLM Guide](https://collabnix.com/running-llms-with-tensorrt-llm-on-nvidia-jetson-orin-nano-super/)

### Research Papers
- [arXiv:2506.09554 - LLM Performance and Power on Jetson](https://arxiv.org/pdf/2506.09554)
- [Georgia Southern University - Edge AI Benchmarking](https://scholars.georgiasouthern.edu/en/publications/benchmarking-edge-ai-platforms-performance-analysis-of-nvidia-jet/)

### GitHub Resources
- [NVIDIA-AI-IOT/jetson_benchmarks](https://github.com/NVIDIA-AI-IOT/jetson_benchmarks)
- [dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers)
- [dusty-nv/jetson-containers Issue #532](https://github.com/dusty-nv/jetson-containers/issues/532) - Benchmark reproducibility discussion

---

*Document compiled: February 2026*
*Research conducted using NVIDIA official documentation, community benchmarks, and academic sources*
