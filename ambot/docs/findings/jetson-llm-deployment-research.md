# Jetson Orin Nano LLM Deployment Research

> Date: 2026-02-03
> Status: Complete
> Keywords: jetson orin nano, llm, ollama, edge ai, deployment, benchmarks

## Summary

Comprehensive research on deploying LLMs to Jetson Orin Nano (8GB) for the bootylicious component. Focus on pre-built solutions (Ollama, containers) without retraining or quantization.

---

## Key Findings

### Recommended Models for Jetson Orin Nano (8GB)

| Priority | Model | Parameters | Memory (Q4) | Tokens/sec | Best For |
|----------|-------|------------|-------------|------------|----------|
| 1 | **Llama 3.2 3B** | 3B | ~2.5 GB | 28-43 | General purpose, RAG |
| 2 | **Phi-3 Mini 4K** | 3.8B | ~2.5 GB | 25-38 | Reasoning, coding |
| 3 | **Qwen2.5 3B** | 3B | ~2 GB | 20-30 | Multilingual |
| 4 | **Gemma 2 2B** | 2B | ~1.5 GB | 30-35 | Conversational AI |
| 5 | **TinyLlama 1.1B** | 1.1B | ~637 MB | 40-65 | Ultra-fast, low memory |
| 6 | **SmolLM2 1.7B** | 1.7B | ~1 GB | 50-65 | Compact, efficient |

**Note**: Models above 4B parameters (7B, 8B) can run but are slower (8-20 tok/s).

### GPU vs CPU Utilization

Critical finding for running LLM alongside locomotion/pathfinder:

- **LLM inference uses GPU** (93-99% GPU utilization)
- **CPU remains free** (only 3-5% during LLM inference)
- This means motor control and lidar can run on CPU while LLM uses GPU

### Memory Budget (8GB Unified Memory)

| Component | Memory |
|-----------|--------|
| System/OS | ~1.5 GB |
| LLM Model (Q4 3B) | ~2.5 GB |
| KV Cache | ~1.0 GB |
| RAG stack (PostgreSQL, Redis, FastAPI) | ~1.5 GB |
| **Remaining for other apps** | **~1.5 GB** |

---

## Deployment Methods (Ranked by Ease)

### 1. Ollama (Easiest - Recommended)

```bash
# Native install
curl -fsSL https://ollama.com/install.sh | sh

# Or via jetson-containers (more reliable GPU detection)
jetson-containers run $(autotag ollama)

# Pull and run model
ollama run llama3.2:3b
```

**Pros**: One command, automatic GPU detection, model management built-in
**Cons**: Some versions have GPU detection issues (use 0.5.7 if problems)

### 2. Jetson Containers (Recommended for Production)

```bash
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh

# Run any LLM framework
jetson-containers run $(autotag ollama)
jetson-containers run $(autotag vllm)
jetson-containers run $(autotag llama_cpp)
```

**Pros**: Handles all GPU configuration, version compatibility
**Cons**: Requires container setup

### 3. vLLM (Best for Concurrent Requests)

```bash
docker pull ghcr.io/nvidia-ai-iot/vllm:latest-jetson-orin
docker run --rm -it --runtime nvidia \
  --gpus all --network host --shm-size=8g \
  ghcr.io/nvidia-ai-iot/vllm:latest-jetson-orin
```

**Pros**: High throughput, production-ready
**Cons**: Higher memory usage than llama.cpp

### 4. TensorRT-LLM (Maximum Performance)

```bash
jetson-containers run dustynv/tensorrt_llm:0.12-r36.4.0
```

**Pros**: Maximum GPU optimization
**Cons**: Complex engine building, may need cross-compilation for 8GB device

---

## Performance Benchmarks

### Jetson Orin Nano Super Mode (25W) vs Regular (15W)

| Model | 15W Mode | Super Mode (25W) |
|-------|----------|------------------|
| SmolLM2 | ~45 tok/s | 64.5 tok/s |
| Llama 3.2 3B | ~30 tok/s | 43 tok/s |
| Phi 3.5 3B | ~26 tok/s | 38 tok/s |
| Qwen2.5 7B | ~15 tok/s | 22 tok/s |
| Llama 3.1 8B | ~13 tok/s | 19 tok/s |

### Memory Efficiency by Framework

| Framework | 7B Model Memory | Notes |
|-----------|-----------------|-------|
| llama.cpp | 4.2 GB | Most memory efficient |
| Ollama | ~4.5 GB | Uses llama.cpp backend |
| vLLM | 6.4 GB | Higher due to batching overhead |

---

## Recommendation for Ambot

### Configuration

1. **LLM**: Llama 3.2 3B (Q4_K_M quantization) via Ollama
2. **Deployment**: jetson-containers for reliability
3. **Power Mode**: Super mode (25W) for demos
4. **RAG**: HuggingFace backend with TinyLlama as fallback

### Expected Performance

- **LLM**: 30-40 tokens/second
- **Latency**: ~25-35ms per token
- **Memory**: ~5-6 GB total (LLM + RAG)
- **CPU free for**: Motor control, lidar processing

### Ollama Commands for Testing

```bash
# Small models (< 2GB, fastest)
ollama run tinyllama:1.1b
ollama run qwen2.5:0.5b

# Recommended models (2-3GB, good balance)
ollama run llama3.2:3b
ollama run phi3
ollama run gemma2:2b

# Larger models (slower but more capable)
ollama run qwen2.5:7b
ollama run llama3.1:8b
```

---

## Known Issues

1. **Ollama 0.5.8+ GPU Detection**: May fail to detect GPU. Use version 0.5.7 or jetson-containers.

2. **JetPack 6.2 Compatibility**: Some issues reported. JetPack 6.1 is more stable.

3. **Memory Pressure**: Running Open WebUI locally wastes ~800MB. Access from another device instead.

---

## Sources

- [NVIDIA Jetson AI Lab - Ollama Tutorial](https://tokk-nv.github.io/jetson-generative-ai-playground/tutorial_ollama.html)
- [dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers)
- [NVIDIA Blog - Edge AI on Jetson](https://developer.nvidia.com/blog/getting-started-with-edge-ai-on-nvidia-jetson-llms-vlms-and-foundation-models-for-robotics/)
- [vLLM on Jetson - LearnOpenCV](https://learnopencv.com/deployment-on-edge-vllm-on-jetson/)
- [Ollama VRAM Requirements](https://localllm.in/blog/ollama-vram-requirements-for-local-llms)
- [RidgeRun - Jetson Orin Nano Super Mode](https://developer.ridgerun.com/wiki/index.php/Exploring_NVIDIA_Jetson_Orin_Nano_Super_Mode_performance_using_Generative_AI)
