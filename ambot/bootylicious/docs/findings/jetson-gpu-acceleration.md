# Jetson Orin Nano GPU Acceleration Status

> Date: 2026-02-17
> Hardware: NVIDIA Jetson Orin Nano Developer Kit

## GPU Specifications

| Property | Value |
|----------|-------|
| GPU | Orin (nvgpu) — integrated Ampere-architecture GPU |
| Driver | 540.4.0 |
| CUDA | 12.6 (toolkit + runtime) |
| cuDNN | 9.3.0 for CUDA 12.6 |
| TensorRT | 10.3.0 for CUDA 12.5 (compatible with 12.6) |
| Memory | **Unified** — 7.4 GiB shared between CPU and GPU (no separate VRAM) |
| JetPack | R36.4.4 (based on Ubuntu 22.04.5 LTS) |

### Key Jetson Differences from Desktop GPUs

- **Unified memory**: CPU and GPU share the same 7.4 GiB pool. No VRAM vs system RAM distinction.
- **nvidia-smi memory**: Shows "Not Supported" — use `free -h` or `jtop` instead.
- **No discrete GPU power**: Tegra SoC, no separate fan/power for GPU.
- **PyTorch**: Must use NVIDIA's Jetson wheels (`nvcr.io/nvidia/l4t-pytorch` or NVIDIA PyTorch index), NOT standard pip PyTorch.

## Current GPU Usage

### Ollama LLM Inference: GPU ACTIVE
Ollama uses llama.cpp/GGML which directly interfaces with CUDA. Confirmed via systemd logs:

```
llama_kv_cache:      CUDA0 KV buffer size =   448.00 MiB
llama_context:       CUDA0 compute buffer size = 256.50 MiB
llama_context:  CUDA_Host output buffer size =     0.50 MiB
```

**Total GPU allocation for llama3.2:3b**: ~704 MiB (KV cache + compute)

### Sentence-Transformers Embeddings: CPU ONLY
The Docker API container uses `python:3.11-slim` base image with PyTorch CPU-only (`2.10.0+cpu`).

```
PyTorch version: 2.10.0+cpu
CUDA available: False
Model device: cpu
```

**Why this is acceptable**: The all-MiniLM-L6-v2 model is only 22 MB (384-dim). Embedding a single query takes ~50-100ms on CPU. GPU would reduce this to ~5-10ms, but the LLM generation (10-20 seconds) dominates the pipeline latency.

**To enable GPU for embeddings** (optional, not recommended currently):
1. Switch to NVIDIA L4T base image: `nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3`
2. This image is ~5-8 GB vs ~1 GB for python-slim
3. Would reduce embedding latency from 50ms to 5ms
4. Only worth it if doing batch embedding of 1000+ chunks

## CUDA Installation Status

| Component | Installed? | Version | Path |
|-----------|-----------|---------|------|
| CUDA Toolkit | Yes | 12.6.11 | `/usr/local/cuda-12.6/` |
| nvcc | Yes (not in PATH) | 12.6 | `/usr/local/cuda/bin/nvcc` |
| cuDNN | Yes | 9.3.0 | apt package |
| TensorRT | Yes | 10.3.0 | apt package |
| CUDA libraries | Yes | 12.6 | `/usr/local/cuda/targets/aarch64-linux/lib/` |

### Fix nvcc PATH

```bash
# Add to ~/.bashrc or run manually:
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

## Framework Recommendations

| Framework | Use Case | GPU Support on Jetson |
|-----------|----------|----------------------|
| **Ollama** | LLM inference | Built-in CUDA via llama.cpp |
| **PyTorch (NVIDIA)** | Custom models, sentence-transformers | Use NVIDIA wheels or L4T images |
| **TensorRT** | Optimized inference | Pre-installed, supports ONNX |
| **TensorFlow** | Not recommended | Complex setup on Jetson, prefer PyTorch |

### Ollama (Recommended for LLM)
- Zero configuration — detects CUDA automatically
- Uses GGUF quantized models (Q4_K_M = 4-bit quantization)
- Manages GPU memory automatically (offloads to CPU if needed)
- Best for: Chat, RAG ask pipeline, text generation

### PyTorch on Jetson (If Needed)
```bash
# NVIDIA Jetson PyTorch wheels (aarch64):
pip3 install --no-cache-dir \
  torch torchvision torchaudio \
  --index-url https://developer.download.nvidia.com/compute/redist/jp/v60

# Or use NVIDIA L4T Docker image:
# nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3
```

### CUDA Version Policy
**DO NOT install multiple CUDA versions.** The Jetson has a specific CUDA version matched to the JetPack release:

| JetPack | CUDA | cuDNN |
|---------|------|-------|
| R36.4.4 | 12.6 | 9.3.0 |

Installing additional CUDA versions (e.g., CUDA 11.x) via apt or runfile:
- Wastes 2-5 GB per version
- Creates PATH/library conflicts
- Not needed — all NVIDIA Jetson libraries are built for the JetPack CUDA version

## Gotchas

1. **`nvidia-smi` memory shows "Not Supported"** — Normal for Jetson (unified memory). Use `free -h` instead.
2. **`nvcc --version` fails** — CUDA is installed but `/usr/local/cuda/bin` not in PATH. See fix above.
3. **Standard pip PyTorch has NO CUDA on aarch64** — Must use NVIDIA index or L4T images.
4. **Docker GPU access**: Requires `--runtime=nvidia` or `default-runtime: nvidia` in daemon.json (already configured).
5. **Multiple CUDA versions**: AVOID. JetPack includes the correct version. Installing others creates conflicts.
