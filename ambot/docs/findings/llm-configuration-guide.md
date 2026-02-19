# LLM Configuration Guide for Constrained Hardware

> Last updated: 2026-02-19
> Hardware: Jetson Orin Nano, 7.4 GiB unified CPU/GPU memory

This document covers Ollama and HuggingFace configuration for running small LLMs on the Jetson Orin Nano, with emphasis on parameters that affect memory usage, response quality, and common gotchas.

---

## Current Setup

| Setting | Value |
|---------|-------|
| Backend | Ollama (recommended) |
| Model | llama3.2:3b (Q4_K_M, 2.0 GB) |
| Context window | 4096 tokens (Ollama default for <24 GiB VRAM) |
| Temperature | 0.3 |
| GPU acceleration | CUDA 12.6, Ollama uses llama.cpp/GGML |

---

## Ollama API Parameters

### Context Window (`num_ctx`)

Controls how many tokens the model can "see" at once (prompt + response combined).

```
Ollama VRAM-based defaults:
  < 24 GiB VRAM  →  4096 tokens  ← Jetson falls here
  24-48 GiB VRAM →  32768 tokens
  ≥ 48 GiB VRAM  →  262144 tokens
```

**Impact on memory**: Each additional 1K context tokens costs ~50-100 MB of KV cache memory (varies by model architecture). On the Jetson with ~1.9 GiB headroom, increasing context to 8K is feasible but 16K+ is risky.

**How to set** (per-request):
```json
{
  "model": "llama3.2:3b",
  "prompt": "...",
  "options": {
    "num_ctx": 4096
  }
}
```

**How to set globally** (environment variable):
```bash
OLLAMA_CONTEXT_LENGTH=8192 ollama serve
```

**How to set via Modelfile** (persistent per-model):
```
FROM llama3.2:3b
PARAMETER num_ctx 4096
PARAMETER temperature 0.3
```

**Recommendation for Jetson**: Keep at 4096 (default). RAG chunks are small (512 tokens × 3 = ~1500 tokens + system prompt + question ≈ 2000 tokens total). 4K is sufficient. Only increase if ingesting larger documents or needing longer responses.

### Response Length (`num_predict`)

Controls maximum number of tokens the model will generate in its response.

| Setting | Default | Effect |
|---------|---------|--------|
| `num_predict` | 128 (Ollama default) | Maximum response tokens |
| `-1` | Unlimited (until context fills) | No token limit |
| `-2` | Fill entire context | Uses all remaining context |

**How to set**:
```json
{
  "model": "llama3.2:3b",
  "prompt": "...",
  "options": {
    "num_predict": 512
  }
}
```

**GOTCHA — Thinking models (qwen3, deepseek-r1)**:
Models with "thinking" mode emit a hidden `<think>` block that counts against `num_predict`. If `num_predict=500` and the model uses 500 tokens thinking, the visible response is **empty**. Fix: Set `num_predict` to at least 2000 for thinking models, or disable thinking:
```json
{
  "model": "qwen3:8b",
  "prompt": "...",
  "think": false,
  "options": {
    "num_predict": 2000
  }
}
```

**Recommendation for AMBOT RAG**: Set `num_predict` to 512-1024. RAG answers should be concise. The default of 128 may truncate longer answers with citations.

### Temperature, Top-P, Top-K

| Parameter | Default | Description | Recommendation |
|-----------|---------|-------------|----------------|
| `temperature` | 0.8 | Randomness (0=deterministic, 1=creative) | 0.3 for RAG (factual answers) |
| `top_p` | 0.9 | Nucleus sampling threshold | 0.9 (default is fine) |
| `top_k` | 40 | Top-K token sampling | 40 (default is fine) |
| `repeat_penalty` | 1.1 | Penalizes repetition | 1.1 (default is fine) |

### All Ollama Options

Pass via `"options": {}` in API calls:

```json
{
  "options": {
    "num_ctx": 4096,
    "num_predict": 512,
    "temperature": 0.3,
    "top_p": 0.9,
    "top_k": 40,
    "repeat_penalty": 1.1,
    "seed": 42,
    "num_gpu": 99,
    "num_thread": 4
  }
}
```

### API Endpoints

| Endpoint | Use Case | Notes |
|----------|----------|-------|
| `/api/generate` | Raw prompt completion | Current AMBOT usage. Simple prompt → response. |
| `/api/chat` | Multi-turn conversation | Messages array with roles. Better for chat UIs. |
| `/api/tags` | List available models | Health check, model discovery. |
| `/api/ps` | Show loaded models | Check memory usage, context allocation. |
| `/api/show` | Model info | Parameters, template, size. |

### Memory Management

| Env Variable | Default | Description |
|--------------|---------|-------------|
| `OLLAMA_MAX_LOADED_MODELS` | 1 (GPU) | Max models in memory. Keep at 1 on Jetson. |
| `OLLAMA_NUM_PARALLEL` | 1 | Concurrent requests per model. Keep at 1 on Jetson. |
| `OLLAMA_KEEP_ALIVE` | 5m | How long loaded model stays in memory. |
| `OLLAMA_CONTEXT_LENGTH` | auto | Global context length override. |

**Jetson recommendation**: Leave defaults. One model loaded at a time, one request at a time. The 5-minute keep-alive means the model unloads after inactivity, freeing ~2 GB for other tasks.

---

## Thinking Models (Important Gotcha)

### What Are Thinking Models?

Some models (qwen3, deepseek-r1, gpt-oss) emit a hidden reasoning trace before their answer. In Ollama, this shows up as a separate `thinking` field in the response.

### The Token Budget Problem

**Thinking tokens count against `num_predict`**. If you set `num_predict=500` and the model "thinks" for 500 tokens, the visible response is empty.

### How to Handle

1. **Disable thinking** (simplest, recommended for RAG):
   ```json
   {"think": false}
   ```

2. **Increase num_predict** (if you want thinking):
   ```json
   {"options": {"num_predict": 2000}}
   ```

3. **Use budget_tokens** (qwen3 specific):
   Limits thinking length separately from response.

### Supported Thinking Models

| Model | Think Field | Notes |
|-------|-------------|-------|
| qwen3 | Boolean | Default: enabled. Use `think: false` to disable. |
| deepseek-r1 | Boolean | Default: enabled. |
| gpt-oss | "low"/"medium"/"high" | Cannot fully disable. |

**AMBOT recommendation**: Don't use thinking models for RAG. The overhead isn't worth it for retrieval-based Q&A. If testing qwen3, always set `think: false`.

---

## Model Selection for Jetson Orin Nano

### Memory Budget

```
Total:     7.4 GiB (unified CPU/GPU)
OS + services: ~1.5 GiB
Docker RAG:    ~0.15 GiB
Available:     ~5.7 GiB
Ollama overhead: ~0.5 GiB
Model budget:  ~3-4 GiB (including KV cache)
```

### Recommended Models (Ollama)

| Model | Params | Size (Q4) | Context | Quality | Notes |
|-------|--------|-----------|---------|---------|-------|
| **llama3.2:3b** | 3.2B | 2.0 GB | 128K | Good | **Current choice**. Good context grounding. |
| **phi3:mini** | 3.8B | 2.2 GB | 128K | Good | Microsoft. Strong reasoning for size. |
| **qwen2.5:3b** | 3B | 1.9 GB | 32K | Good | Alibaba. Multilingual. No thinking mode. |
| **gemma2:2b** | 2.6B | 1.6 GB | 8K | Decent | Google. Smallest footprint. |
| **smollm2:1.7b** | 1.7B | 1.0 GB | 8K | Basic | HuggingFace. Ultra-lightweight. |
| **tinyllama** | 1.1B | 0.6 GB | 2K | Poor | Hallucinated badly in testing. Not recommended. |

### Models to Avoid on Jetson

| Model | Why |
|-------|-----|
| Any 7B+ model | Exceeds memory budget (4-5 GB for Q4) |
| qwen3:8b | 5.2 GB + thinking overhead |
| llama3.1:8b | 4.9 GB, tight fit |
| Any fp16 model | 2x the size of Q4 quantized |

### Quantization Guide

| Quantization | Size vs FP16 | Quality | Notes |
|--------------|-------------|---------|-------|
| Q2_K | ~25% | Poor | Too lossy for most uses |
| Q3_K_M | ~35% | Usable | Minimum recommended |
| **Q4_K_M** | ~45% | Good | **Best balance** (Ollama default) |
| Q5_K_M | ~55% | Very good | If memory allows |
| Q6_K | ~65% | Excellent | Near-original quality |
| Q8_0 | ~80% | Near-perfect | Large, limited to 2B models on Jetson |
| FP16 | 100% | Original | Too large for Jetson |

---

## HuggingFace Configuration

### When to Use HuggingFace vs Ollama

| Factor | Ollama | HuggingFace |
|--------|--------|-------------|
| Setup complexity | Simple (one binary) | Complex (PyTorch + transformers) |
| GPU acceleration | Automatic (llama.cpp/CUDA) | Requires NVIDIA PyTorch wheels for Jetson |
| Model format | GGUF (quantized) | SafeTensors/PyTorch (larger) |
| Memory efficiency | Better (GGUF quantization) | Worse (FP16 minimum usually) |
| Model variety | Curated library | Full HuggingFace Hub |
| Fine-tuned models | Limited | Full support |
| Inference speed | Faster (optimized C++) | Slower (Python overhead) |

**Recommendation**: Use Ollama for production. Use HuggingFace only if you need a specific model not available in Ollama's library, or for fine-tuning experiments.

### HuggingFace Key Parameters

```python
# Generation parameters (transformers pipeline)
max_new_tokens = 512      # Max response tokens (like num_predict)
temperature = 0.3         # Same as Ollama
top_p = 0.9               # Nucleus sampling
top_k = 40                # Top-K sampling
do_sample = True          # Enable sampling (False = greedy)
repetition_penalty = 1.1  # Penalize repeats

# Model loading parameters
torch_dtype = "float16"   # FP16 for GPU (saves memory)
device_map = "auto"       # Let accelerate decide CPU/GPU split
low_cpu_mem_usage = True   # Important for Jetson
```

### PyTorch on Jetson

Standard pip `torch` does NOT have CUDA support on aarch64. You must use:
- **NVIDIA PyTorch wheels** from `https://developer.download.nvidia.com/compute/redist/jp/`
- **L4T Docker images** (e.g., `nvcr.io/nvidia/l4t-pytorch:r36.4.0-pth2.5-py3`)

The current RAG Docker container uses `python:3.11-slim` with CPU-only PyTorch. To add HuggingFace GPU support, the Docker image would need to change to an L4T base (+4-7 GB image size).

---

## Configuration Recommendations for AMBOT

### Immediate (current config is mostly fine)

1. Add `num_predict: 512` to Ollama API calls (default 128 may truncate RAG answers)
2. Add `num_ctx: 4096` explicitly (document the choice, don't rely on VRAM auto-detect)
3. Update config.py default `LLM_MODEL` from `tinyllama` to `llama3.2:3b`

### Future (when testing other models)

1. Always test with the RAG pipeline, not just raw prompts
2. For thinking models (qwen3): always set `think: false` for RAG
3. Monitor memory with `ollama ps` and `free -h` during testing
4. Keep `OLLAMA_MAX_LOADED_MODELS=1` to prevent OOM

---

## Quick Reference: Testing a New Model

```bash
# 1. Pull the model
ssh jetson "ollama pull phi3:mini"

# 2. Check size
ssh jetson "ollama list"

# 3. Quick test (raw)
ssh jetson "echo 'What is AMBOT?' | ollama run phi3:mini"

# 4. Check memory when loaded
ssh jetson "ollama ps && free -h"

# 5. Test through RAG API
curl -X POST http://10.33.255.82:8000/api/ask \
  -H 'Content-Type: application/json' \
  -d '{"question": "What are the components of AMBOT?"}'

# 6. Unload model (free memory)
ssh jetson "curl -s http://localhost:11434/api/generate -d '{\"model\":\"phi3:mini\",\"keep_alive\":0}'"
```

---

*This is a living document. Update when testing new models or discovering new gotchas.*
