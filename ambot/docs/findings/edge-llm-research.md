# Edge LLM Research: Findings from ExudeAI & Web Research

> Date: 2026-02-19 (Session 14)
> Source: `~/exudeai/` repository exploration + web research
> Scope: Model selection, quantization, context management, deployment patterns for Jetson Orin Nano

---

## Summary

This document captures research findings relevant to AMBOT's LLM deployment on Jetson Orin Nano (7.4 GiB unified RAM, CUDA 12.6, Compute 8.7). The focus is on **operational configuration** -- selecting the right models, parameters, and deployment patterns -- NOT fine-tuning or training (out of scope for AMBOT).

---

## 1. Small Language Models (SLMs) for Edge Deployment

### Recommended Models for Jetson Orin Nano

| Model | Parameters | GGUF Size (Q4_K_M) | Context | License | Notes |
|-------|-----------|-------------------|---------|---------|-------|
| **llama3.2:3b** | 3.2B | 2.0 GB | 8K | Meta | **Current choice**. Good context grounding. |
| **phi3:mini** | 3.8B | 2.2 GB | 4K/128K | MIT | Strong reasoning per parameter. |
| **qwen2.5:3b** | 3B | 1.9 GB | 32K | Apache 2.0 | Long context, good code/reasoning. |
| **gemma2:2b** | 2.6B | 1.6 GB | 8K | Google | Smallest with good quality. |
| **smollm2:1.7b** | 1.7B | 1.0 GB | 8K | Apache 2.0 | Ultra-lightweight, basic tasks only. |
| **tinyllama** | 1.1B | 637 MB | 2K | Apache 2.0 | Tested -- hallucinated on RAG. Not recommended. |

### Model Selection Criteria

1. **RAM budget**: Jetson has 7.4 GiB shared CPU/GPU. Ollama + model + KV cache + Docker stack must fit.
   - Current usage: ~5.5 GiB (Docker 148 MiB + Ollama ~2 GB model + KV cache)
   - Headroom: ~1.9 GiB
   - Models up to ~3 GB GGUF are safe; 4+ GB models may cause OOM under load

2. **RAG quality**: Model must ground answers in retrieved context, not hallucinate.
   - llama3.2:3b: Accurately cites sources from retrieved chunks
   - tinyllama: Ignored context, invented answers
   - **Minimum recommended: 3B parameters** for RAG use cases

3. **Context window**: RAG queries need ~2-3K tokens (system + context + question + answer).
   - 4K context is sufficient for current RAG setup
   - 8K gives headroom for larger document chunks
   - 32K+ context costs significant KV cache memory (not recommended on 7.4 GiB)

### Hardware-Model Mapping (from ExudeAI research)

| Device | RAM | Recommended Model | Size | Latency |
|--------|-----|-------------------|------|---------|
| ESP32-S3 | 2 MB | Quantized TinyML | <250 KB | <100 ms |
| Raspberry Pi 5 | 4-8 GB | SmolLM / TinyLlama | ~500 MB | <100 ms |
| **Jetson Orin Nano** | **7.4 GB** | **Phi-3 / Qwen-1.5B-3B** | **2-4 GB** | **<10 ms/token** |
| Jetson Orin Nano Super | 8 GB | Multiple SLMs | 4-8 GB | <10 ms/token |

---

## 2. Quantization

### Key Findings

- **Q4_K_M is the recommended default** for Ollama on Jetson
  - 4x size reduction with excellent quality preservation
  - Standard format in Ollama model library

- **NF4 (4-bit NormalFloat)** is information-theoretically optimal for LLM weights
  - Places quantization levels at normal distribution quantiles (not uniform)
  - ~0.5 perplexity points better than FP4 on LLM benchmarks
  - Used internally by QLoRA for training; Ollama uses GGUF Q4_K_M for inference

- **Quantization level comparison** (from ExudeAI research):

| Level | Size Reduction | Quality | Use Case |
|-------|---------------|---------|----------|
| Q2_K | 8x | Lower | Extreme constraint (not recommended) |
| Q3_K_M | 6x | Good-Fair | Mobile with quality tradeoff |
| **Q4_K_M** | **4x** | **Excellent** | **Default recommendation** |
| Q5_K_M | 3x | Near-lossless | High accuracy required |
| Q6_K | 2.5x | Lossless | Quality critical |
| Q8_0 | 2x | Lossless | Minimal compression |

### Practical Implications for AMBOT

- All Ollama models are pre-quantized (typically Q4_K_M). No action needed.
- If custom models are ever added, use the GGUF export pipeline:
  `HuggingFace model -> merge adapters -> convert to GGUF F16 -> quantize Q4_K_M -> ollama create`
- Do NOT use Q2_K or Q3_K for RAG -- quality degradation causes worse context grounding.

---

## 3. Ollama Configuration for Edge

### Critical Parameters

| Parameter | Default | Recommended | Why |
|-----------|---------|-------------|-----|
| `num_ctx` | 2048 (Ollama) | 4096 | RAG needs ~2-3K for system+context+question. 4K is safe. |
| `num_predict` | 128 (Ollama) | 512 | Default 128 truncates RAG answers with citations. 512 is enough for concise answers. |
| `temperature` | 0.8 | 0.3 | Lower for factual RAG answers. Higher causes hallucination. |
| `top_p` | 0.9 | 0.9 | Fine as default. |
| `top_k` | 40 | 40 | Fine as default. |

### Thinking Model Gotcha

**Critical finding**: Models like `qwen3`, `deepseek-r1`, and `qwq` have a "thinking mode" enabled by default in Ollama. The thinking tokens (inside `<think>...</think>` blocks) **consume the `num_predict` budget**.

Example failure mode:
```
num_predict = 512
Model spends 500 tokens thinking in <think>...</think>
Only 12 tokens left for visible answer
User sees: empty or truncated response
```

**Mitigation options:**
1. Set `"think": false` in API options (disables thinking mode entirely)
2. Set `num_predict` very high (1024-2048) to accommodate both thinking and answer
3. Avoid thinking models for RAG (use standard models like llama3.2, phi3, gemma2)

**AMBOT recommendation**: Use standard (non-thinking) models for RAG. If thinking models are ever tested, always set `think: false` in the API payload.

### Memory Budget

```
Jetson Orin Nano: 7.4 GiB unified memory
- OS + system:          ~1.0 GiB
- Docker containers:    ~0.2 GiB (PostgreSQL + Redis + FastAPI)
- Ollama model (3B Q4): ~2.0 GiB
- KV cache (4K ctx):    ~0.4 GiB
- Compute buffers:      ~0.3 GiB
- Embedding model:      ~0.1 GiB (MiniLM, CPU)
─────────────────────────────────
Total:                  ~4.0 GiB
Headroom:               ~3.4 GiB

With 8K context:        +0.4 GiB KV cache
With nomic-embed-text:  +0.5 GiB (vs 0.1 GiB for MiniLM)
```

---

## 4. Context Window Management

### "Lost in the Middle" Problem

LLMs exhibit a U-shaped attention pattern (from ExudeAI research):
- **First position**: ~80% accuracy
- **Middle positions**: ~55% accuracy (20+ point drop)
- **Last position**: ~78% accuracy

**Practical impact for RAG**: Place the most relevant retrieved chunks at the **beginning** and **end** of the context, not in the middle.

### Current AMBOT RAG Prompt Structure

```
[System prompt]        ~100-200 tokens (beginning -- high attention)
[Source 1: most relevant]  -- beginning of context (high attention)
---
[Source 2: medium relevant]  -- middle (lower attention)
---
[Source 3: least relevant]  -- end of context before question
---
[Question]             ~50-100 tokens (end -- high attention)
```

**Recommendation**: The current `ask_with_context()` in `llm.py` already orders chunks by relevance score (most relevant first). This naturally puts the best context at the beginning where attention is highest.

### Token Budgeting

- **Rule of thumb**: 1 token ~ 0.75 words (English)
- **Current allocation** (4096 context window):
  - System prompt: ~200 tokens
  - Retrieved context (3 chunks): ~1,500-2,000 tokens
  - User question: ~100 tokens
  - Reserved for generation: 512 tokens (num_predict)
  - Buffer: ~1,200-1,800 tokens

This is comfortable. No need to increase context window unless chunks get much larger.

---

## 5. Embedding Model Options

### Current: all-MiniLM-L6-v2
- Dimensions: 384
- Context: 512 tokens
- Size: ~22 MB
- Speed: ~50 ms/embedding (CPU)
- MTEB score: ~58

### Alternative: nomic-embed-text v1.5
- Dimensions: 768 (with Matryoshka support for 512/256)
- Context: 8,192 tokens (16x current)
- Size: ~548 MB
- MTEB score: ~62
- Supports task-specific prefixes (`search_document:`, `search_query:`)

### Recommendation

Stick with **MiniLM for now**. The 16x context increase from nomic-embed-text is irrelevant when chunks are 512 tokens (current `CHUNK_SIZE`). The quality improvement (~4 MTEB points) is marginal for the AMBOT use case (small knowledge base, straightforward queries).

**When to switch**: If RAG quality is poor despite good chunks, or if chunk sizes increase significantly, evaluate nomic-embed-text. This is tracked in roadmap.md Phase 3.

---

## 6. Edge AI Architecture Patterns

### Three-Layer Cognitive Architecture (from ExudeAI)

| Layer | Time Scale | Function | Model Size | AMBOT Mapping |
|-------|-----------|----------|------------|---------------|
| 1: Edge/Telemetry | ms-seconds | Sensor processing, classification | <500M params | Pathfinder (LiDAR, camera, IMU) |
| 2: History Analysis | seconds-minutes | Pattern recognition, context building | 500M-3B | RAG retrieval + embedding |
| 3: Strategic | minutes-hours | Task routing, decision making | 3B+ | LLM generation (Ollama) |

**AMBOT already follows this pattern** (unintentionally):
- Layer 1: Pathfinder does real-time sensor processing (LiDAR scan → obstacle map, camera → face detection)
- Layer 2: RAG system stores and retrieves historical context (pgvector embeddings)
- Layer 3: Ollama LLM makes strategic decisions (answering questions, future: behavior selection)

### Integration Points (Future)

When Demo 3 is implemented:
- Pathfinder events (face detected, obstacle pattern) → trigger RAG queries
- RAG context (conversation history, knowledge) → inform LLM decisions
- LLM decisions → locomotion commands (via MCP server, Milestone 5)

---

## 7. Knowledge Distillation & Fine-Tuning (Reference Only)

> **Out of scope for AMBOT** -- documented here as reference for when/if needed.

### QLoRA Memory Requirements (for fine-tuning)

| Model | QLoRA VRAM | Full FT VRAM |
|-------|-----------|-------------|
| 1.5B | ~4 GB | ~12 GB |
| 3B | ~6 GB | ~24 GB |
| 7B | ~10 GB | ~60 GB |

The Jetson Orin Nano (7.4 GiB) could theoretically QLoRA-train a 1.5B model, but this is out of scope.

### Knowledge Distillation

- Teacher-student framework: Large model generates training data for small model
- 10-20x size reduction possible with teacher guidance
- Capability-specific distillation can extract targeted skills (e.g., "answer EECS questions")

**If AMBOT ever needs a custom model**: Distill from a larger model on a GPU server, export to GGUF, deploy via Ollama. Do NOT train on the Jetson.

---

## 8. Key Takeaways for AMBOT

1. **llama3.2:3b is a good choice** -- 3B parameters is the sweet spot for RAG quality on 7.4 GiB
2. **Test phi3:mini and qwen2.5:3b** -- similar size, potentially better for specific tasks
3. **Always set num_predict >= 512** for RAG (default 128 truncates answers)
4. **Always set num_ctx = 4096** (sufficient for current RAG, matches Jetson memory budget)
5. **Avoid thinking models** (qwen3, deepseek-r1) for RAG unless `think: false` is set
6. **Q4_K_M quantization** is the right level -- all Ollama models use this by default
7. **Stick with MiniLM embeddings** for now -- switch to nomic-embed-text only if quality demands it
8. **Context ordering matters** -- most relevant chunks should go first (current code does this)

---

## Source Documents (ExudeAI Repository)

| Topic | Path |
|-------|------|
| Edge LLM optimization | `~/exudeai/docs/findings/edgeai-llm-optimization-techniques.md` |
| Edge AI layer architecture | `~/exudeai/docs/findings/edge-ai-layer-architecture.md` |
| Base LLM selection | `~/exudeai/code-world-models-research/docs/findings/base-llm-selection.md` |
| Embedding model selection | `~/exudeai/rag-atc-testing/docs/findings/embedding-model-selection-2026.md` |
| Context window management | `~/exudeai/docs/findings/context-window-management-rag.md` |
| QLoRA specifics | `~/exudeai/fine-tuning-research/docs/findings/qlora-specifics.md` |
| LoRA fundamentals | `~/exudeai/fine-tuning-research/docs/findings/lora-fundamentals.md` |
| Embedding model comparison | `~/exudeai/docs/findings/embedding-model-comparison-retrieval.md` |

---

*See also: [llm-configuration-guide.md](llm-configuration-guide.md) for Ollama/HuggingFace API parameter reference.*
