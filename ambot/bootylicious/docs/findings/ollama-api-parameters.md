# Ollama API Parameters for Constrained Hardware

**Date**: 2026-02-19
**Target**: Jetson Orin Nano (7.4 GiB unified CPU/GPU memory)
**Ollama Version**: 0.6.2+ (installed on Jetson)

---

## 1. Context Window Configuration (`num_ctx`)

### What It Does
`num_ctx` sets the maximum number of tokens the model can access in memory as context for generating the next token. This includes both the input prompt/conversation history and the generated output.

### Defaults (Automatic, Based on VRAM)
Ollama auto-selects context length based on available VRAM:

| VRAM | Default Context |
|------|----------------|
| < 24 GiB | **4096 tokens (4K)** |
| 24-48 GiB | 32K tokens |
| >= 48 GiB | 256K tokens |

**For Jetson Orin Nano (7.4 GiB): Default is 4096 tokens.**

### How to Set It

**Per-request via API** (recommended for fine control):
```bash
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.2:3b",
  "prompt": "Why is the sky blue?",
  "options": {
    "num_ctx": 4096
  }
}'
```

**Global default via environment variable** (server-wide):
```bash
# In /etc/systemd/system/ollama.service.d/override.conf
[Service]
Environment="OLLAMA_CONTEXT_LENGTH=4096"
```

**Via CLI interactive session**:
```
/set parameter num_ctx 4096
```

**Via Modelfile** (baked into a custom model):
```
FROM llama3.2:3b
PARAMETER num_ctx 4096
```

### VRAM Impact
Context window is the **primary knob for memory usage** after model size. The KV cache grows linearly with context length. Rough formula:

```
Total VRAM = Model Weights + KV Cache(num_ctx) + Compute Buffers
```

For a 3B Q4_K_M model (~2 GB weights) on the Jetson:
- `num_ctx 2048`: ~2.5 GB total (safe, lots of headroom)
- `num_ctx 4096`: ~3.0 GB total (default, good balance)
- `num_ctx 8192`: ~4.0 GB total (workable but tighter)
- `num_ctx 16384`: ~6.0 GB total (risky on 7.4 GiB unified memory)

**Recommendation for Jetson**: Keep `num_ctx` at 2048-4096. Only increase if the task genuinely requires longer context. Check with `ollama ps` to verify GPU offloading.

### Verifying Context Length and GPU Offload
```bash
ollama ps
```
```
NAME             ID              SIZE      PROCESSOR    CONTEXT    UNTIL
llama3.2:3b      a2af6cc3eb7f    3.0 GB    100% GPU     4096       2 minutes from now
```

If `PROCESSOR` shows `48%/52% CPU/GPU`, the model is partially offloaded to CPU (slower). Reduce `num_ctx` or use a smaller model.

---

## 2. `num_predict` Parameter

### What It Does
Controls the **maximum number of tokens to generate** in the response. This is the output token budget.

### Default
**-1 (infinite generation)** -- the model generates until it hits a stop token or the context window fills up.

### Setting It
```bash
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.2:3b",
  "prompt": "Summarize this in 2 sentences.",
  "options": {
    "num_predict": 256
  }
}'
```

### Why You Might Want to Limit It
- **Latency control**: On the Jetson, a 3B model generates ~20-40 tokens/sec. Setting `num_predict: 128` caps response time to ~3-6 seconds.
- **Memory stability**: Prevents runaway generation that could fill the KV cache.
- **Robot use case**: For AMBOT, responses should be short actions/decisions, not essays. `num_predict: 128-256` is usually sufficient.

### CRITICAL GOTCHA: Thinking Models and `num_predict`

**This is the most important gotcha for models with thinking/reasoning mode (Qwen3, DeepSeek R1).**

When thinking mode is enabled, the `<think>...</think>` reasoning block tokens **count against the `num_predict` budget**. If the model spends all its tokens thinking, the visible response will be **empty or truncated**.

Example scenario with `num_predict: 128`:
1. Model receives prompt
2. Model begins `<think>` reasoning (hidden from user) -- uses 120 tokens
3. Model closes `</think>` -- only 8 tokens left for actual response
4. Response is truncated or empty

**Workarounds**:
- Set `num_predict` much higher for thinking models (e.g., 2048-4096)
- Disable thinking mode: `"think": false` in API request
- Use `/set nothink` in CLI sessions
- Use non-thinking models (llama3.2, qwen2.5) for constrained hardware where token budgets matter

---

## 3. Sampling Parameters

### Temperature
| | |
|---|---|
| **What** | Controls randomness/creativity of output |
| **Default** | **0.8** |
| **Range** | 0.0 - 2.0 |
| **Low (0.1-0.3)** | Deterministic, focused, repetitive |
| **Medium (0.5-0.8)** | Balanced creativity and coherence |
| **High (1.0-2.0)** | Creative, diverse, potentially incoherent |

**Recommendation for robot tasks**: Use **0.3-0.5** for decision-making (wandering, navigation). Use **0.7-0.8** for conversational/creative tasks.

### top_k
| | |
|---|---|
| **What** | Limits token selection to top K most probable tokens |
| **Default** | **40** |
| **Low (10)** | More conservative, less diverse |
| **High (100)** | More diverse answers |

### top_p (Nucleus Sampling)
| | |
|---|---|
| **What** | Selects from smallest set of tokens whose cumulative probability >= p |
| **Default** | **0.9** |
| **Low (0.5)** | Focused and conservative |
| **High (0.95)** | More diverse |

### min_p
| | |
|---|---|
| **What** | Minimum probability threshold relative to most likely token |
| **Default** | **0.0** (disabled) |
| **Recommended** | 0.05 for balanced quality/variety |

### repeat_penalty
| | |
|---|---|
| **What** | Penalizes token repetition |
| **Default** | **1.1** |
| **Range** | 0.0 - 2.0 (higher = stronger penalty) |

### repeat_last_n
| | |
|---|---|
| **What** | How far back the model looks to detect repetition |
| **Default** | **64** |
| **Special values** | 0 = disabled, -1 = entire context (num_ctx) |

### Recommended Settings for Constrained Systems
```json
{
  "options": {
    "temperature": 0.4,
    "top_k": 40,
    "top_p": 0.9,
    "repeat_penalty": 1.1,
    "num_predict": 256,
    "num_ctx": 4096
  }
}
```

Lower temperature saves nothing on compute but produces more predictable outputs, which matters when the model is making decisions for a robot.

---

## 4. Modelfile Customization

### Purpose
Modelfiles let you create a custom model variant with baked-in defaults for parameters, system prompt, and template. This means you do not need to pass `options` on every API call.

### Syntax

```dockerfile
# Modelfile for AMBOT navigation assistant
FROM llama3.2:3b

# System prompt
SYSTEM """You are a navigation assistant for a mobile robot.
Give concise directional commands. Be brief."""

# Parameters
PARAMETER temperature 0.4
PARAMETER num_ctx 4096
PARAMETER num_predict 256
PARAMETER top_k 40
PARAMETER top_p 0.9
PARAMETER repeat_penalty 1.1
PARAMETER stop "<|eot_id|>"
```

### Available Directives

| Directive | Required | Description |
|-----------|----------|-------------|
| `FROM` | Yes | Base model (e.g., `llama3.2:3b`) |
| `PARAMETER` | No | Set runtime parameters |
| `TEMPLATE` | No | Custom prompt template (Go template syntax) |
| `SYSTEM` | No | System message |
| `ADAPTER` | No | LoRA/QLoRA adapter path |
| `LICENSE` | No | License text |
| `MESSAGE` | No | Few-shot examples (role + message pairs) |
| `REQUIRES` | No | Minimum Ollama version |

### Creating and Using

```bash
# Create the custom model
ollama create ambot-nav -f ./Modelfile

# Run it
ollama run ambot-nav

# Use via API (no options needed -- defaults are baked in)
curl http://localhost:11434/api/generate -d '{
  "model": "ambot-nav",
  "prompt": "LiDAR shows obstacle 0.5m ahead, clear path 45 degrees left"
}'

# View the modelfile of any model
ollama show --modelfile llama3.2:3b
```

### Few-Shot Examples via MESSAGE

```dockerfile
FROM llama3.2:3b
SYSTEM "You are a robot navigation assistant."

MESSAGE user "Obstacle ahead, clear left and right"
MESSAGE assistant "TURN_LEFT 45"
MESSAGE user "Path clear ahead, face detected at 30 degrees right"
MESSAGE assistant "TURN_RIGHT 30"
```

---

## 5. Memory Management

### Key Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `OLLAMA_MAX_LOADED_MODELS` | 3 * num_GPUs (or 3 for CPU) | Max models in memory simultaneously |
| `OLLAMA_NUM_PARALLEL` | 1 | Parallel requests per model |
| `OLLAMA_CONTEXT_LENGTH` | Auto (see VRAM table above) | Global default context size |
| `OLLAMA_MAX_QUEUE` | 512 | Max queued requests before 503 error |
| `OLLAMA_KEEP_ALIVE` | 5m | How long models stay loaded after last request |
| `OLLAMA_FLASH_ATTENTION` | 0 | Enable flash attention (reduces memory) |
| `OLLAMA_KV_CACHE_TYPE` | f16 | KV cache quantization type |

### Critical Settings for Jetson (7.4 GiB)

**Set in `/etc/systemd/system/ollama.service.d/override.conf`:**
```ini
[Service]
Environment="OLLAMA_HOST=0.0.0.0"
Environment="OLLAMA_MAX_LOADED_MODELS=1"
Environment="OLLAMA_NUM_PARALLEL=1"
Environment="OLLAMA_CONTEXT_LENGTH=4096"
Environment="OLLAMA_FLASH_ATTENTION=1"
Environment="OLLAMA_KV_CACHE_TYPE=q8_0"
```

After changes:
```bash
sudo systemctl daemon-reload
sudo systemctl restart ollama
```

### Memory Impact Breakdown

**`OLLAMA_MAX_LOADED_MODELS=1`** (CRITICAL for Jetson):
- Default is 3, meaning Ollama may try to keep 3 models in memory
- On 7.4 GiB, this will cause out-of-memory or CPU fallback
- Set to 1 to ensure only one model is loaded at a time

**`OLLAMA_NUM_PARALLEL=1`** (CRITICAL for Jetson):
- Each parallel slot multiplies the KV cache memory
- `OLLAMA_NUM_PARALLEL=2` with 4K context = 8K worth of KV cache memory
- RAM scales as: `OLLAMA_NUM_PARALLEL * OLLAMA_CONTEXT_LENGTH`
- Keep at 1 for the Jetson

**`OLLAMA_FLASH_ATTENTION=1`** (RECOMMENDED):
- Significantly reduces memory usage as context grows
- Required to enable KV cache quantization
- Modern models support this

**`OLLAMA_KV_CACHE_TYPE`** (RECOMMENDED):
- `f16` (default): Full precision KV cache
- `q8_0`: ~50% memory savings on KV cache, negligible quality loss (recommended)
- `q4_0`: ~75% memory savings, small-to-medium quality loss at large contexts
- Note: Requires `OLLAMA_FLASH_ATTENTION=1`
- Models with high GQA count (Qwen2) may see more quality impact from quantization

### Model Loading/Unloading Behavior

1. **Auto-load**: First request to a model loads it into VRAM
2. **keep_alive timer**: After last request, model stays loaded for 5 minutes (default)
3. **Auto-unload**: When a new model is requested and memory is insufficient, idle models are unloaded
4. **Manual preload**: Send empty prompt to load model into memory ahead of time
5. **Manual unload**: `ollama stop <model>` or `keep_alive: 0` in API

```bash
# Preload model
curl http://localhost:11434/api/generate -d '{"model": "llama3.2:3b"}'

# Keep model loaded indefinitely
curl http://localhost:11434/api/generate -d '{"model": "llama3.2:3b", "keep_alive": -1}'

# Unload model immediately
curl http://localhost:11434/api/generate -d '{"model": "llama3.2:3b", "keep_alive": 0}'

# CLI unload
ollama stop llama3.2:3b
```

---

## 6. API Endpoints: `/api/generate` vs `/api/chat`

### `/api/generate` -- Text Completion

**Use for**: Single-shot prompts, completions, one-off tasks.

```bash
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.2:3b",
  "prompt": "LiDAR scan: obstacle 0.3m ahead. What should the robot do?",
  "stream": false,
  "options": {
    "temperature": 0.3,
    "num_predict": 128
  }
}'
```

**Response field**: `"response": "Turn left 45 degrees to avoid the obstacle."`

### `/api/chat` -- Conversational

**Use for**: Multi-turn conversations, maintaining context, tool calling.

```bash
curl http://localhost:11434/api/chat -d '{
  "model": "llama3.2:3b",
  "messages": [
    {"role": "system", "content": "You are a robot navigation assistant."},
    {"role": "user", "content": "Obstacle 0.3m ahead, clear path at 45 degrees left."}
  ],
  "stream": false,
  "options": {
    "temperature": 0.3,
    "num_predict": 128
  }
}'
```

**Response field**: `"message": {"role": "assistant", "content": "Turn left 45 degrees."}`

### Key Differences

| Feature | `/api/generate` | `/api/chat` |
|---------|-----------------|-------------|
| Input format | `prompt` (string) | `messages` (array of role/content objects) |
| Conversation history | Manual (deprecated `context` param) | Built-in via messages array |
| Tool calling | Not available | Supported |
| System message | Via `system` param or raw template | Via `{"role": "system"}` message |
| Response field | `response` (string) | `message` (object with role/content) |
| Thinking support | `thinking` field in response | `message.thinking` field |
| Best for | One-shot tasks | Multi-turn dialogue, agents |

### Common Parameters (Both Endpoints)

| Parameter | Type | Description |
|-----------|------|-------------|
| `model` | string | Required. Model name. |
| `stream` | bool | Default true. Set false for single response. |
| `format` | string/object | `"json"` or JSON schema for structured output |
| `options` | object | All model parameters (num_ctx, temperature, etc.) |
| `keep_alive` | string/int | Model retention time (e.g., "5m", 0, -1) |
| `think` | bool/string | Enable thinking mode for supported models |

### Structured Output (Both Endpoints)

```bash
curl http://localhost:11434/api/chat -d '{
  "model": "llama3.2:3b",
  "messages": [{"role": "user", "content": "What direction should the robot turn?"}],
  "format": {
    "type": "object",
    "properties": {
      "action": {"type": "string", "enum": ["FORWARD", "LEFT", "RIGHT", "STOP"]},
      "degrees": {"type": "integer"},
      "speed": {"type": "integer"}
    },
    "required": ["action"]
  },
  "stream": false
}'
```

---

## 7. Models Suitable for Jetson Orin Nano (7.4 GiB)

### Memory Budget Analysis

Total unified memory: **7.4 GiB**
- OS + services overhead: ~1.0-1.5 GiB
- Docker containers (PostgreSQL, Redis, FastAPI): ~0.5-1.0 GiB
- **Available for Ollama: ~5.0-5.5 GiB**

This means: **Model weights + KV cache (4K context) must fit in ~5 GiB**.

### Model Comparison Table

| Model | Params | Default Quant | Size | 4K KV Cache | Total Est. | Quality | Notes |
|-------|--------|---------------|------|-------------|------------|---------|-------|
| **llama3.2:3b** | 3B | Q4_K_M | 2.0 GB | ~0.8 GB | **~3.0 GB** | Good | Currently used. Solid general purpose. |
| **llama3.2:1b** | 1B | Q4_K_M | 1.3 GB | ~0.5 GB | **~2.0 GB** | Fair | Very fast, but less capable. |
| **qwen2.5:3b** | 3B | Q4_K_M | 1.9 GB | ~0.8 GB | **~3.0 GB** | Good | Strong multilingual, Apache 2.0 (note: 3B is Qwen license). |
| **qwen2.5:7b** | 7B | Q4_K_M | 4.7 GB | ~1.2 GB | **~6.0 GB** | Very Good | Tight fit. May need num_ctx=2048. |
| **qwen3:4b** | 4B | Q4_K_M | 2.5 GB | ~0.8 GB | **~3.5 GB** | Good+ | Thinking mode available. 256K max context. |
| **qwen3:1.7b** | 1.7B | Q4_K_M | 1.4 GB | ~0.5 GB | **~2.0 GB** | Fair+ | Thinking capable, very lightweight. |
| **phi3:3.8b** | 3.8B | Q4_K_M | 2.2 GB | ~0.8 GB | **~3.2 GB** | Good | Strong reasoning for size. 128K max. |
| **gemma2:2b** | 2B | Q4_K_M | 1.6 GB | ~0.6 GB | **~2.4 GB** | Fair+ | Only 8K max context. Google model. |
| **smollm2:1.7b** | 1.7B | Q4_K_M | 1.8 GB | ~0.5 GB | **~2.5 GB** | Fair | 8K max context. HuggingFace model. |

### Quantization Levels Explained

Quantization reduces model precision to decrease size and VRAM usage.

| Quantization | Bits | Size vs FP16 | Quality | Recommended? |
|-------------|------|-------------|---------|-------------|
| **Q2_K** | 2-bit | ~15% | Poor -- noticeable degradation | No (emergency only) |
| **Q3_K_S** | 3-bit (small) | ~20% | Below average | No |
| **Q3_K_M** | 3-bit (medium) | ~22% | Fair | Marginal |
| **Q3_K_L** | 3-bit (large) | ~24% | Fair+ | Marginal |
| **Q4_0** | 4-bit | ~26% | Good | Yes |
| **Q4_K_S** | 4-bit (small) | ~26% | Good | Yes |
| **Q4_K_M** | 4-bit (medium) | ~28% | **Good+ (sweet spot)** | **Yes (default, recommended)** |
| **Q5_0** | 5-bit | ~34% | Very Good | Yes (if fits) |
| **Q5_K_M** | 5-bit (medium) | ~35% | Very Good | Yes (if fits) |
| **Q6_K** | 6-bit | ~40% | Excellent | Only if fits |
| **Q8_0** | 8-bit | ~50% | Near-FP16 | Only small models |
| **FP16** | 16-bit | 100% | Reference | Too large for Jetson |

**Q4_K_M is the default and recommended for Jetson.** It offers the best quality-to-size ratio.

### Detailed Size Table: Quantization Variants for Candidate Models

**llama3.2:3b variants:**
| Quant | Size |
|-------|------|
| q2_K | 1.4 GB |
| q3_K_M | 1.7 GB |
| q4_K_M | 2.0 GB (default) |
| q5_K_M | 2.3 GB |
| q6_K | 2.6 GB |
| q8_0 | 3.4 GB |
| fp16 | 6.4 GB |

**qwen2.5:3b variants:**
| Quant | Size |
|-------|------|
| q2_K | 1.3 GB |
| q3_K_M | 1.6 GB |
| q4_K_M | 1.9 GB (default) |
| q5_K_M | 2.2 GB |
| q6_K | 2.5 GB |
| q8_0 | 3.3 GB |
| fp16 | 6.2 GB |

**qwen3:4b variants:**
| Quant | Size |
|-------|------|
| q4_K_M | 2.6 GB |
| q8_0 | 4.4 GB |
| fp16 | 8.1 GB (too large) |

**phi3:3.8b variants:**
| Quant | Size |
|-------|------|
| q2_K | 1.4 GB |
| q3_K_M | 2.0 GB |
| q4_K_M | 2.2 GB (default) |
| q5_K_M | 2.8 GB |
| q6_K | 3.1 GB |
| q8_0 | 4.1 GB |
| fp16 | 7.6 GB (too large) |

### Recommendation for AMBOT

**Primary model**: `llama3.2:3b` (Q4_K_M, 2.0 GB) -- already proven, good quality, no thinking overhead.

**If you need better quality**: `qwen2.5:3b` (Q4_K_M, 1.9 GB) -- marginally smaller, strong instruction following.

**If you want thinking/reasoning**: `qwen3:4b` (Q4_K_M, 2.6 GB) -- but disable thinking mode for latency-sensitive tasks (`"think": false`).

**If you need faster inference**: `llama3.2:1b` (Q4_K_M, 1.3 GB) -- fastest, but less capable.

**Avoid**: `qwen2.5:7b` (4.7 GB weights alone leaves very little room for KV cache + system overhead).

---

## 8. Thinking/Reasoning Mode Gotchas

### How Thinking Mode Works
Some models (Qwen3, DeepSeek R1, GPT-OSS) support a "thinking" mode where the model reasons through a problem before answering. The reasoning is emitted in a separate `thinking` field.

### Supported Models
- **Qwen 3** (all sizes): `"think": true` / `"think": false`
- **DeepSeek R1**: `"think": true` / `"think": false`
- **DeepSeek V3.1**: `"think": true` / `"think": false`
- **GPT-OSS**: `"think": "low"` / `"medium"` / `"high"` (booleans ignored)

### Thinking is ON by Default
**Important**: For supported models, thinking is **enabled by default** in both CLI and API. You must explicitly disable it if you do not want it.

### API Usage
```bash
# Thinking enabled (default for thinking models)
curl http://localhost:11434/api/chat -d '{
  "model": "qwen3:4b",
  "messages": [{"role": "user", "content": "How do I navigate around an obstacle?"}],
  "think": true,
  "stream": false
}'
# Response includes: message.thinking (reasoning trace) + message.content (answer)

# Thinking disabled
curl http://localhost:11434/api/chat -d '{
  "model": "qwen3:4b",
  "messages": [{"role": "user", "content": "How do I navigate around an obstacle?"}],
  "think": false,
  "stream": false
}'
# Response includes only: message.content (answer, no thinking)
```

### The `num_predict` Token Budget Problem

**This is the critical gotcha.** When thinking is enabled:

1. The model generates `<think>...</think>` tokens first (hidden reasoning)
2. Then generates the visible response
3. **Both thinking AND response tokens count toward `num_predict`**

If `num_predict` is set too low (e.g., 128), the model may spend all 128 tokens thinking and produce an **empty or truncated visible response**.

**Mitigation strategies:**
- **Disable thinking** for latency-sensitive or token-constrained scenarios: `"think": false`
- **Increase `num_predict`** significantly when using thinking mode (e.g., 2048+)
- **Use non-thinking models** (llama3.2, qwen2.5, phi3) when `num_predict` needs to be small
- **Monitor token usage**: The response includes `eval_count` (total tokens generated) and `prompt_eval_count`

### Latency Impact on Constrained Hardware
Thinking mode **doubles or triples generation time** because the model must generate the reasoning trace before the answer. On a Jetson generating ~30 tokens/sec:
- Without thinking: 128 tokens = ~4 seconds
- With thinking: 128 tokens thinking + 128 tokens answer = ~8 seconds

For real-time robot navigation, **disable thinking mode**.

### CLI Quick Reference
```bash
# Run with thinking
ollama run qwen3:4b --think "What is 17 * 23?"

# Run without thinking
ollama run qwen3:4b --think=false "What is 17 * 23?"

# Hide thinking trace (still thinks, but only shows answer)
ollama run qwen3:4b --hidethinking "What is 17 * 23?"

# Interactive session toggles
/set think
/set nothink
```

---

## Quick Reference: Complete API Call for Jetson

```bash
# Optimal settings for AMBOT on Jetson Orin Nano
curl http://localhost:11434/api/chat -d '{
  "model": "llama3.2:3b",
  "messages": [
    {
      "role": "system",
      "content": "You are a mobile robot navigation assistant. Give concise commands."
    },
    {
      "role": "user",
      "content": "LiDAR: obstacle 0.3m ahead. Camera: face detected 30 degrees right. IMU: heading 270. What should I do?"
    }
  ],
  "stream": false,
  "options": {
    "num_ctx": 4096,
    "num_predict": 256,
    "temperature": 0.3,
    "top_k": 40,
    "top_p": 0.9,
    "repeat_penalty": 1.1
  }
}'
```

### Recommended Ollama Server Configuration for Jetson

```ini
# /etc/systemd/system/ollama.service.d/override.conf
[Service]
Environment="OLLAMA_HOST=0.0.0.0"
Environment="OLLAMA_MAX_LOADED_MODELS=1"
Environment="OLLAMA_NUM_PARALLEL=1"
Environment="OLLAMA_CONTEXT_LENGTH=4096"
Environment="OLLAMA_FLASH_ATTENTION=1"
Environment="OLLAMA_KV_CACHE_TYPE=q8_0"
Environment="OLLAMA_KEEP_ALIVE=5m"
```

---

## Sources

- Ollama API docs: https://github.com/ollama/ollama/blob/main/docs/api.md
- Ollama Modelfile reference: https://github.com/ollama/ollama/blob/main/docs/modelfile.mdx
- Ollama FAQ: https://github.com/ollama/ollama/blob/main/docs/faq.mdx
- Ollama context-length docs: https://github.com/ollama/ollama/blob/main/docs/context-length.mdx
- Ollama GPU docs: https://github.com/ollama/ollama/blob/main/docs/gpu.mdx
- Ollama thinking capability: https://github.com/ollama/ollama/blob/main/docs/capabilities/thinking.mdx
- Ollama model library: https://ollama.com/library
