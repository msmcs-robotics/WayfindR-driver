# Jetson Orin Nano - RAG System Setup

> Date: 2026-01-29
> Status: Complete
> Keywords: jetson orin nano, rag, docker, ollama, huggingface, embeddings, llm

## Summary

Complete setup guide and implementation for deploying a RAG (Retrieval-Augmented Generation) system on the Jetson Orin Nano with support for both Ollama and HuggingFace LLM backends.

---

## Implementation Overview

### Files Created

**Setup Scripts** (`jetson_nano/scripts/`):
- `setup-docker.sh` - Docker and Docker Compose installation
- `setup-ollama.sh` - Ollama LLM server (optional)
- `setup-huggingface.sh` - HuggingFace transformers with CUDA
- `setup-all.sh` - Master setup script
- `rsync-to-jetson.sh` - Sync files to Jetson over SSH

**RAG System** (`jetson_nano/rag/`):
- `docker-compose.yml` - Optimized for 8GB RAM Jetson
- `.env.example` - Configuration template
- `deploy.sh` - Deployment and management script
- `knowledge/` - Directory for EECS documentation
- `app/` - FastAPI application (adapted from rag-bootstrap)

### Key Adaptations for Jetson

1. **Memory Optimization**:
   - PostgreSQL: `shared_buffers=128MB` (vs default 256MB)
   - Redis: `maxmemory=256mb` with LRU eviction
   - API container: 2GB limit with GPU reservation

2. **Dual LLM Backend**:
   - Ollama: For external LLM server
   - HuggingFace: For local GPU-accelerated inference

3. **Docker Network**:
   - Single bridge network (`ambot-rag`)
   - Only API port (8000) exposed
   - Internal service communication

---

## Resource Analysis

### Memory Budget (8GB Total)

| Component | Allocation | Notes |
|-----------|------------|-------|
| OS + System | ~1.0 GB | Ubuntu 22.04 + drivers |
| Docker daemon | ~0.2 GB | Container management |
| PostgreSQL | ~0.5 GB | With pgvector extension |
| Redis | ~0.3 GB | Embedding cache |
| FastAPI + Embeddings | ~0.8 GB | sentence-transformers model |
| **Subtotal (base)** | **~2.8 GB** | Without LLM |
| TinyLlama (HF) | ~2.2 GB | float16, GPU |
| **Total with LLM** | **~5.0 GB** | Leaves ~3GB headroom |

### Recommended Models

**For Jetson Orin Nano (8GB)**:

| Model | Parameters | Size | Backend | Verdict |
|-------|------------|------|---------|---------|
| TinyLlama-1.1B | 1.1B | ~2.2GB | HuggingFace | **Recommended** |
| Phi-2 | 2.7B | ~5.4GB | HuggingFace | Tight fit |
| tinyllama | 1.1B | ~600MB | Ollama | External only |
| qwen2:1.5b | 1.5B | ~950MB | Ollama | External only |

**Embedding Model**:
- `all-MiniLM-L6-v2` (384 dimensions, ~80MB) - **Always use this**

---

## LLM Backend Comparison

### Ollama

**Pros**:
- Simple API, well-documented
- Model management built-in
- Works across devices

**Cons**:
- Limited ARM64/Jetson support (may have issues)
- Requires running separate service
- Less GPU optimization for Jetson

**Use when**: Testing, development, or running LLM on separate machine

### HuggingFace Transformers

**Pros**:
- Native CUDA support via PyTorch
- TensorRT optimization possible
- Full control over inference

**Cons**:
- More complex setup
- Larger dependencies
- Model loading takes time

**Use when**: Production on Jetson, maximum GPU utilization

---

## Deployment Workflow

### From Development Machine

```bash
# 1. Set up SSH keys (one-time)
ssh-copy-id ambot@10.33.183.100

# 2. Sync files to Jetson
cd ambot/jetson_nano/scripts
./rsync-to-jetson.sh

# 3. SSH to Jetson and run setup
ssh ambot@10.33.183.100
cd ~/ambot/scripts
./setup-all.sh
```

### On Jetson

```bash
# 1. Deploy RAG system
cd ~/ambot/rag
cp .env.example .env
# Edit .env to configure LLM backend
./deploy.sh start --build

# 2. Add EECS documents
# Copy documents to ~/ambot/rag/knowledge/

# 3. Ingest documents
./deploy.sh ingest

# 4. Test
curl http://localhost:8000/api/health
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is EECS?"}'
```

---

## Configuration Options

### For Ollama Backend (External LLM)

```bash
# .env
LLM_BACKEND=ollama
OLLAMA_BASE_URL=http://host.docker.internal:11434
LLM_MODEL=tinyllama
```

### For HuggingFace Backend (Local GPU)

```bash
# .env
LLM_BACKEND=huggingface
HF_MODEL=TinyLlama/TinyLlama-1.1B-Chat-v1.0
HF_DEVICE=cuda
HF_TORCH_DTYPE=float16
```

### For Embedding-Only Mode (No LLM)

If resources are too tight for LLM, use only search:

```bash
# Just use /api/search endpoint, skip /api/ask
curl -X POST http://localhost:8000/api/search \
  -d '{"query": "...", "mode": "hybrid"}'
```

---

## Testing Strategy

### Phase 1: Docker Infrastructure
- [ ] Verify Docker runs on Jetson
- [ ] Verify GPU access in containers
- [ ] Test PostgreSQL + Redis startup

### Phase 2: RAG Core (No LLM)
- [ ] Deploy without LLM
- [ ] Ingest test documents
- [ ] Test search endpoints

### Phase 3: LLM Integration
- [ ] Test Ollama backend (if available)
- [ ] Test HuggingFace backend
- [ ] Measure memory usage with `docker stats`

### Phase 4: Production
- [ ] Ingest full EECS documentation
- [ ] Tune RAG parameters (top_k, similarity)
- [ ] Monitor performance over time

---

## Troubleshooting

### "No space left on device"
```bash
# Clean Docker
docker system prune -a
docker volume prune

# Check disk usage
df -h
du -sh /var/lib/docker
```

### Out of Memory
```bash
# Check memory
free -h
docker stats

# Reduce model size
HF_MODEL=TinyLlama/TinyLlama-1.1B-Chat-v1.0
```

### GPU Not Detected
```bash
# Check NVIDIA runtime
docker info | grep -i nvidia

# Test GPU access
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

## Related

**Project Documents**:
- [jetson-orin-nano-ubuntu-flash.md](./jetson-orin-nano-ubuntu-flash.md) - OS installation
- [initial-system-research.md](./initial-system-research.md) - Resource planning

**Code**:
- [jetson_nano/rag/](../../jetson_nano/rag/) - RAG system
- [jetson_nano/scripts/](../../jetson_nano/scripts/) - Setup scripts

**External**:
- [rag-bootstrap](https://github.com/exudeai/rag-bootstrap) - Original template
- [Jetson AI Lab](https://www.jetson-ai-lab.com/) - NVIDIA optimization guides

---

## Notes

- The system defaults to Ollama but HuggingFace is recommended for production
- Embedding model (all-MiniLM-L6-v2) runs in the API container, not the LLM
- Redis caches embeddings for 30 days to avoid recomputation
- Docker network isolates all services, only port 8000 is exposed

---

*Document created: 2026-01-29*
