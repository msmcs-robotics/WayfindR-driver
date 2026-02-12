# Jetson Orin Nano - System Inventory

> Date: 2026-02-12
> Status: Complete
> Keywords: jetson, orin nano, system inventory, hardware, docker, cuda

## Summary

Initial system inventory of the Jetson Orin Nano Developer Kit after fresh JetPack installation. Docker and NVIDIA runtime are pre-installed; pip and Ollama need installation.

---

## Hardware

| Spec | Value |
|------|-------|
| Model | NVIDIA Jetson Orin Nano Developer Kit |
| CPU | 6x Cortex-A78AE (aarch64) |
| RAM | 7.4 GiB total, ~5.9 GiB available at idle |
| Swap | 3.7 GiB |
| Storage | 113 GB eMMC, 86 GB available (21% used) |
| GPU | Orin (nvgpu) — integrated, unified memory |

## Software

| Component | Version | Status |
|-----------|---------|--------|
| OS | Ubuntu 22.04.5 LTS (Jammy Jellyfish) | Working |
| Kernel | 5.15.148-tegra | Working |
| JetPack | R36.4.4 (JetPack 6.x) | Working |
| NVIDIA Driver | 540.4.0 | Working |
| CUDA Runtime | 12.6 | Working (nvidia-smi) |
| CUDA Toolkit (nvcc) | Not installed | Dev headers not needed for Docker workflows |
| Docker | 28.2.2 | Active, service running |
| Docker Compose | v2.36.2 | Working |
| NVIDIA Container Toolkit | 1.16.2 | Installed |
| Python | 3.10.12 | Working |
| pip | 22.0.2 | Installed via apt |
| Ollama | 0.6.2 | Installed, tinyllama model pulled |

## Network

| Interface | MAC Address | Status |
|-----------|-------------|--------|
| wlP1p1s0 (WiFi) | f8:3d:c6:57:05:77 | UP, connected |
| enP8p1s0 (Ethernet) | 3c:6d:66:b4:7f:ed | DOWN (no cable) |
| docker0 | auto-assigned | UP (RAG stack running) |
| l4tbr0 | auto-assigned | DOWN |
| can0 | — | DOWN (CAN bus interface) |
| usb0, usb1 | auto-assigned | DOWN (USB gadget mode) |

## SSH Access

```bash
# From WSL (with SSH config)
ssh jetson

# Or explicitly
ssh -i ~/.ssh/id_git georgejetson@10.33.255.82
```

## Memory Budget for AMBOT

Based on inventory (7.4 GiB unified memory):

| Component | Estimated Memory |
|-----------|-----------------|
| OS + System | ~1.5 GB |
| Docker overhead | ~200 MB |
| PostgreSQL (pgvector) | 512 MB (capped) |
| Redis | 300 MB (capped at 256 MB) |
| FastAPI + Embeddings | ~500 MB |
| LLM (TinyLlama 1.1B Q4) | ~637 MB |
| **Total estimated** | **~3.6 GB** |
| **Available headroom** | **~3.8 GB** |

This gives comfortable headroom for a 1B model + RAG stack.

## Key Findings

1. **Docker is pre-installed** — No need to install from scratch. Just need to add user to docker group.
2. **NVIDIA Container Toolkit ready** — GPU-accelerated Docker containers should work out of the box.
3. **No nvcc needed** — For Docker-based workflows (Ollama, containerized LLM), the CUDA runtime is sufficient. CUDA toolkit dev headers are only needed for building native CUDA code.
4. **86 GB storage available** — Plenty for Docker images, models, and data.
5. **JetPack R36.4.4** — This is JetPack 6.x, which the existing research says has good Ollama compatibility.
6. **Python 3.10** — Older than RPi's 3.13 but well-supported by all libraries.

## Setup Steps Completed

1. [x] `sudo apt install python3-pip` — pip 22.0.2
2. [x] `sudo usermod -aG docker georgejetson` — Docker without sudo
3. [x] NOPASSWD sudo configured (`/etc/sudoers.d/georgejetson`)
4. [x] Ollama installed (0.6.2) with `OLLAMA_HOST=0.0.0.0` (listens on all interfaces)
5. [x] tinyllama model pulled (~637 MB) — verified working
6. [x] Docker `default-runtime: nvidia` set in `/etc/docker/daemon.json`
7. [x] RAG Docker stack running (PostgreSQL + Redis + FastAPI)

All handled by `install-jetson.sh`.

## RAG Stack Status

| Service | Container | Status | Notes |
|---------|-----------|--------|-------|
| PostgreSQL | ambot-rag-postgres | Healthy | pgvector/pgvector:pg16, 512MB limit |
| Redis | ambot-rag-redis | Healthy | redis:7-alpine, 256MB limit |
| FastAPI API | ambot-rag-api | Healthy | sentence-transformers + Ollama backend |
| Ollama | host service | Running | tinyllama model, port 11434 (all interfaces) |

### API Endpoints

- `GET /api/health` — Health check (all services)
- `POST /api/ask` — Ask a question (RAG)
- `POST /api/search` — Search documents
- `POST /api/ingest/file` — Ingest single file
- `POST /api/ingest/directory` — Ingest directory
- `GET /api/documents` — List documents
- `GET /api/models` — List available models

### Key Configuration

- Ollama URL: `http://172.18.0.1:11434` (Docker bridge gateway, NOT `host.docker.internal`)
- Ollama systemd override: `/etc/systemd/system/ollama.service.d/override.conf` sets `OLLAMA_HOST=0.0.0.0`
- `.env` file: `~/ambot/bootylicious/rag/.env` (created from `.env.example`)

---

*See also: [jetson-llm-deployment-research.md](../../docs/findings/jetson-llm-deployment-research.md) for LLM model recommendations.*
