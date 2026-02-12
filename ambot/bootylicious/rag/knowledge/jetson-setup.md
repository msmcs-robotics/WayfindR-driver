# Jetson Orin Nano Setup Guide

## System Information

- Hostname: georgejetson-desktop
- IP Address: 10.33.255.82
- WiFi MAC (wlP1p1s0): f8:3d:c6:57:05:77
- Ethernet MAC (enP8p1s0): 3c:6d:66:b4:7f:ed

## Installed Software

| Component | Version | Status |
|-----------|---------|--------|
| Ubuntu | 22.04.5 LTS | Working |
| JetPack | R36.4.4 | Working |
| NVIDIA Driver | 540.4.0 | Working |
| CUDA Runtime | 12.6 | Working |
| Docker | 28.2.2 | Active |
| Docker Compose | v2.36.2 | Working |
| NVIDIA Container Toolkit | 1.16.2 | Working |
| Python | 3.10.12 | Working |
| pip | 22.0.2 | Working |
| Ollama | 0.6.2 | Running |

## LLM Configuration

Ollama is the LLM backend, running as a systemd service on the host. Currently loaded model: tinyllama (1.1B parameters, 637 MB).

Ollama is configured to listen on all interfaces (0.0.0.0:11434) via a systemd override at /etc/systemd/system/ollama.service.d/override.conf. This allows Docker containers to reach it via the Docker bridge gateway IP (172.18.0.1:11434).

Note: host.docker.internal does NOT work on Jetson/Linux Docker. Always use the Docker bridge gateway IP.

## RAG Stack

The RAG system runs as three Docker containers:

1. **PostgreSQL** (pgvector/pgvector:pg16) - Vector database for document embeddings, 512MB memory limit
2. **Redis** (redis:7-alpine) - Embedding cache, 256MB limit
3. **FastAPI API** - RAG endpoints, sentence-transformers embeddings, Ollama LLM backend

API endpoints:
- GET /api/health - Health check
- POST /api/ask - Ask a question using RAG context
- POST /api/search - Search documents
- POST /api/ingest/file - Upload and ingest a file
- POST /api/ingest/directory - Ingest all files in a directory
- GET /api/documents - List ingested documents

## Recommended Models

For the 8GB Jetson Orin Nano:

| Model | Parameters | Memory (Q4) | Speed | Best For |
|-------|------------|-------------|-------|----------|
| TinyLlama 1.1B | 1.1B | ~637 MB | 40-65 tok/s | Ultra-fast, low memory |
| SmolLM2 1.7B | 1.7B | ~1 GB | 50-65 tok/s | Compact, efficient |
| Llama 3.2 3B | 3B | ~2.5 GB | 28-43 tok/s | General purpose |
| Phi-3 Mini 4K | 3.8B | ~2.5 GB | 25-38 tok/s | Reasoning, coding |

## Docker Permissions

If you get "permission denied" errors with Docker, the user needs to be in the docker group AND have logged out/back in:

```bash
# Check group membership
id | grep docker

# Quick fix for current session
newgrp docker

# Permanent fix: log out completely and log back in
```
