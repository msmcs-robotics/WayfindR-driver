# Bootylicious - LLM & RAG System

The "brain" of the Ambot platform. Handles LLM conversation, RAG knowledge retrieval, and visual person detection.

> **Note**: Motor control is in `ambot/locomotion/`, LiDAR is in `ambot/pathfinder/` - separate components.

## Quick Start

```bash
# On development machine (Windows/WSL)
cd ambot/bootylicious/scripts
./rsync-to-jetson.sh <jetson_ip>

# On Jetson
cd ~/bootylicious
./deploy.sh setup     # Initial setup (Docker, etc.)
./deploy.sh start     # Start services
./deploy.sh diagnose  # Run diagnostics
```

## Directory Structure

```
bootylicious/
├── deploy.sh           # Master deployment script
├── README.md           # This file
├── scripts/            # Setup and utility scripts
│   ├── setup-docker.sh
│   ├── setup-ollama.sh
│   ├── setup-huggingface.sh
│   ├── setup-all.sh
│   └── rsync-to-jetson.sh
├── rag/                # RAG/LLM system (Dockerized)
│   ├── deploy.sh
│   ├── docker-compose.yml
│   ├── app/            # FastAPI application
│   └── knowledge/      # EECS documentation
├── tests/              # System tests
│   └── test-system.sh
└── logs/               # Deployment logs
```

> **Sibling components** (in `ambot/` folder):
> - `locomotion/` - Motor control (YAHBOOM G1)
> - `pathfinder/` - LiDAR obstacle avoidance

## Modules

### RAG System (`rag/`)

Retrieval-Augmented Generation for EECS knowledge base.

```bash
cd rag
./deploy.sh start --build    # Start containers
./deploy.sh ingest           # Ingest documents
curl http://localhost:8000/api/health
```

## Master Deploy Script

```bash
# Service Commands
./deploy.sh setup              # Initial setup (idempotent)
./deploy.sh start [--build]    # Start services
./deploy.sh stop               # Stop services
./deploy.sh restart            # Restart services

# Monitoring
./deploy.sh status             # System status overview
./deploy.sh health             # Health checks
./deploy.sh diagnose           # Comprehensive diagnostics
./deploy.sh logs [service]     # View logs

# Troubleshooting
./deploy.sh ssh-fix            # Fix stale SSH tunnels
./deploy.sh network            # Network diagnostics

# Maintenance
./deploy.sh clean              # Stop and optionally clean up
```

All commands are **idempotent** - safe to run multiple times.

## Rsync from Development Machine

```bash
# Basic sync
./scripts/rsync-to-jetson.sh 192.168.1.50

# With options
./scripts/rsync-to-jetson.sh --dry-run    # Preview only
./scripts/rsync-to-jetson.sh --watch      # Continuous sync
./scripts/rsync-to-jetson.sh --delete     # Remove stale files
```

## System Tests

```bash
# Run all tests
./tests/test-system.sh

# Run specific test suite
./tests/test-system.sh docker
./tests/test-system.sh gpu
./tests/test-system.sh rag
./tests/test-system.sh network
```

## Components

| Module | Purpose | Status |
|--------|---------|--------|
| **rag/** | LLM + RAG knowledge system | Ready |
| **vision/** | Person detection | Future |

> **External components** (in `ambot/` folder):
> - **locomotion/** - Motor control (TB6612FNG) - Ready
> - **pathfinder/** - LiDAR obstacle avoidance - Ready

## System Access

```
Host: 10.33.183.100 (or check with hostname -I on Jetson)
User: ambot
SSH:  ssh ambot@<ip_address>
```

## Resource Strategy (8GB RAM)

| Component | Allocation |
|-----------|------------|
| OS + System | ~1 GB |
| Docker | ~200 MB |
| PostgreSQL | ~500 MB |
| Redis | ~256 MB |
| API + Embeddings | ~800 MB |
| LLM (TinyLlama) | ~2.2 GB |
| **Total** | **~5 GB** |

## LLM Backends

Configure in `rag/.env`:

```bash
# Ollama (external server)
LLM_BACKEND=ollama
LLM_MODEL=tinyllama

# HuggingFace (local GPU)
LLM_BACKEND=huggingface
HF_MODEL=TinyLlama/TinyLlama-1.1B-Chat-v1.0
```

## Related Documentation

- [docs/findings/jetson-orin-nano-ubuntu-flash.md](../docs/findings/jetson-orin-nano-ubuntu-flash.md) - Flash guide
- [docs/findings/jetson-rag-system-setup.md](../docs/findings/jetson-rag-system-setup.md) - RAG setup
- [rag/README.md](rag/README.md) - RAG system details

**Sibling components:**
- [locomotion/README.md](../locomotion/README.md) - Motor control
- [pathfinder/README.md](../pathfinder/README.md) - LiDAR system
