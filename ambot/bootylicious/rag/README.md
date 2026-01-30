# Ambot RAG System - Jetson Orin Nano

RAG (Retrieval-Augmented Generation) system for the Ambot project, optimized for Jetson Orin Nano (8GB RAM).

## Overview

This system provides:
- **Document ingestion** - Ingest PDFs, Markdown, text files into a vector database
- **Semantic search** - Find relevant documents using embeddings
- **RAG Q&A** - Answer questions using retrieved context + LLM

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Docker Network (ambot-rag)           │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐ │
│  │ PostgreSQL  │  │   Redis     │  │   FastAPI API   │ │
│  │ + pgvector  │  │   (cache)   │  │   (port 8000)   │ │
│  └─────────────┘  └─────────────┘  └─────────────────┘ │
│                                                         │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
              ┌────────────────────────┐
              │   LLM Backend          │
              │   - Ollama (external)  │
              │   - HuggingFace (local)│
              └────────────────────────┘
```

## Quick Start

### 1. Configure

```bash
cp .env.example .env
# Edit .env as needed
```

### 2. Deploy

```bash
chmod +x deploy.sh
./deploy.sh start --build
```

### 3. Ingest Documents

Place documents in `./knowledge/` directory, then:

```bash
./deploy.sh ingest
```

### 4. Query

```bash
# Health check
curl http://localhost:8000/api/health

# Search
curl -X POST http://localhost:8000/api/search \
  -H "Content-Type: application/json" \
  -d '{"query": "EECS department", "mode": "hybrid", "limit": 5}'

# Ask a question (requires LLM backend)
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What courses are offered in EECS?"}'
```

## LLM Backends

### Ollama (Default)

Uses an external Ollama server for LLM inference.

```bash
# .env
LLM_BACKEND=ollama
OLLAMA_BASE_URL=http://host.docker.internal:11434
LLM_MODEL=tinyllama
```

### HuggingFace (Recommended for Jetson)

Runs models locally using HuggingFace transformers with GPU acceleration.

```bash
# .env
LLM_BACKEND=huggingface
HF_MODEL=TinyLlama/TinyLlama-1.1B-Chat-v1.0
HF_DEVICE=cuda
HF_TORCH_DTYPE=float16
```

## Resource Usage (8GB Jetson)

| Component | Memory |
|-----------|--------|
| PostgreSQL | ~500 MB |
| Redis | ~256 MB (capped) |
| API + Embeddings | ~800 MB |
| **Total Base** | **~1.5 GB** |
| LLM (TinyLlama) | ~2 GB |
| **Total with LLM** | **~3.5 GB** |

## Directory Structure

```
rag/
├── app/                    # FastAPI application
│   ├── main.py            # API routes
│   ├── config.py          # Configuration
│   ├── database.py        # SQLAlchemy models
│   ├── embeddings.py      # Embedding service
│   ├── ingestion.py       # Document processing
│   ├── search.py          # Search algorithms
│   ├── llm.py             # LLM clients (Ollama/HuggingFace)
│   ├── Dockerfile
│   └── requirements.txt
├── knowledge/              # Put EECS documents here
├── docker-compose.yml      # Container orchestration
├── deploy.sh              # Deployment script
├── .env.example           # Environment template
└── README.md              # This file
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/health` | GET | Health status |
| `/api/search` | POST | Search documents |
| `/api/ask` | POST | RAG Q&A |
| `/api/ingest/file` | POST | Upload file |
| `/api/ingest/directory` | POST | Ingest directory |
| `/api/documents` | GET | List documents |
| `/api/documents/{id}` | DELETE | Delete document |

## Recommended Models

For Jetson Orin Nano (8GB):

| Model | Size | Quality | Speed |
|-------|------|---------|-------|
| TinyLlama-1.1B | ~2.2 GB | Good | Fast |
| Phi-2 (2.7B) | ~5.4 GB | Better | Medium |
| Qwen2-1.5B | ~3 GB | Good | Fast |

## Troubleshooting

### Out of Memory

```bash
# Reduce PostgreSQL buffers
# Edit docker-compose.yml postgres command

# Use smaller model
LLM_MODEL=tinyllama  # or smaller HF model
```

### GPU Not Detected

```bash
# Check NVIDIA runtime
docker info | grep -i nvidia

# Run with explicit GPU
docker compose up -d --build
```

### Slow Ingestion

```bash
# Increase chunk size (fewer chunks)
CHUNK_SIZE=1024
```

## Integration with Ambot

This RAG system is designed to work with the Ambot conversational robot:

1. **Person Detection** (Jetson camera) triggers conversation
2. **RAG Search** retrieves relevant EECS information
3. **LLM** generates contextual response
4. **Display** shows response to user

## Related

- [../scripts/](../scripts/) - Setup scripts
- [../../docs/](../../docs/) - Project documentation
- [rag-bootstrap](https://github.com/exudeai/rag-bootstrap) - Original template
