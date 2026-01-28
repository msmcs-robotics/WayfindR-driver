# Ambot - Jetson Orin Nano Subsystem

The "brain" of the Ambot platform. Handles LLM conversation, RAG knowledge retrieval, and visual person detection.

## Components

- **LLM**: nanoLLM framework (TinyLlama-1.1B / Phi-2)
- **RAG**: Docker-based stack (PostgreSQL + pgvector + Redis + FastAPI)
- **Vision**: USB camera person/face detection
- **Output**: Text display

## System Access

```
Host: 10.33.183.100
User: ambot
SSH:  ssh ambot@10.33.183.100
```

## Resource Strategy

- **GPU**: Reserved for nanoLLM inference (TensorRT optimized)
- **CPU**: RAG stack (PostgreSQL, Redis, FastAPI, embeddings)
- **Total RAM**: 8GB shared (tight budget, monitor closely)

## Directory Structure

```
jetson_nano/
├── README.md       # This file
├── llm/            # LLM configuration and scripts (future)
├── rag/            # RAG system adapted from rag-bootstrap (future)
└── vision/         # Person detection scripts (future)
```

## References

- [nanoLLM Tutorial](https://www.jetson-ai-lab.com/archive/tutorial_nano-llm.html)
- RAG template: `~/exudeai/rag-bootstrap/`
