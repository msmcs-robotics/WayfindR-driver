# Ambot

An autonomous conversational robot platform running on a Jetson Orin Nano (LLM + RAG + vision) and Raspberry Pi (LiDAR + movement).

## Overview

Ambot splits robot intelligence across two embedded Linux systems to enable parallel development by two teams. The Jetson handles conversation (small LLM via nanoLLM, RAG via rag-bootstrap, person detection via USB camera) while the Raspberry Pi handles navigation (LiDAR obstacle avoidance, algorithmic movement). Both run Ubuntu-based operating systems.

## Project Structure

```
ambot/
├── docs/
│   ├── README.md          # This file
│   ├── scope.md           # Project boundaries and decisions
│   ├── roadmap.md         # Feature roadmap and milestones
│   ├── todo.md            # Current tasks
│   └── findings/          # Research and investigation docs
├── jetson_nano/           # Jetson Orin Nano subsystem (LLM + RAG + vision)
├── raspberry_pi/          # Raspberry Pi subsystem (LiDAR + movement)
└── tests/                 # Test scripts
```

## Systems

### Jetson Orin Nano (Brain)
- **LLM**: nanoLLM framework, TinyLlama-1.1B / Phi-2 candidates
- **RAG**: PostgreSQL + pgvector + Redis + FastAPI (Dockerized, based on rag-bootstrap)
- **Vision**: USB camera person/face detection
- **Output**: Text display (no speaker initially)
- **Access**: `ssh ambot@10.33.183.100`

### Raspberry Pi (Body)
- **Sensing**: LiDAR-based obstacle detection
- **Movement**: Algorithmic object avoidance and basic movement
- **Stack**: Python (no ROS2), future C port
- **Access**: TBD

## Documentation

- [scope.md](scope.md) - What this project is and isn't
- [roadmap.md](roadmap.md) - Feature progress and milestones
- [todo.md](todo.md) - Current tasks

## Related Projects

- **WayfindR-driver**: Reference for LiDAR code, motor control, navigation patterns
- **rag-bootstrap** (`~/exudeai/rag-bootstrap`): Template for RAG system deployment
- **External LLM optimization project**: Will provide optimized models
- **llm-project-bootstrap**: Project management templates and guides

---

*For detailed project boundaries and technical decisions, see `docs/scope.md`*
