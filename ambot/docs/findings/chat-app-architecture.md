# Chat App Architecture & Design Decisions

> Date: 2026-03-24
> Status: Implemented (Session 21)
> Keywords: chat, fastapi, ollama, rag, streaming, conversation context, smart routing

## Overview

The `chat_app/` provides a focused LLM chat interface that replaces the complex
`web_control/` Flask dashboard. It's a lightweight FastAPI app with vanilla JS
frontend, SSE streaming, and smart RAG routing.

## Architecture

```
Browser (localhost:5050)
    │
    ▼
FastAPI chat_app (port 5050, host process)
    │
    ├── Casual queries ──► Ollama /api/chat (localhost:11434)
    │                       Multi-turn messages array
    │
    └── Knowledge queries ──► RAG API /api/search (localhost:8000, Docker)
                               │
                               ▼
                            Ollama /api/chat (with retrieved context)
```

### Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Framework | FastAPI (not Flask) | User preference, async support, SSE native |
| LLM endpoint | Ollama `/api/chat` (not `/api/generate`) | Multi-turn conversation support via messages array |
| RAG integration | Use RAG API for search only, chat_app handles LLM | Avoids double-prompting; conversation history stays in chat_app |
| Streaming | SSE (Server-Sent Events) | Simpler than WebSocket for one-way token streaming |
| Session state | In-memory dict per session_id | Simple, sufficient for single-user Jetson deployment |
| Frontend | Vanilla HTML/CSS/JS | No build tools, runs anywhere, easy to deploy |

## Smart RAG Routing

### Problem
"Hello! How are you?" was triggering a knowledge base search, adding latency
for no benefit. General conversation doesn't need RAG retrieval.

### Solution: Two-tier classification
1. **Casual patterns checked first** — greetings, thanks, yes/no, "how are you"
2. **Domain keywords** — EECS, ERAU, robotics, lab, professor, etc. trigger RAG
3. **Question starters** — "what", "where", "how does" etc. only trigger RAG for 5+ word queries
4. **Default** — short/ambiguous messages treated as casual (faster response)

### Performance Impact
- Casual: ~4-5s (direct Ollama, no RAG search overhead)
- Knowledge: ~20-30s (RAG search ~0.5s + LLM generation with context)

## Conversation Context

### Approach
- chat_app maintains conversation history per session as a messages array
- Each request sends full history to Ollama `/api/chat`
- User's clean question (without RAG context) stored in history
- RAG context injected into the user message but NOT stored in history

### Context Window Management (4096 tokens)
- Rough token estimation: ~4 chars per token
- When history reaches ~70% of MAX_CONTEXT_TOKENS (3200), auto-condense
- Condensation: older messages summarized by Ollama into 2-3 sentences
- Last 4 exchanges (8 messages) kept verbatim
- Condensation notice shown in chat UI ("condensed x1")

### Future Improvements
- Proper tokenizer-based counting (tiktoken or Ollama's tokenize endpoint)
- Persistent conversation storage (SQLite)
- Session expiration/cleanup

## Response Timing

Each response includes server-side timing:
- `total_ms`: Wall clock from request to completion
- `search_ms`: Time spent on RAG search (0 for casual)
- `generate_ms`: Time spent on LLM token generation

Displayed in the chat footer: `llama3.2:3b · 4.4s · direct` or `llama3.2:3b · 25.7s (search: 0.6s) · 5 sources`

## MCP Tour Guide Vision (Future)

The chat app will evolve to include MCP (Model Context Protocol) tools for
location-based tour guide functionality:

1. **Location list**: Named locations with descriptions (buildings, labs, landmarks)
2. **MCP tools**: `check_location()`, `move_to(location_name)`
3. **Intent classification**: "tell me about the RASL lab" (info) vs "take me to the RASL lab" (movement)
4. **Chat UI integration**: MCP actions shown as italic text below assistant messages
5. **Confirmation flow**: LLM confirms before triggering movement commands
6. **Autonomous actions**: Some MCP commands don't need a chat response

### MCP Action Display
```
Assistant: Sure! Let me take you to the Robotics and Autonomous Systems Laboratory.
  → MCP: move_to("RASL") — Initiating navigation...
```

Actions that don't need a chat response:
```
User: Go to the RASL lab
  → MCP: move_to("RASL") — Navigation started
Assistant: On our way to the Robotics and Autonomous Systems Laboratory!
```

## Files

| File | Purpose |
|------|---------|
| `chat_app/main.py` | FastAPI backend — routing, sessions, streaming, condensation |
| `chat_app/static/index.html` | Chat UI page |
| `chat_app/static/style.css` | Dark theme styles |
| `chat_app/static/app.js` | Frontend — SSE parsing, DOM updates, timing display |
| `chat_app/requirements.txt` | Python dependencies (fastapi, uvicorn, httpx) |

## Running

```bash
# On Jetson
cd ~/ambot
RAG_API_URL=http://localhost:8000 OLLAMA_URL=http://localhost:11434 \
  python3 -m uvicorn chat_app.main:app --host 0.0.0.0 --port 5050

# From dev machine (SSH tunnel)
ssh -f -N -L 5050:localhost:5050 jetson
# Open http://localhost:5050
```
