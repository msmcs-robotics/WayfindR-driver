# MCP & Conversation Pattern Research — cars_demo_13 & exudeai

> Date: 2026-03-24
> Status: Research complete, patterns documented for future implementation
> Keywords: MCP, conversation context, intent classification, context condensation, tour guide

## Source Repositories

- `~/cars_demo_13/` — Multi-agent GPS simulation with MCP, HITL, and streaming chat
- `~/exudeai/` — RAG system with context window management documentation

## Key Patterns for AMBOT

### 1. Intent Classification (from cars_demo_13)

The most critical pattern for the tour guide MCP feature. Distinguishes
"consultation" (asking about something) from "execution" (wanting to do something).

**Pattern**: `classify_message_intent()` returns:
- `intent`: "movement" | "consultation" | "report_request"
- `is_execution`: bool (True if requesting action)
- `confidence`: float (0.0-1.0)

**Consultation patterns** (override execution):
- "should I", "what should", "is it safe", "tell me about", "explain"
- Questions with "?" default to consultation

**Execution patterns**:
- Direct imperatives: "move to", "go to", "take me to"
- Explicit verbs: "execute", "proceed", "navigate"

**Critical rule**: Consultation overrides execution when both match.
For a moving robot, always safer to confirm before acting.

### 2. MCP Tool Registration (from cars_demo_13)

Uses FastMCP v2.x with decorator-based tool registration:
```python
from fastmcp import FastMCP
mcp = FastMCP("ambot")

@mcp.tool()
async def move_to(location_name: str) -> dict:
    """Navigate robot to a named location."""
    ...

@mcp.tool()
async def check_location() -> dict:
    """Query current robot position."""
    ...
```

Tools integrate with HITL (Human-In-The-Loop) approval workflow.
Movement commands go through `/hitl/submit/*` endpoints.

### 3. Context Assembly Priority (from cars_demo_13)

Multi-source context built in priority order:
1. Knowledge base (domain expertise) — max 1500 chars
2. System documentation (if relevant keywords detected)
3. Live API data (source of truth)
4. Position history (optional)
5. Recent errors (limited to 2)

**Quick vs Full context paths**:
- Status commands → quick path (live API only, ~500 chars)
- General queries → full path (all sources, ~3000-5000 chars)

### 4. Context Window Management (from exudeai)

**Dynamic budget calculation**:
```
available = effective_limit - system_prompt - query - history - max_output - safety_margin
```

**Budget allocation for 4K model (llama3.2:3b)**:
| Component | Tokens |
|-----------|--------|
| System prompt | ~300 |
| Retrieved context | ~1800 |
| Query + history | ~300 |
| Reserved output | ~600 |
| Safety (5%) | ~200 |

**Conversation strategies**:
1. Sliding window with summarization (what we implemented)
2. Query rewriting for context independence (future)
3. Retrieval-aware history pruning (future)

### 5. Thought Streaming (from cars_demo_13)

SSE phases for real-time progress:
```
ANALYZING     → Understanding user input
FETCHING_DATA → Retrieving context
REASONING     → Analysis phase
GENERATING    → Composing response
COMPLETE      → Done (with final_response)
```

We already have similar stages (searching → generating → done).

### 6. Graceful Degradation (from cars_demo_13)

Fallback chain:
1. LLM available → full chat with context
2. LLM unavailable → algorithmic control (pattern matching)
3. API unavailable → use cached data
4. All failures → log but don't crash

### 7. Lost in the Middle (from exudeai)

LLMs attend best to information at the beginning and end of context.
Middle positions lose ~25% accuracy.

**Mitigation**: Sandwich ordering (best-worst-best) for multi-chunk RAG.

## Implementation Priority for AMBOT

| Priority | Feature | Complexity | Dependency |
|----------|---------|------------|------------|
| 1 | Intent classification (consultation vs execution) | Medium | None |
| 2 | Location list with descriptions | Low | None |
| 3 | MCP tool registration (FastMCP) | Medium | Location list |
| 4 | Confirmation flow before movement | Medium | Intent classification |
| 5 | MCP action display in chat UI | Low | MCP tools |
| 6 | Query rewriting with conversation context | Medium | Current context system |
| 7 | Proper token counting | Low | None |
| 8 | Conversation persistence (SQLite) | Medium | None |

## Files Referenced

- `~/cars_demo_13/chatapp/mcp_commands.py` — Intent classification, routing
- `~/cars_demo_13/chatapp/mcp_tools.py` — MCP tool registration
- `~/cars_demo_13/chatapp/mcp_llm_agent.py` — Context assembly, data schema
- `~/cars_demo_13/rag/rag.py` — Multi-source context assembly
- `~/cars_demo_13/core/ollama_manager.py` — Connection state, request queuing
- `~/exudeai/docs/findings/context-window-management-rag.md` — Token budgeting
- `~/exudeai/rag-atc-testing/app/search.py` — Adaptive semantic weighting
