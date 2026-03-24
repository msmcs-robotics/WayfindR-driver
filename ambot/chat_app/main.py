"""AMBOT Chat — FastAPI chat interface with conversation context and smart routing.

Architecture:
- Manages conversation history per session (in-memory)
- Smart routing: casual chat goes direct to Ollama, knowledge questions use RAG
- Uses Ollama /api/chat (multi-turn messages) for conversation context
- Uses RAG API /api/search for knowledge retrieval when needed
- SSE streaming with state indicators and timing
- Context condensation when approaching model limits
"""

import json
import logging
import os
import time
import uuid
from collections import defaultdict

import httpx
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from chat_app.locations import LocationManager
from chat_app.mcp_tools import (
    classify_location_intent, execute_check_location,
    execute_move_to, execute_list_locations,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── Configuration ────────────────────────────────────────────────
RAG_API_URL = os.environ.get("RAG_API_URL", "http://localhost:8000")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://localhost:11434")
OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "llama3.2:3b")
OLLAMA_TIMEOUT = float(os.environ.get("OLLAMA_TIMEOUT", "120"))
MAX_CONTEXT_TOKENS = int(os.environ.get("MAX_CONTEXT_TOKENS", "3200"))
MAX_HISTORY_TURNS = int(os.environ.get("MAX_HISTORY_TURNS", "20"))

SYSTEM_PROMPT = (
    "You are AMBOT, a helpful and friendly conversational robot assistant and tour guide at "
    "Embry-Riddle Aeronautical University (ERAU). You help with questions about "
    "EECS, robotics, engineering, campus life, and general conversation. "
    "You can also help people navigate to locations on campus. "
    "Be concise but thorough. When you have retrieved knowledge context, "
    "cite sources by their source number. When you don't have context, "
    "answer from your general knowledge or say you're not sure. "
    "When a user wants to go to a location, confirm the destination before navigating."
)

# ── App Setup ────────────────────────────────────────────────────
app = FastAPI(title="AMBOT Chat", docs_url=None, redoc_url=None)

static_dir = os.path.join(os.path.dirname(__file__), "static")
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# ── Location Manager ─────────────────────────────────────────────
location_mgr = LocationManager()

# ── Session Store ────────────────────────────────────────────────
# In-memory conversation history keyed by session_id
_sessions: dict[str, dict] = defaultdict(lambda: {
    "messages": [],
    "created": time.time(),
    "turn_count": 0,
    "condensation_count": 0,
})


def _get_session(session_id: str) -> dict:
    """Get or create a session."""
    return _sessions[session_id]


def _estimate_tokens(text: str) -> int:
    """Rough token estimate: ~3.5 chars per token for Llama models."""
    return int(len(text) / 3.5)


def _estimate_messages_tokens(messages: list[dict]) -> int:
    """Estimate total tokens in a messages list."""
    total = 0
    for msg in messages:
        total += _estimate_tokens(msg.get("content", ""))
        total += 4  # overhead per message (role, formatting)
    return total


# ── Query Classification ────────────────────────────────────────
_CASUAL_PATTERNS = {
    "hello", "hi", "hey", "greetings", "good morning", "good afternoon",
    "good evening", "thanks", "thank you", "bye", "goodbye", "see you",
    "ok", "okay", "sure", "yes", "no", "cool", "nice", "great",
    "how are you", "what's up", "sup", "yo",
    "lol", "haha", "hmm", "wow",
}


def classify_query(question: str) -> str:
    """Classify whether a query needs RAG retrieval or is casual conversation.

    Returns: 'rag' or 'casual'
    """
    lower = question.lower().strip().rstrip("?!.")
    words = lower.split()

    # Check casual patterns FIRST (regardless of length)
    for pattern in _CASUAL_PATTERNS:
        if lower == pattern or lower.startswith(pattern):
            return "casual"

    # Very short messages (1-2 words) without RAG keywords are casual
    if len(words) <= 2:
        return "casual"

    # Check for domain-specific RAG indicators (nouns, not question words)
    _DOMAIN_KEYWORDS = {
        "program", "degree", "lab", "faculty", "professor", "department",
        "course", "class", "major", "minor", "accreditation", "research",
        "club", "student", "campus", "erau", "embry", "riddle",
        "eecs", "engineering", "computer science", "electrical",
        "robotics", "ambot", "robot", "lidar", "sensor", "jetson",
        "rag", "knowledge", "document",
    }
    for keyword in _DOMAIN_KEYWORDS:
        if keyword in lower:
            return "rag"

    # Question words + enough length suggest a knowledge question
    _QUESTION_STARTERS = {"what", "where", "which", "how", "who", "when", "does", "is there",
                          "tell me about", "explain", "describe", "list", "show me"}
    if len(words) > 4:
        for starter in _QUESTION_STARTERS:
            if lower.startswith(starter):
                return "rag"

    # Questions (ending with ?) with 5+ words likely need lookup
    if question.strip().endswith("?") and len(words) > 4:
        return "rag"

    return "casual"


# ── Context Condensation ────────────────────────────────────────
async def condense_history(session: dict) -> None:
    """Summarize older conversation turns to free up context window.

    Keeps the last 4 turns verbatim, summarizes everything before that.
    """
    messages = session["messages"]
    if len(messages) < 10:
        return  # Not enough to condense

    # Keep last 4 exchanges (8 messages: user+assistant pairs)
    keep_count = 8
    to_condense = messages[:-keep_count]
    to_keep = messages[-keep_count:]

    # Build condensation prompt
    convo_text = ""
    for msg in to_condense:
        role = msg["role"].capitalize()
        convo_text += f"{role}: {msg['content']}\n"

    condense_prompt = (
        "Summarize this conversation in 2-3 sentences, preserving key topics "
        "discussed and any important facts or decisions:\n\n" + convo_text
    )

    try:
        async with httpx.AsyncClient(timeout=30) as client:
            resp = await client.post(
                f"{OLLAMA_URL}/api/generate",
                json={
                    "model": OLLAMA_MODEL,
                    "prompt": condense_prompt,
                    "stream": False,
                    "options": {"num_predict": 150, "temperature": 0.1},
                },
            )
            if resp.status_code == 200:
                summary = resp.json().get("response", "").strip()
                if summary:
                    # Replace old messages with a summary message
                    session["messages"] = [
                        {"role": "system",
                         "content": f"[Previous conversation summary: {summary}]"}
                    ] + to_keep
                    session["condensation_count"] += 1
                    logger.info("Condensed %d messages into summary (condensation #%d)",
                                len(to_condense), session["condensation_count"])
                    return
    except Exception as e:
        logger.warning("Condensation failed: %s", e)

    # Fallback: just trim old messages
    session["messages"] = to_keep


# ── Routes ───────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def index():
    """Serve the chat UI."""
    html_path = os.path.join(static_dir, "index.html")
    with open(html_path) as f:
        return HTMLResponse(f.read())


@app.get("/api/health")
async def health():
    """Check RAG API and Ollama health."""
    result = {"status": "ready", "rag_api": False, "ollama": False}

    async with httpx.AsyncClient(timeout=10) as client:
        # Check RAG API
        try:
            resp = await client.get(f"{RAG_API_URL}/api/health")
            if resp.status_code == 200:
                result["rag_api"] = True
                result["details"] = resp.json()
        except Exception:
            pass

        # Check Ollama directly
        try:
            resp = await client.get(f"{OLLAMA_URL}/api/tags")
            if resp.status_code == 200:
                result["ollama"] = True
        except Exception:
            pass

    if result["ollama"]:
        result["status"] = "ready"
    elif result["rag_api"]:
        result["status"] = "degraded"
    else:
        result["status"] = "offline"

    return result


@app.get("/api/models")
async def models():
    """Return available LLM models."""
    try:
        async with httpx.AsyncClient(timeout=10) as client:
            resp = await client.get(f"{OLLAMA_URL}/api/tags")
            if resp.status_code == 200:
                model_list = [m["name"] for m in resp.json().get("models", [])]
                return {"models": model_list, "current": OLLAMA_MODEL}
    except Exception as e:
        logger.warning("Failed to fetch models: %s", e)
    return {"models": [], "current": OLLAMA_MODEL}


@app.post("/api/session")
async def create_session():
    """Create a new chat session."""
    session_id = str(uuid.uuid4())[:8]
    _get_session(session_id)  # Initialize
    return {"session_id": session_id}


@app.delete("/api/session/{session_id}")
async def clear_session(session_id: str):
    """Clear a session's conversation history."""
    if session_id in _sessions:
        del _sessions[session_id]
    return {"cleared": True}


@app.get("/api/session/{session_id}/history")
async def get_history(session_id: str):
    """Return conversation history for a session."""
    session = _get_session(session_id)
    return {
        "messages": session["messages"],
        "turn_count": session["turn_count"],
        "condensation_count": session["condensation_count"],
    }


@app.get("/api/locations")
async def list_locations():
    """List all available locations."""
    return {"locations": location_mgr.list_all(), "count": len(location_mgr.locations)}


@app.get("/api/locations/reload")
async def reload_locations():
    """Reload locations from markdown files."""
    location_mgr.load()
    return {"reloaded": True, "count": len(location_mgr.locations)}


@app.post("/api/chat")
async def chat(request: Request):
    """Stream a chat response with conversation context and smart routing.

    Expects JSON: {"question": "...", "session_id": "..."}
    Returns: text/event-stream with state transitions and timing.
    """
    body = await request.json()
    question = body.get("question", "").strip()
    if not question:
        return {"error": "Empty question"}

    session_id = body.get("session_id", "default")
    session = _get_session(session_id)

    # Classify the query
    query_type = classify_query(question)
    t_start = time.time()

    # Check for location intent
    loc_intent = classify_location_intent(question, location_mgr)

    async def event_stream():
        sources = []
        rag_context = ""
        mcp_actions = []
        t_search_done = t_start

        # ── MCP: Location execution ─────────────────────────────
        if loc_intent.intent == "execution" and loc_intent.matched_locations:
            top_match = loc_intent.matched_locations[0]
            if top_match["score"] >= 0.8 and loc_intent.confidence >= 0.8:
                # High confidence — execute move
                action = execute_move_to(top_match["name"], location_mgr)
                mcp_actions.append(action)
                yield _sse("mcp_action", {
                    "tool": action.tool,
                    "text": action.display_text,
                    "result": action.result,
                })
            # Let the LLM still respond about the navigation

        # ── RAG Search (if needed) ──────────────────────────────
        if query_type == "rag":
            yield _sse("state", {"stage": "searching",
                                  "message": "Searching knowledge base..."})
            try:
                async with httpx.AsyncClient(timeout=30) as client:
                    resp = await client.post(
                        f"{RAG_API_URL}/api/search",
                        json={"query": question, "mode": "hybrid", "limit": 5},
                    )
                    if resp.status_code == 200:
                        sources = resp.json()
                        if sources:
                            parts = []
                            for i, s in enumerate(sources, 1):
                                fname = s.get("document_filename", "unknown")
                                content = s.get("content", "")
                                parts.append(f"[Source {i}: {fname}]\n{content}")
                            rag_context = "\n\n---\n\n".join(parts)

                            yield _sse("sources", {
                                "sources": sources,
                                "model": OLLAMA_MODEL,
                            })
            except Exception as e:
                logger.warning("RAG search failed: %s", e)

            t_search_done = time.time()

        # ── Build Messages Array ────────────────────────────────
        yield _sse("state", {"stage": "generating",
                              "message": "Generating response..."})

        # Check if we need to condense history
        history_tokens = _estimate_messages_tokens(session["messages"])
        if history_tokens > MAX_CONTEXT_TOKENS * 0.7:
            yield _sse("state", {"stage": "condensing",
                                  "message": "Condensing conversation history..."})
            await condense_history(session)

        # Build the messages array for Ollama /api/chat
        system_content = SYSTEM_PROMPT
        # Add location context if relevant
        if loc_intent.intent != "none" and location_mgr.locations:
            system_content += "\n\n" + location_mgr.format_for_llm()

        messages = [{"role": "system", "content": system_content}]

        # Add conversation history
        messages.extend(session["messages"])

        # Build the user message (with optional RAG context + MCP results)
        extra_context = ""
        if rag_context:
            extra_context += f"## Retrieved Knowledge\n\n{rag_context}\n\n---\n\n"
        if mcp_actions:
            action_texts = [f"[MCP {a.tool}: {a.display_text}]" for a in mcp_actions]
            extra_context += "## Actions Taken\n\n" + "\n".join(action_texts) + "\n\n---\n\n"

        if extra_context:
            user_content = extra_context + f"## Question\n\n{question}"
        else:
            user_content = question

        messages.append({"role": "user", "content": user_content})

        # ── Stream from Ollama /api/chat ────────────────────────
        full_response = ""
        try:
            async with httpx.AsyncClient(timeout=OLLAMA_TIMEOUT) as client:
                async with client.stream(
                    "POST",
                    f"{OLLAMA_URL}/api/chat",
                    json={
                        "model": OLLAMA_MODEL,
                        "messages": messages,
                        "stream": True,
                        "options": {
                            "num_ctx": 4096,
                            "num_predict": 512,
                            "temperature": 0.4,
                        },
                    },
                ) as resp:
                    if resp.status_code != 200:
                        yield _sse("error", {
                            "message": f"Ollama returned {resp.status_code}",
                        })
                        return

                    async for line in resp.aiter_lines():
                        if not line:
                            continue
                        try:
                            data = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        token = data.get("message", {}).get("content", "")
                        if token:
                            full_response += token
                            yield _sse("token", {"text": token})

                        if data.get("done", False):
                            break

        except httpx.ConnectError:
            yield _sse("error", {
                "message": "Cannot connect to Ollama. Is it running?",
            })
            return
        except httpx.ReadTimeout:
            yield _sse("error", {
                "message": "Response timed out. The model may still be loading.",
            })
            return
        except Exception as e:
            yield _sse("error", {"message": str(e)})
            return

        # ── Update Session History ──────────────────────────────
        # Store the clean question (without RAG context) in history
        session["messages"].append({"role": "user", "content": question})
        session["messages"].append({"role": "assistant", "content": full_response})
        session["turn_count"] += 1

        # Trim if too many turns
        if len(session["messages"]) > MAX_HISTORY_TURNS * 2:
            session["messages"] = session["messages"][-(MAX_HISTORY_TURNS * 2):]

        # ── Done ────────────────────────────────────────────────
        t_end = time.time()
        yield _sse("state", {
            "stage": "done",
            "message": "Complete",
            "timing": {
                "total_ms": round((t_end - t_start) * 1000),
                "search_ms": round((t_search_done - t_start) * 1000) if query_type == "rag" else 0,
                "generate_ms": round((t_end - t_search_done) * 1000),
            },
            "query_type": query_type,
            "location_intent": loc_intent.intent if loc_intent.intent != "none" else None,
            "mcp_actions": [a.tool for a in mcp_actions] if mcp_actions else None,
            "turn": session["turn_count"],
            "condensation_count": session["condensation_count"],
        })

    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


def _sse(event: str, data: dict) -> str:
    """Format a Server-Sent Event."""
    return f"event: {event}\ndata: {json.dumps(data)}\n\n"


if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", "5050"))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
