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
    execute_move_to, execute_list_locations, execute_waypoint_route,
    init_motors, cleanup_motors,
)
from chat_app import db

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── Configuration ────────────────────────────────────────────────
RAG_API_URL = os.environ.get("RAG_API_URL", "http://localhost:8000")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://localhost:11434")
OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "llama3.2:3b")
OLLAMA_TIMEOUT = float(os.environ.get("OLLAMA_TIMEOUT", "120"))
MAX_CONTEXT_TOKENS = int(os.environ.get("MAX_CONTEXT_TOKENS", "3200"))
MAX_HISTORY_TURNS = int(os.environ.get("MAX_HISTORY_TURNS", "20"))
# Reserve tokens for system prompt, RAG context injected into user message,
# and the new user message itself — these are NOT counted in session["messages"].
RESERVED_CONTEXT_TOKENS = 800

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

# ── Database Init ────────────────────────────────────────────────
_db_available = db.init_db()
if _db_available:
    logger.info("PostgreSQL conversation persistence enabled")
else:
    logger.info("PostgreSQL unavailable — using in-memory sessions only")

# ── Motor Bridge Init ───────────────────────────────────────────
# Set ENABLE_MOTORS=1 to activate hardware motor control from MCP tools.
# Without it, move_to commands update state only (no physical movement).
_enable_motors = os.environ.get("ENABLE_MOTORS", "0")
if _enable_motors == "1":
    _simulate_motors = False
    logger.info("ENABLE_MOTORS=1 — initializing hardware motor bridge")
elif _enable_motors == "sim":
    _simulate_motors = True
    logger.info("ENABLE_MOTORS=sim — initializing simulated motor bridge")
else:
    _simulate_motors = None  # sentinel: skip init entirely

if _simulate_motors is not None:
    _motors_ready = init_motors(simulate=_simulate_motors)
    if _motors_ready:
        logger.info("Motor bridge ready (simulate=%s)", _simulate_motors)
    else:
        logger.warning("Motor bridge init failed — move_to will use state-only simulation")
else:
    logger.info("Motors not enabled (set ENABLE_MOTORS=1 or ENABLE_MOTORS=sim)")

# ── Session Store ────────────────────────────────────────────────
# In-memory conversation history keyed by session_id
_sessions: dict[str, dict] = defaultdict(lambda: {
    "messages": [],
    "created": time.time(),
    "turn_count": 0,
    "condensation_count": 0,
})


def _get_session(session_id: str) -> dict:
    """Get or create a session. Loads from DB if available."""
    if session_id not in _sessions and _db_available:
        loaded = db.load_session(session_id)
        if loaded:
            _sessions[session_id] = loaded
            logger.info("Loaded session %s from DB (%d messages)",
                        session_id, len(loaded["messages"]))
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
# Exact-match casual phrases (greeting, acknowledgment, farewell).
# These only match when the ENTIRE query is the pattern (after stripping
# trailing punctuation), NOT when the pattern is a prefix of a longer query.
_CASUAL_EXACT = {
    "hello", "hi", "hey", "greetings", "good morning", "good afternoon",
    "good evening", "thanks", "thank you", "bye", "goodbye", "see you",
    "ok", "okay", "sure", "yes", "no", "cool", "nice", "great",
    "how are you", "what's up", "sup", "yo",
    "lol", "haha", "hmm", "wow",
    "help", "what can you do", "what do you do",
}

# Casual phrases that are also valid as prefixes of short messages
# (e.g. "hi there", "hey!", "thanks a lot").  Matched only when the
# total query is short enough that it's clearly still casual.
_CASUAL_PREFIXES = {
    "hi", "hey", "hello", "thanks", "thank you", "bye", "goodbye",
    "good morning", "good afternoon", "good evening",
}
_CASUAL_PREFIX_MAX_WORDS = 4  # "good morning how are you" = 5 words → still casual

# Out-of-scope question patterns: questions a campus robot can't answer.
_OUT_OF_SCOPE_PREFIXES = (
    "what time", "what day", "what date", "what year",
    "how's the weather", "hows the weather", "what's the weather",
    "whats the weather",
)

# Follow-up cues: when conversation history exists and the query is short,
# these indicate the user wants more on the current topic (no new RAG search).
_FOLLOWUP_CUES = {
    "more", "why", "elaborate", "explain that", "what about",
    "how so", "really", "interesting", "go on", "continue",
    "and", "also", "what else", "tell me more", "can you explain",
}

_DOMAIN_KEYWORDS = {
    "program", "degree", "lab", "faculty", "professor", "department",
    "course", "class", "major", "minor", "accreditation", "research",
    "club", "student", "campus", "erau", "embry", "riddle",
    "eecs", "engineering", "computer science", "electrical",
    "robotics", "ambot", "robot", "lidar", "sensor", "jetson",
    "rag", "knowledge", "document",
}

_QUESTION_STARTERS = {
    "what", "where", "which", "how", "who", "when", "does", "is there",
    "tell me about", "explain", "describe", "list", "show me",
}


def classify_query(question: str, has_history: bool = False) -> str:
    """Classify whether a query needs RAG retrieval or is casual conversation.

    Returns: 'rag' or 'casual'
    """
    lower = question.lower().strip().rstrip("?!.")
    words = lower.split()

    if not words:
        return "casual"

    # 1. Exact casual match (entire query after punctuation strip)
    if lower in _CASUAL_EXACT:
        return "casual"

    # 2. Casual prefix match — only when the total query is short.
    #    Prevents "hi where is the lab" from being classified as casual.
    if len(words) <= _CASUAL_PREFIX_MAX_WORDS:
        for prefix in _CASUAL_PREFIXES:
            if lower == prefix or lower.startswith(prefix + " "):
                # Make sure the remaining words aren't domain keywords
                remainder = lower[len(prefix):].strip()
                remainder_words = remainder.split() if remainder else []
                has_domain = any(kw in remainder for kw in _DOMAIN_KEYWORDS)
                if not has_domain:
                    return "casual"

    # 3. Out-of-scope questions the robot can't answer
    for prefix in _OUT_OF_SCOPE_PREFIXES:
        if lower.startswith(prefix):
            return "casual"

    # 4. Follow-up detection: short referential queries use conversation context
    if has_history and len(words) <= 5:
        if any(cue in lower for cue in _FOLLOWUP_CUES):
            return "casual"  # Use conversation context, skip RAG

    # 5. Very short messages (1-2 words) without domain keywords are casual
    if len(words) <= 2:
        # Still check for domain keywords even in short queries
        for keyword in _DOMAIN_KEYWORDS:
            if keyword in lower:
                return "rag"
        return "casual"

    # 6. Domain keyword check — any mention of a known topic triggers RAG
    for keyword in _DOMAIN_KEYWORDS:
        if keyword in lower:
            return "rag"

    # 7. Question words + enough length suggest a knowledge question
    if len(words) > 4:
        for starter in _QUESTION_STARTERS:
            if lower.startswith(starter):
                return "rag"

    # 8. Questions (ending with ?) with 5+ words likely need lookup
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
async def delete_session(session_id: str):
    """Clear a session's conversation history."""
    if session_id in _sessions:
        del _sessions[session_id]
    if _db_available:
        db.clear_session(session_id)
    return {"cleared": True}


@app.get("/api/sessions")
async def get_sessions():
    """List recent conversation sessions."""
    if _db_available:
        return {"sessions": db.list_sessions()}
    return {"sessions": []}


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
    has_history = len(session["messages"]) > 0
    query_type = classify_query(question, has_history=has_history)
    t_start = time.time()

    # Check for location intent
    loc_intent = classify_location_intent(question, location_mgr)

    async def event_stream():
        sources = []
        rag_context = ""
        mcp_actions = []
        t_search_done = t_start

        # ── MCP: Multi-stop waypoint route handling ─────────────
        if loc_intent.intent == "multi_stop" and loc_intent.matched_locations:
            # Plan the full route
            stop_names = [m["name"] for m in loc_intent.matched_locations]
            action = execute_waypoint_route(stop_names, location_mgr)
            mcp_actions.append(action)
            yield _sse("mcp_action", {
                "tool": action.tool,
                "text": action.display_text,
                "result": action.result,
            })

            # Store waypoint queue in session for future stop-by-stop navigation
            if action.result.get("success"):
                session["waypoint_queue"] = {
                    "stops": action.result["stops"],
                    "current_index": 0,
                    "total": action.result["stop_count"],
                }
                logger.info("Waypoint queue stored: %d stops — %s",
                            len(action.result["stops"]),
                            " -> ".join(s["name"] for s in action.result["stops"]))

        # ── MCP: Single location intent handling ─────────────────
        elif loc_intent.intent == "execution" and loc_intent.matched_locations:
            top_match = loc_intent.matched_locations[0]

            # Check if user is confirming a pending navigation
            pending = session.get("pending_navigation")
            is_confirmation = pending and question.lower().strip() in (
                "yes", "yeah", "yep", "sure", "ok", "okay", "confirm",
                "go", "let's go", "do it", "yes please",
            )

            if is_confirmation:
                # User confirmed — execute the pending move
                action = execute_move_to(pending["name"], location_mgr)
                mcp_actions.append(action)
                yield _sse("mcp_action", {
                    "tool": action.tool,
                    "text": action.display_text,
                    "result": action.result,
                })
                session.pop("pending_navigation", None)

            elif top_match["score"] >= 0.9 and loc_intent.confidence >= 0.8:
                # Very high confidence exact match — execute directly
                action = execute_move_to(top_match["name"], location_mgr)
                mcp_actions.append(action)
                yield _sse("mcp_action", {
                    "tool": action.tool,
                    "text": action.display_text,
                    "result": action.result,
                })
                session.pop("pending_navigation", None)

            else:
                # Lower confidence — store as pending, let LLM ask for confirmation
                session["pending_navigation"] = top_match
                yield _sse("mcp_action", {
                    "tool": "pending_confirmation",
                    "text": f"Did you mean: {top_match['name']}?",
                    "result": {"pending": True, "candidates": loc_intent.matched_locations[:3]},
                })

        elif loc_intent.intent == "none":
            # Check if this is a confirmation for a pending navigation
            pending = session.get("pending_navigation")
            if pending and question.lower().strip() in (
                "yes", "yeah", "yep", "sure", "ok", "okay", "confirm",
                "go", "let's go", "do it", "yes please",
            ):
                action = execute_move_to(pending["name"], location_mgr)
                mcp_actions.append(action)
                yield _sse("mcp_action", {
                    "tool": action.tool,
                    "text": action.display_text,
                    "result": action.result,
                })
                session.pop("pending_navigation", None)
            elif pending and question.lower().strip() in ("no", "nope", "cancel", "nevermind", "never mind"):
                session.pop("pending_navigation", None)
                yield _sse("mcp_action", {
                    "tool": "cancelled",
                    "text": "Navigation cancelled.",
                    "result": {"cancelled": True},
                })

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
        # Budget accounts for system prompt + RAG context + new user message
        # which are added to the Ollama request but not tracked in session history.
        history_budget = MAX_CONTEXT_TOKENS - RESERVED_CONTEXT_TOKENS
        if history_tokens > history_budget * 0.7:
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

        # Persist to DB
        if _db_available:
            db.save_message(session_id, "user", question, session["turn_count"])
            db.save_message(session_id, "assistant", full_response, session["turn_count"])

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


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up motor resources on server shutdown."""
    cleanup_motors()


if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", "5050"))
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
