"""AMBOT Chat — Lightweight FastAPI chat interface for the RAG API.

Proxies to the bootylicious RAG API and serves a simple chat frontend
with streaming responses and state indicators.
"""

import json
import logging
import os

import httpx
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

RAG_API_URL = os.environ.get("RAG_API_URL", "http://localhost:8000")
RAG_TIMEOUT = float(os.environ.get("RAG_TIMEOUT", "120"))

app = FastAPI(title="AMBOT Chat", docs_url=None, redoc_url=None)

# Serve static files (CSS, JS)
static_dir = os.path.join(os.path.dirname(__file__), "static")
app.mount("/static", StaticFiles(directory=static_dir), name="static")


@app.get("/", response_class=HTMLResponse)
async def index():
    """Serve the chat UI."""
    html_path = os.path.join(static_dir, "index.html")
    with open(html_path) as f:
        return HTMLResponse(f.read())


@app.get("/api/health")
async def health():
    """Check RAG API health and return combined status."""
    try:
        async with httpx.AsyncClient(timeout=10) as client:
            resp = await client.get(f"{RAG_API_URL}/api/health")
            if resp.status_code == 200:
                data = resp.json()
                return {
                    "status": "ready",
                    "rag_api": True,
                    "details": data,
                }
    except httpx.ConnectError:
        return {"status": "rag_unavailable", "rag_api": False,
                "error": "Cannot reach RAG API"}
    except Exception as e:
        return {"status": "error", "rag_api": False, "error": str(e)}

    return {"status": "degraded", "rag_api": False}


@app.get("/api/models")
async def models():
    """Return available LLM models."""
    try:
        async with httpx.AsyncClient(timeout=10) as client:
            resp = await client.get(f"{RAG_API_URL}/api/models")
            if resp.status_code == 200:
                return resp.json()
    except Exception as e:
        logger.warning("Failed to fetch models: %s", e)
    return {"models": [], "current": "unknown"}


@app.get("/api/documents")
async def documents():
    """Return ingested document list."""
    try:
        async with httpx.AsyncClient(timeout=10) as client:
            resp = await client.get(f"{RAG_API_URL}/api/documents")
            if resp.status_code == 200:
                docs = resp.json()
                return {"documents": docs, "count": len(docs)}
    except Exception as e:
        logger.warning("Failed to fetch documents: %s", e)
    return {"documents": [], "count": 0}


@app.post("/api/chat")
async def chat(request: Request):
    """Stream a chat response from the RAG API via SSE.

    Expects JSON: {"question": "...", "mode": "hybrid"}
    Returns: text/event-stream with state transitions.
    """
    body = await request.json()
    question = body.get("question", "").strip()
    if not question:
        return {"error": "Empty question"}

    mode = body.get("mode", "hybrid")

    async def event_stream():
        # Stage 1: Searching
        yield _sse("state", {"stage": "searching",
                              "message": "Searching knowledge base..."})

        try:
            async with httpx.AsyncClient(timeout=RAG_TIMEOUT) as client:
                payload = {"question": question, "mode": mode}
                async with client.stream(
                    "POST",
                    f"{RAG_API_URL}/api/ask/stream",
                    json=payload,
                ) as resp:
                    if resp.status_code != 200:
                        yield _sse("error", {
                            "message": f"RAG API returned {resp.status_code}"
                        })
                        return

                    async for line in resp.aiter_lines():
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            data = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        event = data.get("event")

                        if event == "sources":
                            # Stage 2: Generating
                            yield _sse("sources", {
                                "sources": data.get("sources", []),
                                "model": data.get("model", "unknown"),
                            })
                            yield _sse("state", {
                                "stage": "generating",
                                "message": "Generating response...",
                            })

                        elif event == "token":
                            yield _sse("token", {
                                "text": data.get("token", ""),
                            })

                        elif event == "done":
                            yield _sse("state", {
                                "stage": "done",
                                "message": "Complete",
                            })

                        elif event == "error":
                            yield _sse("error", {
                                "message": data.get("error", "Unknown error"),
                            })

        except httpx.ConnectError:
            yield _sse("error", {
                "message": "Cannot connect to RAG API. Is the Docker stack running?",
            })
        except httpx.ReadTimeout:
            yield _sse("error", {
                "message": "Response timed out. The model may still be loading.",
            })
        except Exception as e:
            yield _sse("error", {"message": str(e)})

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
