"""
FastMCP server exposing the RAG system as MCP tools.

Allows LLM agents to search, ingest, and manage documents
through the Model Context Protocol.
"""

from __future__ import annotations

import asyncio
from typing import Any

from fastmcp import FastMCP
from sqlalchemy import select

from .config import settings
from .database import Document, async_session, init_db
from .embeddings import EmbeddingService
from .ingestion import ingest_file as _ingest_file
from .search import hybrid_search, keyword_search, semantic_search

mcp = FastMCP(
    name="rag-bootstrap",
    description="RAG document search and ingestion system",
)

# ---------------------------------------------------------------------------
# Shared embedding service (lazy init)
# ---------------------------------------------------------------------------

_embedding_service: EmbeddingService | None = None


def _get_embedding_service() -> EmbeddingService:
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service


# ---------------------------------------------------------------------------
# MCP Tools
# ---------------------------------------------------------------------------


@mcp.tool()
async def search_documents(
    query: str,
    mode: str = "hybrid",
    limit: int = 10,
) -> dict[str, Any]:
    """Search ingested documents using semantic, keyword, or hybrid search.

    Args:
        query: The search query string.
        mode: Search mode - one of "semantic", "keyword", or "hybrid".
        limit: Maximum number of results to return (1-100).
    """
    if mode not in ("semantic", "keyword", "hybrid"):
        return {"error": f"Invalid mode '{mode}'. Use 'semantic', 'keyword', or 'hybrid'."}

    limit = max(1, min(limit, 100))
    embed_svc = _get_embedding_service()

    async with async_session() as session:
        if mode == "semantic":
            results = await semantic_search(query, session, embed_svc, limit)
        elif mode == "keyword":
            results = await keyword_search(query, session, limit)
        else:
            results = await hybrid_search(query, session, embed_svc, limit)

    return {
        "query": query,
        "mode": mode,
        "count": len(results),
        "results": [
            {
                "chunk_id": r.chunk_id,
                "document_id": r.document_id,
                "document_filename": r.document_filename,
                "chunk_index": r.chunk_index,
                "content": r.content,
                "score": r.score,
            }
            for r in results
        ],
    }


@mcp.tool()
async def ingest_file(filepath: str) -> dict[str, Any]:
    """Ingest a single file into the RAG system.

    Args:
        filepath: Absolute path to the file to ingest.
    """
    embed_svc = _get_embedding_service()

    async with async_session() as session:
        doc = await _ingest_file(filepath, session, embed_svc)

    return {
        "status": "ok",
        "document_id": doc.id,
        "filename": doc.filename,
        "chunk_count": doc.chunk_count,
    }


@mcp.tool()
async def list_documents() -> dict[str, Any]:
    """List all documents that have been ingested into the RAG system."""
    async with async_session() as session:
        stmt = select(Document).order_by(Document.id)
        result = await session.execute(stmt)
        documents = result.scalars().all()

    return {
        "count": len(documents),
        "documents": [
            {
                "id": doc.id,
                "filename": doc.filename,
                "filepath": doc.filepath,
                "file_type": doc.file_type,
                "chunk_count": doc.chunk_count,
                "created_at": doc.created_at.isoformat() if doc.created_at else None,
            }
            for doc in documents
        ],
    }


@mcp.tool()
async def get_document(document_id: int) -> dict[str, Any]:
    """Get detailed information about a specific ingested document.

    Args:
        document_id: The numeric ID of the document to retrieve.
    """
    async with async_session() as session:
        doc = await session.get(Document, document_id)

    if doc is None:
        return {"error": f"Document with id {document_id} not found."}

    return {
        "id": doc.id,
        "filename": doc.filename,
        "filepath": doc.filepath,
        "file_type": doc.file_type,
        "file_size": doc.file_size,
        "chunk_count": doc.chunk_count,
        "created_at": doc.created_at.isoformat() if doc.created_at else None,
    }


# ---------------------------------------------------------------------------
# Server entry point
# ---------------------------------------------------------------------------


async def _startup() -> None:
    """Initialise the database before serving requests."""
    await init_db()


def run_mcp_server() -> None:
    """Run the MCP server (stdio transport by default)."""
    asyncio.get_event_loop().run_until_complete(_startup())
    mcp.run()


if __name__ == "__main__":
    run_mcp_server()
