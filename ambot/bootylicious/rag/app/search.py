from __future__ import annotations

from dataclasses import dataclass

from sqlalchemy import func, select, text
from sqlalchemy.ext.asyncio import AsyncSession

from .config import settings
from .database import Chunk, Document
from .embeddings import EmbeddingService


@dataclass
class SearchResult:
    chunk_id: int
    document_id: int
    document_filename: str
    chunk_index: int
    content: str
    score: float


# =============================================================================
# Semantic search (pgvector cosine distance)
# =============================================================================


async def semantic_search(
    query: str,
    session: AsyncSession,
    embedding_service: EmbeddingService,
    limit: int = 10,
) -> list[SearchResult]:
    query_embedding = await embedding_service.embed_text(query)

    distance = Chunk.embedding.cosine_distance(query_embedding).label("distance")

    stmt = (
        select(Chunk, Document.filename, distance)
        .join(Document, Chunk.document_id == Document.id)
        .order_by(distance)
        .limit(limit)
    )

    rows = (await session.execute(stmt)).all()

    return [
        SearchResult(
            chunk_id=row.Chunk.id,
            document_id=row.Chunk.document_id,
            document_filename=row.filename,
            chunk_index=row.Chunk.chunk_index,
            content=row.Chunk.content,
            score=1.0 - float(row.distance),  # cosine similarity
        )
        for row in rows
    ]


# =============================================================================
# Keyword search (PostgreSQL full-text search)
# =============================================================================


async def keyword_search(
    query: str,
    session: AsyncSession,
    limit: int = 10,
) -> list[SearchResult]:
    ts_query = func.plainto_tsquery("english", query)
    ts_vector = func.to_tsvector("english", Chunk.content)
    rank = func.ts_rank(ts_vector, ts_query).label("rank")

    stmt = (
        select(Chunk, Document.filename, rank)
        .join(Document, Chunk.document_id == Document.id)
        .where(ts_vector.op("@@")(ts_query))
        .order_by(rank.desc())
        .limit(limit)
    )

    rows = (await session.execute(stmt)).all()

    return [
        SearchResult(
            chunk_id=row.Chunk.id,
            document_id=row.Chunk.document_id,
            document_filename=row.filename,
            chunk_index=row.Chunk.chunk_index,
            content=row.Chunk.content,
            score=float(row.rank),
        )
        for row in rows
    ]


# =============================================================================
# Hybrid search (Reciprocal Rank Fusion)
# =============================================================================


def _rrf(
    semantic_results: list[SearchResult],
    keyword_results: list[SearchResult],
    semantic_weight: float = 0.7,
    k: int = 60,
) -> list[SearchResult]:
    """Combine two ranked lists using Reciprocal Rank Fusion."""
    keyword_weight = 1.0 - semantic_weight
    scores: dict[int, float] = {}
    result_map: dict[int, SearchResult] = {}

    for rank, r in enumerate(semantic_results, start=1):
        scores[r.chunk_id] = scores.get(r.chunk_id, 0.0) + semantic_weight / (k + rank)
        result_map[r.chunk_id] = r

    for rank, r in enumerate(keyword_results, start=1):
        scores[r.chunk_id] = scores.get(r.chunk_id, 0.0) + keyword_weight / (k + rank)
        result_map.setdefault(r.chunk_id, r)

    ranked_ids = sorted(scores, key=lambda cid: scores[cid], reverse=True)
    return [
        SearchResult(
            chunk_id=result_map[cid].chunk_id,
            document_id=result_map[cid].document_id,
            document_filename=result_map[cid].document_filename,
            chunk_index=result_map[cid].chunk_index,
            content=result_map[cid].content,
            score=scores[cid],
        )
        for cid in ranked_ids
    ]


async def hybrid_search(
    query: str,
    session: AsyncSession,
    embedding_service: EmbeddingService,
    limit: int = 10,
    semantic_weight: float = 0.7,
) -> list[SearchResult]:
    # Fetch 5x candidates for better RRF fusion quality (from rag-atc-testing)
    fetch_limit = limit * 5
    sem_results = await semantic_search(query, session, embedding_service, fetch_limit)
    kw_results = await keyword_search(query, session, fetch_limit)
    fused = _rrf(sem_results, kw_results, semantic_weight)
    return fused[:limit]
