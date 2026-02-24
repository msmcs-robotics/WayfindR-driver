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
# Keyword search (dual: English stemmed AND + Simple exact OR)
# =============================================================================
#
# English config stems words and uses AND logic — great for natural language
# but mangles acronyms (e.g. "LLM" gets stemmed away).
# Simple config does exact matching with OR logic — preserves acronyms.
# Running both and merging (with 20% boost for overlap) gives best results.
# See: rag-optimization-review.md §3.1


async def _keyword_english(
    query: str,
    session: AsyncSession,
    limit: int = 10,
) -> list[SearchResult]:
    """English-stemmed keyword search (AND logic via plainto_tsquery)."""
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


async def _keyword_simple(
    query: str,
    session: AsyncSession,
    limit: int = 10,
) -> list[SearchResult]:
    """Simple exact keyword search (OR logic) — preserves acronyms."""
    # Build OR query: "LLM RAG" → "LLM | RAG"
    words = query.split()
    if not words:
        return []
    or_query = " | ".join(words)

    ts_query = func.to_tsquery("simple", or_query)
    ts_vector = func.to_tsvector("simple", Chunk.content)
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


async def keyword_search(
    query: str,
    session: AsyncSession,
    limit: int = 10,
) -> list[SearchResult]:
    """Dual keyword search: merge English stemmed + Simple exact results.

    Chunks found by both methods get a 20% score boost.
    """
    expanded = _expand_acronyms(query)
    english_results = await _keyword_english(expanded, session, limit)
    simple_results = await _keyword_simple(query, session, limit)  # simple uses original (exact match)

    # Merge with overlap boost
    result_map: dict[int, SearchResult] = {}
    scores: dict[int, float] = {}

    for r in english_results:
        result_map[r.chunk_id] = r
        scores[r.chunk_id] = r.score

    for r in simple_results:
        if r.chunk_id in scores:
            scores[r.chunk_id] *= 1.2  # 20% boost for overlap
        else:
            result_map[r.chunk_id] = r
            scores[r.chunk_id] = r.score

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
        for cid in ranked_ids[:limit]
    ]


# =============================================================================
# Domain acronym expansion
# =============================================================================
#
# Expand acronyms to include their full form so keyword search matches both.
# E.g., query "LLM" becomes "LLM large language model" — this lets the English
# stemmed search find chunks containing the full phrase too.

ACRONYM_TABLE: dict[str, str] = {
    # AI / ML
    "LLM": "large language model",
    "RAG": "retrieval augmented generation",
    "NLP": "natural language processing",
    "ML": "machine learning",
    "AI": "artificial intelligence",
    "GPU": "graphics processing unit",
    "CUDA": "compute unified device architecture",
    "VRAM": "video random access memory",
    "KV": "key value cache",
    "GGUF": "GPT-Generated Unified Format quantization",
    # Robotics / Sensors
    "IMU": "inertial measurement unit",
    "SLAM": "simultaneous localization and mapping",
    "ICP": "iterative closest point",
    "LiDAR": "light detection and ranging",
    "PWM": "pulse width modulation",
    "GPIO": "general purpose input output",
    "I2C": "inter-integrated circuit",
    "PID": "proportional integral derivative",
    "DOF": "degrees of freedom",
    "MPU": "motion processing unit",
    # Computing / Networking
    "SSH": "secure shell",
    "API": "application programming interface",
    "MCP": "model context protocol",
    "ROS": "robot operating system",
    "STT": "speech to text",
    "TTS": "text to speech",
    # Hardware
    "RPi": "Raspberry Pi",
    "SBC": "single board computer",
    "ARM": "Advanced RISC Machine",
    "EECS": "electrical engineering computer science",
}


def _expand_acronyms(query: str) -> str:
    """Append expanded forms for any acronyms found in the query."""
    words = query.split()
    expansions: list[str] = []
    for word in words:
        # Strip common punctuation for matching
        clean = word.strip("?.,!:;")
        if clean in ACRONYM_TABLE:
            expansions.append(ACRONYM_TABLE[clean])
        elif clean.upper() in ACRONYM_TABLE:
            expansions.append(ACRONYM_TABLE[clean.upper()])
    if expansions:
        return query + " " + " ".join(expansions)
    return query


# =============================================================================
# Adaptive semantic weight
# =============================================================================


def _adaptive_weight(query: str) -> float:
    """Choose semantic vs keyword weight based on query characteristics.

    Short acronym queries benefit from keyword search (exact match).
    Long natural language queries benefit from semantic search (meaning).

    Returns the semantic weight (0.0–1.0); keyword weight = 1 - semantic.
    """
    words = query.split()
    word_count = len(words)
    upper_ratio = sum(1 for w in words if w.isupper() and len(w) >= 2) / max(word_count, 1)

    # Short acronym query (e.g. "LLM", "RAG API"): heavy keyword
    if word_count <= 2 and upper_ratio > 0.5:
        return 0.2

    # Contains acronyms but also natural language
    if upper_ratio > 0.0:
        return 0.3

    # Short natural language (e.g. "what is a motor")
    if word_count <= 4:
        return 0.5

    # Long natural language (e.g. "how does the LiDAR navigation system avoid obstacles")
    return 0.7


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
    semantic_weight: float | None = None,
) -> list[SearchResult]:
    # Use adaptive weight unless caller specifies a fixed weight
    if semantic_weight is None:
        semantic_weight = _adaptive_weight(query)

    # Fetch 5x candidates for better RRF fusion quality (from rag-atc-testing)
    fetch_limit = limit * 5
    sem_results = await semantic_search(query, session, embedding_service, fetch_limit)
    kw_results = await keyword_search(query, session, fetch_limit)
    fused = _rrf(sem_results, kw_results, semantic_weight)
    return fused[:limit]
