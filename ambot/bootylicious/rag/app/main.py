from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from enum import Enum
from pathlib import Path
from typing import Annotated

import redis.asyncio as aioredis
from fastapi import Depends, FastAPI, HTTPException, UploadFile, status
from pydantic import BaseModel, Field
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from .config import settings
from .database import Document, async_session, engine, init_db
from .embeddings import EmbeddingService
from .ingestion import ingest_directory, ingest_file
from .search import hybrid_search, keyword_search, semantic_search
from .llm import LLMResponse, OllamaClient

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# =============================================================================
# Lifespan
# =============================================================================


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Initializing database...")
    await init_db()

    logger.info("Connecting to Redis at %s", settings.REDIS_URL)
    redis_client = aioredis.from_url(settings.REDIS_URL, decode_responses=True)
    app.state.redis = redis_client

    embedding_service = EmbeddingService(redis_client=redis_client)
    app.state.embedding_service = embedding_service

    llm_client = OllamaClient()
    app.state.llm_client = llm_client

    logger.info("Startup complete.")
    yield

    # Shutdown
    await redis_client.aclose()
    await engine.dispose()
    logger.info("Shutdown complete.")


app = FastAPI(title=settings.PROJECT_NAME, lifespan=lifespan)


# =============================================================================
# Dependencies
# =============================================================================


async def get_session():
    async with async_session() as session:
        yield session


SessionDep = Annotated[AsyncSession, Depends(get_session)]


def get_embedding_service() -> EmbeddingService:
    return app.state.embedding_service


EmbeddingDep = Annotated[EmbeddingService, Depends(get_embedding_service)]


def get_llm_client() -> OllamaClient:
    return app.state.llm_client


LLMDep = Annotated[OllamaClient, Depends(get_llm_client)]


# =============================================================================
# Schemas
# =============================================================================


class SearchMode(str, Enum):
    semantic = "semantic"
    keyword = "keyword"
    hybrid = "hybrid"


class SearchRequest(BaseModel):
    query: str
    mode: SearchMode = SearchMode.hybrid
    limit: int = Field(default=10, ge=1, le=100)


class SearchResultSchema(BaseModel):
    chunk_id: int
    document_id: int
    document_filename: str
    chunk_index: int
    content: str
    score: float


class DocumentSchema(BaseModel):
    id: int
    filename: str
    filepath: str
    file_type: str
    file_size: int
    chunk_count: int
    created_at: str
    updated_at: str


class DirectoryIngestRequest(BaseModel):
    path: str


class AskRequest(BaseModel):
    question: str
    mode: SearchMode = SearchMode.hybrid
    limit: int = Field(default=5, ge=1, le=20)
    system_prompt: str | None = None


class AskResponse(BaseModel):
    answer: str
    model: str
    sources: list[SearchResultSchema]


class MultiDirectoryIngestRequest(BaseModel):
    paths: list[str]


class HealthStatus(BaseModel):
    status: str
    database: bool
    redis: bool
    embedding_service: bool
    llm: bool


# =============================================================================
# Routes
# =============================================================================


@app.get("/api/health", response_model=HealthStatus)
async def health_check(session: SessionDep, embed: EmbeddingDep):
    db_ok = False
    redis_ok = False
    embed_ok = False

    try:
        await session.execute(select(func.now()))
        db_ok = True
    except Exception:
        logger.exception("DB health check failed")

    try:
        await app.state.redis.ping()
        redis_ok = True
    except Exception:
        logger.exception("Redis health check failed")

    try:
        embed_ok = await embed.health_check()
    except Exception:
        logger.exception("Embedding health check failed")

    llm_ok = False
    try:
        llm_ok = await app.state.llm_client.health_check()
    except Exception:
        logger.exception("LLM health check failed")

    overall = "healthy" if all([db_ok, redis_ok, embed_ok, llm_ok]) else "degraded"
    return HealthStatus(
        status=overall, database=db_ok, redis=redis_ok, embedding_service=embed_ok, llm=llm_ok
    )


@app.post("/api/ingest/file", response_model=DocumentSchema, status_code=status.HTTP_201_CREATED)
async def ingest_upload(
    file: UploadFile,
    session: SessionDep,
    embed: EmbeddingDep,
):
    import tempfile

    suffix = Path(file.filename or "upload.bin").suffix
    with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as tmp:
        content = await file.read()
        tmp.write(content)
        tmp_path = Path(tmp.name)

    try:
        doc = await ingest_file(tmp_path, session, embed)
    except (FileNotFoundError, ValueError) as exc:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc))
    finally:
        tmp_path.unlink(missing_ok=True)

    return _doc_to_schema(doc)


@app.post(
    "/api/ingest/directory",
    response_model=list[DocumentSchema],
    status_code=status.HTTP_201_CREATED,
)
async def ingest_dir(
    body: DirectoryIngestRequest,
    session: SessionDep,
    embed: EmbeddingDep,
):
    try:
        docs = await ingest_directory(body.path, session, embed)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc))
    return [_doc_to_schema(d) for d in docs]


@app.get("/api/documents", response_model=list[DocumentSchema])
async def list_documents(session: SessionDep):
    result = await session.execute(select(Document).order_by(Document.created_at.desc()))
    return [_doc_to_schema(d) for d in result.scalars().all()]


@app.get("/api/documents/{doc_id}", response_model=DocumentSchema)
async def get_document(doc_id: int, session: SessionDep):
    doc = await session.get(Document, doc_id)
    if doc is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Document not found")
    return _doc_to_schema(doc)


@app.delete("/api/documents/{doc_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_document(doc_id: int, session: SessionDep):
    doc = await session.get(Document, doc_id)
    if doc is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Document not found")
    await session.delete(doc)
    await session.commit()


@app.post("/api/search", response_model=list[SearchResultSchema])
async def search(body: SearchRequest, session: SessionDep, embed: EmbeddingDep):
    if body.mode == SearchMode.semantic:
        results = await semantic_search(body.query, session, embed, body.limit)
    elif body.mode == SearchMode.keyword:
        results = await keyword_search(body.query, session, body.limit)
    else:
        results = await hybrid_search(body.query, session, embed, body.limit)

    return [
        SearchResultSchema(
            chunk_id=r.chunk_id,
            document_id=r.document_id,
            document_filename=r.document_filename,
            chunk_index=r.chunk_index,
            content=r.content,
            score=r.score,
        )
        for r in results
    ]



@app.post("/api/ask", response_model=AskResponse)
async def ask_question(
    body: AskRequest,
    session: SessionDep,
    embed: EmbeddingDep,
    llm: LLMDep,
):
    """Ask a question and get an answer using RAG-retrieved context."""
    # First, search for relevant context
    if body.mode == SearchMode.semantic:
        results = await semantic_search(body.question, session, embed, body.limit)
    elif body.mode == SearchMode.keyword:
        results = await keyword_search(body.question, session, body.limit)
    else:
        results = await hybrid_search(body.question, session, embed, body.limit)

    if not results:
        return AskResponse(
            answer="No relevant documents found to answer this question. Please ingest documents first.",
            model=llm.model,
            sources=[],
        )

    # Format chunks for LLM
    context_chunks = [
        {
            "content": r.content,
            "document_filename": r.document_filename,
            "score": r.score,
        }
        for r in results
    ]

    # Generate answer
    llm_response = await llm.ask_with_context(
        question=body.question,
        context_chunks=context_chunks,
        system_prompt=body.system_prompt,
    )

    sources = [
        SearchResultSchema(
            chunk_id=r.chunk_id,
            document_id=r.document_id,
            document_filename=r.document_filename,
            chunk_index=r.chunk_index,
            content=r.content,
            score=r.score,
        )
        for r in results
    ]

    return AskResponse(
        answer=llm_response.answer,
        model=llm_response.model,
        sources=sources,
    )


@app.post(
    "/api/ingest/directories",
    response_model=list[DocumentSchema],
    status_code=status.HTTP_201_CREATED,
)
async def ingest_dirs(
    body: MultiDirectoryIngestRequest,
    session: SessionDep,
    embed: EmbeddingDep,
):
    """Ingest documents from multiple directories."""
    all_docs = []
    for path in body.paths:
        try:
            docs = await ingest_directory(path, session, embed)
            all_docs.extend(docs)
        except FileNotFoundError:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Directory not found: {path}",
            )
    return [_doc_to_schema(d) for d in all_docs]


@app.get("/api/models")
async def list_models(llm: LLMDep):
    """List available Ollama models."""
    models = await llm.list_models()
    return {"models": models, "current": llm.model}


# =============================================================================
# Helpers
# =============================================================================


def _doc_to_schema(doc: Document) -> DocumentSchema:
    return DocumentSchema(
        id=doc.id,
        filename=doc.filename,
        filepath=doc.filepath,
        file_type=doc.file_type,
        file_size=doc.file_size,
        chunk_count=doc.chunk_count,
        created_at=doc.created_at.isoformat() if doc.created_at else "",
        updated_at=doc.updated_at.isoformat() if doc.updated_at else "",
    )
