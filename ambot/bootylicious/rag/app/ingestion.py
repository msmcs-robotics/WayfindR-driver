from __future__ import annotations

import hashlib
import json
import logging
import os
import re
from pathlib import Path
from typing import Protocol, runtime_checkable

import yaml
from sqlalchemy.ext.asyncio import AsyncSession

from .config import settings
from .database import Chunk, Document
from .embeddings import EmbeddingService

logger = logging.getLogger(__name__)

# =============================================================================
# File handlers
# =============================================================================


@runtime_checkable
class FileHandler(Protocol):
    def extract_text(self, filepath: Path) -> str: ...


class PDFHandler:
    def extract_text(self, filepath: Path) -> str:
        import fitz  # pymupdf

        text_parts: list[str] = []
        with fitz.open(filepath) as doc:
            for page in doc:
                text_parts.append(page.get_text())
        return "\n".join(text_parts)


class MarkdownHandler:
    def extract_text(self, filepath: Path) -> str:
        return filepath.read_text(encoding="utf-8")


class TextHandler:
    def extract_text(self, filepath: Path) -> str:
        return filepath.read_text(encoding="utf-8")


class JSONHandler:
    def extract_text(self, filepath: Path) -> str:
        data = json.loads(filepath.read_text(encoding="utf-8"))
        return json.dumps(data, indent=2, ensure_ascii=False)


class YAMLHandler:
    def extract_text(self, filepath: Path) -> str:
        data = yaml.safe_load(filepath.read_text(encoding="utf-8"))
        return yaml.dump(data, default_flow_style=False, allow_unicode=True)


class HandlerRegistry:
    _handlers: dict[str, FileHandler] = {
        ".pdf": PDFHandler(),
        ".md": MarkdownHandler(),
        ".txt": TextHandler(),
        ".log": TextHandler(),
        ".json": JSONHandler(),
        ".yaml": YAMLHandler(),
        ".yml": YAMLHandler(),
    }

    @classmethod
    def get(cls, extension: str) -> FileHandler | None:
        return cls._handlers.get(extension.lower())

    @classmethod
    def supported_extensions(cls) -> set[str]:
        return set(cls._handlers.keys())


# =============================================================================
# Chunking
# =============================================================================

_SENTENCE_RE = re.compile(r"(?<=[.!?])\s+")


def chunk_text(
    text: str,
    chunk_size: int = settings.CHUNK_SIZE,
    chunk_overlap: int = settings.CHUNK_OVERLAP,
) -> list[str]:
    """Split *text* into token-aware chunks along sentence boundaries.

    ``chunk_size`` and ``chunk_overlap`` are measured in *whitespace tokens*
    (a good-enough proxy that avoids a tokenizer dependency).
    """
    sentences = _SENTENCE_RE.split(text.strip())
    sentences = [s.strip() for s in sentences if s.strip()]

    chunks: list[str] = []
    current_tokens: list[str] = []
    current_len = 0

    for sentence in sentences:
        tokens = sentence.split()
        token_count = len(tokens)

        if current_len + token_count > chunk_size and current_tokens:
            chunks.append(" ".join(current_tokens))
            # Keep overlap tokens from the end
            overlap_tokens: list[str] = []
            overlap_len = 0
            for t in reversed(current_tokens):
                if overlap_len + 1 > chunk_overlap:
                    break
                overlap_tokens.insert(0, t)
                overlap_len += 1
            current_tokens = overlap_tokens
            current_len = overlap_len

        current_tokens.extend(tokens)
        current_len += token_count

    if current_tokens:
        chunks.append(" ".join(current_tokens))

    return chunks if chunks else [text.strip()] if text.strip() else []


# =============================================================================
# Ingestion pipeline
# =============================================================================


async def ingest_file(
    filepath: str | Path,
    session: AsyncSession,
    embedding_service: EmbeddingService,
) -> Document:
    filepath = Path(filepath)
    if not filepath.is_file():
        raise FileNotFoundError(f"File not found: {filepath}")

    handler = HandlerRegistry.get(filepath.suffix)
    if handler is None:
        raise ValueError(
            f"Unsupported file type: {filepath.suffix}. "
            f"Supported: {HandlerRegistry.supported_extensions()}"
        )

    # Extract
    text = handler.extract_text(filepath)

    # Chunk
    chunks_text = chunk_text(text, settings.CHUNK_SIZE, settings.CHUNK_OVERLAP)

    # Embed
    embeddings = await embedding_service.embed_batch(chunks_text)

    # Store
    content_hash = hashlib.sha256(text.encode()).hexdigest()
    doc = Document(
        filename=filepath.name,
        filepath=str(filepath.resolve()),
        file_type=filepath.suffix.lstrip("."),
        content_hash=content_hash,
        file_size=filepath.stat().st_size,
        content=text,
        chunk_count=len(chunks_text),
    )
    session.add(doc)
    await session.flush()  # get doc.id

    for idx, (chunk_content, embedding) in enumerate(zip(chunks_text, embeddings)):
        chunk = Chunk(
            document_id=doc.id,
            content=chunk_content,
            chunk_index=idx,
            embedding=embedding,
        )
        session.add(chunk)

    await session.commit()
    await session.refresh(doc)
    logger.info("Ingested %s (%d chunks)", filepath.name, len(chunks_text))
    return doc


async def ingest_directory(
    dirpath: str | Path,
    session: AsyncSession,
    embedding_service: EmbeddingService,
) -> list[Document]:
    dirpath = Path(dirpath)
    if not dirpath.is_dir():
        raise FileNotFoundError(f"Directory not found: {dirpath}")

    supported = HandlerRegistry.supported_extensions()
    documents: list[Document] = []

    for entry in sorted(dirpath.rglob("*")):
        if entry.is_file() and entry.suffix.lower() in supported:
            try:
                doc = await ingest_file(entry, session, embedding_service)
                documents.append(doc)
            except Exception:
                logger.exception("Failed to ingest %s", entry)

    return documents
