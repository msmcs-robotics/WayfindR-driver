from __future__ import annotations

import hashlib
import json
import logging
import os
import re
from pathlib import Path
from typing import Protocol, runtime_checkable

import yaml
from sqlalchemy import select
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
_DOT_LEADER_RE = re.compile(r"\.{2,}")  # two or more consecutive dots


def is_junk_chunk(
    text: str,
    digit_threshold: float = settings.JUNK_DIGIT_THRESHOLD,
    dot_leader_threshold: float = settings.JUNK_DOT_LEADER_THRESHOLD,
) -> bool:
    """Return True if *text* looks like a table-of-contents, index, or
    other non-prose content that would pollute embeddings.

    Heuristics (from rag-atc-testing findings):
      - >25% of characters are digits (page-number-heavy content)
      - >10% of characters are part of dot-leader patterns (TOC lines)
    """
    if not text:
        return True
    length = len(text)
    digit_count = sum(c.isdigit() for c in text)
    if digit_count / length > digit_threshold:
        return True
    dot_leader_chars = sum(len(m.group()) for m in _DOT_LEADER_RE.finditer(text))
    if dot_leader_chars / length > dot_leader_threshold:
        return True
    return False


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
    all_chunks = chunk_text(text, settings.CHUNK_SIZE, settings.CHUNK_OVERLAP)

    # Filter junk chunks (TOC, indexes, page-number-heavy content)
    chunks_text = [c for c in all_chunks if not is_junk_chunk(c)]
    junk_count = len(all_chunks) - len(chunks_text)
    if junk_count:
        logger.info(
            "Filtered %d/%d junk chunks from %s",
            junk_count, len(all_chunks), filepath.name,
        )

    if not chunks_text:
        logger.warning("No usable chunks after filtering for %s", filepath.name)
        chunks_text = all_chunks[:1] if all_chunks else [text.strip()[:500]]

    # Embed
    embeddings = await embedding_service.embed_batch(chunks_text)

    # Store — batch commit every INGEST_BATCH_SIZE chunks for resume support
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

    batch_size = settings.INGEST_BATCH_SIZE
    for idx, (chunk_content, embedding) in enumerate(zip(chunks_text, embeddings)):
        chunk = Chunk(
            document_id=doc.id,
            content=chunk_content,
            chunk_index=idx,
            embedding=embedding,
        )
        session.add(chunk)

        # Commit in batches to survive interruptions
        if (idx + 1) % batch_size == 0:
            await session.commit()
            logger.debug("Batch commit at chunk %d/%d", idx + 1, len(chunks_text))

    await session.commit()
    await session.refresh(doc)
    logger.info("Ingested %s (%d chunks, %d junk filtered)", filepath.name, len(chunks_text), junk_count)
    return doc


async def _existing_hashes(session: AsyncSession) -> set[str]:
    """Return content hashes already in the database (for resume support)."""
    result = await session.execute(select(Document.content_hash))
    return {row[0] for row in result.fetchall()}


async def ingest_directory(
    dirpath: str | Path,
    session: AsyncSession,
    embedding_service: EmbeddingService,
) -> list[Document]:
    dirpath = Path(dirpath)
    if not dirpath.is_dir():
        raise FileNotFoundError(f"Directory not found: {dirpath}")

    supported = HandlerRegistry.supported_extensions()
    known_hashes = await _existing_hashes(session)
    documents: list[Document] = []
    skipped = 0

    for entry in sorted(dirpath.rglob("*")):
        if not (entry.is_file() and entry.suffix.lower() in supported):
            continue

        # Resume support: skip files whose content is already ingested
        try:
            content_hash = hashlib.sha256(entry.read_bytes()).hexdigest()
        except OSError:
            logger.warning("Cannot read %s, skipping", entry)
            continue

        if content_hash in known_hashes:
            skipped += 1
            logger.debug("Skipping already-ingested %s", entry.name)
            continue

        try:
            doc = await ingest_file(entry, session, embedding_service)
            documents.append(doc)
            known_hashes.add(content_hash)
        except Exception:
            logger.exception("Failed to ingest %s", entry)

    if skipped:
        logger.info("Skipped %d already-ingested files", skipped)
    return documents
