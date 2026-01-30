from __future__ import annotations

import hashlib
import json
import logging
from typing import TYPE_CHECKING

import httpx
import redis.asyncio as aioredis

from .config import settings

if TYPE_CHECKING:
    from sentence_transformers import SentenceTransformer

logger = logging.getLogger(__name__)

_CACHE_TTL = 60 * 60 * 24 * 30  # 30 days


class EmbeddingService:
    """Generates embeddings via sentence-transformers (local) or Ollama."""

    def __init__(
        self,
        backend: str = settings.EMBEDDING_BACKEND,
        model_name: str = settings.EMBEDDING_MODEL,
        ollama_base_url: str = settings.OLLAMA_BASE_URL,
        redis_client: aioredis.Redis | None = None,
    ) -> None:
        self.backend = backend
        self.model_name = model_name
        self.ollama_base_url = ollama_base_url.rstrip("/")
        self.redis: aioredis.Redis | None = redis_client
        self._local_model: SentenceTransformer | None = None

    # -- lazy loading for sentence-transformers ---------------------------------

    def _get_local_model(self) -> SentenceTransformer:
        if self._local_model is None:
            from sentence_transformers import SentenceTransformer

            logger.info("Loading sentence-transformers model: %s", self.model_name)
            self._local_model = SentenceTransformer(self.model_name)
        return self._local_model

    # -- cache helpers ----------------------------------------------------------

    @staticmethod
    def _cache_key(text: str) -> str:
        h = hashlib.sha256(text.encode()).hexdigest()
        return f"emb:{h}"

    async def _cache_get(self, text: str) -> list[float] | None:
        if self.redis is None:
            return None
        raw = await self.redis.get(self._cache_key(text))
        if raw is not None:
            return json.loads(raw)
        return None

    async def _cache_set(self, text: str, vector: list[float]) -> None:
        if self.redis is None:
            return
        await self.redis.set(self._cache_key(text), json.dumps(vector), ex=_CACHE_TTL)

    # -- public API -------------------------------------------------------------

    async def embed_text(self, text: str) -> list[float]:
        cached = await self._cache_get(text)
        if cached is not None:
            return cached

        if self.backend == "ollama":
            vector = await self._embed_ollama(text)
        else:
            vector = self._embed_local(text)

        await self._cache_set(text, vector)
        return vector

    async def embed_batch(self, texts: list[str]) -> list[list[float]]:
        results: list[list[float] | None] = [None] * len(texts)
        uncached_indices: list[int] = []
        uncached_texts: list[str] = []

        # Check cache first
        for i, t in enumerate(texts):
            cached = await self._cache_get(t)
            if cached is not None:
                results[i] = cached
            else:
                uncached_indices.append(i)
                uncached_texts.append(t)

        if uncached_texts:
            if self.backend == "ollama":
                vectors = [await self._embed_ollama(t) for t in uncached_texts]
            else:
                vectors = self._embed_local_batch(uncached_texts)

            for idx, text, vec in zip(uncached_indices, uncached_texts, vectors):
                results[idx] = vec
                await self._cache_set(text, vec)

        return results  # type: ignore[return-value]

    # -- backends ---------------------------------------------------------------

    def _embed_local(self, text: str) -> list[float]:
        model = self._get_local_model()
        return model.encode(text, normalize_embeddings=True).tolist()

    def _embed_local_batch(self, texts: list[str]) -> list[list[float]]:
        model = self._get_local_model()
        embeddings = model.encode(texts, normalize_embeddings=True, batch_size=64)
        return [e.tolist() for e in embeddings]

    async def _embed_ollama(self, text: str) -> list[float]:
        url = f"{self.ollama_base_url}/api/embeddings"
        payload = {"model": self.model_name, "prompt": text}
        async with httpx.AsyncClient(timeout=60.0) as client:
            resp = await client.post(url, json=payload)
            resp.raise_for_status()
            return resp.json()["embedding"]

    # -- health -----------------------------------------------------------------

    async def health_check(self) -> bool:
        try:
            await self.embed_text("health check")
            return True
        except Exception:
            logger.exception("Embedding service health check failed")
            return False
