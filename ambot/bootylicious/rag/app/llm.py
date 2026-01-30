"""
Ambot RAG System - LLM Client
Supports both Ollama and HuggingFace backends for text generation.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import httpx

from .config import settings

logger = logging.getLogger(__name__)


@dataclass
class LLMResponse:
    """Response from LLM generation."""
    answer: str
    model: str
    context_chunks: list[dict]
    done: bool = True


class BaseLLMClient(ABC):
    """Abstract base class for LLM clients."""

    @abstractmethod
    async def generate(
        self,
        prompt: str,
        system: str | None = None,
        temperature: float | None = None,
    ) -> str:
        """Generate a response from the LLM."""
        pass

    @abstractmethod
    async def health_check(self) -> bool:
        """Check if the LLM backend is available."""
        pass

    @abstractmethod
    def get_model_name(self) -> str:
        """Get the name of the current model."""
        pass

    async def ask_with_context(
        self,
        question: str,
        context_chunks: list[dict],
        system_prompt: str | None = None,
    ) -> LLMResponse:
        """Answer a question using retrieved RAG context chunks."""
        # Format context for injection
        context_parts = []
        for i, chunk in enumerate(context_chunks, 1):
            source = chunk.get("document_filename", "unknown")
            content = chunk.get("content", "")
            context_parts.append(f"[Source {i}: {source}]\n{content}")

        context_str = "\n\n---\n\n".join(context_parts)

        default_system = (
            f"You are a knowledgeable assistant for '{settings.PROJECT_NAME}'. "
            "You help answer questions about EECS (Electrical Engineering and Computer Science) "
            "at Embry-Riddle Aeronautical University. "
            "Answer questions accurately using ONLY the provided context. "
            "If the context does not contain enough information to answer, say so clearly. "
            "Cite sources by their source number when possible."
        )

        user_prompt = f"""## Retrieved Context

{context_str}

---

## Question

{question}"""

        answer = await self.generate(
            prompt=user_prompt,
            system=system_prompt or default_system,
        )

        return LLMResponse(
            answer=answer,
            model=self.get_model_name(),
            context_chunks=context_chunks,
        )


class OllamaClient(BaseLLMClient):
    """Async client for Ollama LLM API."""

    def __init__(
        self,
        base_url: str | None = None,
        model: str | None = None,
        timeout: int = 300,
        temperature: float = 0.3,
    ):
        self.base_url = (base_url or settings.OLLAMA_BASE_URL).rstrip("/")
        self.model = model or settings.LLM_MODEL
        self.timeout = timeout
        self.temperature = temperature

    def get_model_name(self) -> str:
        return self.model

    async def generate(
        self,
        prompt: str,
        system: str | None = None,
        temperature: float | None = None,
    ) -> str:
        """Generate a response from Ollama."""
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": temperature if temperature is not None else self.temperature,
            },
        }
        if system:
            payload["system"] = system

        async with httpx.AsyncClient(timeout=self.timeout) as client:
            resp = await client.post(f"{self.base_url}/api/generate", json=payload)
            resp.raise_for_status()
            data = resp.json()
            return data.get("response", "")

    async def health_check(self) -> bool:
        """Check if Ollama server is reachable and model exists."""
        try:
            async with httpx.AsyncClient(timeout=10) as client:
                resp = await client.get(f"{self.base_url}/api/tags")
                resp.raise_for_status()
                models = resp.json().get("models", [])
                model_names = [m.get("name", "") for m in models]
                return any(self.model in name or name in self.model for name in model_names)
        except Exception:
            return False

    async def list_models(self) -> list[str]:
        """List available models on the Ollama server."""
        try:
            async with httpx.AsyncClient(timeout=10) as client:
                resp = await client.get(f"{self.base_url}/api/tags")
                resp.raise_for_status()
                models = resp.json().get("models", [])
                return [m.get("name", "") for m in models]
        except Exception:
            return []


class HuggingFaceClient(BaseLLMClient):
    """
    Local HuggingFace Transformers client for LLM inference.

    Designed for Jetson Orin Nano with GPU acceleration.
    Uses lazy loading to avoid memory usage until first inference.
    """

    def __init__(
        self,
        model_name: str | None = None,
        device: str | None = None,
        torch_dtype: str | None = None,
        max_new_tokens: int | None = None,
        temperature: float = 0.3,
    ):
        self.model_name = model_name or settings.HF_MODEL
        self.device = device or settings.HF_DEVICE
        self.torch_dtype_str = torch_dtype or settings.HF_TORCH_DTYPE
        self.max_new_tokens = max_new_tokens or settings.HF_MAX_NEW_TOKENS
        self.temperature = temperature

        # Lazy-loaded model and tokenizer
        self._model = None
        self._tokenizer = None
        self._pipeline = None

    def get_model_name(self) -> str:
        return self.model_name

    def _get_torch_dtype(self):
        """Convert string dtype to torch dtype."""
        import torch
        dtype_map = {
            "float16": torch.float16,
            "float32": torch.float32,
            "bfloat16": torch.bfloat16,
        }
        return dtype_map.get(self.torch_dtype_str, torch.float16)

    def _load_model(self):
        """Lazy-load the model and tokenizer."""
        if self._pipeline is not None:
            return

        logger.info(f"Loading HuggingFace model: {self.model_name}")

        try:
            import torch
            from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline

            # Determine device
            if self.device == "auto":
                device_map = "auto"
                device = None
            elif self.device == "cuda" and torch.cuda.is_available():
                device_map = None
                device = "cuda"
            else:
                device_map = None
                device = "cpu"

            # Load tokenizer
            self._tokenizer = AutoTokenizer.from_pretrained(self.model_name)

            # Load model with appropriate settings for Jetson
            self._model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype=self._get_torch_dtype(),
                device_map=device_map,
                low_cpu_mem_usage=True,  # Important for constrained memory
            )

            if device and device_map is None:
                self._model = self._model.to(device)

            # Create pipeline for easier inference
            self._pipeline = pipeline(
                "text-generation",
                model=self._model,
                tokenizer=self._tokenizer,
                device=0 if device == "cuda" else -1,
            )

            logger.info(f"Model loaded successfully on {device or 'auto'}")

        except Exception as e:
            logger.error(f"Failed to load HuggingFace model: {e}")
            raise

    async def generate(
        self,
        prompt: str,
        system: str | None = None,
        temperature: float | None = None,
    ) -> str:
        """Generate a response using HuggingFace transformers."""
        import asyncio

        # Load model if not already loaded
        self._load_model()

        # Format prompt with system message (TinyLlama chat format)
        if system:
            full_prompt = f"<|system|>\n{system}</s>\n<|user|>\n{prompt}</s>\n<|assistant|>\n"
        else:
            full_prompt = f"<|user|>\n{prompt}</s>\n<|assistant|>\n"

        # Run generation in thread pool to not block async
        def _generate():
            result = self._pipeline(
                full_prompt,
                max_new_tokens=self.max_new_tokens,
                temperature=temperature or self.temperature,
                do_sample=settings.HF_DO_SAMPLE,
                top_p=settings.HF_TOP_P,
                pad_token_id=self._tokenizer.eos_token_id,
                return_full_text=False,
            )
            return result[0]["generated_text"]

        loop = asyncio.get_event_loop()
        response = await loop.run_in_executor(None, _generate)

        return response.strip()

    async def health_check(self) -> bool:
        """Check if HuggingFace backend is available."""
        try:
            import torch
            from transformers import AutoTokenizer

            # Check if CUDA is available (for Jetson GPU)
            cuda_available = torch.cuda.is_available()
            logger.info(f"CUDA available: {cuda_available}")

            # Try to load just the tokenizer as a quick check
            AutoTokenizer.from_pretrained(self.model_name)

            return True
        except Exception as e:
            logger.error(f"HuggingFace health check failed: {e}")
            return False


def create_llm_client() -> BaseLLMClient:
    """
    Factory function to create the appropriate LLM client based on configuration.

    Returns:
        BaseLLMClient: Either OllamaClient or HuggingFaceClient
    """
    if settings.is_huggingface_backend:
        logger.info("Using HuggingFace backend for LLM")
        return HuggingFaceClient()
    else:
        logger.info("Using Ollama backend for LLM")
        return OllamaClient()


# Default client instance (can be overridden)
def get_default_client() -> BaseLLMClient:
    """Get the default LLM client based on configuration."""
    return create_llm_client()
