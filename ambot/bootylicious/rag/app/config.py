"""
Ambot RAG System - Configuration
Supports both Ollama and HuggingFace backends for LLM inference.
"""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application configuration loaded from environment variables."""

    # -------------------------------------------------------------------------
    # PostgreSQL
    # -------------------------------------------------------------------------
    POSTGRES_HOST: str = "localhost"
    POSTGRES_PORT: int = 5432
    POSTGRES_DB: str = "ambot_rag"
    POSTGRES_USER: str = "ambot"
    POSTGRES_PASSWORD: str = "ambot_secure_pass"

    # -------------------------------------------------------------------------
    # Redis
    # -------------------------------------------------------------------------
    REDIS_URL: str = "redis://localhost:6379/0"

    # -------------------------------------------------------------------------
    # Embedding
    # -------------------------------------------------------------------------
    EMBEDDING_MODEL: str = "all-MiniLM-L6-v2"
    EMBEDDING_DIMENSION: int = 384
    EMBEDDING_BACKEND: str = "sentence-transformers"  # or "ollama"

    # -------------------------------------------------------------------------
    # Chunking
    # -------------------------------------------------------------------------
    CHUNK_SIZE: int = 512
    CHUNK_OVERLAP: int = 50

    # -------------------------------------------------------------------------
    # LLM Backend Selection
    # -------------------------------------------------------------------------
    # Options: "ollama" or "huggingface"
    LLM_BACKEND: str = "ollama"

    # -------------------------------------------------------------------------
    # Ollama Configuration (when LLM_BACKEND = "ollama")
    # -------------------------------------------------------------------------
    OLLAMA_BASE_URL: str = "http://localhost:11434"
    LLM_MODEL: str = "tinyllama"
    LLM_TEMPERATURE: float = 0.3
    LLM_TIMEOUT: int = 300

    # -------------------------------------------------------------------------
    # HuggingFace Configuration (when LLM_BACKEND = "huggingface")
    # -------------------------------------------------------------------------
    HF_MODEL: str = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"
    HF_DEVICE: str = "cuda"  # "cuda", "cpu", or "auto"
    HF_TORCH_DTYPE: str = "float16"  # "float16", "float32", "bfloat16"
    HF_MAX_NEW_TOKENS: int = 512
    HF_DO_SAMPLE: bool = True
    HF_TOP_P: float = 0.9

    # -------------------------------------------------------------------------
    # RAG Retrieval
    # -------------------------------------------------------------------------
    RAG_TOP_K: int = 3  # Reduced for resource efficiency
    RAG_MIN_SIMILARITY: float = 0.75

    # -------------------------------------------------------------------------
    # Project
    # -------------------------------------------------------------------------
    PROJECT_NAME: str = "ambot-eecs"

    @property
    def DATABASE_URL(self) -> str:
        return (
            f"postgresql+asyncpg://{self.POSTGRES_USER}:{self.POSTGRES_PASSWORD}"
            f"@{self.POSTGRES_HOST}:{self.POSTGRES_PORT}/{self.POSTGRES_DB}"
        )

    @property
    def is_ollama_backend(self) -> bool:
        return self.LLM_BACKEND.lower() == "ollama"

    @property
    def is_huggingface_backend(self) -> bool:
        return self.LLM_BACKEND.lower() == "huggingface"

    model_config = {"env_file": ".env", "env_file_encoding": "utf-8"}


settings = Settings()
