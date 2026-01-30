# =============================================================================
# Ambot RAG System - Dockerfile (Jetson Orin Nano Optimized)
# =============================================================================
# Supports both Ollama and HuggingFace LLM backends
# For Jetson with GPU: use NVIDIA L4T base image
# For CPU-only or Ollama backend: python:3.11-slim works fine
# =============================================================================

ARG BASE_IMAGE=python:3.11-slim
FROM ${BASE_IMAGE}

WORKDIR /src

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        curl \
        git \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
# Note: torch may need to be installed separately for Jetson
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . ./app/

EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:8000/api/health || exit 1

# Run with uvicorn
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
