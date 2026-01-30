#!/bin/bash
# =============================================================================
# Jetson Orin Nano - HuggingFace Transformers Setup
# =============================================================================
# Installs HuggingFace transformers with CUDA support for Jetson.
# This is the RECOMMENDED approach for running LLMs on Jetson Orin Nano
# as it leverages the NVIDIA GPU with TensorRT optimizations.
#
# Usage:
#   chmod +x setup-huggingface.sh
#   ./setup-huggingface.sh
#
# This script can be run with or without sudo:
#   - With sudo: System-wide installation
#   - Without sudo: User-local installation (recommended)
# =============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[STEP]${NC} $1"; }

# =============================================================================
# Check prerequisites
# =============================================================================
check_prerequisites() {
    log_step "Checking prerequisites..."

    # Check architecture
    ARCH=$(uname -m)
    if [[ "$ARCH" != "aarch64" ]]; then
        log_warn "Expected aarch64 (ARM64), got $ARCH"
    fi
    log_info "Architecture: $ARCH"

    # Check Python
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version)
        log_info "Python: $PYTHON_VERSION"
    else
        log_error "Python3 is required but not installed"
        exit 1
    fi

    # Check pip
    if command -v pip3 &> /dev/null; then
        log_info "pip3: $(pip3 --version | head -1)"
    else
        log_warn "pip3 not found, will install"
    fi

    # Check CUDA
    if command -v nvcc &> /dev/null; then
        CUDA_VERSION=$(nvcc --version | grep release | awk '{print $6}')
        log_info "CUDA: $CUDA_VERSION"
    else
        log_warn "nvcc not found - CUDA may not be properly configured"
    fi

    # Check for JetPack
    if [ -f /etc/nv_tegra_release ]; then
        log_info "JetPack detected: $(cat /etc/nv_tegra_release | head -1)"
    fi
}

# =============================================================================
# Install system dependencies
# =============================================================================
install_system_deps() {
    log_step "Installing system dependencies..."

    sudo apt update
    sudo apt install -y \
        python3-pip \
        python3-venv \
        python3-dev \
        build-essential \
        cmake \
        git \
        libopenblas-dev \
        liblapack-dev \
        libjpeg-dev \
        zlib1g-dev \
        libpng-dev

    log_info "System dependencies installed"
}

# =============================================================================
# Create virtual environment (optional but recommended)
# =============================================================================
setup_venv() {
    log_step "Setting up Python virtual environment..."

    VENV_DIR="${HOME}/.venvs/ambot-llm"

    if [ -d "$VENV_DIR" ]; then
        log_warn "Virtual environment already exists at $VENV_DIR"
        read -p "Recreate? (y/N): " recreate
        if [[ "$recreate" =~ ^[Yy]$ ]]; then
            rm -rf "$VENV_DIR"
        else
            log_info "Using existing virtual environment"
            source "$VENV_DIR/bin/activate"
            return 0
        fi
    fi

    python3 -m venv "$VENV_DIR"
    source "$VENV_DIR/bin/activate"

    # Upgrade pip
    pip install --upgrade pip wheel setuptools

    log_info "Virtual environment created at $VENV_DIR"
    echo ""
    echo "To activate in the future:"
    echo "  source $VENV_DIR/bin/activate"
    echo ""
}

# =============================================================================
# Install PyTorch for Jetson
# =============================================================================
install_pytorch_jetson() {
    log_step "Installing PyTorch for Jetson..."

    # NVIDIA provides pre-built PyTorch wheels for Jetson
    # Check https://forums.developer.nvidia.com/t/pytorch-for-jetson/

    # For JetPack 6.x (L4T R36.x), use these wheels
    # Adjust version as needed based on your JetPack version

    echo ""
    echo "PyTorch installation options for Jetson:"
    echo "  1) Install from NVIDIA Jetson wheels (recommended)"
    echo "  2) Install from pip (may not have full CUDA support)"
    echo "  3) Skip PyTorch (if already installed)"
    echo ""
    read -p "Choose (1/2/3): " choice

    case $choice in
        1)
            log_info "Installing PyTorch from NVIDIA Jetson wheels..."

            # JetPack 6.0+ / L4T R36.x wheels
            # These URLs may change - check NVIDIA forums for latest
            PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.3.0a0+ebedce2.nv24.02-cp310-cp310-linux_aarch64.whl"

            # Try the wheel, fall back to pip if it fails
            pip install "$PYTORCH_URL" 2>/dev/null || {
                log_warn "NVIDIA wheel failed, trying pip..."
                pip install torch torchvision torchaudio
            }
            ;;
        2)
            log_info "Installing PyTorch from pip..."
            pip install torch torchvision torchaudio
            ;;
        3)
            log_info "Skipping PyTorch installation"
            ;;
    esac

    # Verify PyTorch and CUDA
    log_info "Verifying PyTorch installation..."
    python3 -c "
import torch
print(f'PyTorch version: {torch.__version__}')
print(f'CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'CUDA version: {torch.version.cuda}')
    print(f'GPU: {torch.cuda.get_device_name(0)}')
" || log_warn "PyTorch verification failed"
}

# =============================================================================
# Install HuggingFace libraries
# =============================================================================
install_huggingface() {
    log_step "Installing HuggingFace libraries..."

    # Core transformers library
    pip install transformers

    # Accelerate for efficient inference
    pip install accelerate

    # Sentence transformers for embeddings
    pip install sentence-transformers

    # BitsAndBytes for quantization (4-bit/8-bit models)
    pip install bitsandbytes

    # Tokenizers (usually installed with transformers, but ensure it's there)
    pip install tokenizers

    # Datasets for loading datasets
    pip install datasets

    log_info "HuggingFace libraries installed"
}

# =============================================================================
# Install optimizations for Jetson
# =============================================================================
install_jetson_optimizations() {
    log_step "Installing Jetson-specific optimizations..."

    # These are optional but can improve performance

    echo ""
    echo "Optional optimizations:"
    echo "  1) Install all optimizations"
    echo "  2) Skip optimizations"
    echo ""
    read -p "Choose (1/2): " opt_choice

    if [[ "$opt_choice" == "1" ]]; then
        # ONNX Runtime for optimized inference
        pip install onnxruntime-gpu 2>/dev/null || {
            log_warn "onnxruntime-gpu failed, trying CPU version"
            pip install onnxruntime
        }

        # Optimum for model optimization
        pip install optimum

        log_info "Optimizations installed"
    else
        log_info "Skipping optimizations"
    fi
}

# =============================================================================
# Download recommended models
# =============================================================================
download_models() {
    log_step "Model download options..."

    echo ""
    echo "Recommended models for Jetson Orin Nano (8GB RAM):"
    echo ""
    echo "LLM Models (pick ONE):"
    echo "  - TinyLlama/TinyLlama-1.1B-Chat-v1.0 (~2.2GB) - Fastest"
    echo "  - microsoft/phi-2 (~5.4GB) - Best quality for size"
    echo "  - Qwen/Qwen2-1.5B (~3GB) - Multilingual"
    echo ""
    echo "Embedding Models:"
    echo "  - sentence-transformers/all-MiniLM-L6-v2 (~80MB) - Recommended"
    echo "  - sentence-transformers/all-mpnet-base-v2 (~420MB) - Higher quality"
    echo ""

    read -p "Download TinyLlama-1.1B now? (Y/n): " dl_llm
    if [[ ! "$dl_llm" =~ ^[Nn]$ ]]; then
        log_info "Downloading TinyLlama-1.1B (this may take a while)..."
        python3 -c "
from transformers import AutoModelForCausalLM, AutoTokenizer
model_name = 'TinyLlama/TinyLlama-1.1B-Chat-v1.0'
print(f'Downloading {model_name}...')
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModelForCausalLM.from_pretrained(model_name)
print('Download complete!')
"
    fi

    read -p "Download all-MiniLM-L6-v2 embedding model? (Y/n): " dl_embed
    if [[ ! "$dl_embed" =~ ^[Nn]$ ]]; then
        log_info "Downloading all-MiniLM-L6-v2..."
        python3 -c "
from sentence_transformers import SentenceTransformer
model = SentenceTransformer('all-MiniLM-L6-v2')
print('Embedding model downloaded!')
"
    fi
}

# =============================================================================
# Create test script
# =============================================================================
create_test_script() {
    log_step "Creating test script..."

    TEST_SCRIPT="${HOME}/test_huggingface.py"

    cat > "$TEST_SCRIPT" << 'EOF'
#!/usr/bin/env python3
"""
Test script for HuggingFace on Jetson Orin Nano
Run: python3 ~/test_huggingface.py
"""

import torch
print("=" * 50)
print("HuggingFace Jetson Test")
print("=" * 50)

# Check PyTorch and CUDA
print(f"\nPyTorch: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA version: {torch.version.cuda}")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")

# Test embedding model
print("\n--- Testing Embedding Model ---")
try:
    from sentence_transformers import SentenceTransformer
    model = SentenceTransformer('all-MiniLM-L6-v2')
    embedding = model.encode("Hello, Jetson!")
    print(f"Embedding shape: {embedding.shape}")
    print("Embedding model: OK")
except Exception as e:
    print(f"Embedding model: FAILED - {e}")

# Test LLM (if downloaded)
print("\n--- Testing LLM (TinyLlama) ---")
try:
    from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline

    model_name = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"

    # Check if model is cached
    from huggingface_hub import scan_cache_dir
    cache = scan_cache_dir()
    model_cached = any(model_name.split('/')[-1] in str(r) for r in cache.repos)

    if model_cached:
        tokenizer = AutoTokenizer.from_pretrained(model_name)

        # Load with appropriate device
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model = AutoModelForCausalLM.from_pretrained(
            model_name,
            torch_dtype=torch.float16 if device == "cuda" else torch.float32,
            device_map="auto" if device == "cuda" else None
        )

        # Simple test
        pipe = pipeline("text-generation", model=model, tokenizer=tokenizer, max_new_tokens=50)
        result = pipe("The capital of France is")
        print(f"LLM output: {result[0]['generated_text'][:100]}...")
        print("LLM: OK")
    else:
        print("LLM: SKIPPED (model not downloaded)")
except Exception as e:
    print(f"LLM: FAILED - {e}")

print("\n" + "=" * 50)
print("Test complete!")
print("=" * 50)
EOF

    chmod +x "$TEST_SCRIPT"
    log_info "Test script created at $TEST_SCRIPT"
}

# =============================================================================
# Summary
# =============================================================================
print_summary() {
    echo ""
    echo "========================================"
    echo "HuggingFace Setup Complete"
    echo "========================================"

    if [ -n "$VIRTUAL_ENV" ]; then
        echo "Virtual environment: $VIRTUAL_ENV"
        echo ""
        echo "To activate:"
        echo "  source $VIRTUAL_ENV/bin/activate"
    fi

    echo ""
    echo "Installed packages:"
    pip list | grep -E "torch|transformers|sentence|accelerate" 2>/dev/null || echo "  (check with: pip list)"

    echo ""
    echo "Test the installation:"
    echo "  python3 ~/test_huggingface.py"
    echo ""
    echo "Usage in Python:"
    echo "  from transformers import pipeline"
    echo "  from sentence_transformers import SentenceTransformer"
    echo ""
    echo "========================================"
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "Jetson Orin Nano - HuggingFace Setup"
    echo "========================================"
    echo ""

    check_prerequisites

    read -p "Install system dependencies (requires sudo)? (Y/n): " install_sys
    if [[ ! "$install_sys" =~ ^[Nn]$ ]]; then
        install_system_deps
    fi

    read -p "Create/use virtual environment? (Y/n): " use_venv
    if [[ ! "$use_venv" =~ ^[Nn]$ ]]; then
        setup_venv
    fi

    install_pytorch_jetson
    install_huggingface
    install_jetson_optimizations
    create_test_script
    download_models
    print_summary

    echo ""
    log_info "HuggingFace setup complete!"
    echo ""
}

main "$@"
