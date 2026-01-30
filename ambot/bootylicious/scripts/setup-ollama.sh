#!/bin/bash
# =============================================================================
# Jetson Orin Nano - Ollama Setup (Optional)
# =============================================================================
# Installs Ollama for LLM inference. This is OPTIONAL - the RAG system
# can also use HuggingFace transformers directly (recommended for Jetson).
#
# Usage:
#   chmod +x setup-ollama.sh
#   ./setup-ollama.sh
#
# Note: Ollama may have limited ARM64/Jetson support. For production on
# Jetson, consider using HuggingFace transformers with TensorRT instead.
# =============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# =============================================================================
# Check architecture
# =============================================================================
check_architecture() {
    ARCH=$(uname -m)
    log_info "Architecture: $ARCH"

    if [[ "$ARCH" != "aarch64" ]]; then
        log_warn "This script is designed for ARM64 (aarch64) Jetson systems"
        log_warn "Current architecture: $ARCH"
    fi
}

# =============================================================================
# Install Ollama via official script
# =============================================================================
install_ollama_official() {
    log_info "Installing Ollama via official install script..."

    # Check if already installed
    if command -v ollama &> /dev/null; then
        log_warn "Ollama is already installed: $(ollama --version 2>/dev/null || echo 'version unknown')"
        read -p "Reinstall? (y/N): " reinstall
        if [[ ! "$reinstall" =~ ^[Yy]$ ]]; then
            return 0
        fi
    fi

    # Official install script
    curl -fsSL https://ollama.com/install.sh | sh

    log_info "Ollama installed"
}

# =============================================================================
# Install Ollama manually (alternative method)
# =============================================================================
install_ollama_manual() {
    log_info "Installing Ollama manually..."

    # This is an alternative if the official script doesn't work on Jetson

    OLLAMA_VERSION="0.5.4"  # Update as needed
    DOWNLOAD_URL="https://github.com/ollama/ollama/releases/download/v${OLLAMA_VERSION}/ollama-linux-arm64.tgz"

    log_info "Downloading Ollama v${OLLAMA_VERSION}..."

    cd /tmp
    curl -L -o ollama.tgz "$DOWNLOAD_URL"

    log_info "Extracting..."
    sudo tar -xzf ollama.tgz -C /usr/local/bin

    # Make executable
    sudo chmod +x /usr/local/bin/ollama

    rm ollama.tgz

    log_info "Ollama installed to /usr/local/bin/ollama"
}

# =============================================================================
# Create systemd service
# =============================================================================
setup_ollama_service() {
    log_info "Setting up Ollama systemd service..."

    # Create service file
    sudo tee /etc/systemd/system/ollama.service > /dev/null <<EOF
[Unit]
Description=Ollama LLM Service
After=network-online.target

[Service]
Type=simple
User=$USER
Environment="OLLAMA_HOST=0.0.0.0"
Environment="OLLAMA_MODELS=/home/$USER/.ollama/models"
ExecStart=/usr/local/bin/ollama serve
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

    # Reload systemd and enable service
    sudo systemctl daemon-reload
    sudo systemctl enable ollama
    sudo systemctl start ollama

    log_info "Ollama service enabled and started"
}

# =============================================================================
# Pull recommended models
# =============================================================================
pull_models() {
    echo ""
    log_info "Recommended models for Jetson Orin Nano (8GB RAM):"
    echo "  - tinyllama (1.1B) - ~600MB, fastest"
    echo "  - phi (2.7B) - ~1.7GB, good balance"
    echo "  - gemma:2b - ~1.5GB, Google's small model"
    echo "  - qwen2:1.5b - ~950MB, multilingual"
    echo ""

    read -p "Pull tinyllama now? (Y/n): " pull_tiny
    if [[ ! "$pull_tiny" =~ ^[Nn]$ ]]; then
        log_info "Pulling tinyllama..."
        ollama pull tinyllama
    fi

    read -p "Pull phi (2.7B)? (y/N): " pull_phi
    if [[ "$pull_phi" =~ ^[Yy]$ ]]; then
        log_info "Pulling phi..."
        ollama pull phi
    fi
}

# =============================================================================
# Verify installation
# =============================================================================
verify_installation() {
    log_info "Verifying Ollama installation..."

    echo ""
    echo "========================================"
    echo "Ollama Installation Summary"
    echo "========================================"
    echo "Ollama:    $(ollama --version 2>/dev/null || echo 'NOT INSTALLED')"
    echo "Service:   $(systemctl is-active ollama 2>/dev/null || echo 'NOT RUNNING')"
    echo "API:       http://localhost:11434"
    echo "========================================"
    echo ""

    # Test API
    log_info "Testing Ollama API..."
    sleep 2  # Give service time to start

    if curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
        log_info "Ollama API is responding!"
        echo "Available models:"
        curl -s http://localhost:11434/api/tags | python3 -c "
import sys, json
data = json.load(sys.stdin)
for m in data.get('models', []):
    print(f\"  - {m['name']} ({m.get('size', 'unknown size')})\")" 2>/dev/null || echo "  (none yet)"
    else
        log_warn "Ollama API not responding yet. May need a moment to start."
        log_warn "Check with: curl http://localhost:11434/api/tags"
    fi
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "Jetson Orin Nano - Ollama Setup"
    echo "========================================"
    echo ""
    log_warn "NOTE: For Jetson production, HuggingFace + TensorRT may be better."
    log_warn "Ollama is useful for development and testing."
    echo ""

    check_architecture

    echo ""
    echo "Installation method:"
    echo "  1) Official install script (recommended)"
    echo "  2) Manual download (if official fails)"
    echo "  3) Skip installation"
    echo ""
    read -p "Choose (1/2/3): " method

    case $method in
        1)
            install_ollama_official
            ;;
        2)
            install_ollama_manual
            setup_ollama_service
            ;;
        3)
            log_info "Skipping Ollama installation"
            exit 0
            ;;
        *)
            log_error "Invalid choice"
            exit 1
            ;;
    esac

    verify_installation
    pull_models

    echo ""
    log_info "Ollama setup complete!"
    echo ""
    echo "Usage:"
    echo "  ollama run tinyllama     # Interactive chat"
    echo "  ollama list              # List models"
    echo "  ollama pull <model>      # Download model"
    echo "  curl http://localhost:11434/api/generate -d '{...}'  # API"
    echo ""
}

main "$@"
