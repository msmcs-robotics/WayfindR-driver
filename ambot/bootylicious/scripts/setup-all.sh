#!/bin/bash
# =============================================================================
# Jetson Orin Nano - Complete Setup Script
# =============================================================================
# Master script to set up the Jetson Orin Nano for the Ambot project.
# Runs Docker, Ollama (optional), and HuggingFace setup scripts.
#
# Usage:
#   chmod +x setup-all.sh
#   ./setup-all.sh
#
# This script is interactive and will prompt for each component.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_step() { echo -e "${BLUE}[====]${NC} $1"; }

# =============================================================================
# System info
# =============================================================================
print_system_info() {
    echo ""
    echo "========================================"
    echo "System Information"
    echo "========================================"
    echo "Hostname:     $(hostname)"
    echo "User:         $(whoami)"
    echo "Architecture: $(uname -m)"
    echo "Kernel:       $(uname -r)"
    echo "OS:           $(lsb_release -d 2>/dev/null | cut -f2 || cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"

    if [ -f /etc/nv_tegra_release ]; then
        echo "JetPack:      $(head -1 /etc/nv_tegra_release)"
    fi

    echo ""
    echo "Memory:"
    free -h | head -2

    echo ""
    echo "Disk:"
    df -h / | tail -1

    echo "========================================"
    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "Ambot - Jetson Orin Nano Complete Setup"
    echo "========================================"
    echo ""
    echo "This script will guide you through setting up:"
    echo "  1. Docker & Docker Compose"
    echo "  2. Ollama (optional - for LLM inference)"
    echo "  3. HuggingFace Transformers (recommended for Jetson)"
    echo "  4. RAG System deployment"
    echo ""

    print_system_info

    read -p "Continue with setup? (Y/n): " continue_setup
    if [[ "$continue_setup" =~ ^[Nn]$ ]]; then
        log_info "Setup cancelled."
        exit 0
    fi

    # Step 1: Docker
    echo ""
    log_step "STEP 1: Docker Setup"
    echo ""
    read -p "Install/configure Docker? (Y/n): " do_docker
    if [[ ! "$do_docker" =~ ^[Nn]$ ]]; then
        chmod +x "$SCRIPT_DIR/setup-docker.sh"
        "$SCRIPT_DIR/setup-docker.sh"
    else
        log_info "Skipping Docker setup"
    fi

    # Step 2: Ollama (optional)
    echo ""
    log_step "STEP 2: Ollama Setup (Optional)"
    echo ""
    log_warn "Ollama is optional. HuggingFace is recommended for Jetson."
    read -p "Install Ollama? (y/N): " do_ollama
    if [[ "$do_ollama" =~ ^[Yy]$ ]]; then
        chmod +x "$SCRIPT_DIR/setup-ollama.sh"
        "$SCRIPT_DIR/setup-ollama.sh"
    else
        log_info "Skipping Ollama setup"
    fi

    # Step 3: HuggingFace
    echo ""
    log_step "STEP 3: HuggingFace Setup"
    echo ""
    read -p "Install HuggingFace Transformers? (Y/n): " do_hf
    if [[ ! "$do_hf" =~ ^[Nn]$ ]]; then
        chmod +x "$SCRIPT_DIR/setup-huggingface.sh"
        "$SCRIPT_DIR/setup-huggingface.sh"
    else
        log_info "Skipping HuggingFace setup"
    fi

    # Step 4: RAG System
    echo ""
    log_step "STEP 4: RAG System Deployment"
    echo ""
    if command -v docker &> /dev/null && docker compose version &> /dev/null 2>&1; then
        RAG_DIR="$SCRIPT_DIR/../rag"
        if [ -d "$RAG_DIR" ]; then
            read -p "Deploy RAG system with Docker Compose? (Y/n): " do_rag
            if [[ ! "$do_rag" =~ ^[Nn]$ ]]; then
                cd "$RAG_DIR"
                if [ -f "deploy.sh" ]; then
                    chmod +x deploy.sh
                    ./deploy.sh start
                else
                    docker compose up -d
                fi
                log_info "RAG system deployed!"
            fi
        else
            log_warn "RAG system not found at $RAG_DIR"
            log_warn "Run rsync from development machine first."
        fi
    else
        log_warn "Docker not available. Skipping RAG deployment."
    fi

    # Summary
    echo ""
    echo "========================================"
    echo "Setup Complete!"
    echo "========================================"
    echo ""
    echo "Installed components:"
    echo "  Docker:     $(docker --version 2>/dev/null || echo 'Not installed')"
    echo "  Compose:    $(docker compose version 2>/dev/null || echo 'Not installed')"
    echo "  Ollama:     $(ollama --version 2>/dev/null || echo 'Not installed')"
    echo "  Python:     $(python3 --version 2>/dev/null || echo 'Not installed')"
    echo ""
    echo "Next steps:"
    echo "  1. If Docker was installed, log out and back in (or run: newgrp docker)"
    echo "  2. Copy your EECS documentation to: ~/ambot/knowledge/"
    echo "  3. Ingest documents: curl -X POST http://localhost:8000/api/ingest/directory -d '{\"path\": \"/data/knowledge\"}'"
    echo "  4. Test RAG: curl http://localhost:8000/api/health"
    echo ""
    echo "========================================"
}

main "$@"
