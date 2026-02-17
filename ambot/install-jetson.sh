#!/bin/bash
# =============================================================================
# Ambot - Jetson Orin Nano Install Script
# =============================================================================
# Comprehensive, idempotent, non-interactive setup for Jetson.
# Safe to run multiple times. Designed to survive SSH drops via nohup.
#
# Usage (on Jetson):
#   sudo ./install-jetson.sh              # Full install (all components)
#   sudo ./install-jetson.sh --check      # Check what's installed
#   sudo ./install-jetson.sh --docker     # Docker only
#   sudo ./install-jetson.sh --ollama     # Ollama only
#   sudo ./install-jetson.sh --pip        # Python pip only
#   sudo ./install-jetson.sh --model      # Pull LLM model only
#   sudo ./install-jetson.sh --rag        # Start RAG Docker stack only
#   sudo ./install-jetson.sh --cuda       # CUDA verification & PATH fix
#   sudo ./install-jetson.sh --inventory  # System inventory (no installs)
#
# Background-safe (survives SSH drops):
#   nohup sudo ./install-jetson.sh > install.log 2>&1 &
#   tail -f install.log
#
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="${SCRIPT_DIR}/install-jetson.log"

# Colors (disabled if not a terminal)
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    CYAN='\033[0;36m'
    NC='\033[0m'
else
    RED='' GREEN='' YELLOW='' BLUE='' CYAN='' NC=''
fi

log_info()    { echo -e "${GREEN}[INFO]${NC} $(date '+%H:%M:%S') $1" | tee -a "$LOG_FILE"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $(date '+%H:%M:%S') $1" | tee -a "$LOG_FILE"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $(date '+%H:%M:%S') $1" | tee -a "$LOG_FILE"; }
log_step()    { echo -e "${BLUE}[====]${NC} $(date '+%H:%M:%S') $1" | tee -a "$LOG_FILE"; }
log_success() { echo -e "${GREEN}[OK]${NC} $(date '+%H:%M:%S') $1" | tee -a "$LOG_FILE"; }
log_fail()    { echo -e "${RED}[FAIL]${NC} $(date '+%H:%M:%S') $1" | tee -a "$LOG_FILE"; }

# Track results for final summary
declare -a RESULTS=()
result_ok()   { RESULTS+=("OK: $1"); }
result_skip() { RESULTS+=("SKIP: $1"); }
result_fail() { RESULTS+=("FAIL: $1"); }

# =============================================================================
# System Inventory (no installs)
# =============================================================================
system_inventory() {
    log_step "System Inventory"

    echo "" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    echo "  Jetson Orin Nano System Inventory" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"

    echo "Hostname:     $(hostname)" | tee -a "$LOG_FILE"
    echo "User:         $(whoami)" | tee -a "$LOG_FILE"
    echo "Model:        $(cat /proc/device-tree/model 2>/dev/null || echo 'unknown')" | tee -a "$LOG_FILE"
    echo "OS:           $(lsb_release -ds 2>/dev/null || cat /etc/os-release | grep PRETTY_NAME | cut -d= -f2 | tr -d '\"')" | tee -a "$LOG_FILE"
    echo "Kernel:       $(uname -r)" | tee -a "$LOG_FILE"
    echo "Arch:         $(uname -m)" | tee -a "$LOG_FILE"

    # JetPack
    if [ -f /etc/nv_tegra_release ]; then
        echo "JetPack:      $(head -1 /etc/nv_tegra_release)" | tee -a "$LOG_FILE"
    fi

    # Memory
    echo "" | tee -a "$LOG_FILE"
    echo "--- Memory ---" | tee -a "$LOG_FILE"
    free -h | tee -a "$LOG_FILE"

    # Disk
    echo "" | tee -a "$LOG_FILE"
    echo "--- Disk ---" | tee -a "$LOG_FILE"
    df -h / | tee -a "$LOG_FILE"

    # Network interfaces
    echo "" | tee -a "$LOG_FILE"
    echo "--- Network ---" | tee -a "$LOG_FILE"
    ip -br link show | tee -a "$LOG_FILE"

    # NVIDIA
    echo "" | tee -a "$LOG_FILE"
    echo "--- GPU ---" | tee -a "$LOG_FILE"
    nvidia-smi 2>/dev/null | head -12 | tee -a "$LOG_FILE" || echo "nvidia-smi not available" | tee -a "$LOG_FILE"

    # CUDA
    echo "" | tee -a "$LOG_FILE"
    echo "--- CUDA ---" | tee -a "$LOG_FILE"
    if command -v nvcc &>/dev/null; then
        nvcc --version | tee -a "$LOG_FILE"
    else
        echo "nvcc: not installed (runtime CUDA via nvidia-smi: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null || echo 'N/A'))" | tee -a "$LOG_FILE"
    fi

    # Docker
    echo "" | tee -a "$LOG_FILE"
    echo "--- Docker ---" | tee -a "$LOG_FILE"
    echo "Docker:           $(docker --version 2>/dev/null || echo 'NOT INSTALLED')" | tee -a "$LOG_FILE"
    echo "Docker Compose:   $(docker compose version 2>/dev/null || echo 'NOT INSTALLED')" | tee -a "$LOG_FILE"
    echo "Docker Service:   $(systemctl is-active docker 2>/dev/null || echo 'unknown')" | tee -a "$LOG_FILE"
    echo "NVIDIA Runtime:   $(dpkg -l nvidia-container-toolkit 2>/dev/null | grep -c '^ii' || echo '0') package(s)" | tee -a "$LOG_FILE"

    # Python
    echo "" | tee -a "$LOG_FILE"
    echo "--- Python ---" | tee -a "$LOG_FILE"
    echo "Python:  $(python3 --version 2>/dev/null || echo 'NOT INSTALLED')" | tee -a "$LOG_FILE"
    echo "pip:     $(python3 -m pip --version 2>/dev/null || echo 'NOT INSTALLED')" | tee -a "$LOG_FILE"

    # Ollama
    echo "" | tee -a "$LOG_FILE"
    echo "--- Ollama ---" | tee -a "$LOG_FILE"
    echo "Ollama:  $(ollama --version 2>/dev/null || echo 'NOT INSTALLED')" | tee -a "$LOG_FILE"
    if systemctl is-active ollama &>/dev/null; then
        echo "Service: active" | tee -a "$LOG_FILE"
        echo "Models:" | tee -a "$LOG_FILE"
        ollama list 2>/dev/null | tee -a "$LOG_FILE" || echo "  (unable to list)" | tee -a "$LOG_FILE"
    fi

    echo "" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
}

# =============================================================================
# Install pip
# =============================================================================
install_pip() {
    log_step "Installing Python pip"

    if python3 -m pip --version &>/dev/null; then
        log_info "pip already installed: $(python3 -m pip --version)"
        result_skip "pip (already installed)"
        return 0
    fi

    log_info "Installing python3-pip..."
    apt-get update -qq
    apt-get install -y python3-pip

    if python3 -m pip --version &>/dev/null; then
        log_success "pip installed: $(python3 -m pip --version)"
        result_ok "pip"
    else
        log_fail "pip installation failed"
        result_fail "pip"
        return 1
    fi
}

# =============================================================================
# Docker verification (already installed, just verify)
# =============================================================================
verify_docker() {
    log_step "Verifying Docker Installation"

    local all_good=true

    # Docker engine
    if command -v docker &>/dev/null; then
        log_success "Docker: $(docker --version)"
    else
        log_error "Docker not installed"
        log_info "Installing docker.io..."
        apt-get update -qq
        apt-get install -y docker.io
        systemctl start docker
        systemctl enable docker
        all_good=false
    fi

    # Docker Compose
    if docker compose version &>/dev/null; then
        log_success "Docker Compose: $(docker compose version)"
    else
        log_warn "Docker Compose not installed, installing..."
        DOCKER_CONFIG=${DOCKER_CONFIG:-/usr/local/lib/docker}
        mkdir -p $DOCKER_CONFIG/cli-plugins
        curl -SL "https://github.com/docker/compose/releases/download/v2.36.2/docker-compose-linux-aarch64" \
            -o $DOCKER_CONFIG/cli-plugins/docker-compose
        chmod +x $DOCKER_CONFIG/cli-plugins/docker-compose
    fi

    # Docker service
    if systemctl is-active docker &>/dev/null; then
        log_success "Docker service: active"
    else
        log_warn "Docker service not running, starting..."
        systemctl start docker
        systemctl enable docker
    fi

    # NVIDIA container runtime
    if dpkg -l nvidia-container-toolkit &>/dev/null; then
        log_success "NVIDIA Container Toolkit: installed"
    else
        log_warn "NVIDIA Container Toolkit not found, installing..."
        apt-get update -qq
        apt-get install -y nvidia-container-toolkit
        systemctl restart docker
    fi

    # Docker group for current user (determine the real user even under sudo)
    REAL_USER="${SUDO_USER:-$USER}"
    if groups "$REAL_USER" 2>/dev/null | grep -q docker; then
        log_success "User $REAL_USER in docker group"
    else
        usermod -aG docker "$REAL_USER"
        log_warn "Added $REAL_USER to docker group"
        log_warn "IMPORTANT: You must log out and back in (or run 'newgrp docker') for this to take effect!"
        log_warn "Until then, you'll get 'permission denied' when running docker without sudo."
    fi

    # Docker socket permissions check
    if [ -S /var/run/docker.sock ]; then
        local sock_group
        sock_group=$(stat -c '%G' /var/run/docker.sock 2>/dev/null)
        if [ "$sock_group" = "docker" ]; then
            log_success "Docker socket owned by docker group"
        else
            log_warn "Docker socket owned by '$sock_group' (expected 'docker')"
            chgrp docker /var/run/docker.sock 2>/dev/null || true
        fi
    fi

    # Test NVIDIA GPU access in Docker
    log_info "Testing NVIDIA GPU in Docker..."
    if docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi &>/dev/null; then
        log_success "Docker + NVIDIA GPU: working"
        result_ok "Docker + NVIDIA GPU"
    else
        log_warn "Docker NVIDIA GPU test failed (may need image pull or runtime config)"
        log_info "Trying to configure nvidia runtime in /etc/docker/daemon.json..."

        # Ensure nvidia runtime is configured
        if [ ! -f /etc/docker/daemon.json ]; then
            cat > /etc/docker/daemon.json <<'DAEMON_EOF'
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "default-runtime": "nvidia"
}
DAEMON_EOF
            systemctl restart docker
            log_info "Configured nvidia as default Docker runtime"
        fi
        result_ok "Docker (GPU test skipped — may need image pull)"
    fi
}

# =============================================================================
# Install Ollama
# =============================================================================
install_ollama() {
    log_step "Installing Ollama"

    if command -v ollama &>/dev/null; then
        log_info "Ollama already installed: $(ollama --version 2>/dev/null || echo 'version unknown')"
        result_skip "Ollama (already installed)"

        # Ensure service is running
        if ! systemctl is-active ollama &>/dev/null; then
            log_info "Starting Ollama service..."
            systemctl start ollama 2>/dev/null || true
        fi
        return 0
    fi

    log_info "Installing Ollama via official script..."
    curl -fsSL https://ollama.com/install.sh | sh

    if command -v ollama &>/dev/null; then
        log_success "Ollama installed: $(ollama --version 2>/dev/null || echo 'OK')"

        # Ensure service is enabled
        systemctl enable ollama 2>/dev/null || true
        systemctl start ollama 2>/dev/null || true

        # Wait for API
        log_info "Waiting for Ollama API..."
        for i in $(seq 1 15); do
            if curl -s http://localhost:11434/api/tags &>/dev/null; then
                log_success "Ollama API responding"
                break
            fi
            sleep 2
        done

        result_ok "Ollama"
    else
        log_fail "Ollama installation failed"
        result_fail "Ollama"
        return 1
    fi
}

# =============================================================================
# Pull LLM Model
# =============================================================================
pull_model() {
    log_step "Pulling LLM Model"

    # Default: llama3.2:3b (~3.2B params, ~2GB, good context grounding)
    # tinyllama hallucinates badly; llama3.2:3b answers from retrieved context
    # Other options: phi-3-mini-4k (3.8B), smollm2 (1.7B)
    MODEL="${MODEL:-llama3.2:3b}"

    # Check if Ollama is running
    if ! curl -s http://localhost:11434/api/tags &>/dev/null; then
        log_warn "Ollama API not responding. Starting service..."
        systemctl start ollama 2>/dev/null || ollama serve &>/dev/null &
        sleep 5
    fi

    # Check if model already exists
    if ollama list 2>/dev/null | grep -q "$MODEL"; then
        log_info "Model '$MODEL' already pulled"
        result_skip "Model $MODEL (already present)"
        return 0
    fi

    log_info "Pulling model: $MODEL (this may take a few minutes)..."
    if ollama pull "$MODEL"; then
        log_success "Model '$MODEL' pulled successfully"

        # Quick test
        log_info "Testing model with a simple prompt..."
        RESPONSE=$(echo "Say hello in one sentence." | timeout 60 ollama run "$MODEL" 2>/dev/null | head -3)
        if [ -n "$RESPONSE" ]; then
            log_success "Model test passed: $RESPONSE"
        else
            log_warn "Model test: no response (may still be loading)"
        fi

        result_ok "Model $MODEL"
    else
        log_fail "Failed to pull model '$MODEL'"
        result_fail "Model $MODEL"
        return 1
    fi
}

# =============================================================================
# Start RAG Docker Stack
# =============================================================================
start_rag() {
    log_step "Starting RAG Docker Stack"

    RAG_DIR="${SCRIPT_DIR}/bootylicious/rag"

    if [ ! -d "$RAG_DIR" ]; then
        log_error "RAG directory not found: $RAG_DIR"
        log_info "Make sure bootylicious has been rsynced to this location"
        result_fail "RAG stack (directory not found)"
        return 1
    fi

    cd "$RAG_DIR"

    # Create .env from example if it doesn't exist
    if [ ! -f .env ] && [ -f .env.example ]; then
        cp .env.example .env
        # Update LLM backend to use Ollama (since we just installed it)
        sed -i 's/^LLM_BACKEND=.*/LLM_BACKEND=ollama/' .env
        sed -i 's|^OLLAMA_BASE_URL=.*|OLLAMA_BASE_URL=http://host.docker.internal:11434|' .env
        log_info "Created .env from .env.example (configured for Ollama)"
    fi

    # Check if containers are already running
    if docker compose ps --format json 2>/dev/null | grep -q "running"; then
        log_info "RAG containers already running"
        docker compose ps 2>/dev/null | tee -a "$LOG_FILE"
        result_skip "RAG stack (already running)"
        return 0
    fi

    # Start the stack
    log_info "Starting RAG Docker stack (postgres + redis + api)..."
    if docker compose up -d --build 2>&1 | tee -a "$LOG_FILE"; then
        log_info "Waiting for services to be healthy..."
        sleep 10

        # Check health
        if curl -s http://localhost:8000/api/health &>/dev/null; then
            log_success "RAG API is healthy"
            result_ok "RAG stack"
        else
            log_warn "RAG API not responding yet (may still be starting)"
            docker compose ps 2>/dev/null | tee -a "$LOG_FILE"
            result_ok "RAG stack (started, health pending)"
        fi
    else
        log_fail "Docker compose failed"
        result_fail "RAG stack"
        return 1
    fi
}

# =============================================================================
# Check (show what's installed without changing anything)
# =============================================================================
check_only() {
    system_inventory
}

# =============================================================================
# Print Summary
# =============================================================================
print_summary() {
    echo "" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    echo "  Install Summary" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    for r in "${RESULTS[@]}"; do
        echo "  $r" | tee -a "$LOG_FILE"
    done
    echo "========================================" | tee -a "$LOG_FILE"
    echo "" | tee -a "$LOG_FILE"
    echo "Log saved to: $LOG_FILE" | tee -a "$LOG_FILE"
    echo "" | tee -a "$LOG_FILE"
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo "" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    echo "  Ambot - Jetson Orin Nano Setup" | tee -a "$LOG_FILE"
    echo "  $(date)" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
    echo "" | tee -a "$LOG_FILE"

    # Parse arguments
    local do_all=true
    local do_check=false
    local do_inventory=false
    local do_docker=false
    local do_pip=false
    local do_ollama=false
    local do_model=false
    local do_rag=false
    local do_cuda=false

    for arg in "$@"; do
        case $arg in
            --check)     do_check=true; do_all=false ;;
            --inventory) do_inventory=true; do_all=false ;;
            --docker)    do_docker=true; do_all=false ;;
            --pip)       do_pip=true; do_all=false ;;
            --ollama)    do_ollama=true; do_all=false ;;
            --model)     do_model=true; do_all=false ;;
            --rag)       do_rag=true; do_all=false ;;
            --cuda)      do_cuda=true; do_all=false ;;
            --help|-h)
                echo "Usage: sudo $0 [options]"
                echo ""
                echo "Options:"
                echo "  --check      Check what's installed (no changes)"
                echo "  --inventory  Detailed system inventory"
                echo "  --docker     Verify/install Docker"
                echo "  --pip        Install Python pip"
                echo "  --ollama     Install Ollama LLM server"
                echo "  --model      Pull LLM model (default: llama3.2:3b)"
                echo "  --rag        Start RAG Docker stack"
                echo "  --cuda       CUDA verification & PATH fix"
                echo "  --help       Show this help"
                echo ""
                echo "No args = full install (all components)"
                echo ""
                echo "Environment:"
                echo "  MODEL=<name>  Override default model (e.g., MODEL=phi-3-mini-4k)"
                echo ""
                echo "Background-safe:"
                echo "  nohup sudo ./install-jetson.sh > install.log 2>&1 &"
                exit 0
                ;;
            *)
                log_error "Unknown option: $arg (use --help)"
                exit 1
                ;;
        esac
    done

    if [ "$do_check" = true ]; then
        check_only
        exit 0
    fi

    if [ "$do_inventory" = true ]; then
        system_inventory
        exit 0
    fi

    # Full install or selective
    if [ "$do_all" = true ] || [ "$do_pip" = true ]; then
        install_pip
    fi

    if [ "$do_all" = true ] || [ "$do_docker" = true ]; then
        verify_docker
    fi

    if [ "$do_all" = true ] || [ "$do_ollama" = true ]; then
        install_ollama
    fi

    if [ "$do_all" = true ] || [ "$do_model" = true ]; then
        pull_model
    fi

    if [ "$do_rag" = true ]; then
        start_rag
    fi

    if [ "$do_all" = true ] || [ "$do_cuda" = true ]; then
        log_step "CUDA Verification"
        local cuda_script="${SCRIPT_DIR}/scripts/setup-cuda.sh"
        if [ -f "$cuda_script" ]; then
            bash "$cuda_script"
        else
            log_warn "CUDA setup script not found: $cuda_script"
            log_info "Run scripts/setup-cuda.sh separately"
        fi
    fi

    print_summary

    if [ "$do_all" = true ]; then
        echo "Next steps:" | tee -a "$LOG_FILE"
        echo "  1. Verify: sudo ./install-jetson.sh --check" | tee -a "$LOG_FILE"
        echo "  2. Test LLM: ollama run llama3.2:3b" | tee -a "$LOG_FILE"
        echo "  3. Start RAG: sudo ./install-jetson.sh --rag" | tee -a "$LOG_FILE"
        echo "  4. Test RAG: curl http://localhost:8000/api/health" | tee -a "$LOG_FILE"
        echo "" | tee -a "$LOG_FILE"
    fi
}

main "$@"
