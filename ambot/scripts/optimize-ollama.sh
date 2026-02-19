#!/usr/bin/env bash
# =============================================================================
# optimize-ollama.sh - Apply Ollama memory optimizations for Jetson Orin Nano
# =============================================================================
# Adds environment variables to Ollama's systemd override to reduce memory usage.
# Run on the Jetson (or via SSH): sudo bash scripts/optimize-ollama.sh
#
# Optimizations:
#   OLLAMA_MAX_LOADED_MODELS=1   - Only keep 1 model in memory (default: 3)
#   OLLAMA_NUM_PARALLEL=1        - Single request slot (saves KV cache duplication)
#   OLLAMA_FLASH_ATTENTION=1     - Reduces memory at larger context windows
#   OLLAMA_KV_CACHE_TYPE=q8_0    - Halves KV cache memory with negligible quality loss
# =============================================================================

set -euo pipefail

OVERRIDE_DIR="/etc/systemd/system/ollama.service.d"
OVERRIDE_FILE="${OVERRIDE_DIR}/override.conf"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    log_error "Must run as root (sudo)"
    exit 1
fi

# Check if Ollama is installed
if ! command -v ollama &>/dev/null; then
    log_error "Ollama not found"
    exit 1
fi

# Show current config
log_info "Current Ollama override:"
if [ -f "$OVERRIDE_FILE" ]; then
    cat "$OVERRIDE_FILE"
else
    echo "  (none)"
fi
echo ""

# Create override directory if needed
mkdir -p "$OVERRIDE_DIR"

# Write optimized override
cat > "$OVERRIDE_FILE" << 'EOF'
[Service]
Environment="OLLAMA_HOST=0.0.0.0"
Environment="OLLAMA_MAX_LOADED_MODELS=1"
Environment="OLLAMA_NUM_PARALLEL=1"
Environment="OLLAMA_FLASH_ATTENTION=1"
Environment="OLLAMA_KV_CACHE_TYPE=q8_0"
EOF

log_info "Updated Ollama override:"
cat "$OVERRIDE_FILE"
echo ""

# Reload systemd and restart Ollama
log_info "Reloading systemd daemon..."
systemctl daemon-reload

log_info "Restarting Ollama..."
systemctl restart ollama

# Wait for Ollama to come up
sleep 3

# Verify
if systemctl is-active --quiet ollama; then
    log_info "Ollama restarted successfully"

    # Quick health check
    if curl -sf http://localhost:11434/api/tags > /dev/null 2>&1; then
        log_info "Ollama API responding"

        # Show loaded models
        echo ""
        log_info "Available models:"
        curl -sf http://localhost:11434/api/tags | python3 -c "
import sys, json
data = json.load(sys.stdin)
for m in data.get('models', []):
    name = m.get('name', '?')
    size = m.get('size', 0) / (1024**3)
    print(f'  {name}: {size:.1f} GB')
" 2>/dev/null || echo "  (couldn't list)"
    else
        log_warn "Ollama API not responding yet (may still be starting)"
    fi
else
    log_error "Ollama failed to restart!"
    systemctl status ollama --no-pager
    exit 1
fi

echo ""
log_info "Optimizations applied:"
echo "  OLLAMA_MAX_LOADED_MODELS=1  (prevents loading multiple models, saves ~2-4 GB)"
echo "  OLLAMA_NUM_PARALLEL=1       (single request slot, saves KV cache duplication)"
echo "  OLLAMA_FLASH_ATTENTION=1    (reduces memory at larger contexts)"
echo "  OLLAMA_KV_CACHE_TYPE=q8_0   (halves KV cache memory, negligible quality loss)"
