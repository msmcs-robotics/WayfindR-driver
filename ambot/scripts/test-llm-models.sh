#!/usr/bin/env bash
# =============================================================================
# test-llm-models.sh - Compare LLM models on Jetson for RAG quality
# =============================================================================
# Usage: bash scripts/test-llm-models.sh [--pull] [--skip-pull]
#
# Pulls candidate models, runs the same RAG question through each,
# and reports timing + answer quality for comparison.
# =============================================================================

set -euo pipefail

OLLAMA_URL="http://localhost:11434"
RAG_URL="http://localhost:8000"

# Models to test (name:tag as Ollama expects)
MODELS=(
    "llama3.2:3b"
    "phi3:mini"
    "gemma2:2b"
    "smollm2:1.7b"
)

# Test question for RAG
TEST_QUESTION="What are the main components of the ambot system and what hardware does each run on?"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_test()  { echo -e "${CYAN}[TEST]${NC} $*"; }

# -------------------------------------------------------------------------
# Check prerequisites
# -------------------------------------------------------------------------
check_services() {
    log_info "Checking services..."

    # Check Ollama
    if ! curl -sf "${OLLAMA_URL}/api/tags" > /dev/null 2>&1; then
        log_error "Ollama not responding at ${OLLAMA_URL}"
        exit 1
    fi
    log_info "Ollama: OK"

    # Check RAG API
    local health
    health=$(curl -sf "${RAG_URL}/api/health" 2>/dev/null || echo '{}')
    if echo "$health" | python3 -c "import sys,json; d=json.load(sys.stdin); assert d.get('status')=='healthy'" 2>/dev/null; then
        log_info "RAG API: healthy"
    else
        log_error "RAG API not healthy at ${RAG_URL}"
        exit 1
    fi
}

# -------------------------------------------------------------------------
# Pull models
# -------------------------------------------------------------------------
pull_models() {
    log_info "Pulling models (this may take a while)..."

    for model in "${MODELS[@]}"; do
        # Check if already available
        if curl -sf "${OLLAMA_URL}/api/tags" | python3 -c "
import sys, json
data = json.load(sys.stdin)
names = [m['name'] for m in data.get('models', [])]
found = any('$model' in n or n in '$model' for n in names)
sys.exit(0 if found else 1)
" 2>/dev/null; then
            log_info "  $model: already available"
        else
            log_info "  $model: pulling..."
            if curl -sf -X POST "${OLLAMA_URL}/api/pull" -d "{\"name\": \"$model\", \"stream\": false}" > /dev/null 2>&1; then
                log_info "  $model: pulled OK"
            else
                log_warn "  $model: pull failed (skipping)"
            fi
        fi
    done
}

# -------------------------------------------------------------------------
# Test a single model via direct Ollama API (bypass RAG for raw speed test)
# -------------------------------------------------------------------------
test_model_direct() {
    local model="$1"
    local prompt="Answer in 2-3 sentences: What is the capital of France and why is it famous?"

    log_test "Direct test: $model"

    local start end elapsed
    start=$(date +%s%N)

    local response
    response=$(curl -sf -X POST "${OLLAMA_URL}/api/generate" \
        -d "{
            \"model\": \"$model\",
            \"prompt\": \"$prompt\",
            \"stream\": false,
            \"options\": {
                \"temperature\": 0.3,
                \"num_ctx\": 4096,
                \"num_predict\": 256
            }
        }" 2>/dev/null || echo '{"error": "request failed"}')

    end=$(date +%s%N)
    elapsed=$(( (end - start) / 1000000 ))

    local answer
    answer=$(echo "$response" | python3 -c "import sys,json; print(json.load(sys.stdin).get('response','ERROR')[:200])" 2>/dev/null || echo "PARSE ERROR")

    local total_duration eval_count
    total_duration=$(echo "$response" | python3 -c "import sys,json; d=json.load(sys.stdin); print(f'{d.get(\"total_duration\",0)/1e9:.1f}s')" 2>/dev/null || echo "?")
    eval_count=$(echo "$response" | python3 -c "import sys,json; print(json.load(sys.stdin).get('eval_count','?'))" 2>/dev/null || echo "?")

    echo "  Time: ${elapsed}ms (Ollama: ${total_duration})"
    echo "  Tokens generated: ${eval_count}"
    echo "  Answer: ${answer}"
    echo ""
}

# -------------------------------------------------------------------------
# Test a model via RAG API (full pipeline)
# -------------------------------------------------------------------------
test_model_rag() {
    local model="$1"

    log_test "RAG test: $model"

    # Temporarily update the .env to use this model, then restart API
    # Instead, we'll use the direct Ollama API with RAG-like prompt
    # This avoids restarting the container for each model test

    # First, get relevant context from RAG search
    local search_result
    search_result=$(curl -sf -X POST "${RAG_URL}/api/search" \
        -H 'Content-Type: application/json' \
        -d "{\"query\": \"$TEST_QUESTION\", \"top_k\": 3}" 2>/dev/null || echo '{"results":[]}')

    # Extract context from search results
    local context
    context=$(echo "$search_result" | python3 -c "
import sys, json
data = json.load(sys.stdin)
results = data.get('results', [])
parts = []
for i, r in enumerate(results, 1):
    src = r.get('document_filename', 'unknown')
    content = r.get('content', '')[:500]
    parts.append(f'[Source {i}: {src}]\n{content}')
print('\n---\n'.join(parts))
" 2>/dev/null || echo "No context available")

    # Build RAG prompt
    local system_prompt="You are a knowledgeable assistant for ambot-eecs. Answer questions accurately using ONLY the provided context. Cite sources by their source number."
    local user_prompt="## Retrieved Context\n\n${context}\n\n---\n\n## Question\n\n${TEST_QUESTION}"

    local start end elapsed
    start=$(date +%s%N)

    local response
    response=$(curl -sf -X POST "${OLLAMA_URL}/api/generate" \
        -d "$(python3 -c "
import json
print(json.dumps({
    'model': '$model',
    'prompt': '''$user_prompt''',
    'system': '''$system_prompt''',
    'stream': False,
    'options': {
        'temperature': 0.3,
        'num_ctx': 4096,
        'num_predict': 512
    }
}))
")" 2>/dev/null || echo '{"error": "request failed"}')

    end=$(date +%s%N)
    elapsed=$(( (end - start) / 1000000 ))

    local answer
    answer=$(echo "$response" | python3 -c "import sys,json; print(json.load(sys.stdin).get('response','ERROR')[:500])" 2>/dev/null || echo "PARSE ERROR")

    local total_duration eval_count
    total_duration=$(echo "$response" | python3 -c "import sys,json; d=json.load(sys.stdin); print(f'{d.get(\"total_duration\",0)/1e9:.1f}s')" 2>/dev/null || echo "?")
    eval_count=$(echo "$response" | python3 -c "import sys,json; print(json.load(sys.stdin).get('eval_count','?'))" 2>/dev/null || echo "?")

    echo "  Time: ${elapsed}ms (Ollama: ${total_duration})"
    echo "  Tokens generated: ${eval_count}"
    echo "  Answer (first 500 chars):"
    echo "  ${answer}"
    echo ""
}

# -------------------------------------------------------------------------
# Show memory usage
# -------------------------------------------------------------------------
show_memory() {
    echo ""
    log_info "Memory usage:"
    free -h | grep Mem | awk '{printf "  Total: %s, Used: %s, Free: %s, Available: %s\n", $2, $3, $4, $7}'

    # Check Ollama loaded models
    local ps_result
    ps_result=$(curl -sf "${OLLAMA_URL}/api/ps" 2>/dev/null || echo '{}')
    echo "  Ollama loaded models:"
    echo "$ps_result" | python3 -c "
import sys, json
data = json.load(sys.stdin)
models = data.get('models', [])
if not models:
    print('    (none loaded)')
else:
    for m in models:
        name = m.get('name', '?')
        size = m.get('size', 0) / (1024**3)
        print(f'    {name}: {size:.1f} GiB')
" 2>/dev/null || echo "    (couldn't read)"
    echo ""
}

# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------
main() {
    local skip_pull=false

    for arg in "$@"; do
        case "$arg" in
            --skip-pull) skip_pull=true ;;
            --pull)      skip_pull=false ;;
            -h|--help)
                echo "Usage: $0 [--pull|--skip-pull]"
                echo "  --pull       Pull missing models before testing (default)"
                echo "  --skip-pull  Skip pulling, only test available models"
                exit 0
                ;;
        esac
    done

    echo "============================================================"
    echo "  LLM Model Comparison Test for Jetson Orin Nano"
    echo "============================================================"
    echo ""

    check_services

    # List available models
    echo ""
    log_info "Available Ollama models:"
    curl -sf "${OLLAMA_URL}/api/tags" | python3 -c "
import sys, json
data = json.load(sys.stdin)
for m in data.get('models', []):
    name = m.get('name', '?')
    size = m.get('size', 0) / (1024**3)
    print(f'  {name}: {size:.1f} GB')
" 2>/dev/null || echo "  (couldn't list)"
    echo ""

    if [ "$skip_pull" = false ]; then
        pull_models
    fi

    show_memory

    # Run direct speed tests
    echo "============================================================"
    echo "  DIRECT SPEED TEST (simple question, no RAG)"
    echo "============================================================"
    echo ""

    for model in "${MODELS[@]}"; do
        # Check if model is available before testing
        if curl -sf "${OLLAMA_URL}/api/tags" | python3 -c "
import sys, json
data = json.load(sys.stdin)
names = [m['name'] for m in data.get('models', [])]
found = any('$model' in n or n in '$model' for n in names)
sys.exit(0 if found else 1)
" 2>/dev/null; then
            test_model_direct "$model"
            show_memory
        else
            log_warn "Skipping $model (not available)"
        fi
    done

    # Run RAG tests
    echo "============================================================"
    echo "  RAG QUALITY TEST (with retrieved context)"
    echo "============================================================"
    echo ""

    for model in "${MODELS[@]}"; do
        if curl -sf "${OLLAMA_URL}/api/tags" | python3 -c "
import sys, json
data = json.load(sys.stdin)
names = [m['name'] for m in data.get('models', [])]
found = any('$model' in n or n in '$model' for n in names)
sys.exit(0 if found else 1)
" 2>/dev/null; then
            test_model_rag "$model"
        else
            log_warn "Skipping $model (not available)"
        fi
    done

    show_memory

    echo "============================================================"
    echo "  TEST COMPLETE"
    echo "============================================================"
}

main "$@"
