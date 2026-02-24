#!/usr/bin/env bash
# =============================================================================
# rag-ctl.sh — RAG System Control Script (runs ON the Jetson)
# =============================================================================
#
# Single entry point for all RAG operations. deploy.sh calls this remotely
# via SSH so only ONE connection is needed per operation.
#
# Usage:
#   ./rag-ctl.sh status              # Docker container status
#   ./rag-ctl.sh health              # API health check (JSON)
#   ./rag-ctl.sh rebuild             # Rebuild + restart + wait for healthy
#   ./rag-ctl.sh restart             # Restart containers (no rebuild)
#   ./rag-ctl.sh test                # Run full RAG test suite
#   ./rag-ctl.sh logs [LINES]        # Show recent API logs (default: 50)
#   ./rag-ctl.sh docs                # List ingested documents
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAG_DIR="$(cd "$SCRIPT_DIR/../rag" && pwd)"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[rag-ctl]${NC} $*"; }
warn() { echo -e "${YELLOW}[rag-ctl]${NC} $*"; }
err()  { echo -e "${RED}[rag-ctl]${NC} $*" >&2; }

API_URL="http://localhost:8000"

# --- Helpers ----------------------------------------------------------------

wait_healthy() {
    local max_wait="${1:-60}"
    local waited=0
    log "Waiting for API to become healthy (timeout: ${max_wait}s)..."
    while [ "$waited" -lt "$max_wait" ]; do
        if curl -sf "${API_URL}/api/health" > /dev/null 2>&1; then
            log "API is healthy (${waited}s)"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
    done
    err "API did not become healthy within ${max_wait}s"
    return 1
}

# --- Commands ---------------------------------------------------------------

cmd_status() {
    log "Docker container status:"
    cd "$RAG_DIR"
    sudo docker compose ps 2>/dev/null || docker compose ps
}

cmd_health() {
    if curl -sf "${API_URL}/api/health" 2>/dev/null; then
        echo ""  # newline after JSON
    else
        err "API not reachable at ${API_URL}"
        return 1
    fi
}

cmd_rebuild() {
    log "Rebuilding RAG containers..."
    cd "$RAG_DIR"
    sudo docker compose up -d --build api 2>&1
    wait_healthy 90
    log "Rebuild complete."
}

cmd_restart() {
    log "Restarting RAG containers..."
    cd "$RAG_DIR"
    sudo docker compose restart api 2>&1
    wait_healthy 60
    log "Restart complete."
}

cmd_logs() {
    local lines="${1:-50}"
    cd "$RAG_DIR"
    sudo docker compose logs --tail="$lines" api 2>/dev/null || docker compose logs --tail="$lines" api
}

cmd_docs() {
    python3 "$SCRIPT_DIR/rag-docs.py" "${API_URL}"
}

cmd_test() {
    python3 "$SCRIPT_DIR/rag-test.py" "${API_URL}"
}

# --- Main -------------------------------------------------------------------

case "${1:-help}" in
    status)   cmd_status ;;
    health)   cmd_health ;;
    rebuild)  cmd_rebuild ;;
    restart)  cmd_restart ;;
    test)     cmd_test ;;
    logs)     cmd_logs "${2:-50}" ;;
    docs)     cmd_docs ;;
    help|--help|-h)
        echo "Usage: rag-ctl.sh {status|health|rebuild|restart|test|logs|docs}"
        echo ""
        echo "Commands:"
        echo "  status    Docker container status"
        echo "  health    API health check (JSON)"
        echo "  rebuild   Rebuild + restart + wait for healthy"
        echo "  restart   Restart containers (no rebuild)"
        echo "  test      Run full RAG test suite"
        echo "  logs [N]  Show recent API logs (default: 50)"
        echo "  docs      List ingested documents"
        ;;
    *)
        err "Unknown command: $1"
        echo "Use 'rag-ctl.sh help' for usage."
        exit 1
        ;;
esac
