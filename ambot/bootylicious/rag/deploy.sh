#!/bin/bash
# =============================================================================
# Ambot RAG System - Deploy Script
# =============================================================================
# Deployment script for the RAG system on Jetson Orin Nano.
#
# Usage:
#   ./deploy.sh start [--build]   Start all services
#   ./deploy.sh stop              Stop all services
#   ./deploy.sh restart           Restart services
#   ./deploy.sh status            Show service status
#   ./deploy.sh logs [service]    View logs
#   ./deploy.sh health            Check system health
#   ./deploy.sh ingest [path]     Ingest documents
#   ./deploy.sh clean             Stop and remove all data
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[====]${NC} $1"; }

# =============================================================================
# Check prerequisites
# =============================================================================
check_prereqs() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed. Run setup-docker.sh first."
        exit 1
    fi

    if ! docker compose version &> /dev/null 2>&1; then
        log_error "Docker Compose is not installed. Run setup-docker.sh first."
        exit 1
    fi
}

# =============================================================================
# Create .env file if not exists
# =============================================================================
ensure_env() {
    if [ ! -f .env ]; then
        if [ -f .env.example ]; then
            log_info "Creating .env from .env.example"
            cp .env.example .env
        else
            log_warn "No .env file found. Using defaults."
        fi
    fi
}

# =============================================================================
# Commands
# =============================================================================

cmd_start() {
    log_step "Starting Ambot RAG System..."

    ensure_env

    BUILD_FLAG=""
    if [[ "$1" == "--build" ]]; then
        BUILD_FLAG="--build"
        log_info "Building containers..."
    fi

    docker compose up -d $BUILD_FLAG

    log_info "Waiting for services to be healthy..."
    sleep 5

    cmd_health

    echo ""
    log_info "RAG system started!"
    echo ""
    echo "API endpoint: http://localhost:${API_PORT:-8000}"
    echo "Health check: http://localhost:${API_PORT:-8000}/api/health"
    echo ""
    echo "Ingest documents:"
    echo "  ./deploy.sh ingest ./knowledge"
    echo ""
}

cmd_stop() {
    log_step "Stopping Ambot RAG System..."
    docker compose down
    log_info "Services stopped."
}

cmd_restart() {
    cmd_stop
    cmd_start "$@"
}

cmd_status() {
    log_step "Service Status"
    docker compose ps
}

cmd_logs() {
    SERVICE="${1:-}"
    if [ -n "$SERVICE" ]; then
        docker compose logs -f "$SERVICE"
    else
        docker compose logs -f
    fi
}

cmd_health() {
    log_step "Health Check"

    API_PORT="${API_PORT:-8000}"
    HEALTH_URL="http://localhost:${API_PORT}/api/health"

    echo ""

    # Check if API is responding
    if curl -s "$HEALTH_URL" > /dev/null 2>&1; then
        HEALTH=$(curl -s "$HEALTH_URL")
        echo "API Health: $HEALTH"
    else
        log_warn "API not responding yet. Services may still be starting."
        echo ""
        echo "Container status:"
        docker compose ps --format "table {{.Name}}\t{{.Status}}\t{{.Ports}}"
    fi

    echo ""
}

cmd_ingest() {
    INGEST_PATH="${1:-./knowledge}"

    log_step "Ingesting documents from: $INGEST_PATH"

    API_PORT="${API_PORT:-8000}"

    # Check if path exists
    if [ ! -d "$INGEST_PATH" ]; then
        log_error "Directory not found: $INGEST_PATH"
        exit 1
    fi

    # Convert to absolute path
    ABS_PATH=$(realpath "$INGEST_PATH")

    # For Docker, we need to use the mounted path
    DOCKER_PATH="/data/knowledge"

    log_info "Ingesting from Docker path: $DOCKER_PATH"

    RESPONSE=$(curl -s -X POST "http://localhost:${API_PORT}/api/ingest/directory" \
        -H "Content-Type: application/json" \
        -d "{\"path\": \"$DOCKER_PATH\"}")

    echo "$RESPONSE" | python3 -m json.tool 2>/dev/null || echo "$RESPONSE"

    echo ""
    log_info "Ingestion complete!"
}

cmd_clean() {
    log_warn "This will delete ALL data including the database!"
    read -p "Are you sure? (y/N): " confirm

    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        log_step "Cleaning up..."
        docker compose down -v
        log_info "All services and data removed."
    else
        log_info "Cancelled."
    fi
}

cmd_shell() {
    SERVICE="${1:-api}"
    log_info "Opening shell in $SERVICE container..."
    docker compose exec "$SERVICE" /bin/sh
}

cmd_help() {
    echo ""
    echo "Ambot RAG System - Deploy Script"
    echo ""
    echo "Usage: ./deploy.sh <command> [options]"
    echo ""
    echo "Commands:"
    echo "  start [--build]    Start all services (rebuild if --build)"
    echo "  stop               Stop all services"
    echo "  restart            Restart all services"
    echo "  status             Show service status"
    echo "  logs [service]     View logs (all or specific service)"
    echo "  health             Check system health"
    echo "  ingest [path]      Ingest documents from path (default: ./knowledge)"
    echo "  clean              Stop and remove all data (volumes)"
    echo "  shell [service]    Open shell in container (default: api)"
    echo "  help               Show this help"
    echo ""
    echo "Examples:"
    echo "  ./deploy.sh start --build    # Build and start"
    echo "  ./deploy.sh logs api         # View API logs"
    echo "  ./deploy.sh ingest ./docs    # Ingest from ./docs"
    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    check_prereqs

    COMMAND="${1:-help}"
    shift || true

    case "$COMMAND" in
        start)
            cmd_start "$@"
            ;;
        stop)
            cmd_stop
            ;;
        restart)
            cmd_restart "$@"
            ;;
        status)
            cmd_status
            ;;
        logs)
            cmd_logs "$@"
            ;;
        health)
            cmd_health
            ;;
        ingest)
            cmd_ingest "$@"
            ;;
        clean)
            cmd_clean
            ;;
        shell)
            cmd_shell "$@"
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            log_error "Unknown command: $COMMAND"
            cmd_help
            exit 1
            ;;
    esac
}

main "$@"
