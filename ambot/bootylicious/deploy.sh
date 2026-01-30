#!/bin/bash
# =============================================================================
# Bootylicious - Comprehensive Deployment Script
# =============================================================================
# Master deployment script for the Ambot Jetson system.
# Handles setup, deployment, diagnostics, and teardown with idempotency.
#
# Usage:
#   ./deploy.sh <command> [options]
#
# Commands:
#   setup       Run initial system setup (Docker, etc.)
#   start       Start all services
#   stop        Stop all services
#   restart     Restart all services
#   status      Show system status
#   health      Run health checks
#   diagnose    Run comprehensive diagnostics
#   logs        View logs
#   clean       Clean up (with confirmation)
#   ssh-fix     Fix stale SSH tunnels
#   network     Network diagnostics
#   help        Show this help
#
# All commands are idempotent - safe to run multiple times.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# =============================================================================
# Configuration
# =============================================================================
PROJECT_NAME="bootylicious"
RAG_DIR="$SCRIPT_DIR/rag"
SCRIPTS_DIR="$SCRIPT_DIR/scripts"
LOG_DIR="$SCRIPT_DIR/logs"
STATE_FILE="$SCRIPT_DIR/.deploy_state"

# Create log directory
mkdir -p "$LOG_DIR"

# =============================================================================
# Colors and Logging
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()    { echo -e "${BLUE}[====]${NC} $1"; }
log_check()   { echo -e "${CYAN}[CHECK]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_fail()    { echo -e "${RED}[✗]${NC} $1"; }

# Logging with timestamp
log_to_file() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$LOG_DIR/deploy.log"
}

# =============================================================================
# State Management (for idempotency)
# =============================================================================
save_state() {
    echo "$1=$(date '+%Y-%m-%d %H:%M:%S')" >> "$STATE_FILE"
}

check_state() {
    grep -q "^$1=" "$STATE_FILE" 2>/dev/null
}

clear_state() {
    rm -f "$STATE_FILE"
}

# =============================================================================
# System Checks (Idempotent)
# =============================================================================
check_docker() {
    if command -v docker &> /dev/null; then
        if docker ps &> /dev/null 2>&1; then
            log_success "Docker: Running ($(docker --version | cut -d' ' -f3 | tr -d ','))"
            return 0
        else
            log_warn "Docker: Installed but not accessible (run: newgrp docker)"
            return 1
        fi
    else
        log_fail "Docker: Not installed"
        return 1
    fi
}

check_docker_compose() {
    if docker compose version &> /dev/null 2>&1; then
        log_success "Docker Compose: $(docker compose version --short)"
        return 0
    else
        log_fail "Docker Compose: Not installed"
        return 1
    fi
}

check_nvidia() {
    if command -v nvidia-smi &> /dev/null; then
        GPU_INFO=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null || echo "Error reading GPU")
        log_success "NVIDIA GPU: $GPU_INFO"
        return 0
    else
        log_warn "NVIDIA GPU: nvidia-smi not found"
        return 1
    fi
}

check_memory() {
    MEM_TOTAL=$(free -h | awk '/^Mem:/ {print $2}')
    MEM_USED=$(free -h | awk '/^Mem:/ {print $3}')
    MEM_AVAIL=$(free -h | awk '/^Mem:/ {print $7}')
    log_info "Memory: ${MEM_USED} used / ${MEM_TOTAL} total (${MEM_AVAIL} available)"
}

check_disk() {
    DISK_INFO=$(df -h / | awk 'NR==2 {print $3 " used / " $2 " total (" $5 " used)"}')
    log_info "Disk (/): $DISK_INFO"
}

check_network() {
    # Check internet connectivity
    if ping -c 1 -W 2 8.8.8.8 &> /dev/null; then
        log_success "Internet: Connected"
    else
        log_warn "Internet: No connection"
    fi

    # Get IP addresses
    IP_ADDRS=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^$' | head -3)
    if [ -n "$IP_ADDRS" ]; then
        log_info "IP Addresses: $(echo $IP_ADDRS | tr '\n' ' ')"
    fi
}

check_ssh_tunnels() {
    STALE_SSH=$(ps aux | grep -E 'ssh.*-[LR]' | grep -v grep | wc -l)
    if [ "$STALE_SSH" -gt 0 ]; then
        log_warn "SSH Tunnels: $STALE_SSH active tunnel(s) found"
        return 1
    else
        log_success "SSH Tunnels: None (clean)"
        return 0
    fi
}

check_ports() {
    log_info "Checking common ports..."

    # Check if ports are in use
    for PORT in 8000 5432 6379 11434; do
        if ss -tuln 2>/dev/null | grep -q ":$PORT "; then
            SERVICE=$(ss -tulnp 2>/dev/null | grep ":$PORT " | awk '{print $NF}' | head -1)
            log_info "  Port $PORT: IN USE ($SERVICE)"
        else
            log_info "  Port $PORT: Available"
        fi
    done
}

# =============================================================================
# Service Management
# =============================================================================
rag_is_running() {
    if [ -d "$RAG_DIR" ]; then
        cd "$RAG_DIR"
        docker compose ps --status running 2>/dev/null | grep -q "ambot-rag" && return 0
    fi
    return 1
}

cmd_start() {
    log_step "Starting Bootylicious Services..."
    log_to_file "START command initiated"

    # Pre-flight checks
    if ! check_docker; then
        log_error "Docker is required. Run: ./deploy.sh setup"
        exit 1
    fi

    # Start RAG system
    if [ -d "$RAG_DIR" ]; then
        log_info "Starting RAG system..."
        cd "$RAG_DIR"

        # Create .env if not exists
        if [ ! -f .env ] && [ -f .env.example ]; then
            cp .env.example .env
            log_info "Created .env from .env.example"
        fi

        # Build if needed, otherwise just start
        if [ "$1" == "--build" ]; then
            docker compose up -d --build
        else
            docker compose up -d
        fi

        log_info "Waiting for services to be healthy..."
        sleep 5

        save_state "rag_started"
    fi

    cd "$SCRIPT_DIR"
    cmd_health

    log_success "Services started!"
    log_to_file "START command completed"
}

cmd_stop() {
    log_step "Stopping Bootylicious Services..."
    log_to_file "STOP command initiated"

    # Stop RAG system
    if [ -d "$RAG_DIR" ]; then
        log_info "Stopping RAG system..."
        cd "$RAG_DIR"
        docker compose down 2>/dev/null || true
    fi

    cd "$SCRIPT_DIR"
    log_success "Services stopped!"
    log_to_file "STOP command completed"
}

cmd_restart() {
    cmd_stop
    sleep 2
    cmd_start "$@"
}

# =============================================================================
# Status and Health
# =============================================================================
cmd_status() {
    echo ""
    echo "========================================"
    echo "  Bootylicious System Status"
    echo "========================================"
    echo ""

    log_step "System Resources"
    check_memory
    check_disk
    echo ""

    log_step "Infrastructure"
    check_docker
    check_docker_compose
    check_nvidia
    echo ""

    log_step "Network"
    check_network
    check_ssh_tunnels
    echo ""

    log_step "Docker Containers"
    if command -v docker &> /dev/null && docker ps &> /dev/null 2>&1; then
        docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" 2>/dev/null || echo "  (none running)"
    else
        echo "  Docker not accessible"
    fi
    echo ""

    echo "========================================"
}

cmd_health() {
    log_step "Health Checks"

    HEALTH_PASS=0
    HEALTH_FAIL=0

    # Docker health
    if check_docker &>/dev/null; then
        ((HEALTH_PASS++))
    else
        ((HEALTH_FAIL++))
    fi

    # RAG API health
    if [ -d "$RAG_DIR" ]; then
        API_PORT="${API_PORT:-8000}"
        if curl -sf "http://localhost:${API_PORT}/api/health" &>/dev/null; then
            log_success "RAG API: Healthy"
            ((HEALTH_PASS++))
        else
            log_fail "RAG API: Not responding (http://localhost:${API_PORT})"
            ((HEALTH_FAIL++))
        fi
    fi

    # Summary
    echo ""
    if [ $HEALTH_FAIL -eq 0 ]; then
        log_success "All health checks passed ($HEALTH_PASS/$HEALTH_PASS)"
    else
        log_warn "Health checks: $HEALTH_PASS passed, $HEALTH_FAIL failed"
    fi

    return $HEALTH_FAIL
}

# =============================================================================
# Diagnostics
# =============================================================================
cmd_diagnose() {
    echo ""
    echo "========================================"
    echo "  Bootylicious Comprehensive Diagnostics"
    echo "========================================"
    echo "  Time: $(date)"
    echo "========================================"
    echo ""

    # System info
    log_step "System Information"
    echo "  Hostname: $(hostname)"
    echo "  User: $(whoami)"
    echo "  OS: $(lsb_release -d 2>/dev/null | cut -f2 || cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d'"' -f2)"
    echo "  Kernel: $(uname -r)"
    echo "  Arch: $(uname -m)"
    echo ""

    # JetPack info (if on Jetson)
    if [ -f /etc/nv_tegra_release ]; then
        log_step "JetPack / L4T"
        cat /etc/nv_tegra_release
        echo ""
    fi

    # Run all checks
    log_step "Resource Checks"
    check_memory
    check_disk
    echo ""

    log_step "Infrastructure Checks"
    check_docker || true
    check_docker_compose || true
    check_nvidia || true
    echo ""

    log_step "Network Checks"
    check_network
    check_ssh_tunnels || true
    check_ports
    echo ""

    log_step "Docker Status"
    if command -v docker &> /dev/null && docker ps &> /dev/null 2>&1; then
        echo "  Running containers:"
        docker ps --format "    {{.Names}}: {{.Status}}" 2>/dev/null || echo "    (none)"
        echo ""
        echo "  Docker disk usage:"
        docker system df 2>/dev/null | sed 's/^/    /'
    else
        echo "  Docker not accessible"
    fi
    echo ""

    log_step "Service Health"
    cmd_health 2>/dev/null || true
    echo ""

    # Recent errors from logs
    log_step "Recent Errors (last 10)"
    if [ -f "$LOG_DIR/deploy.log" ]; then
        grep -i "error\|fail\|warn" "$LOG_DIR/deploy.log" 2>/dev/null | tail -10 | sed 's/^/    /' || echo "    (none)"
    else
        echo "    (no log file)"
    fi
    echo ""

    echo "========================================"
    echo "  Diagnostics Complete"
    echo "========================================"
}

# =============================================================================
# Network and SSH Fixes
# =============================================================================
cmd_ssh_fix() {
    log_step "Fixing Stale SSH Tunnels..."

    # Find and kill stale SSH tunnel processes
    STALE_PIDS=$(ps aux | grep -E 'ssh.*-[LR]' | grep -v grep | awk '{print $2}')

    if [ -n "$STALE_PIDS" ]; then
        echo "Found stale SSH tunnels:"
        ps aux | grep -E 'ssh.*-[LR]' | grep -v grep
        echo ""

        read -p "Kill these processes? (y/N): " confirm
        if [[ "$confirm" =~ ^[Yy]$ ]]; then
            echo "$STALE_PIDS" | xargs kill -9 2>/dev/null || true
            log_success "Killed stale SSH tunnels"
        else
            log_info "Cancelled"
        fi
    else
        log_success "No stale SSH tunnels found"
    fi
}

cmd_network() {
    log_step "Network Diagnostics"
    echo ""

    # IP addresses
    log_info "Network Interfaces:"
    ip addr show 2>/dev/null | grep -E "^[0-9]+:|inet " | sed 's/^/    /'
    echo ""

    # Routing
    log_info "Default Route:"
    ip route show default 2>/dev/null | sed 's/^/    /'
    echo ""

    # DNS
    log_info "DNS Servers:"
    cat /etc/resolv.conf 2>/dev/null | grep nameserver | sed 's/^/    /'
    echo ""

    # Connectivity tests
    log_info "Connectivity Tests:"
    for HOST in "8.8.8.8" "google.com"; do
        if ping -c 1 -W 2 "$HOST" &>/dev/null; then
            echo "    $HOST: OK"
        else
            echo "    $HOST: FAILED"
        fi
    done
    echo ""

    # Port listening
    log_info "Listening Ports:"
    ss -tuln 2>/dev/null | grep LISTEN | sed 's/^/    /' | head -20
}

# =============================================================================
# Setup (Idempotent)
# =============================================================================
cmd_setup() {
    log_step "Running System Setup..."
    log_to_file "SETUP command initiated"

    echo ""
    echo "This will run the setup scripts if needed."
    echo "All operations are idempotent (safe to run multiple times)."
    echo ""

    # Check what's already set up
    DOCKER_OK=false
    COMPOSE_OK=false

    if check_docker &>/dev/null; then
        DOCKER_OK=true
        log_info "Docker already installed, skipping..."
    fi

    if check_docker_compose &>/dev/null; then
        COMPOSE_OK=true
        log_info "Docker Compose already installed, skipping..."
    fi

    # Run setup if needed
    if [ "$DOCKER_OK" = false ] || [ "$COMPOSE_OK" = false ]; then
        if [ -f "$SCRIPTS_DIR/setup-docker.sh" ]; then
            read -p "Run Docker setup? (Y/n): " run_docker
            if [[ ! "$run_docker" =~ ^[Nn]$ ]]; then
                chmod +x "$SCRIPTS_DIR/setup-docker.sh"
                "$SCRIPTS_DIR/setup-docker.sh"
            fi
        else
            log_warn "setup-docker.sh not found"
        fi
    fi

    # Optional: HuggingFace setup
    echo ""
    read -p "Run HuggingFace setup? (y/N): " run_hf
    if [[ "$run_hf" =~ ^[Yy]$ ]]; then
        if [ -f "$SCRIPTS_DIR/setup-huggingface.sh" ]; then
            chmod +x "$SCRIPTS_DIR/setup-huggingface.sh"
            "$SCRIPTS_DIR/setup-huggingface.sh"
        fi
    fi

    save_state "setup_complete"
    log_success "Setup complete!"
    log_to_file "SETUP command completed"
}

# =============================================================================
# Logs
# =============================================================================
cmd_logs() {
    SERVICE="${1:-}"

    if [ -n "$SERVICE" ]; then
        # Specific service logs
        if [ -d "$RAG_DIR" ]; then
            cd "$RAG_DIR"
            docker compose logs -f "$SERVICE"
        fi
    else
        # All logs or deployment log
        echo "Log options:"
        echo "  1) RAG services (docker compose logs)"
        echo "  2) Deployment log ($LOG_DIR/deploy.log)"
        echo ""
        read -p "Choose (1/2): " choice

        case $choice in
            1)
                if [ -d "$RAG_DIR" ]; then
                    cd "$RAG_DIR"
                    docker compose logs -f
                fi
                ;;
            2)
                if [ -f "$LOG_DIR/deploy.log" ]; then
                    tail -100 "$LOG_DIR/deploy.log"
                else
                    echo "No deployment log found"
                fi
                ;;
        esac
    fi
}

# =============================================================================
# Clean
# =============================================================================
cmd_clean() {
    echo ""
    log_warn "This will stop all services and optionally remove data."
    echo ""
    echo "Options:"
    echo "  1) Stop services only"
    echo "  2) Stop services + remove containers"
    echo "  3) Stop services + remove containers + volumes (DATA LOSS)"
    echo "  4) Cancel"
    echo ""
    read -p "Choose (1/2/3/4): " choice

    case $choice in
        1)
            cmd_stop
            ;;
        2)
            cmd_stop
            if [ -d "$RAG_DIR" ]; then
                cd "$RAG_DIR"
                docker compose rm -f 2>/dev/null || true
            fi
            log_success "Containers removed"
            ;;
        3)
            log_warn "This will DELETE ALL DATA including the database!"
            read -p "Type 'DELETE' to confirm: " confirm
            if [ "$confirm" == "DELETE" ]; then
                if [ -d "$RAG_DIR" ]; then
                    cd "$RAG_DIR"
                    docker compose down -v 2>/dev/null || true
                fi
                clear_state
                log_success "All data removed"
            else
                log_info "Cancelled"
            fi
            ;;
        4|*)
            log_info "Cancelled"
            ;;
    esac
}

# =============================================================================
# Help
# =============================================================================
cmd_help() {
    echo ""
    echo "Bootylicious - Comprehensive Deployment Script"
    echo ""
    echo "Usage: ./deploy.sh <command> [options]"
    echo ""
    echo "Service Commands:"
    echo "  setup              Run initial system setup (idempotent)"
    echo "  start [--build]    Start all services"
    echo "  stop               Stop all services"
    echo "  restart            Restart all services"
    echo ""
    echo "Monitoring Commands:"
    echo "  status             Show system status overview"
    echo "  health             Run health checks"
    echo "  diagnose           Run comprehensive diagnostics"
    echo "  logs [service]     View logs"
    echo ""
    echo "Troubleshooting Commands:"
    echo "  ssh-fix            Fix stale SSH tunnels"
    echo "  network            Network diagnostics"
    echo ""
    echo "Maintenance Commands:"
    echo "  clean              Stop services and optionally clean up"
    echo "  help               Show this help"
    echo ""
    echo "Examples:"
    echo "  ./deploy.sh setup           # Initial setup"
    echo "  ./deploy.sh start --build   # Build and start"
    echo "  ./deploy.sh diagnose        # Full diagnostics"
    echo "  ./deploy.sh logs api        # View API logs"
    echo ""
    echo "All commands are idempotent - safe to run multiple times."
    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    COMMAND="${1:-help}"
    shift || true

    case "$COMMAND" in
        setup)
            cmd_setup "$@"
            ;;
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
        health)
            cmd_health
            ;;
        diagnose|diag)
            cmd_diagnose
            ;;
        logs)
            cmd_logs "$@"
            ;;
        ssh-fix|sshfix)
            cmd_ssh_fix
            ;;
        network|net)
            cmd_network
            ;;
        clean)
            cmd_clean
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
