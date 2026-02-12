#!/bin/bash
# =============================================================================
# Jetson Monitor - Watch Docker, SSH, and System Activity
# =============================================================================
# Auto-detects if running on the Jetson or remotely.
# When on Jetson: runs commands locally (no SSH needed)
# When remote: runs commands via SSH to Jetson
#
# Usage:
#   ./jetson-monitor.sh docker          # Watch Docker containers (default)
#   ./jetson-monitor.sh logs [service]  # Follow Docker Compose logs
#   ./jetson-monitor.sh ssh             # Watch active SSH/remote connections
#   ./jetson-monitor.sh system          # Watch CPU/mem/GPU usage
#   ./jetson-monitor.sh rag             # Watch RAG stack health
#   ./jetson-monitor.sh all             # Combined dashboard (tmux-like)
#   ./jetson-monitor.sh --help          # Show help
#
# Options:
#   -n SECONDS    Refresh interval (default: 2)
#   --local       Force local mode (skip auto-detection)
#   --remote      Force remote mode via SSH
#
# Examples:
#   ./jetson-monitor.sh docker              # Watch containers
#   ./jetson-monitor.sh docker -n 5         # Refresh every 5s
#   ./jetson-monitor.sh logs api            # Follow API container logs
#   ./jetson-monitor.sh logs                # Follow all RAG logs
#   ./jetson-monitor.sh ssh                 # Watch remote connections
#   ./jetson-monitor.sh system              # System resources
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_HOST="${JETSON_HOST:-jetson}"
INTERVAL=2
LOCAL_MODE=""  # empty = auto-detect

# Auto-detect: are we on a Jetson/Tegra device?
detect_jetson() {
    # Check for Tegra kernel (Jetson signature)
    if uname -r 2>/dev/null | grep -q tegra; then
        return 0
    fi
    # Check for NVIDIA Jetson in device tree
    if [ -f /proc/device-tree/model ] && grep -qi "jetson" /proc/device-tree/model 2>/dev/null; then
        return 0
    fi
    # Check for Jetson-specific GPU path
    if [ -d /sys/devices/17000000.ga10b ] || [ -d /sys/devices/gpu.0 ]; then
        return 0
    fi
    return 1
}

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

show_help() {
    echo "Usage: $0 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  docker          Watch Docker containers (ps + stats)"
    echo "  logs [service]  Follow Docker Compose logs (api, postgres, redis, or all)"
    echo "  ssh             Watch active SSH and remote connections"
    echo "  system          Watch CPU, memory, GPU, and disk usage"
    echo "  rag             Watch RAG stack health (API + Ollama)"
    echo "  all             Combined overview (docker + ssh + system)"
    echo ""
    echo "Options:"
    echo "  -n SECONDS      Refresh interval (default: 2)"
    echo "  --local         Force local mode (when ON the Jetson)"
    echo "  --remote        Force remote mode (SSH to Jetson)"
    echo "  --help          Show this help"
    echo ""
    echo "Auto-detection:"
    echo "  The script auto-detects if running on a Jetson (Tegra kernel)."
    echo "  On Jetson: runs commands locally. Elsewhere: uses SSH."
    echo ""
    echo "Environment:"
    echo "  JETSON_HOST     SSH host (default: jetson, from ~/.ssh/config)"
    echo ""
    exit 0
}

# Run command on Jetson (or locally)
run_cmd() {
    if [ "$LOCAL_MODE" = true ]; then
        eval "$@"
    else
        ssh -o ConnectTimeout=5 "$JETSON_HOST" "$@"
    fi
}

# =============================================================================
# Docker Watch
# =============================================================================
watch_docker() {
    local mode_label; [ "$LOCAL_MODE" = true ] && mode_label="local" || mode_label="via SSH → ${JETSON_HOST}"
    echo -e "${CYAN}Watching Docker containers (${mode_label}) (Ctrl+C to stop)${NC}"
    echo ""
    while true; do
        # Buffer all output before displaying (prevents jagged incremental rendering)
        local buffer=""
        buffer+="$(echo -e "${BLUE}=== Docker Containers === $(date '+%H:%M:%S') ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(run_cmd "docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}\t{{.Image}}'" 2>/dev/null || \
            run_cmd "sudo docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}\t{{.Image}}'" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${BLUE}=== Docker Stats ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(run_cmd "docker stats --no-stream --format 'table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.BlockIO}}'" 2>/dev/null || \
            run_cmd "sudo docker stats --no-stream --format 'table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.BlockIO}}'" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${YELLOW}Refreshing every ${INTERVAL}s... (Ctrl+C to stop)${NC}")"
        clear
        echo -e "$buffer"
        sleep "$INTERVAL"
    done
}

# =============================================================================
# Docker Compose Logs
# =============================================================================
watch_logs() {
    local service="$1"
    local mode_label; [ "$LOCAL_MODE" = true ] && mode_label="local" || mode_label="via SSH → ${JETSON_HOST}"
    echo -e "${CYAN}Following Docker Compose logs (${mode_label}) (Ctrl+C to stop)${NC}"
    echo ""
    if [ -n "$service" ]; then
        run_cmd "docker compose -f $RAG_COMPOSE logs -f --tail=50 $service" 2>/dev/null || \
            run_cmd "sudo docker compose -f $RAG_COMPOSE logs -f --tail=50 $service" 2>/dev/null
    else
        run_cmd "docker compose -f $RAG_COMPOSE logs -f --tail=50" 2>/dev/null || \
            run_cmd "sudo docker compose -f $RAG_COMPOSE logs -f --tail=50" 2>/dev/null
    fi
}

# =============================================================================
# SSH / Remote Connections Watch
# =============================================================================
watch_ssh() {
    local mode_label; [ "$LOCAL_MODE" = true ] && mode_label="local" || mode_label="via SSH → ${JETSON_HOST}"
    echo -e "${CYAN}Watching SSH/remote connections (${mode_label}) (Ctrl+C to stop)${NC}"
    echo ""
    while true; do
        local buffer=""
        buffer+="$(echo -e "${BLUE}=== Active SSH Sessions === $(date '+%H:%M:%S') ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(run_cmd "who -a 2>/dev/null | head -20" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${BLUE}=== SSH Connections (ss) ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(run_cmd "ss -tnp 2>/dev/null | grep -E ':22\b' | head -20" 2>/dev/null || \
            run_cmd "sudo ss -tnp 2>/dev/null | grep -E ':22\b' | head -20" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${BLUE}=== Listening Services ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(run_cmd "ss -tlnp 2>/dev/null | grep -E '(LISTEN|State)'" 2>/dev/null || \
            run_cmd "sudo ss -tlnp 2>/dev/null | grep -E '(LISTEN|State)'" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${YELLOW}Refreshing every ${INTERVAL}s... (Ctrl+C to stop)${NC}")"
        clear
        echo -e "$buffer"
        sleep "$INTERVAL"
    done
}

# =============================================================================
# System Resources Watch
# =============================================================================
watch_system() {
    local mode_label; [ "$LOCAL_MODE" = true ] && mode_label="local" || mode_label="via SSH → ${JETSON_HOST}"
    echo -e "${CYAN}Watching system resources (${mode_label}) (Ctrl+C to stop)${NC}"
    echo ""
    while true; do
        local buffer=""
        buffer+="$(echo -e "${BLUE}=== System Resources === $(date '+%H:%M:%S') ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(run_cmd "
            echo '--- CPU ---'
            uptime
            echo ''
            echo '--- Memory ---'
            free -h | head -3
            echo ''
            echo '--- GPU ---'
            if [ -f /sys/devices/gpu.0/load ]; then
                echo \"GPU Load: \$(cat /sys/devices/gpu.0/load)\"
            elif [ -f /sys/devices/17000000.ga10b/load ]; then
                echo \"GPU Load: \$(cat /sys/devices/17000000.ga10b/load)\"
            else
                echo 'GPU load file not found'
            fi
            echo ''
            echo '--- Disk ---'
            df -h / | tail -1
            echo ''
            echo '--- Top Processes ---'
            ps aux --sort=-%mem | head -8
            echo ''
            echo '--- Jetson Power Mode ---'
            nvpmodel -q 2>/dev/null || echo 'nvpmodel not available'
        " 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${YELLOW}Refreshing every ${INTERVAL}s... (Ctrl+C to stop)${NC}")"
        clear
        echo -e "$buffer"
        sleep "$INTERVAL"
    done
}

# =============================================================================
# RAG Health Watch
# =============================================================================
watch_rag() {
    local mode_label; [ "$LOCAL_MODE" = true ] && mode_label="local" || mode_label="via SSH → ${JETSON_HOST}"
    echo -e "${CYAN}Watching RAG stack health (${mode_label}) (Ctrl+C to stop)${NC}"
    echo ""
    while true; do
        local buffer=""
        buffer+="$(echo -e "${BLUE}=== RAG Stack Health === $(date '+%H:%M:%S') ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${BLUE}--- API Health ---${NC}")"$'\n'
        buffer+="$(run_cmd "curl -s http://localhost:8000/api/health 2>/dev/null | python3 -m json.tool 2>/dev/null || echo 'API not responding'" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${BLUE}--- Ollama ---${NC}")"$'\n'
        buffer+="$(run_cmd "curl -s http://localhost:11434/api/tags 2>/dev/null | python3 -c 'import json,sys; d=json.load(sys.stdin); [print(f\"  {m[\"name\"]} ({m[\"details\"][\"parameter_size\"]}, {m[\"size\"]/1e6:.0f}MB)\") for m in d[\"models\"]]' 2>/dev/null || echo 'Ollama not responding'" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${BLUE}--- Containers ---${NC}")"$'\n'
        buffer+="$(run_cmd "docker compose -f $RAG_COMPOSE ps --format 'table {{.Name}}\t{{.Status}}\t{{.Service}}'" 2>/dev/null || \
            run_cmd "sudo docker compose -f $RAG_COMPOSE ps --format 'table {{.Name}}\t{{.Status}}\t{{.Service}}'" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${BLUE}--- Documents ---${NC}")"$'\n'
        buffer+="$(run_cmd "curl -s http://localhost:8000/api/documents 2>/dev/null | python3 -c 'import json,sys; d=json.load(sys.stdin); print(f\"  {len(d)} documents ingested\")' 2>/dev/null || echo '  Could not query documents'" 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${YELLOW}Refreshing every ${INTERVAL}s... (Ctrl+C to stop)${NC}")"
        clear
        echo -e "$buffer"
        sleep "$INTERVAL"
    done
}

# =============================================================================
# Combined Overview
# =============================================================================
watch_all() {
    local mode_label; [ "$LOCAL_MODE" = true ] && mode_label="local" || mode_label="via SSH → ${JETSON_HOST}"
    echo -e "${CYAN}Combined overview (${mode_label}) (Ctrl+C to stop)${NC}"
    echo ""
    while true; do
        local buffer=""
        buffer+="$(echo -e "${BLUE}=== Jetson Overview === $(date '+%H:%M:%S') ===${NC}")"$'\n'
        buffer+=$'\n'
        buffer+="$(run_cmd "
            echo '--- System ---'
            uptime
            free -h | grep Mem
            df -h / | tail -1
            echo ''
            echo '--- Docker ---'
            docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}' 2>/dev/null || \
                sudo docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}' 2>/dev/null
            echo ''
            echo '--- Connections ---'
            who 2>/dev/null
            echo ''
            echo '--- RAG API ---'
            curl -s --max-time 2 http://localhost:8000/api/health 2>/dev/null || echo 'API not responding'
            echo ''
            echo '--- Ollama ---'
            curl -s --max-time 2 http://localhost:11434/api/tags 2>/dev/null | python3 -c 'import json,sys; d=json.load(sys.stdin); [print(f\"  {m[\"name\"]}\") for m in d[\"models\"]]' 2>/dev/null || echo 'Ollama not responding'
        " 2>/dev/null)"$'\n'
        buffer+=$'\n'
        buffer+="$(echo -e "${YELLOW}Refreshing every ${INTERVAL}s... (Ctrl+C to stop)${NC}")"
        clear
        echo -e "$buffer"
        sleep "$INTERVAL"
    done
}

# =============================================================================
# Parse Arguments
# =============================================================================
COMMAND=""
LOG_SERVICE=""

while [ $# -gt 0 ]; do
    case "$1" in
        docker|ssh|system|rag|all)
            COMMAND="$1"
            shift
            ;;
        logs)
            COMMAND="logs"
            shift
            # Next arg might be a service name
            if [ $# -gt 0 ] && [[ ! "$1" =~ ^- ]]; then
                LOG_SERVICE="$1"
                shift
            fi
            ;;
        -n)
            shift
            INTERVAL="${1:-2}"
            shift
            ;;
        --local)
            LOCAL_MODE=true
            shift
            ;;
        --remote)
            LOCAL_MODE=false
            shift
            ;;
        --help|-h)
            show_help
            ;;
        *)
            echo "Unknown argument: $1 (use --help)"
            exit 1
            ;;
    esac
done

# Default command
if [ -z "$COMMAND" ]; then
    COMMAND="docker"
fi

# Auto-detect platform if not forced via --local or --remote
if [ -z "$LOCAL_MODE" ]; then
    if detect_jetson; then
        LOCAL_MODE=true
        echo -e "${GREEN}Detected Jetson platform — running locally${NC}"
    else
        LOCAL_MODE=false
    fi
fi

# Set compose path based on mode
if [ "$LOCAL_MODE" = true ]; then
    RAG_COMPOSE="$HOME/ambot/bootylicious/rag/docker-compose.yml"
else
    RAG_COMPOSE="~/ambot/bootylicious/rag/docker-compose.yml"
fi

# Check SSH connectivity (unless local mode)
if [ "$LOCAL_MODE" = false ]; then
    if ! ssh -o ConnectTimeout=3 -o BatchMode=yes "$JETSON_HOST" exit 2>/dev/null; then
        echo -e "${RED}Cannot connect to ${JETSON_HOST}${NC}"
        echo ""
        echo "Options:"
        echo "  1. Ensure SSH is configured: ssh $JETSON_HOST"
        echo "  2. Run with --local if you're ON the Jetson"
        echo "  3. Set JETSON_HOST=<ip> if using a different address"
        exit 1
    fi
fi

# Dispatch
case "$COMMAND" in
    docker)  watch_docker ;;
    logs)    watch_logs "$LOG_SERVICE" ;;
    ssh)     watch_ssh ;;
    system)  watch_system ;;
    rag)     watch_rag ;;
    all)     watch_all ;;
esac
