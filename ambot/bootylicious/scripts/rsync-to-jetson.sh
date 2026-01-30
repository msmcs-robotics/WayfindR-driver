#!/bin/bash
# =============================================================================
# Bootylicious - Rsync to Jetson Orin Nano
# =============================================================================
# Syncs the bootylicious folder to ~/bootylicious on the Jetson.
# The entire folder structure is preserved for easy organization.
#
# Usage:
#   ./rsync-to-jetson.sh [options] [jetson_ip]
#
# Options:
#   --watch     Continuously sync on file changes
#   --dry-run   Show what would be synced without doing it
#   --delete    Delete files on Jetson that don't exist locally
#   --help      Show help
#
# Examples:
#   ./rsync-to-jetson.sh                    # Sync to default IP
#   ./rsync-to-jetson.sh 192.168.1.50       # Sync to specific IP
#   ./rsync-to-jetson.sh --watch            # Watch for changes
#   ./rsync-to-jetson.sh --dry-run          # Preview sync
#
# Environment Variables:
#   JETSON_USER  - Username on Jetson (default: ambot)
#   JETSON_IP    - IP address (can also be passed as argument)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOOTYLICIOUS_DIR="$(dirname "$SCRIPT_DIR")"  # Parent: bootylicious/

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()    { echo -e "${BLUE}[====]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }

# =============================================================================
# Configuration
# =============================================================================
JETSON_USER="${JETSON_USER:-ambot}"
JETSON_IP="${JETSON_IP:-10.33.183.100}"
REMOTE_DIR="~/bootylicious"

# Parse arguments
DRY_RUN=false
WATCH=false
DELETE=false

for arg in "$@"; do
    case $arg in
        --dry-run)
            DRY_RUN=true
            ;;
        --watch)
            WATCH=true
            ;;
        --delete)
            DELETE=true
            ;;
        --help|-h)
            echo "Usage: $0 [options] [jetson_ip]"
            echo ""
            echo "Syncs bootylicious folder to ~/bootylicious on Jetson"
            echo ""
            echo "Options:"
            echo "  --watch     Continuously sync on file changes"
            echo "  --dry-run   Show what would be synced without doing it"
            echo "  --delete    Delete files on Jetson that don't exist locally"
            echo "  --help      Show this help"
            echo ""
            echo "Environment Variables:"
            echo "  JETSON_USER  Username (default: ambot)"
            echo "  JETSON_IP    IP address (default: 10.33.183.100)"
            echo ""
            exit 0
            ;;
        -*)
            log_error "Unknown option: $arg"
            exit 1
            ;;
        *)
            # Assume it's an IP address
            JETSON_IP="$arg"
            ;;
    esac
done

# =============================================================================
# SSH Connection Check
# =============================================================================
check_ssh() {
    log_info "Checking SSH connection to ${JETSON_USER}@${JETSON_IP}..."

    # First try with BatchMode (no password prompt)
    if ssh -o ConnectTimeout=5 -o BatchMode=yes "${JETSON_USER}@${JETSON_IP}" exit 2>/dev/null; then
        log_success "SSH connection successful (key-based auth)"
        return 0
    fi

    # If that fails, check if host is reachable at all
    if ping -c 1 -W 2 "$JETSON_IP" &>/dev/null; then
        log_warn "Host reachable but SSH key auth failed"
        echo ""
        echo "SSH keys not set up. Run these commands:"
        echo ""
        echo "  # Generate key if you don't have one"
        echo "  ssh-keygen -t ed25519"
        echo ""
        echo "  # Copy key to Jetson"
        echo "  ssh-copy-id ${JETSON_USER}@${JETSON_IP}"
        echo ""
        echo "  # Then run this script again"
        echo ""

        read -p "Try with password authentication? (y/N): " try_pass
        if [[ "$try_pass" =~ ^[Yy]$ ]]; then
            if ssh -o ConnectTimeout=10 "${JETSON_USER}@${JETSON_IP}" exit; then
                log_success "SSH connection successful (password auth)"
                return 0
            fi
        fi
        return 1
    else
        log_error "Cannot reach ${JETSON_IP}"
        echo ""
        echo "Please ensure:"
        echo "  1. Jetson is powered on"
        echo "  2. Jetson is connected to the network"
        echo "  3. IP address is correct (current: ${JETSON_IP})"
        echo ""
        echo "To find Jetson IP, run on Jetson: hostname -I"
        echo ""
        return 1
    fi
}

# =============================================================================
# Pre-sync Checks
# =============================================================================
presync_checks() {
    log_step "Pre-sync Checks"

    # Check local directory exists
    if [ ! -d "$BOOTYLICIOUS_DIR" ]; then
        log_error "Source directory not found: $BOOTYLICIOUS_DIR"
        exit 1
    fi

    # Show what we're syncing
    LOCAL_SIZE=$(du -sh "$BOOTYLICIOUS_DIR" 2>/dev/null | cut -f1)
    LOCAL_FILES=$(find "$BOOTYLICIOUS_DIR" -type f | wc -l)
    log_info "Local: $BOOTYLICIOUS_DIR ($LOCAL_SIZE, $LOCAL_FILES files)"

    # Check remote directory (create if needed)
    ssh "${JETSON_USER}@${JETSON_IP}" "mkdir -p $REMOTE_DIR"
    log_info "Remote: ${JETSON_USER}@${JETSON_IP}:$REMOTE_DIR"

    echo ""
}

# =============================================================================
# Rsync
# =============================================================================
sync_files() {
    log_step "Syncing files..."

    # Build rsync options
    RSYNC_OPTS="-avz --progress"

    # Add optional flags
    if [ "$DRY_RUN" = true ]; then
        RSYNC_OPTS="$RSYNC_OPTS --dry-run"
        log_warn "DRY RUN - no files will be transferred"
    fi

    if [ "$DELETE" = true ]; then
        RSYNC_OPTS="$RSYNC_OPTS --delete"
        log_warn "DELETE mode - files not in source will be removed from destination"
    fi

    # Excludes
    RSYNC_OPTS="$RSYNC_OPTS \
        --exclude '__pycache__' \
        --exclude '*.pyc' \
        --exclude '.git' \
        --exclude '.venv' \
        --exclude 'venv' \
        --exclude 'node_modules' \
        --exclude '*.egg-info' \
        --exclude '.pytest_cache' \
        --exclude '.mypy_cache' \
        --exclude '*.log' \
        --exclude '.DS_Store' \
        --exclude 'logs/*.log'"

    # Run rsync
    eval rsync $RSYNC_OPTS \
        "${BOOTYLICIOUS_DIR}/" \
        "${JETSON_USER}@${JETSON_IP}:${REMOTE_DIR}/"

    log_success "Sync complete!"
}

# =============================================================================
# Post-sync Setup
# =============================================================================
postsync_setup() {
    log_step "Post-sync Setup"

    # Make scripts executable on Jetson
    ssh "${JETSON_USER}@${JETSON_IP}" "
        chmod +x ${REMOTE_DIR}/deploy.sh 2>/dev/null || true
        chmod +x ${REMOTE_DIR}/scripts/*.sh 2>/dev/null || true
        chmod +x ${REMOTE_DIR}/rag/deploy.sh 2>/dev/null || true
    "
    log_success "Scripts are executable"

    # Create logs directory
    ssh "${JETSON_USER}@${JETSON_IP}" "mkdir -p ${REMOTE_DIR}/logs"
}

# =============================================================================
# Show Next Steps
# =============================================================================
show_next_steps() {
    echo ""
    echo "========================================"
    echo "  Sync Complete!"
    echo "========================================"
    echo ""
    echo "Files synced to: ${JETSON_USER}@${JETSON_IP}:~/bootylicious"
    echo ""
    echo "Next steps on the Jetson:"
    echo ""
    echo "  1. SSH into the Jetson:"
    echo "     ssh ${JETSON_USER}@${JETSON_IP}"
    echo ""
    echo "  2. Run the master deploy script:"
    echo "     cd ~/bootylicious"
    echo "     ./deploy.sh setup      # Initial setup"
    echo "     ./deploy.sh start      # Start services"
    echo "     ./deploy.sh diagnose   # Run diagnostics"
    echo ""
    echo "  3. Add EECS documents:"
    echo "     cp /path/to/docs/* ~/bootylicious/rag/knowledge/"
    echo ""
    echo "  4. Ingest documents:"
    echo "     cd ~/bootylicious/rag"
    echo "     ./deploy.sh ingest"
    echo ""
    echo "========================================"
}

# =============================================================================
# Watch Mode
# =============================================================================
watch_mode() {
    log_step "Watch Mode"
    log_info "Watching for changes... (Ctrl+C to stop)"
    echo ""

    # Check for fswatch (macOS) or inotifywait (Linux)
    if command -v fswatch &> /dev/null; then
        fswatch -o "$BOOTYLICIOUS_DIR" \
            --exclude '__pycache__' \
            --exclude '\.git' \
            --exclude '\.pyc$' \
            | while read change; do
                log_info "Changes detected, syncing..."
                sync_files
                echo ""
                log_info "Watching for changes..."
            done
    elif command -v inotifywait &> /dev/null; then
        while inotifywait -r -e modify,create,delete,move \
            --exclude '__pycache__|\.git|\.pyc' \
            "$BOOTYLICIOUS_DIR" 2>/dev/null; do
            log_info "Changes detected, syncing..."
            sync_files
            echo ""
            log_info "Watching for changes..."
        done
    else
        log_error "Watch mode requires 'fswatch' or 'inotifywait'"
        echo ""
        echo "Install with:"
        echo "  macOS:  brew install fswatch"
        echo "  Ubuntu: sudo apt install inotify-tools"
        echo ""
        exit 1
    fi
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "  Bootylicious → Jetson Rsync"
    echo "========================================"
    echo ""

    check_ssh || exit 1
    presync_checks
    sync_files

    if [ "$DRY_RUN" = false ]; then
        postsync_setup
    fi

    if [ "$WATCH" = true ]; then
        echo ""
        watch_mode
    else
        show_next_steps
    fi
}

main "$@"
