#!/bin/bash
# =============================================================================
# Pathfinder - LiDAR Deployment Script
# =============================================================================
# Deployment and management script for the Pathfinder LiDAR system.
# Handles setup, diagnostics, and service management on Raspberry Pi.
#
# Usage:
#   ./deploy.sh <command>
#
# Commands:
#   setup       Run initial system setup (Python, dependencies, udev)
#   start       Start LiDAR service
#   stop        Stop LiDAR service
#   status      Show system and device status
#   diagnose    Run comprehensive diagnostics
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
PROJECT_NAME="pathfinder"
SCRIPTS_DIR="$SCRIPT_DIR/scripts"
LOG_DIR="$SCRIPT_DIR/logs"
STATE_FILE="$SCRIPT_DIR/.deploy_state"
PID_FILE="$SCRIPT_DIR/.lidar.pid"

# LiDAR device paths
LIDAR_SYMLINK="/dev/rplidar"
LIDAR_FALLBACK="/dev/ttyUSB0"

# Create directories
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
log_success() { echo -e "${GREEN}[OK]${NC} $1"; }
log_fail()    { echo -e "${RED}[FAIL]${NC} $1"; }

# =============================================================================
# State Management (for idempotency)
# =============================================================================
save_state() {
    echo "$1=$(date '+%Y-%m-%d %H:%M:%S')" >> "$STATE_FILE"
}

check_state() {
    grep -q "^$1=" "$STATE_FILE" 2>/dev/null
}

# =============================================================================
# System Checks
# =============================================================================
check_python() {
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
        log_success "Python 3: $PYTHON_VERSION"
        return 0
    else
        log_fail "Python 3: Not installed"
        return 1
    fi
}

check_pyserial() {
    if python3 -c "import serial" 2>/dev/null; then
        log_success "pyserial: Installed"
        return 0
    else
        log_fail "pyserial: Not installed"
        return 1
    fi
}

check_numpy() {
    if python3 -c "import numpy" 2>/dev/null; then
        log_success "numpy: Installed"
        return 0
    else
        log_warn "numpy: Not installed (optional)"
        return 1
    fi
}

check_matplotlib() {
    if python3 -c "import matplotlib" 2>/dev/null; then
        log_success "matplotlib: Installed"
        return 0
    else
        log_warn "matplotlib: Not installed (optional, for visualization)"
        return 1
    fi
}

check_lidar_device() {
    # Check for symlink first
    if [ -e "$LIDAR_SYMLINK" ]; then
        REAL_PATH=$(readlink -f "$LIDAR_SYMLINK")
        log_success "LiDAR device: $LIDAR_SYMLINK -> $REAL_PATH"
        return 0
    fi

    # Check for fallback
    if [ -e "$LIDAR_FALLBACK" ]; then
        log_warn "LiDAR device: $LIDAR_FALLBACK (no symlink)"
        log_info "  Run './deploy.sh setup' to create udev rules"
        return 0
    fi

    # Check for any ttyUSB
    for i in $(seq 0 9); do
        if [ -e "/dev/ttyUSB$i" ]; then
            log_warn "LiDAR device: /dev/ttyUSB$i (no symlink)"
            return 0
        fi
    done

    log_fail "LiDAR device: Not found"
    log_info "  Check USB connection"
    return 1
}

check_lidar_permissions() {
    local device=""

    if [ -e "$LIDAR_SYMLINK" ]; then
        device="$LIDAR_SYMLINK"
    elif [ -e "$LIDAR_FALLBACK" ]; then
        device="$LIDAR_FALLBACK"
    else
        return 1
    fi

    if [ -r "$device" ] && [ -w "$device" ]; then
        log_success "Permissions: Read/Write access to $device"
        return 0
    else
        log_fail "Permissions: No access to $device"
        log_info "  Run: sudo chmod 666 $device"
        log_info "  Or:  sudo usermod -a -G dialout \$USER (then logout/login)"
        return 1
    fi
}

check_udev_rules() {
    if [ -f /etc/udev/rules.d/99-rplidar.rules ]; then
        log_success "udev rules: Installed"
        return 0
    else
        log_warn "udev rules: Not installed"
        log_info "  Run './deploy.sh setup' to install"
        return 1
    fi
}

check_user_dialout() {
    if groups | grep -q dialout; then
        log_success "User in dialout group: Yes"
        return 0
    else
        log_warn "User in dialout group: No"
        log_info "  Run: sudo usermod -a -G dialout \$USER"
        return 1
    fi
}

check_usb_device() {
    # Check for Silicon Labs CP210x (RPLidar USB chip)
    if lsusb 2>/dev/null | grep -qi "10c4:ea60\|silicon.*labs\|cp210"; then
        log_success "USB device: Silicon Labs CP210x detected"
        return 0
    else
        log_warn "USB device: CP210x not detected in lsusb"
        return 1
    fi
}

# =============================================================================
# Commands
# =============================================================================
cmd_setup() {
    log_step "Running Pathfinder Setup..."
    echo ""
    echo "This will:"
    echo "  1. Check Python 3 installation"
    echo "  2. Install Python dependencies"
    echo "  3. Setup udev rules for LiDAR"
    echo ""

    # Check Python
    log_step "Checking Python..."
    if ! check_python; then
        log_error "Python 3 is required. Install with: sudo apt install python3"
        exit 1
    fi

    # Install dependencies
    log_step "Installing Python dependencies..."
    if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
        if pip3 install -r "$SCRIPT_DIR/requirements.txt" --user 2>/dev/null || \
           pip3 install -r "$SCRIPT_DIR/requirements.txt" 2>/dev/null; then
            log_success "Python dependencies installed"
        else
            log_warn "Some dependencies failed to install"
            log_info "  Try: pip3 install pyserial numpy"
        fi
    fi

    # Check pyserial
    if ! check_pyserial; then
        log_info "Installing pyserial..."
        pip3 install pyserial --user 2>/dev/null || pip3 install pyserial 2>/dev/null
    fi

    # Setup udev rules
    log_step "Setting up udev rules..."
    if [ -f "$SCRIPTS_DIR/setup-udev.sh" ]; then
        if [ "$EUID" -eq 0 ]; then
            bash "$SCRIPTS_DIR/setup-udev.sh"
        else
            log_info "udev setup requires sudo. Run:"
            echo "  sudo bash $SCRIPTS_DIR/setup-udev.sh"
            echo ""
            read -p "Run now with sudo? (Y/n): " run_udev
            if [[ ! "$run_udev" =~ ^[Nn]$ ]]; then
                sudo bash "$SCRIPTS_DIR/setup-udev.sh"
            fi
        fi
    else
        log_warn "setup-udev.sh not found"
    fi

    # Check dialout group
    log_step "Checking user permissions..."
    if ! check_user_dialout; then
        read -p "Add user to dialout group? (Y/n): " add_dialout
        if [[ ! "$add_dialout" =~ ^[Nn]$ ]]; then
            sudo usermod -a -G dialout "$USER"
            log_success "Added $USER to dialout group"
            log_warn "You must logout and login for this to take effect"
        fi
    fi

    save_state "setup_complete"
    echo ""
    log_success "Setup complete!"
    echo ""
    log_info "Next steps:"
    echo "  1. Plug in the LiDAR via USB"
    echo "  2. Run: ./deploy.sh status"
    echo "  3. Run: python3 test_lidar.py --check"
}

cmd_start() {
    log_step "Starting LiDAR service..."

    # Check prerequisites
    if ! check_lidar_device &>/dev/null; then
        log_error "LiDAR device not found. Check USB connection."
        exit 1
    fi

    if ! check_lidar_permissions &>/dev/null; then
        log_error "No permission to access LiDAR. Run './deploy.sh setup'"
        exit 1
    fi

    # Check if already running
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if kill -0 "$PID" 2>/dev/null; then
            log_warn "LiDAR service already running (PID: $PID)"
            return 0
        else
            rm -f "$PID_FILE"
        fi
    fi

    # Start continuous detection in background
    log_info "Starting continuous obstacle detection..."
    nohup python3 "$SCRIPT_DIR/test_lidar.py" --continuous > "$LOG_DIR/lidar.log" 2>&1 &
    echo $! > "$PID_FILE"

    sleep 1

    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        log_success "LiDAR service started (PID: $(cat "$PID_FILE"))"
        log_info "Logs: $LOG_DIR/lidar.log"
    else
        log_fail "Failed to start LiDAR service"
        log_info "Check logs: $LOG_DIR/lidar.log"
        exit 1
    fi
}

cmd_stop() {
    log_step "Stopping LiDAR service..."

    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if kill -0 "$PID" 2>/dev/null; then
            kill "$PID"
            sleep 1
            if kill -0 "$PID" 2>/dev/null; then
                kill -9 "$PID" 2>/dev/null
            fi
            log_success "LiDAR service stopped (was PID: $PID)"
        else
            log_info "Service was not running"
        fi
        rm -f "$PID_FILE"
    else
        log_info "No PID file found"
    fi
}

cmd_status() {
    echo ""
    echo "========================================"
    echo "  Pathfinder LiDAR Status"
    echo "========================================"
    echo ""

    log_step "System"
    echo "  Hostname: $(hostname)"
    echo "  User: $(whoami)"
    echo "  Platform: $(uname -m)"
    echo ""

    log_step "Software"
    check_python || true
    check_pyserial || true
    check_numpy || true
    check_matplotlib || true
    echo ""

    log_step "Hardware"
    check_usb_device || true
    check_lidar_device || true
    check_lidar_permissions || true
    echo ""

    log_step "Configuration"
    check_udev_rules || true
    check_user_dialout || true
    echo ""

    log_step "Service"
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if kill -0 "$PID" 2>/dev/null; then
            log_success "LiDAR service: Running (PID: $PID)"
        else
            log_warn "LiDAR service: Stale PID file"
        fi
    else
        log_info "LiDAR service: Not running"
    fi
    echo ""

    echo "========================================"
}

cmd_diagnose() {
    echo ""
    echo "========================================"
    echo "  Pathfinder Comprehensive Diagnostics"
    echo "========================================"
    echo "  Time: $(date)"
    echo "========================================"
    echo ""

    # System info
    log_step "System Information"
    echo "  Hostname: $(hostname)"
    echo "  User: $(whoami)"
    echo "  OS: $(cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d'"' -f2 || uname -s)"
    echo "  Kernel: $(uname -r)"
    echo "  Architecture: $(uname -m)"
    echo ""

    # Check if Raspberry Pi
    if [ -f /proc/device-tree/model ]; then
        log_step "Hardware"
        echo "  Model: $(cat /proc/device-tree/model | tr '\0' '\n')"
        echo ""
    fi

    # USB devices
    log_step "USB Devices"
    echo "  Relevant USB devices:"
    lsusb 2>/dev/null | grep -iE "silicon|cp210|ftdi|ch340|serial" | sed 's/^/    /' || echo "    (none found)"
    echo ""

    # Serial ports
    log_step "Serial Ports"
    echo "  Available ports:"
    ls -la /dev/ttyUSB* /dev/ttyACM* /dev/rplidar 2>/dev/null | sed 's/^/    /' || echo "    (none found)"
    echo ""

    # Software checks
    log_step "Software Checks"
    check_python || true
    check_pyserial || true
    check_numpy || true
    check_matplotlib || true
    echo ""

    # Hardware checks
    log_step "Hardware Checks"
    check_usb_device || true
    check_lidar_device || true
    check_lidar_permissions || true
    echo ""

    # Configuration checks
    log_step "Configuration Checks"
    check_udev_rules || true
    check_user_dialout || true
    echo ""

    # udev rules content
    if [ -f /etc/udev/rules.d/99-rplidar.rules ]; then
        log_step "udev Rules Content"
        cat /etc/udev/rules.d/99-rplidar.rules | sed 's/^/    /'
        echo ""
    fi

    # Try quick LiDAR check
    log_step "LiDAR Quick Check"
    if python3 "$SCRIPT_DIR/test_lidar.py" --check 2>/dev/null; then
        log_success "LiDAR check passed"
    else
        log_warn "LiDAR check failed (see above for details)"
    fi
    echo ""

    # Recent logs
    log_step "Recent Logs"
    if [ -f "$LOG_DIR/lidar.log" ]; then
        echo "  Last 10 lines of lidar.log:"
        tail -10 "$LOG_DIR/lidar.log" 2>/dev/null | sed 's/^/    /'
    else
        echo "  (no log file)"
    fi
    echo ""

    echo "========================================"
    echo "  Diagnostics Complete"
    echo "========================================"
}

cmd_help() {
    echo ""
    echo "Pathfinder - LiDAR Deployment Script"
    echo ""
    echo "Usage: ./deploy.sh <command>"
    echo ""
    echo "Commands:"
    echo "  setup       Run initial system setup (idempotent)"
    echo "              - Check Python 3"
    echo "              - Install pyserial, numpy"
    echo "              - Setup udev rules for /dev/rplidar"
    echo ""
    echo "  start       Start LiDAR obstacle detection service"
    echo "  stop        Stop LiDAR service"
    echo ""
    echo "  status      Show system and device status"
    echo "  diagnose    Run comprehensive diagnostics"
    echo ""
    echo "  help        Show this help"
    echo ""
    echo "Examples:"
    echo "  ./deploy.sh setup      # Initial setup"
    echo "  ./deploy.sh status     # Check everything"
    echo "  ./deploy.sh diagnose   # Full diagnostics"
    echo ""
    echo "Testing:"
    echo "  python3 test_lidar.py --check       # Verify device"
    echo "  python3 test_lidar.py --scan        # Single scan"
    echo "  python3 test_lidar.py --visualize   # Polar plot"
    echo "  python3 test_lidar.py --continuous  # Obstacle detection"
    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    COMMAND="${1:-help}"

    case "$COMMAND" in
        setup)
            cmd_setup
            ;;
        start)
            cmd_start
            ;;
        stop)
            cmd_stop
            ;;
        status)
            cmd_status
            ;;
        diagnose|diag)
            cmd_diagnose
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
