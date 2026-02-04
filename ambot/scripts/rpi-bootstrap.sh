#!/bin/bash
# =============================================================================
# Raspberry Pi Master Bootstrap Script
# =============================================================================
# Complete bootstrap for ambot on Raspberry Pi.
# Runs both system packages and Python packages installation.
# This script is IDEMPOTENT - safe to run multiple times.
#
# Usage (on Raspberry Pi):
#   ./rpi-bootstrap.sh              # Interactive mode (prompts for sudo)
#   ./rpi-bootstrap.sh --auto       # Auto mode (uses stored password)
#
# Usage (from development machine via SSH):
#   ssh pi@10.33.224.1 "cd ~/ambot/scripts && ./rpi-bootstrap.sh"
#
# Or with password piped:
#   ssh pi@10.33.224.1 "echo 'erau' | sudo -S ~/ambot/scripts/rpi-bootstrap.sh"
#
# Options:
#   --auto       Run without prompts (for automation)
#   --system     Only run system packages bootstrap
#   --python     Only run Python packages bootstrap
#   --minimal    Minimal installation (Python only)
#   --verify     Only verify installation, don't install
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[====]${NC} $1"; }
log_header() { echo -e "${CYAN}$1${NC}"; }

# =============================================================================
# Parse Arguments
# =============================================================================
AUTO_MODE=false
SYSTEM_ONLY=false
PYTHON_ONLY=false
MINIMAL_MODE=false
VERIFY_ONLY=false

for arg in "$@"; do
    case $arg in
        --auto)
            AUTO_MODE=true
            ;;
        --system)
            SYSTEM_ONLY=true
            ;;
        --python)
            PYTHON_ONLY=true
            ;;
        --minimal)
            MINIMAL_MODE=true
            ;;
        --verify)
            VERIFY_ONLY=true
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --auto       Run without prompts (for automation)"
            echo "  --system     Only run system packages bootstrap"
            echo "  --python     Only run Python packages bootstrap"
            echo "  --minimal    Minimal installation (Python only)"
            echo "  --verify     Only verify installation, don't install"
            echo "  --help       Show this help message"
            exit 0
            ;;
    esac
done

# =============================================================================
# Print Banner
# =============================================================================
print_banner() {
    echo ""
    log_header "╔════════════════════════════════════════════════════════════╗"
    log_header "║                                                            ║"
    log_header "║     AMBOT - Raspberry Pi Bootstrap                         ║"
    log_header "║                                                            ║"
    log_header "║     Installs all dependencies for motor control,           ║"
    log_header "║     camera, LiDAR, and system utilities.                   ║"
    log_header "║                                                            ║"
    log_header "╚════════════════════════════════════════════════════════════╝"
    echo ""
}

# =============================================================================
# System Info
# =============================================================================
print_system_info() {
    log_step "System Information"
    echo ""

    if [ -f /proc/device-tree/model ]; then
        echo "  Model:        $(cat /proc/device-tree/model | tr -d '\0')"
    fi

    echo "  Hostname:     $(hostname)"
    echo "  User:         $(whoami)"
    echo "  Architecture: $(uname -m)"
    echo "  Kernel:       $(uname -r)"
    echo "  OS:           $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2 2>/dev/null || echo 'Unknown')"
    echo ""

    # Memory
    echo "  Memory:"
    free -h | head -2 | tail -1 | awk '{print "    Total: " $2 "  Used: " $3 "  Free: " $4}'
    echo ""

    # Disk
    echo "  Disk:"
    df -h / | tail -1 | awk '{print "    Total: " $2 "  Used: " $3 " (" $5 ")  Free: " $4}'
    echo ""
}

# =============================================================================
# Verify Installation
# =============================================================================
verify_installation() {
    log_step "Verifying Installation"
    echo ""

    ERRORS=0

    # System packages
    echo "System Packages:"
    for pkg in python3 git curl pip3; do
        if command -v "$pkg" &> /dev/null; then
            echo -e "  ${GREEN}[✓]${NC} $pkg"
        else
            echo -e "  ${RED}[✗]${NC} $pkg - NOT FOUND"
            ((ERRORS++))
        fi
    done
    echo ""

    # Python packages
    echo "Python Packages:"
    PACKAGES=(
        "RPi.GPIO:RPi.GPIO"
        "gpiozero:gpiozero"
        "serial:pyserial"
        "cv2:opencv"
        "numpy:numpy"
    )

    for item in "${PACKAGES[@]}"; do
        import_name="${item%%:*}"
        pkg_name="${item##*:}"

        if python3 -c "import $import_name" 2>/dev/null; then
            echo -e "  ${GREEN}[✓]${NC} $pkg_name"
        else
            echo -e "  ${RED}[✗]${NC} $pkg_name - NOT INSTALLED"
            ((ERRORS++))
        fi
    done
    echo ""

    # Hardware
    echo "Hardware:"
    if [ -e /dev/video0 ]; then
        echo -e "  ${GREEN}[✓]${NC} Camera (/dev/video0)"
    else
        echo -e "  ${YELLOW}[?]${NC} Camera - not detected"
    fi

    if [ -e /dev/ttyUSB0 ]; then
        echo -e "  ${GREEN}[✓]${NC} LiDAR (/dev/ttyUSB0)"
    else
        echo -e "  ${YELLOW}[?]${NC} LiDAR - not detected"
    fi

    if [ -e /dev/gpiomem ] || [ -e /dev/gpiochip0 ]; then
        echo -e "  ${GREEN}[✓]${NC} GPIO available"
    else
        echo -e "  ${RED}[✗]${NC} GPIO - not available"
        ((ERRORS++))
    fi
    echo ""

    # User groups
    echo "User Groups:"
    for grp in gpio dialout i2c spi; do
        if groups | grep -q "$grp"; then
            echo -e "  ${GREEN}[✓]${NC} $grp"
        else
            echo -e "  ${YELLOW}[?]${NC} $grp - not a member"
        fi
    done
    echo ""

    if [ $ERRORS -eq 0 ]; then
        log_info "All critical components verified!"
        return 0
    else
        log_error "$ERRORS critical components missing"
        return 1
    fi
}

# =============================================================================
# Run System Bootstrap
# =============================================================================
run_system_bootstrap() {
    log_step "Running System Packages Bootstrap..."
    echo ""

    SYSTEM_SCRIPT="$SCRIPT_DIR/rpi-bootstrap-system.sh"

    if [ ! -f "$SYSTEM_SCRIPT" ]; then
        log_error "System bootstrap script not found: $SYSTEM_SCRIPT"
        return 1
    fi

    chmod +x "$SYSTEM_SCRIPT"

    # Check if we need sudo
    if [ "$(id -u)" -eq 0 ]; then
        # Already root
        "$SYSTEM_SCRIPT"
    else
        # Need sudo
        if [ "$AUTO_MODE" = true ]; then
            # In auto mode, assume sudo is configured or password is piped
            sudo "$SYSTEM_SCRIPT"
        else
            log_info "System packages require sudo access."
            sudo "$SYSTEM_SCRIPT"
        fi
    fi
}

# =============================================================================
# Run Python Bootstrap
# =============================================================================
run_python_bootstrap() {
    log_step "Running Python Packages Bootstrap..."
    echo ""

    PYTHON_SCRIPT="$SCRIPT_DIR/rpi-bootstrap-python.sh"

    if [ ! -f "$PYTHON_SCRIPT" ]; then
        log_error "Python bootstrap script not found: $PYTHON_SCRIPT"
        return 1
    fi

    chmod +x "$PYTHON_SCRIPT"

    ARGS=""
    if [ "$MINIMAL_MODE" = true ]; then
        ARGS="--minimal"
    fi

    "$PYTHON_SCRIPT" $ARGS
}

# =============================================================================
# Print Final Summary
# =============================================================================
print_final_summary() {
    echo ""
    log_header "╔════════════════════════════════════════════════════════════╗"
    log_header "║              Bootstrap Complete!                           ║"
    log_header "╚════════════════════════════════════════════════════════════╝"
    echo ""

    echo "Quick Test Commands:"
    echo "  cd ~/ambot/tests && ./run_all_tests.sh"
    echo ""
    echo "Individual Tests:"
    echo "  python3 ~/ambot/tests/test_gpio.py"
    echo "  python3 ~/ambot/tests/test_usb_camera.py"
    echo "  python3 ~/ambot/tests/test_usb_lidar.py"
    echo ""
    echo "Motor Test (after wiring L298N):"
    echo "  cd ~/ambot/locomotion && python3 -m rpi_motors.test_motors --check"
    echo ""

    # Check if re-login needed
    if ! groups | grep -q dialout || ! groups | grep -q gpio; then
        log_warn "═══════════════════════════════════════════════════════════"
        log_warn "ACTION REQUIRED: Logout and login for group changes!"
        log_warn "Or run: newgrp dialout && newgrp gpio"
        log_warn "═══════════════════════════════════════════════════════════"
    fi

    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    print_banner
    print_system_info

    if [ "$VERIFY_ONLY" = true ]; then
        verify_installation
        exit $?
    fi

    # Confirmation (unless auto mode)
    if [ "$AUTO_MODE" = false ]; then
        echo "This script will install system packages and Python libraries."
        echo ""
        read -p "Continue? (Y/n): " confirm
        if [[ "$confirm" =~ ^[Nn]$ ]]; then
            log_info "Bootstrap cancelled."
            exit 0
        fi
    fi

    # Run bootstraps based on options
    if [ "$PYTHON_ONLY" = true ]; then
        run_python_bootstrap
    elif [ "$SYSTEM_ONLY" = true ]; then
        run_system_bootstrap
    else
        # Run both
        run_system_bootstrap
        echo ""
        run_python_bootstrap
    fi

    # Verify
    echo ""
    verify_installation || true

    print_final_summary
}

main "$@"
