#!/bin/bash
# =============================================================================
# Raspberry Pi Python Libraries Bootstrap Script
# =============================================================================
# Installs Python libraries required for ambot on Raspberry Pi.
# This script is IDEMPOTENT - safe to run multiple times.
#
# Usage:
#   chmod +x rpi-bootstrap-python.sh
#   ./rpi-bootstrap-python.sh
#
# Options:
#   --force    Force reinstall all packages
#   --minimal  Install only essential packages
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[====]${NC} $1"; }
log_check() { echo -e "${GREEN}[✓]${NC} $1"; }
log_install() { echo -e "${YELLOW}[+]${NC} Installing $1..."; }

# Parse arguments
FORCE_INSTALL=false
MINIMAL_INSTALL=false

for arg in "$@"; do
    case $arg in
        --force)
            FORCE_INSTALL=true
            ;;
        --minimal)
            MINIMAL_INSTALL=true
            ;;
    esac
done

# =============================================================================
# Check if Python package is installed
# =============================================================================
is_pip_installed() {
    python3 -c "import $1" 2>/dev/null
}

# =============================================================================
# Install Python package if not present
# =============================================================================
ensure_pip_package() {
    local pkg="$1"
    local import_name="${2:-$1}"
    local desc="${3:-$1}"

    if [ "$FORCE_INSTALL" = true ]; then
        log_install "$desc (forced)"
        pip3 install --break-system-packages --upgrade "$pkg"
        log_check "$desc installed"
        return 0
    fi

    if is_pip_installed "$import_name"; then
        log_check "$desc already installed"
        return 0
    fi

    log_install "$desc"
    pip3 install --break-system-packages "$pkg"
    log_check "$desc installed"
}

# =============================================================================
# Essential Packages - Always Installed
# =============================================================================
install_essential() {
    log_step "Installing essential Python packages..."

    # GPIO libraries (may already be system packages)
    ensure_pip_package "RPi.GPIO" "RPi.GPIO" "RPi.GPIO"
    ensure_pip_package "gpiozero" "gpiozero" "gpiozero"

    # Serial communication (for LiDAR)
    ensure_pip_package "pyserial" "serial" "PySerial"
}

# =============================================================================
# Camera Packages
# =============================================================================
install_camera() {
    log_step "Installing camera Python packages..."

    # OpenCV headless (no GUI dependencies)
    ensure_pip_package "opencv-python-headless" "cv2" "OpenCV Headless"

    # Alternative: picamera2 for Pi Camera Module
    # (Usually pre-installed on Raspberry Pi OS)
    if is_pip_installed "picamera2"; then
        log_check "picamera2 already installed"
    else
        log_warn "picamera2 not installed (only needed for Pi Camera Module)"
        log_info "Install with: pip3 install --break-system-packages picamera2"
    fi
}

# =============================================================================
# LiDAR Packages
# =============================================================================
install_lidar() {
    log_step "Installing LiDAR Python packages..."

    # NOTE: We use a custom driver in pathfinder/lidar.py for RPLidar C1M1
    # The rplidar-roboticia library does NOT support C1M1!

    # PySerial is needed (already installed in essential)
    log_check "PySerial (for custom LiDAR driver)"

    # Optional: RPLidar library for A-series devices only
    # ensure_pip_package "rplidar-roboticia" "rplidar" "RPLidar (A-series only)"

    log_info "Custom C1M1 driver: ambot/pathfinder/lidar.py"
    log_info "C1M1 requires 460800 baud rate"
}

# =============================================================================
# Motor Control Packages
# =============================================================================
install_motors() {
    log_step "Installing motor control Python packages..."

    # GPIO packages already installed
    log_check "GPIO packages (RPi.GPIO, gpiozero)"

    # PWM support is built into RPi.GPIO
    log_info "PWM support via RPi.GPIO.PWM"
}

# =============================================================================
# Utility Packages
# =============================================================================
install_utilities() {
    log_step "Installing utility Python packages..."

    # NumPy (often needed for camera/lidar data processing)
    ensure_pip_package "numpy" "numpy" "NumPy"

    # Requests for HTTP communication
    ensure_pip_package "requests" "requests" "Requests HTTP library"

    # YAML parsing
    ensure_pip_package "pyyaml" "yaml" "PyYAML"

    # Dotenv for environment variables
    ensure_pip_package "python-dotenv" "dotenv" "python-dotenv"
}

# =============================================================================
# Development Packages (optional)
# =============================================================================
install_development() {
    log_step "Installing development Python packages..."

    # Code formatting
    ensure_pip_package "black" "black" "Black formatter"

    # Type checking
    ensure_pip_package "mypy" "mypy" "MyPy type checker"

    # Testing
    ensure_pip_package "pytest" "pytest" "Pytest"
}

# =============================================================================
# Verify Installation
# =============================================================================
verify_installation() {
    log_step "Verifying Python package installation..."

    echo ""
    echo "Package Verification:"
    echo "--------------------"

    # Check each critical package
    PACKAGES=(
        "RPi.GPIO:RPi.GPIO"
        "gpiozero:gpiozero"
        "serial:pyserial"
        "cv2:opencv-python-headless"
        "numpy:numpy"
    )

    ALL_OK=true

    for item in "${PACKAGES[@]}"; do
        import_name="${item%%:*}"
        pkg_name="${item##*:}"

        if python3 -c "import $import_name" 2>/dev/null; then
            VERSION=$(python3 -c "import $import_name; print(getattr($import_name, '__version__', getattr($import_name, 'VERSION', 'unknown')))" 2>/dev/null || echo "unknown")
            log_check "$pkg_name ($VERSION)"
        else
            log_error "$pkg_name - NOT INSTALLED"
            ALL_OK=false
        fi
    done

    echo ""

    if [ "$ALL_OK" = true ]; then
        log_info "All critical packages verified!"
    else
        log_error "Some packages failed to install"
        return 1
    fi
}

# =============================================================================
# Print Summary
# =============================================================================
print_summary() {
    echo ""
    echo "========================================"
    echo "Python Bootstrap Complete"
    echo "========================================"
    echo ""
    echo "Installed packages:"
    echo "  GPIO:      RPi.GPIO, gpiozero"
    echo "  Serial:    pyserial"
    echo "  Camera:    opencv-python-headless"
    echo "  Utilities: numpy, requests, pyyaml"

    if [ "$MINIMAL_INSTALL" = false ]; then
        echo "  Dev:       black, mypy, pytest"
    fi

    echo ""
    echo "Python version: $(python3 --version)"
    echo "Pip version:    $(pip3 --version | cut -d' ' -f1-2)"
    echo ""

    echo "========================================"
    echo "Ready to run ambot!"
    echo "========================================"
    echo ""
    echo "Test commands:"
    echo "  GPIO:   python3 ~/ambot/tests/test_gpio.py"
    echo "  Camera: python3 ~/ambot/tests/test_usb_camera.py"
    echo "  LiDAR:  python3 ~/ambot/tests/test_usb_lidar.py"
    echo "  Motors: cd ~/ambot/locomotion && python3 -m rpi_motors.test_motors --check"
    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "Ambot - Raspberry Pi Python Bootstrap"
    echo "========================================"
    echo ""

    if [ "$FORCE_INSTALL" = true ]; then
        log_warn "Force install mode - all packages will be reinstalled"
    fi

    if [ "$MINIMAL_INSTALL" = true ]; then
        log_info "Minimal install mode - only essential packages"
    fi

    echo ""

    # Always install essential
    install_essential

    # Camera and LiDAR
    install_camera
    install_lidar

    # Motor control
    install_motors

    # Utilities
    install_utilities

    # Development packages (unless minimal)
    if [ "$MINIMAL_INSTALL" = false ]; then
        install_development
    fi

    # Verify
    verify_installation

    print_summary
}

main "$@"
