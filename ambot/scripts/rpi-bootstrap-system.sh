#!/bin/bash
# =============================================================================
# Raspberry Pi System Packages Bootstrap Script
# =============================================================================
# Installs system-level packages required for ambot on Raspberry Pi.
# This script is IDEMPOTENT - safe to run multiple times.
#
# Usage:
#   chmod +x rpi-bootstrap-system.sh
#   ./rpi-bootstrap-system.sh
#
# Or with sudo password:
#   echo "erau" | sudo -S ./rpi-bootstrap-system.sh
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

# =============================================================================
# Check if running on Raspberry Pi
# =============================================================================
check_platform() {
    log_step "Checking platform..."

    if [ -f /proc/device-tree/model ]; then
        MODEL=$(cat /proc/device-tree/model | tr -d '\0')
        log_info "Platform: $MODEL"
    elif [ -f /etc/rpi-issue ]; then
        log_info "Platform: Raspberry Pi (detected via /etc/rpi-issue)"
    else
        log_warn "May not be running on Raspberry Pi - continuing anyway"
    fi

    log_info "Architecture: $(uname -m)"
    log_info "Kernel: $(uname -r)"
    log_info "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2 2>/dev/null || echo 'Unknown')"
}

# =============================================================================
# Check if package is installed
# =============================================================================
is_installed() {
    dpkg -s "$1" &> /dev/null
}

# =============================================================================
# Install package if not present
# =============================================================================
ensure_package() {
    local pkg="$1"
    local desc="${2:-$1}"

    if is_installed "$pkg"; then
        log_check "$desc already installed"
        return 0
    fi

    log_install "$desc"
    sudo apt-get install -y "$pkg"
    log_check "$desc installed"
}

# =============================================================================
# System Packages - Core
# =============================================================================
install_core_packages() {
    log_step "Installing core system packages..."

    # Update package lists (only if stale)
    CACHE_TIME=3600  # 1 hour
    CACHE_FILE="/var/cache/apt/pkgcache.bin"

    if [ ! -f "$CACHE_FILE" ] || [ $(($(date +%s) - $(stat -c %Y "$CACHE_FILE"))) -gt $CACHE_TIME ]; then
        log_info "Updating package lists..."
        sudo apt-get update
    else
        log_check "Package lists up to date"
    fi

    # Core utilities
    ensure_package "git" "Git version control"
    ensure_package "curl" "cURL"
    ensure_package "wget" "wget"
    ensure_package "vim" "Vim editor"
    ensure_package "htop" "htop process viewer"
    ensure_package "tree" "Tree directory viewer"
}

# =============================================================================
# System Packages - Python Development
# =============================================================================
install_python_packages() {
    log_step "Installing Python development packages..."

    ensure_package "python3" "Python 3"
    ensure_package "python3-pip" "Python pip"
    ensure_package "python3-venv" "Python venv"
    ensure_package "python3-dev" "Python development headers"
    ensure_package "python3-setuptools" "Python setuptools"
}

# =============================================================================
# System Packages - GPIO and Hardware
# =============================================================================
install_gpio_packages() {
    log_step "Installing GPIO and hardware packages..."

    ensure_package "python3-rpi.gpio" "RPi.GPIO (system package)"
    ensure_package "python3-gpiozero" "gpiozero (system package)"
    ensure_package "python3-lgpio" "lgpio (system package)"
    ensure_package "python3-spidev" "SPI device support"
    ensure_package "i2c-tools" "I2C tools"
    ensure_package "python3-smbus" "SMBus (I2C) support"
}

# =============================================================================
# System Packages - Camera and Video
# =============================================================================
install_camera_packages() {
    log_step "Installing camera and video packages..."

    ensure_package "v4l-utils" "Video4Linux utilities"
    ensure_package "libv4l-dev" "Video4Linux development"
    ensure_package "ffmpeg" "FFmpeg"
    ensure_package "libopencv-dev" "OpenCV development libraries"

    # Camera detection tools
    if [ -x /usr/bin/rpicam-hello ]; then
        log_check "rpicam tools available"
    elif [ -x /usr/bin/libcamera-hello ]; then
        log_check "libcamera tools available"
    else
        log_warn "No Pi camera tools found (may be USB camera only)"
    fi
}

# =============================================================================
# System Packages - Serial and LiDAR
# =============================================================================
install_serial_packages() {
    log_step "Installing serial and LiDAR packages..."

    ensure_package "python3-serial" "PySerial (system package)"
    ensure_package "screen" "Screen terminal multiplexer"
    ensure_package "minicom" "Minicom serial terminal"

    # Check serial port permissions
    if groups | grep -q dialout; then
        log_check "User in dialout group (serial access)"
    else
        log_warn "User not in dialout group - adding..."
        sudo usermod -a -G dialout "$USER"
        log_info "Added to dialout group (re-login required)"
    fi

    # Check gpio group
    if groups | grep -q gpio; then
        log_check "User in gpio group (GPIO access)"
    else
        log_warn "User not in gpio group - adding..."
        sudo usermod -a -G gpio "$USER"
        log_info "Added to gpio group (re-login required)"
    fi
}

# =============================================================================
# System Packages - Networking
# =============================================================================
install_network_packages() {
    log_step "Installing networking packages..."

    ensure_package "net-tools" "Network tools (ifconfig, etc.)"
    ensure_package "wireless-tools" "Wireless tools"
    ensure_package "openssh-server" "SSH server"
    ensure_package "rsync" "Rsync"
}

# =============================================================================
# System Packages - Build Tools (for pip packages with native extensions)
# =============================================================================
install_build_packages() {
    log_step "Installing build tools..."

    ensure_package "build-essential" "Build essentials (gcc, make, etc.)"
    ensure_package "cmake" "CMake"
    ensure_package "pkg-config" "pkg-config"
    ensure_package "libffi-dev" "FFI development"
    ensure_package "libssl-dev" "OpenSSL development"
}

# =============================================================================
# Docker Installation (for RAG system and containers)
# =============================================================================
install_docker() {
    log_step "Installing Docker..."

    # Check if Docker is already installed and working
    if command -v docker &> /dev/null && docker --version &> /dev/null; then
        log_check "Docker already installed: $(docker --version)"

        # Check Docker Compose v2
        if docker compose version &> /dev/null; then
            log_check "Docker Compose v2: $(docker compose version --short 2>/dev/null || echo 'installed')"
            return 0
        fi
    fi

    # Install Docker from apt
    log_install "Docker"
    sudo apt-get install -y docker.io docker-compose

    # Create docker group if it doesn't exist
    if ! getent group docker &> /dev/null; then
        sudo groupadd docker
        log_info "Created docker group"
    fi

    # Add user to docker group
    if ! groups | grep -q docker; then
        sudo usermod -aG docker "$USER"
        log_info "Added $USER to docker group (re-login required)"
    else
        log_check "User already in docker group"
    fi

    # Start and enable Docker service
    sudo systemctl start docker || log_warn "Could not start Docker service"
    sudo systemctl enable docker || log_warn "Could not enable Docker service"

    # Install Docker Compose v2 plugin (ARM64 compatible)
    log_info "Installing Docker Compose v2 plugin..."
    DOCKER_CONFIG=${DOCKER_CONFIG:-$HOME/.docker}
    mkdir -p "$DOCKER_CONFIG/cli-plugins"

    # Detect architecture
    ARCH=$(uname -m)
    case $ARCH in
        aarch64|arm64)
            COMPOSE_ARCH="linux-aarch64"
            ;;
        x86_64)
            COMPOSE_ARCH="linux-x86_64"
            ;;
        armv7l)
            COMPOSE_ARCH="linux-armv7"
            ;;
        *)
            log_warn "Unknown architecture $ARCH - trying aarch64"
            COMPOSE_ARCH="linux-aarch64"
            ;;
    esac

    COMPOSE_URL="https://github.com/docker/compose/releases/download/v2.35.0/docker-compose-${COMPOSE_ARCH}"

    if curl -SL "$COMPOSE_URL" -o "$DOCKER_CONFIG/cli-plugins/docker-compose" 2>/dev/null; then
        chmod +x "$DOCKER_CONFIG/cli-plugins/docker-compose"
        log_check "Docker Compose v2 installed"
    else
        log_warn "Could not download Docker Compose v2 - using apt version"
    fi

    # Verify installation
    if docker --version &> /dev/null; then
        log_check "Docker: $(docker --version)"
    fi

    if docker compose version &> /dev/null; then
        log_check "Docker Compose: $(docker compose version --short 2>/dev/null || echo 'v2')"
    fi

    log_warn "NOTE: Log out and back in for docker group to take effect"
    log_info "Or run: newgrp docker"
}

# =============================================================================
# Enable Interfaces
# =============================================================================
enable_interfaces() {
    log_step "Checking hardware interfaces..."

    # Check if raspi-config is available
    if ! command -v raspi-config &> /dev/null; then
        log_warn "raspi-config not available - skipping interface configuration"
        return 0
    fi

    # I2C
    if [ -e /dev/i2c-1 ]; then
        log_check "I2C interface enabled"
    else
        log_info "Enabling I2C interface..."
        sudo raspi-config nonint do_i2c 0 2>/dev/null || log_warn "Could not enable I2C"
    fi

    # SPI
    if [ -e /dev/spidev0.0 ]; then
        log_check "SPI interface enabled"
    else
        log_info "Enabling SPI interface..."
        sudo raspi-config nonint do_spi 0 2>/dev/null || log_warn "Could not enable SPI"
    fi

    # Serial (UART)
    if [ -e /dev/ttyAMA0 ] || [ -e /dev/serial0 ]; then
        log_check "Serial interface available"
    else
        log_warn "Serial interface may not be enabled"
    fi
}

# =============================================================================
# Summary
# =============================================================================
print_summary() {
    echo ""
    echo "========================================"
    echo "System Bootstrap Complete"
    echo "========================================"
    echo ""
    echo "Installed packages:"
    echo "  Core:      git, curl, wget, vim, htop"
    echo "  Python:    python3, pip, venv, dev headers"
    echo "  GPIO:      RPi.GPIO, gpiozero, lgpio, spidev"
    echo "  Camera:    v4l-utils, ffmpeg, opencv-dev"
    echo "  Serial:    pyserial, screen, minicom"
    echo "  Network:   net-tools, wireless-tools, rsync"
    echo "  Build:     gcc, make, cmake, pkg-config"
    echo "  Docker:    docker.io, docker-compose v2"
    echo ""
    echo "User groups: $(groups)"
    echo ""

    # Docker status
    if command -v docker &> /dev/null; then
        echo "Docker: $(docker --version 2>/dev/null || echo 'installed')"
        echo "Compose: $(docker compose version --short 2>/dev/null || echo 'checking...')"
        echo ""
    fi

    # Check if re-login needed
    if ! groups | grep -q dialout || ! groups | grep -q gpio || ! groups | grep -q docker; then
        log_warn "Re-login required for group changes to take effect!"
        log_info "Run: newgrp docker && newgrp dialout && newgrp gpio"
        log_info "Or logout and login again"
    fi

    echo "========================================"
    echo "Next: Run rpi-bootstrap-python.sh"
    echo "========================================"
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "Ambot - Raspberry Pi System Bootstrap"
    echo "========================================"
    echo ""

    check_platform
    echo ""

    install_core_packages
    install_python_packages
    install_gpio_packages
    install_camera_packages
    install_serial_packages
    install_network_packages
    install_build_packages
    install_docker
    enable_interfaces

    print_summary
}

main "$@"
