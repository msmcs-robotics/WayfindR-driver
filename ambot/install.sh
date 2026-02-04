#!/bin/bash
#
# AMBOT Comprehensive Install Script
#
# Installs all system packages and Python dependencies for ambot components.
# This script is IDEMPOTENT - safe to run multiple times.
#
# Components:
#   - pathfinder: LiDAR + sensors (pyserial, numpy, opencv)
#   - locomotion: Motor control (RPi.GPIO, gpiozero)
#   - bootylicious: LLM + RAG (Docker, ollama) - optional
#
# Usage:
#   sudo ./install.sh              # Install everything (pathfinder + locomotion)
#   sudo ./install.sh --check      # Check what's installed
#   sudo ./install.sh --pathfinder # Only pathfinder dependencies
#   sudo ./install.sh --locomotion # Only locomotion dependencies
#   sudo ./install.sh --gui        # Include GUI packages (matplotlib, tkinter)
#   sudo ./install.sh --docker     # Include Docker for bootylicious
#
# This script requires sudo for system packages.
# Python packages are installed with --break-system-packages for Debian 12+.

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Defaults
INSTALL_PATHFINDER=true
INSTALL_LOCOMOTION=true
INSTALL_GUI=false
INSTALL_DOCKER=false
CHECK_ONLY=false
VERBOSE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --check)
            CHECK_ONLY=true
            shift
            ;;
        --pathfinder)
            INSTALL_PATHFINDER=true
            INSTALL_LOCOMOTION=false
            shift
            ;;
        --locomotion)
            INSTALL_PATHFINDER=false
            INSTALL_LOCOMOTION=true
            shift
            ;;
        --gui)
            INSTALL_GUI=true
            shift
            ;;
        --docker)
            INSTALL_DOCKER=true
            shift
            ;;
        --all)
            INSTALL_PATHFINDER=true
            INSTALL_LOCOMOTION=true
            INSTALL_GUI=true
            INSTALL_DOCKER=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            echo "AMBOT Install Script"
            echo ""
            echo "Usage: sudo ./install.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --check        Check what's installed without installing"
            echo "  --pathfinder   Only install pathfinder dependencies"
            echo "  --locomotion   Only install locomotion dependencies"
            echo "  --gui          Include GUI packages (matplotlib, tkinter)"
            echo "  --docker       Include Docker for bootylicious"
            echo "  --all          Install everything including GUI and Docker"
            echo "  -v, --verbose  Verbose output"
            echo "  -h, --help     Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_section() {
    echo ""
    echo -e "${BLUE}════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}════════════════════════════════════════════════════════════${NC}"
}

# Check if running as root
check_root() {
    if [[ $EUID -ne 0 ]] && [[ "$CHECK_ONLY" != "true" ]]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

# Check if package is installed (apt)
is_apt_installed() {
    dpkg -l "$1" 2>/dev/null | grep -q "^ii"
}

# Check if Python package is installed
is_pip_installed() {
    python3 -c "import $1" 2>/dev/null
}

# Install apt package if not installed
install_apt() {
    local pkg="$1"
    if is_apt_installed "$pkg"; then
        if [[ "$VERBOSE" == "true" ]]; then
            log_success "$pkg already installed"
        fi
        return 0
    fi

    if [[ "$CHECK_ONLY" == "true" ]]; then
        log_warning "$pkg NOT installed"
        return 1
    fi

    log_info "Installing $pkg..."
    apt-get install -y "$pkg" > /dev/null 2>&1
    log_success "Installed $pkg"
}

# Install Python package if not installed
install_pip() {
    local pkg="$1"
    local import_name="${2:-$1}"

    if is_pip_installed "$import_name"; then
        if [[ "$VERBOSE" == "true" ]]; then
            log_success "Python: $pkg already installed"
        fi
        return 0
    fi

    if [[ "$CHECK_ONLY" == "true" ]]; then
        log_warning "Python: $pkg NOT installed"
        return 1
    fi

    log_info "Installing Python: $pkg..."
    pip3 install "$pkg" --break-system-packages > /dev/null 2>&1
    log_success "Installed Python: $pkg"
}

# Check if user is in a group
user_in_group() {
    local user="$1"
    local group="$2"
    groups "$user" 2>/dev/null | grep -qw "$group"
}

# Add user to group if not already a member
add_user_to_group() {
    local user="$1"
    local group="$2"

    if user_in_group "$user" "$group"; then
        if [[ "$VERBOSE" == "true" ]]; then
            log_success "User $user already in group $group"
        fi
        return 0
    fi

    if [[ "$CHECK_ONLY" == "true" ]]; then
        log_warning "User $user NOT in group $group"
        return 1
    fi

    log_info "Adding $user to group $group..."
    usermod -aG "$group" "$user"
    log_success "Added $user to group $group"
}

# =============================================================================
# System Package Installation
# =============================================================================

install_system_base() {
    log_section "System Base Packages"

    # Update package list (only if not check-only)
    if [[ "$CHECK_ONLY" != "true" ]]; then
        log_info "Updating package list..."
        apt-get update > /dev/null 2>&1
    fi

    # Essential packages
    local packages=(
        "python3"
        "python3-pip"
        "python3-dev"
        "python3-venv"
        "git"
        "curl"
        "wget"
        "build-essential"
    )

    for pkg in "${packages[@]}"; do
        install_apt "$pkg"
    done
}

install_pathfinder_system() {
    log_section "Pathfinder System Packages"

    # Serial/USB packages
    local packages=(
        "python3-serial"      # pyserial
        "libusb-1.0-0-dev"    # USB development
        "udev"                # Device management
    )

    for pkg in "${packages[@]}"; do
        install_apt "$pkg"
    done

    # Add user to dialout for serial access
    local current_user="${SUDO_USER:-$USER}"
    if [[ -n "$current_user" && "$current_user" != "root" ]]; then
        add_user_to_group "$current_user" "dialout"
    fi
}

install_locomotion_system() {
    log_section "Locomotion System Packages"

    # GPIO packages
    local packages=(
        "python3-rpi.gpio"    # RPi.GPIO
        "python3-gpiozero"    # gpiozero
        "python3-lgpio"       # lgpio (newer alternative)
        "i2c-tools"           # I2C utilities
        "python3-smbus"       # I2C Python library
    )

    for pkg in "${packages[@]}"; do
        install_apt "$pkg" || true  # Some may not be available on all systems
    done

    # Add user to gpio and i2c groups
    local current_user="${SUDO_USER:-$USER}"
    if [[ -n "$current_user" && "$current_user" != "root" ]]; then
        add_user_to_group "$current_user" "gpio" || true
        add_user_to_group "$current_user" "i2c" || true
    fi
}

install_gui_system() {
    log_section "GUI System Packages"

    local packages=(
        "python3-tk"          # Tkinter
        "python3-matplotlib"  # Matplotlib (system version)
        "python3-numpy"       # NumPy (system version)
        "libopencv-dev"       # OpenCV development
        "python3-opencv"      # OpenCV Python
        "opencv-data"         # Haar cascades for face detection
    )

    for pkg in "${packages[@]}"; do
        install_apt "$pkg" || true
    done
}

install_docker() {
    log_section "Docker Installation"

    if command -v docker &> /dev/null; then
        log_success "Docker already installed: $(docker --version)"
    else
        if [[ "$CHECK_ONLY" == "true" ]]; then
            log_warning "Docker NOT installed"
        else
            log_info "Installing Docker..."
            apt-get install -y docker.io docker-compose > /dev/null 2>&1
            log_success "Docker installed"
        fi
    fi

    # Docker Compose v2 plugin
    local compose_dir="${HOME}/.docker/cli-plugins"
    if [[ -f "$compose_dir/docker-compose" ]]; then
        log_success "Docker Compose v2 plugin already installed"
    else
        if [[ "$CHECK_ONLY" == "true" ]]; then
            log_warning "Docker Compose v2 plugin NOT installed"
        else
            log_info "Installing Docker Compose v2 plugin..."

            # Detect architecture
            local arch
            case "$(uname -m)" in
                aarch64|arm64) arch="aarch64" ;;
                x86_64) arch="x86_64" ;;
                armv7l) arch="armv7" ;;
                *) log_error "Unsupported architecture: $(uname -m)"; return 1 ;;
            esac

            mkdir -p "$compose_dir"
            local compose_url="https://github.com/docker/compose/releases/download/v2.35.0/docker-compose-linux-${arch}"
            curl -SL "$compose_url" -o "$compose_dir/docker-compose" 2>/dev/null
            chmod +x "$compose_dir/docker-compose"
            log_success "Docker Compose v2 plugin installed"
        fi
    fi

    # Add user to docker group
    local current_user="${SUDO_USER:-$USER}"
    if [[ -n "$current_user" && "$current_user" != "root" ]]; then
        add_user_to_group "$current_user" "docker" || true
    fi
}

# =============================================================================
# Python Package Installation
# =============================================================================

install_pathfinder_python() {
    log_section "Pathfinder Python Packages"

    local packages=(
        "pyserial:serial"
        "numpy:numpy"
    )

    for entry in "${packages[@]}"; do
        local pkg="${entry%%:*}"
        local import="${entry##*:}"
        install_pip "$pkg" "$import"
    done
}

install_locomotion_python() {
    log_section "Locomotion Python Packages"

    # Note: RPi.GPIO and gpiozero are usually installed via apt
    # These are backup/alternative packages

    local packages=(
        "gpiozero:gpiozero"
    )

    for entry in "${packages[@]}"; do
        local pkg="${entry%%:*}"
        local import="${entry##*:}"
        install_pip "$pkg" "$import" || true
    done
}

install_gui_python() {
    log_section "GUI Python Packages"

    local packages=(
        "matplotlib:matplotlib"
        "opencv-python-headless:cv2"
    )

    for entry in "${packages[@]}"; do
        local pkg="${entry%%:*}"
        local import="${entry##*:}"
        install_pip "$pkg" "$import"
    done
}

# =============================================================================
# Post-Installation
# =============================================================================

setup_udev_rules() {
    log_section "Udev Rules Setup"

    local rules_file="/etc/udev/rules.d/99-ambot.rules"

    if [[ -f "$rules_file" ]]; then
        log_success "Udev rules already exist: $rules_file"
        return 0
    fi

    if [[ "$CHECK_ONLY" == "true" ]]; then
        log_warning "Udev rules NOT configured"
        return 1
    fi

    log_info "Creating udev rules..."

    cat > "$rules_file" << 'EOF'
# AMBOT udev rules
# Allows non-root access to serial devices

# USB serial adapters (LiDAR, etc.)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"

# Generic USB serial
KERNEL=="ttyUSB[0-9]*", MODE="0666", GROUP="dialout"
KERNEL=="ttyACM[0-9]*", MODE="0666", GROUP="dialout"
EOF

    udevadm control --reload-rules
    udevadm trigger

    log_success "Udev rules configured"
}

verify_installation() {
    log_section "Verification"

    local errors=0

    # Python
    if python3 --version > /dev/null 2>&1; then
        log_success "Python3: $(python3 --version)"
    else
        log_error "Python3 not found"
        ((errors++))
    fi

    # pip
    if pip3 --version > /dev/null 2>&1; then
        log_success "pip3: $(pip3 --version | head -1)"
    else
        log_error "pip3 not found"
        ((errors++))
    fi

    # Serial
    if is_pip_installed "serial"; then
        log_success "pyserial: installed"
    else
        log_warning "pyserial: not installed"
    fi

    # NumPy
    if is_pip_installed "numpy"; then
        log_success "numpy: installed"
    else
        log_warning "numpy: not installed"
    fi

    # GPIO
    if is_pip_installed "RPi.GPIO" || is_pip_installed "RPi"; then
        log_success "RPi.GPIO: installed"
    else
        log_warning "RPi.GPIO: not installed (may be normal on non-RPi)"
    fi

    # OpenCV
    if is_pip_installed "cv2"; then
        log_success "OpenCV: installed"
    else
        log_warning "OpenCV: not installed"
    fi

    # Matplotlib
    if is_pip_installed "matplotlib"; then
        log_success "matplotlib: installed"
    else
        log_warning "matplotlib: not installed"
    fi

    # Docker
    if command -v docker &> /dev/null; then
        log_success "Docker: $(docker --version)"
    else
        log_warning "Docker: not installed"
    fi

    # USB devices
    if ls /dev/ttyUSB* 2>/dev/null; then
        log_success "USB serial devices found"
    else
        log_warning "No USB serial devices detected"
    fi

    echo ""
    if [[ $errors -eq 0 ]]; then
        log_success "All critical dependencies are installed!"
    else
        log_error "Some critical dependencies are missing"
    fi

    return $errors
}

# =============================================================================
# Main
# =============================================================================

main() {
    echo ""
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║               AMBOT Installation Script                    ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""

    if [[ "$CHECK_ONLY" == "true" ]]; then
        log_info "Running in CHECK mode (no changes will be made)"
    fi

    echo "Components to install:"
    [[ "$INSTALL_PATHFINDER" == "true" ]] && echo "  - Pathfinder (LiDAR + sensors)"
    [[ "$INSTALL_LOCOMOTION" == "true" ]] && echo "  - Locomotion (motor control)"
    [[ "$INSTALL_GUI" == "true" ]] && echo "  - GUI packages (matplotlib, tkinter)"
    [[ "$INSTALL_DOCKER" == "true" ]] && echo "  - Docker (for bootylicious)"
    echo ""

    # Check root if not check-only
    check_root

    # Base system packages
    install_system_base

    # Component-specific packages
    if [[ "$INSTALL_PATHFINDER" == "true" ]]; then
        install_pathfinder_system
        install_pathfinder_python
    fi

    if [[ "$INSTALL_LOCOMOTION" == "true" ]]; then
        install_locomotion_system
        install_locomotion_python
    fi

    if [[ "$INSTALL_GUI" == "true" ]]; then
        install_gui_system
        install_gui_python
    fi

    if [[ "$INSTALL_DOCKER" == "true" ]]; then
        install_docker
    fi

    # Udev rules
    if [[ "$INSTALL_PATHFINDER" == "true" ]]; then
        setup_udev_rules
    fi

    # Verify
    verify_installation

    echo ""
    log_section "Installation Complete"
    echo ""
    echo "Next steps:"
    echo "  1. Log out and back in (for group changes to take effect)"
    echo "  2. Run: ./deploy.sh rpi  (to sync ambot code)"
    echo "  3. Test: python3 tests/test_ld19_lidar.py"
    echo ""

    if [[ "$INSTALL_DOCKER" == "true" ]]; then
        echo "Docker setup:"
        echo "  1. Start Docker: sudo systemctl start docker"
        echo "  2. Enable on boot: sudo systemctl enable docker"
        echo ""
    fi
}

main "$@"
