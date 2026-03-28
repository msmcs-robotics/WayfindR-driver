#!/bin/bash
# ============================================================================
# AMBOT Jetson Setup Script (comprehensive, idempotent)
#
# Installs ALL dependencies for the AMBOT project on Jetson Orin Nano:
#   1. System apt packages (GTK modules, tkinter, GUI libs, dev tools)
#   2. System groups (dialout, gpio, i2c, video)
#   3. Python packages (pyserial, ollama, fastmcp, gpiod, httpx)
#   4. GPIO permissions and udev rules
#   5. LiDAR serial port access
#   6. Camera + sensor verification
#   7. Ollama + Docker verification
#
# Usage:
#   ./setup-jetson-motors.sh              # Full setup
#   ./setup-jetson-motors.sh --check      # Check only, don't install
#   ./setup-jetson-motors.sh --pip-only   # Only install Python packages
#   ./setup-jetson-motors.sh --apt-only   # Only install apt packages
#   ./setup-jetson-motors.sh --groups-only  # Only fix group memberships
# ============================================================================

set -euo pipefail

USER_NAME="${SUDO_USER:-$(whoami)}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CHECK_ONLY=false
PIP_ONLY=false
APT_ONLY=false
GROUPS_ONLY=false

# Parse args
for arg in "$@"; do
    case "$arg" in
        --check) CHECK_ONLY=true ;;
        --pip-only) PIP_ONLY=true ;;
        --apt-only) APT_ONLY=true ;;
        --groups-only) GROUPS_ONLY=true ;;
        --help|-h)
            echo "Usage: $0 [--check|--pip-only|--apt-only|--groups-only]"
            exit 0
            ;;
    esac
done

# ── Helpers ────────────────────────────────────────────────

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

ok()   { echo -e "  ${GREEN}✓${NC} $1"; }
fail() { echo -e "  ${RED}✗${NC} $1"; }
warn() { echo -e "  ${YELLOW}!${NC} $1"; }
info() { echo -e "  $1"; }

check_group() {
    local group="$1"
    if id -nG "$USER_NAME" 2>/dev/null | grep -qw "$group"; then
        ok "$USER_NAME is in '$group' group"
        return 0
    else
        fail "$USER_NAME is NOT in '$group' group"
        return 1
    fi
}

add_group() {
    local group="$1"
    if id -nG "$USER_NAME" 2>/dev/null | grep -qw "$group"; then
        ok "$USER_NAME already in '$group' group"
    elif $CHECK_ONLY; then
        fail "$USER_NAME needs '$group' group (--check mode, not adding)"
    else
        sudo usermod -aG "$group" "$USER_NAME"
        ok "Added $USER_NAME to '$group' group"
        warn "Re-login required for group change to take effect"
    fi
}

check_pip_package() {
    local package="$1"
    if python3 -c "import $package" 2>/dev/null; then
        ok "Python package '$package' installed"
        return 0
    else
        fail "Python package '$package' NOT installed"
        return 1
    fi
}

install_pip_package() {
    local package="$1"
    local import_name="${2:-$1}"
    if python3 -c "import $import_name" 2>/dev/null; then
        ok "$package already installed"
    elif $CHECK_ONLY; then
        fail "$package needs install (--check mode, not installing)"
    else
        pip3 install --user "$package" 2>&1 | tail -1
        if python3 -c "import $import_name" 2>/dev/null; then
            ok "$package installed successfully"
        else
            fail "$package install failed"
        fi
    fi
}

# ── Main ───────────────────────────────────────────────────

echo "============================================"
echo "  AMBOT Jetson Setup (comprehensive)"
echo "============================================"
echo "  User: $USER_NAME"
echo "  Mode: $($CHECK_ONLY && echo 'CHECK ONLY' || echo 'INSTALL')"
echo ""

# ── 0. System APT Packages ───────────────────────────────

if ! $PIP_ONLY && ! $GROUPS_ONLY; then
    echo "── System APT Packages ──"

    # All apt packages needed for AMBOT on Jetson
    APT_PACKAGES=(
        # GUI / GTK (face tracker, LiDAR GUI)
        "libcanberra-gtk-module"
        "libcanberra-gtk3-module"
        "python3-tk"
        "python3-pil.imagetk"
        # NOTE: Do NOT install opencv-data from apt — it conflicts with NVIDIA's
        # OpenCV 4.8.0 (libopencv). Cascade files are at /usr/share/opencv4/haarcascades/
        # Serial / I2C tools
        "python3-serial"
        "i2c-tools"
        # Device tree tools (for pinmux overlays)
        "device-tree-compiler"
        # GPIO tools
        "gpiod"
        "libgpiod-dev"
        # General utilities
        "busybox"
        "curl"
    )

    missing=()
    for pkg in "${APT_PACKAGES[@]}"; do
        if dpkg -s "$pkg" &>/dev/null; then
            ok "$pkg"
        else
            fail "$pkg NOT installed"
            missing+=("$pkg")
        fi
    done

    if [ ${#missing[@]} -gt 0 ]; then
        if $CHECK_ONLY; then
            warn "Missing ${#missing[@]} packages (--check mode, not installing)"
        else
            info "Installing ${#missing[@]} packages: ${missing[*]}"
            sudo apt update -qq 2>/dev/null
            sudo apt install -y "${missing[@]}" 2>&1 | tail -3
            ok "APT packages installed"
        fi
    else
        ok "All APT packages present"
    fi

    echo ""
fi

# ── 1. System Groups ──────────────────────────────────────

if ! $PIP_ONLY && ! $APT_ONLY; then
    echo "── System Groups ──"
    add_group gpio
    add_group dialout
    add_group i2c
    add_group video
    echo ""
fi

# ── 2. Python Packages ───────────────────────────────────

if ! $GROUPS_ONLY && ! $APT_ONLY; then
    echo "── Python Packages ──"

    # pyserial (for LiDAR LD19)
    install_pip_package pyserial serial

    # ollama Python client
    install_pip_package ollama ollama

    # FastMCP (for MCP ability server)
    install_pip_package fastmcp fastmcp

    # gpiod (libgpiod Python bindings — fallback motor driver)
    install_pip_package gpiod gpiod

    # httpx (for chat app async HTTP)
    install_pip_package httpx httpx

    # requests (for RAG API proxy)
    install_pip_package requests requests

    # uvicorn (for chat app server)
    install_pip_package uvicorn uvicorn

    echo ""
fi

# ── 3. GPIO Verification ─────────────────────────────────

if ! $PIP_ONLY && ! $APT_ONLY; then
    echo "── GPIO Verification ──"

    # Check Jetson.GPIO
    if python3 -c "import Jetson.GPIO; print(f'Jetson.GPIO {Jetson.GPIO.VERSION}')" 2>/dev/null; then
        ok "Jetson.GPIO $(python3 -c 'import Jetson.GPIO; print(Jetson.GPIO.VERSION)' 2>/dev/null)"
    else
        fail "Jetson.GPIO not installed"
        if ! $CHECK_ONLY; then
            pip3 install --user Jetson.GPIO 2>&1 | tail -1
        fi
    fi

    # Check GPIO device permissions
    if [ -c /dev/gpiochip0 ]; then
        perms=$(stat -c '%a %G' /dev/gpiochip0)
        ok "/dev/gpiochip0 exists ($perms)"
    else
        fail "/dev/gpiochip0 not found"
    fi

    # Check udev rules
    if [ -f /etc/udev/rules.d/99-gpio.rules ]; then
        ok "GPIO udev rules present"
    else
        warn "GPIO udev rules missing"
        if ! $CHECK_ONLY; then
            if [ -f /opt/nvidia/jetson-gpio/etc/99-gpio.rules ]; then
                sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
                sudo udevadm control --reload-rules
                sudo udevadm trigger
                ok "GPIO udev rules installed"
            else
                fail "Cannot find source udev rules"
            fi
        fi
    fi

    echo ""
fi

# ── 4. Serial Port (LiDAR) ───────────────────────────────

if ! $PIP_ONLY && ! $APT_ONLY; then
    echo "── Serial Port (LiDAR) ──"

    if [ -c /dev/ttyUSB0 ]; then
        perms=$(stat -c '%a %G' /dev/ttyUSB0)
        ok "/dev/ttyUSB0 exists ($perms)"

        # Test if readable
        if python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB0', 230400, timeout=0.5)
data = s.read(100)
s.close()
print(f'{len(data)} bytes')
" 2>/dev/null; then
            ok "LiDAR serial port accessible"
        else
            warn "LiDAR serial port exists but not readable (re-login for dialout group?)"
        fi
    else
        warn "/dev/ttyUSB0 not found (LiDAR not plugged in?)"
    fi

    echo ""
fi

# ── 5. Camera ─────────────────────────────────────────────

if ! $PIP_ONLY && ! $APT_ONLY; then
    echo "── Camera ──"

    if [ -c /dev/video0 ]; then
        ok "/dev/video0 exists"
        if python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret = cap.isOpened()
cap.release()
print('open' if ret else 'fail')
" 2>/dev/null | grep -q "open"; then
            ok "Camera accessible via OpenCV"
        else
            warn "Camera device exists but OpenCV can't open it"
        fi
    else
        warn "/dev/video0 not found (camera not plugged in?)"
    fi

    echo ""
fi

# ── 6. Ollama ─────────────────────────────────────────────

if ! $GROUPS_ONLY && ! $APT_ONLY; then
    echo "── Ollama ──"

    if command -v ollama &>/dev/null; then
        ok "Ollama CLI installed ($(ollama --version 2>/dev/null | head -1))"
    else
        warn "Ollama CLI not found (expected at /usr/local/bin/ollama)"
    fi

    if curl -s http://localhost:11434/api/tags 2>/dev/null | python3 -c "
import json, sys
data = json.load(sys.stdin)
models = [m['name'] for m in data.get('models', [])]
print(', '.join(models))
" 2>/dev/null; then
        ok "Ollama API responding, models listed above"
    else
        warn "Ollama API not responding at localhost:11434"
    fi

    echo ""
fi

# ── Summary ───────────────────────────────────────────────

echo "============================================"
echo "  Setup complete."
echo "============================================"
echo ""
echo "  If groups were changed, run: newgrp dialout"
echo "  Or log out and back in for all groups."
echo ""
echo "  Test motor chat: cd ambot/ && python3 -m mcp_ability.chat"
echo "  Test motor GPIO: python3 locomotion/jetson_motors/test_motors.py --simulate"
echo ""
