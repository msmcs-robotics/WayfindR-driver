#!/bin/bash
# =============================================================================
# Jetson Orin Nano - Docker & Docker Compose Setup
# =============================================================================
# This script installs Docker and Docker Compose on a fresh Ubuntu 22.04
# Jetson Orin Nano system.
#
# Usage:
#   chmod +x setup-docker.sh
#   ./setup-docker.sh
#
# Requirements:
#   - Ubuntu 22.04 (JetPack 6.x)
#   - sudo privileges
#   - Internet connection
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# =============================================================================
# Step 1: Install Docker
# =============================================================================
install_docker() {
    log_info "Installing Docker..."

    # Check if Docker is already installed
    if command -v docker &> /dev/null; then
        log_warn "Docker is already installed: $(docker --version)"
        read -p "Reinstall? (y/N): " reinstall
        if [[ ! "$reinstall" =~ ^[Yy]$ ]]; then
            log_info "Skipping Docker installation"
            return 0
        fi
    fi

    # Install Docker from Ubuntu repos (works well on Jetson)
    sudo apt update
    sudo apt install -y docker.io

    log_info "Docker installed: $(docker --version)"
}

# =============================================================================
# Step 2: Configure Docker permissions
# =============================================================================
configure_docker_permissions() {
    log_info "Configuring Docker permissions..."

    # Create docker group if it doesn't exist
    if ! getent group docker > /dev/null 2>&1; then
        sudo groupadd docker
        log_info "Created docker group"
    else
        log_info "Docker group already exists"
    fi

    # Add current user to docker group
    if groups $USER | grep -q docker; then
        log_info "User $USER is already in docker group"
    else
        sudo usermod -aG docker $USER
        log_info "Added $USER to docker group"
    fi
}

# =============================================================================
# Step 3: Enable and start Docker service
# =============================================================================
enable_docker_service() {
    log_info "Enabling Docker service..."

    sudo systemctl start docker
    sudo systemctl enable docker

    log_info "Docker service status:"
    sudo systemctl status docker --no-pager -l | head -10
}

# =============================================================================
# Step 4: Install Docker Compose v2 (plugin)
# =============================================================================
install_docker_compose() {
    log_info "Installing Docker Compose v2..."

    # Check if already installed
    if docker compose version &> /dev/null 2>&1; then
        log_warn "Docker Compose is already installed: $(docker compose version)"
        read -p "Reinstall? (y/N): " reinstall
        if [[ ! "$reinstall" =~ ^[Yy]$ ]]; then
            log_info "Skipping Docker Compose installation"
            return 0
        fi
    fi

    # Create CLI plugins directory
    DOCKER_CONFIG=${DOCKER_CONFIG:-$HOME/.docker}
    mkdir -p $DOCKER_CONFIG/cli-plugins

    # Determine architecture
    ARCH=$(uname -m)
    if [[ "$ARCH" == "aarch64" ]]; then
        COMPOSE_ARCH="linux-aarch64"
    else
        COMPOSE_ARCH="linux-x86_64"
    fi

    # Download Docker Compose v2
    COMPOSE_VERSION="v2.35.0"
    log_info "Downloading Docker Compose $COMPOSE_VERSION for $COMPOSE_ARCH..."

    curl -SL "https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-${COMPOSE_ARCH}" \
        -o $DOCKER_CONFIG/cli-plugins/docker-compose

    chmod +x $DOCKER_CONFIG/cli-plugins/docker-compose

    log_info "Docker Compose installed: $(docker compose version)"
}

# =============================================================================
# Step 5: Install legacy docker-compose (optional, for compatibility)
# =============================================================================
install_docker_compose_legacy() {
    log_info "Installing legacy docker-compose (apt)..."

    # This provides the old 'docker-compose' command for compatibility
    sudo apt install -y docker-compose

    log_info "Legacy docker-compose installed: $(docker-compose --version 2>/dev/null || echo 'not available')"
}

# =============================================================================
# Step 6: Verify installation
# =============================================================================
verify_installation() {
    log_info "Verifying installation..."

    echo ""
    echo "========================================"
    echo "Installation Summary"
    echo "========================================"
    echo "Docker:          $(docker --version 2>/dev/null || echo 'NOT INSTALLED')"
    echo "Docker Compose:  $(docker compose version 2>/dev/null || echo 'NOT INSTALLED')"
    echo "Docker Service:  $(systemctl is-active docker)"
    echo "User in docker:  $(groups $USER | grep -q docker && echo 'YES' || echo 'NO')"
    echo "========================================"
    echo ""

    # Test docker without sudo (may fail until re-login)
    log_info "Testing Docker (may require re-login for group changes)..."

    if docker ps &> /dev/null; then
        log_info "Docker is working without sudo!"
    else
        log_warn "Docker requires re-login for group changes to take effect."
        log_warn "Run: newgrp docker"
        log_warn "Or log out and log back in."
    fi
}

# =============================================================================
# Step 7: Apply group changes (optional)
# =============================================================================
apply_group_changes() {
    echo ""
    read -p "Apply docker group changes now? (requires newgrp) (y/N): " apply
    if [[ "$apply" =~ ^[Yy]$ ]]; then
        log_info "Applying group changes..."
        log_info "After this, you'll be in a new shell. Type 'exit' to return."
        newgrp docker
    else
        log_info "Remember to log out and back in for group changes to take effect."
    fi
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "Jetson Orin Nano - Docker Setup"
    echo "========================================"
    echo ""

    install_docker
    configure_docker_permissions
    enable_docker_service
    install_docker_compose
    # install_docker_compose_legacy  # Uncomment if needed
    verify_installation
    apply_group_changes

    echo ""
    log_info "Docker setup complete!"
    echo ""
}

main "$@"
