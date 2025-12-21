#!/bin/bash
#
# 01_system_setup.sh - Basic system configuration
#
# This runs first to prepare the system for ROS2 installation.
#

set -e

LOG_PREFIX="[01_system_setup]"

log() {
    echo "$LOG_PREFIX $1"
}

log "Starting system setup..."

# Wait for any apt processes to finish
wait_for_apt() {
    local max_wait=300
    local waited=0
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        if [[ $waited -ge $max_wait ]]; then
            log "Timeout waiting for apt lock"
            return 1
        fi
        log "Waiting for apt lock... ($waited/$max_wait seconds)"
        sleep 5
        ((waited+=5))
    done
}

log "Waiting for package manager..."
wait_for_apt

log "Updating package lists..."
apt-get update

log "Upgrading installed packages..."
DEBIAN_FRONTEND=noninteractive apt-get upgrade -y

log "Installing essential tools..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    software-properties-common \
    curl \
    wget \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv \
    htop \
    tmux \
    vim \
    net-tools \
    iproute2 \
    iputils-ping \
    dnsutils

log "Setting up locale..."
locale-gen en_US.UTF-8
update-locale LANG=en_US.UTF-8

log "Configuring timezone..."
timedatectl set-timezone America/New_York || true

log "Enabling SSH service..."
systemctl enable ssh
systemctl start ssh

log "Adding user to dialout group for serial port access..."
usermod -aG dialout ubuntu || true

log "System setup complete!"
