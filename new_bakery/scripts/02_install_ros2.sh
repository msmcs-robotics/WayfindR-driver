#!/bin/bash
#
# 02_install_ros2.sh - Install ROS2 Humble on Ubuntu 22.04
#
# This installs ROS2 Humble base packages for Raspberry Pi.
# Using ros-humble-ros-base (no GUI) for headless operation.
#

set -e

LOG_PREFIX="[02_install_ros2]"

log() {
    echo "$LOG_PREFIX $1"
}

log "Starting ROS2 Humble installation..."

# Check if ROS2 is already installed
if [[ -d /opt/ros/humble ]]; then
    log "ROS2 Humble appears to be already installed"
    source /opt/ros/humble/setup.bash
    ros2 --version && log "ROS2 is working!" && exit 0
fi

# Wait for any apt processes
wait_for_apt() {
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        log "Waiting for apt lock..."
        sleep 5
    done
}

wait_for_apt

log "Setting up ROS2 repository..."

# Install prerequisites
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release

# Add ROS2 GPG key
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    log "Adding ROS2 GPG key..."
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

# Add ROS2 repository
if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
    log "Adding ROS2 repository..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

log "Updating package lists with ROS2 repository..."
apt-get update

log "Installing ROS2 Humble base packages..."
# Using ros-base for headless (no GUI packages)
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

log "Initializing rosdep..."
if [[ ! -d /etc/ros/rosdep ]]; then
    rosdep init || log "rosdep already initialized"
fi

# Run rosdep update as the ubuntu user
log "Updating rosdep..."
su - ubuntu -c "rosdep update" || log "rosdep update completed with warnings"

log "Adding ROS2 to ubuntu user's bashrc..."
if ! grep -q "source /opt/ros/humble/setup.bash" /home/ubuntu/.bashrc; then
    echo "" >> /home/ubuntu/.bashrc
    echo "# ROS2 Humble" >> /home/ubuntu/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc
fi

log "Verifying ROS2 installation..."
source /opt/ros/humble/setup.bash
ros2 --version

log "ROS2 Humble installation complete!"
