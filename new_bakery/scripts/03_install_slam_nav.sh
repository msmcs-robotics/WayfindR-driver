#!/bin/bash
#
# 03_install_slam_nav.sh - Install SLAM and Navigation packages
#
# Installs slam_toolbox, Nav2, and related packages for mapping
# and autonomous navigation.
#

set -e

LOG_PREFIX="[03_install_slam_nav]"

log() {
    echo "$LOG_PREFIX $1"
}

log "Starting SLAM and Navigation package installation..."

# Source ROS2
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    log "ERROR: ROS2 not found. Run 02_install_ros2.sh first."
    exit 1
fi

# Wait for apt lock
wait_for_apt() {
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        log "Waiting for apt lock..."
        sleep 5
    done
}

wait_for_apt

log "Installing SLAM Toolbox..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-slam-toolbox

log "Installing Navigation2 packages..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-simple-commander

log "Installing robot localization..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-robot-localization

log "Installing TF tools..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations

log "Installing additional useful packages..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro

log "Creating ROS2 workspace..."
WORKSPACE="/home/ubuntu/ros2_ws"
if [[ ! -d "$WORKSPACE" ]]; then
    mkdir -p "$WORKSPACE/src"
    mkdir -p "$WORKSPACE/maps"
    chown -R ubuntu:ubuntu "$WORKSPACE"
    log "Created workspace at $WORKSPACE"
fi

# Add workspace to bashrc
if ! grep -q "source ~/ros2_ws/install/setup.bash" /home/ubuntu/.bashrc; then
    echo "" >> /home/ubuntu/.bashrc
    echo "# ROS2 Workspace" >> /home/ubuntu/.bashrc
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then" >> /home/ubuntu/.bashrc
    echo "    source ~/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc
    echo "fi" >> /home/ubuntu/.bashrc
fi

log "Verifying installation..."
ros2 pkg list | grep -E "(slam|nav2)" | head -10

log "SLAM and Navigation installation complete!"
