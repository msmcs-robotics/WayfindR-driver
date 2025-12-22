#!/bin/bash
#
# Install ROS2 Humble dependencies for the comprehensive navigation system
#
# Usage: ./install_dependencies.sh
#

set -e

echo "============================================"
echo "  ROS2 Navigation System - Dependency Setup"
echo "============================================"
echo ""

# Check Ubuntu version
if ! grep -q "22.04" /etc/os-release 2>/dev/null; then
    echo "Warning: This script is designed for Ubuntu 22.04"
    echo "Your system may have compatibility issues."
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if ROS2 is already installed
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "ROS2 Humble already installed."
else
    echo "Installing ROS2 Humble..."

    # Set up locale
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Add ROS2 repository
    sudo apt install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    # Install ROS2 Desktop (includes RViz)
    echo "Installing ros-humble-desktop..."
    sudo apt install -y ros-humble-desktop

    # Install dev tools
    sudo apt install -y ros-dev-tools python3-colcon-common-extensions

    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
fi

echo ""
echo "Installing SLAM and Navigation packages..."

sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools

echo ""
echo "Installing LiDAR drivers..."

sudo apt install -y \
    ros-humble-rplidar-ros \
    ros-humble-laser-filters

echo ""
echo "Installing Python dependencies..."

pip3 install --user pyyaml numpy

echo ""
echo "============================================"
echo "  Installation Complete!"
echo "============================================"
echo ""
echo "Add this to your ~/.bashrc:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "Then run: source ~/.bashrc"
echo ""
