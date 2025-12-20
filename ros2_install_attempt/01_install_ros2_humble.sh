#!/usr/bin/env bash
# 01_install_ros2_humble.sh
# Base ROS2 Humble installation for Ubuntu 22.04
# This is a streamlined version for SLAM/Navigation setup

set -e
set -u

echo "=================================================================="
echo "  ROS2 HUMBLE BASE INSTALLATION - Ubuntu 22.04"
echo "=================================================================="
echo ""
echo "This script will install:"
echo "  - ROS2 Humble Desktop (core + visualization)"
echo "  - Development tools (colcon, rosdep)"
echo "  - Create ~/ros2_ws workspace"
echo ""
read -p "Press ENTER to continue (Ctrl+C to cancel)..."

# Check Ubuntu version
echo ""
echo "--- Checking Ubuntu version ---"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "22.04" ]; then
        echo "⚠ WARNING: This script is for Ubuntu 22.04, you have $VERSION_ID"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        echo "✓ Ubuntu 22.04 detected"
    fi
fi

# 1. Update system
echo ""
echo "--- Step 1: System update ---"
sudo apt update
sudo apt upgrade -y

# 2. Locale setup
echo ""
echo "--- Step 2: Locale configuration ---"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "✓ Locale: $(locale | grep LANG)"

# 3. Enable Universe repository
echo ""
echo "--- Step 3: Enable Universe repository ---"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# 4. Add ROS2 repository
echo ""
echo "--- Step 4: Add ROS2 apt repository ---"
ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
ROS2_KEY="/usr/share/keyrings/ros-archive-keyring.gpg"

# Remove old configurations if present
if [ -f "$ROS2_LIST" ]; then
    echo "  Removing old ROS2 sources list..."
    sudo rm -f "$ROS2_LIST"
fi
if [ -f "$ROS2_KEY" ]; then
    echo "  Removing old ROS2 keyring..."
    sudo rm -f "$ROS2_KEY"
fi

sudo apt update
sudo apt install -y curl gnupg2 lsb-release

# Download and add ROS2 GPG key
echo "  Downloading ROS2 GPG key..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$ROS2_KEY"

# Add ROS2 repository
echo "  Adding ROS2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS2_KEY}] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee "$ROS2_LIST" > /dev/null

# 5. Install ROS2 Humble Desktop
echo ""
echo "--- Step 5: Installing ROS2 Humble Desktop ---"
echo "  This may take several minutes..."
sudo apt update
sudo apt install -y ros-humble-desktop

# 6. Install development tools
echo ""
echo "--- Step 6: Installing ROS2 development tools ---"
sudo apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool

# 7. Initialize rosdep
echo ""
echo "--- Step 7: Initialize rosdep ---"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "  Initializing rosdep..."
    sudo rosdep init
else
    echo "  rosdep already initialized"
fi
rosdep update

# 8. Environment setup
echo ""
echo "--- Step 8: Configure shell environment ---"
BASHRC="$HOME/.bashrc"
SOURCE_LINE="source /opt/ros/humble/setup.bash"

if grep -Fxq "$SOURCE_LINE" "$BASHRC"; then
    echo "  ROS2 already sourced in ~/.bashrc"
else
    echo ""
    echo "# ROS2 Humble environment" >> "$BASHRC"
    echo "$SOURCE_LINE" >> "$BASHRC"
    echo "  ✓ Added ROS2 source to ~/.bashrc"
fi

# Source for current session
source /opt/ros/humble/setup.bash

# 9. Create workspace
echo ""
echo "--- Step 9: Create ROS2 workspace ---"
WS_DIR="$HOME/ros2_ws"
if [ ! -d "$WS_DIR/src" ]; then
    mkdir -p "$WS_DIR/src"
    echo "  ✓ Created workspace at $WS_DIR"
else
    echo "  Workspace already exists at $WS_DIR"
fi

# Build empty workspace
cd "$WS_DIR"
colcon build --symlink-install || {
    echo "  Note: Workspace build produced warnings (normal for empty workspace)"
}

# Add workspace to bashrc
WS_SOURCE_LINE="source $WS_DIR/install/setup.bash"
if grep -Fxq "$WS_SOURCE_LINE" "$BASHRC"; then
    echo "  Workspace already sourced in ~/.bashrc"
else
    echo "$WS_SOURCE_LINE" >> "$BASHRC"
    echo "  ✓ Added workspace source to ~/.bashrc"
fi

# 10. Setup colcon mixins
echo ""
echo "--- Step 10: Setup colcon mixins ---"
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true
colcon mixin update default || true

# 11. Create maps and waypoints directories
echo ""
echo "--- Step 11: Create directories for maps and waypoints ---"
mkdir -p "$HOME/maps"
mkdir -p "$HOME/waypoints"
echo "  ✓ Created ~/maps for map files"
echo "  ✓ Created ~/waypoints for waypoint files"

# Summary
echo ""
echo "=================================================================="
echo "  ROS2 HUMBLE INSTALLATION COMPLETE"
echo "=================================================================="
echo ""
echo "✓ ROS2 Humble Desktop installed"
echo "✓ Development tools installed"
echo "✓ Workspace created at ~/ros2_ws"
echo "✓ Directories created:"
echo "    ~/maps - for saved maps"
echo "    ~/waypoints - for waypoint files"
echo ""
echo "Next steps:"
echo "  1. Open a NEW terminal or run: source ~/.bashrc"
echo "  2. Verify installation: ros2 --version"
echo "  3. Test with demo: ros2 run demo_nodes_cpp talker"
echo "  4. Run next script: sudo bash 02_install_slam_navigation.sh"
echo ""
echo "=================================================================="

exit 0
