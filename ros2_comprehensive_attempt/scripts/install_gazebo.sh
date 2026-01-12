#!/bin/bash
################################################################################
# Gazebo Fortress Installation Script for WayfindR
################################################################################
#
# This script installs Gazebo Fortress and ROS2 integration packages for
# the WayfindR robot simulation environment.
#
# Requirements:
#   - Ubuntu 22.04 LTS
#   - ROS2 Humble installed
#   - Internet connection
#   - sudo privileges
#
# Usage:
#   chmod +x install_gazebo.sh
#   ./install_gazebo.sh
#
# Author: WayfindR Development Team
# Date: 2026-01-11
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Header
echo "================================================================================"
echo "  WayfindR Gazebo Fortress Installation Script"
echo "================================================================================"
echo ""

# Check Ubuntu version
log_info "Checking Ubuntu version..."
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "22.04" ]; then
        log_warning "This script is designed for Ubuntu 22.04. You have $VERSION_ID"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        log_success "Ubuntu 22.04 LTS detected"
    fi
else
    log_error "Cannot determine Ubuntu version"
    exit 1
fi

# Check if ROS2 Humble is installed
log_info "Checking for ROS2 Humble..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    log_success "ROS2 Humble found"
    source /opt/ros/humble/setup.bash
else
    log_error "ROS2 Humble not found. Please install ROS2 Humble first."
    exit 1
fi

# Update package list
log_info "Updating package list..."
sudo apt update

# Install prerequisites
log_info "Installing prerequisites..."
sudo apt install -y \
    wget \
    gnupg \
    lsb-release \
    software-properties-common

log_success "Prerequisites installed"

# Add Gazebo repository
log_info "Adding Gazebo repository..."

# Download GPG key
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    sudo wget https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    log_success "Gazebo GPG key added"
else
    log_info "Gazebo GPG key already exists"
fi

# Add repository to sources list
GAZEBO_REPO="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main"

if ! grep -q "packages.osrfoundation.org/gazebo" /etc/apt/sources.list.d/gazebo-stable.list 2>/dev/null; then
    echo "$GAZEBO_REPO" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    log_success "Gazebo repository added"
else
    log_info "Gazebo repository already exists"
fi

# Update package list again
log_info "Updating package list with Gazebo repository..."
sudo apt update

# Install Gazebo Fortress
log_info "Installing Gazebo Fortress (this may take a while)..."
sudo apt install -y ignition-fortress

log_success "Gazebo Fortress installed"

# Install ROS-Gazebo bridge packages
log_info "Installing ROS-Gazebo integration packages..."
sudo apt install -y ros-humble-ros-gz

log_success "ROS-Gazebo bridge packages installed"

# Install additional useful packages
log_info "Installing additional ROS2 packages..."
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher

log_success "Additional packages installed"

# Verify installation
echo ""
log_info "Verifying installation..."

# Check Gazebo version
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -n 1)
    log_success "Gazebo installed: $GZ_VERSION"
else
    log_error "Gazebo 'gz' command not found"
    exit 1
fi

# Check ROS-Gazebo packages
if ros2 pkg list | grep -q ros_gz; then
    log_success "ROS-Gazebo packages found"
else
    log_error "ROS-Gazebo packages not found"
    exit 1
fi

# Create environment setup script
log_info "Creating environment setup script..."

SETUP_SCRIPT="$HOME/.gazebo_wayfinder_env.sh"

cat > "$SETUP_SCRIPT" << 'EOF'
#!/bin/bash
# WayfindR Gazebo Environment Setup
# Source this file to set up Gazebo environment

# Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace (adjust path as needed)
# WORKSPACE_PATH="$HOME/your_workspace"
# if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
#     source "$WORKSPACE_PATH/install/setup.bash"
# fi

# Set Gazebo resource paths (adjust as needed)
# export GZ_SIM_RESOURCE_PATH=$HOME/your_workspace/install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/worlds:$GZ_SIM_RESOURCE_PATH

echo "WayfindR Gazebo environment ready!"
EOF

chmod +x "$SETUP_SCRIPT"
log_success "Environment setup script created at: $SETUP_SCRIPT"

# Installation complete
echo ""
echo "================================================================================"
log_success "Installation Complete!"
echo "================================================================================"
echo ""
echo "Next steps:"
echo ""
echo "1. Source your ROS2 workspace:"
echo "   $ source ~/your_workspace/install/setup.bash"
echo ""
echo "2. Test Gazebo installation:"
echo "   $ gz sim"
echo ""
echo "3. Launch WayfindR simulation:"
echo "   $ ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py"
echo ""
echo "4. Read the full guide:"
echo "   $ less ~/your_workspace/install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/findings/gazebo-simulation-guide.md"
echo ""
echo "Optional: Add the following to your ~/.bashrc for automatic setup:"
echo "   source $SETUP_SCRIPT"
echo ""
echo "================================================================================"

# Summary of installed packages
echo ""
log_info "Installed packages summary:"
echo "  - ignition-fortress (Gazebo Fortress simulator)"
echo "  - ros-humble-ros-gz (ROS-Gazebo bridge)"
echo "  - ros-humble-xacro (URDF processing)"
echo "  - ros-humble-teleop-twist-keyboard (keyboard control)"
echo "  - ros-humble-robot-state-publisher (TF publishing)"
echo "  - ros-humble-joint-state-publisher (joint states)"
echo ""

# Offer to test installation
read -p "Would you like to test Gazebo now? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    log_info "Launching Gazebo Fortress..."
    log_info "Close the window or press Ctrl+C to exit"
    sleep 2
    gz sim
fi

log_success "Installation script finished successfully!"
exit 0
