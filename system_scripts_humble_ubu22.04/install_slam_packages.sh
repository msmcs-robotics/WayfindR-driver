#!/usr/bin/env bash
# install_slam_nav_packages.sh
# Complete ROS2 Humble package installation for autonomous mapping, waypoint navigation
# For Slamtec C1M1 Lidar on Ubuntu 22.04
# Usage: bash install_slam_nav_packages.sh

set -e  # exit on error
# NOTE: Not using 'set -u' because ROS2 setup scripts use unbound variables

echo "======================================================================"
echo "  ROS2 HUMBLE: SLAM + NAVIGATION + EXPLORATION PACKAGE INSTALLER"
echo "======================================================================"
echo ""
echo "This script will install:"
echo "  - SLAM Toolbox (mapping)"
echo "  - Nav2 Stack (navigation & localization with AMCL)"
echo "  - Explore Lite (autonomous exploration)"
echo "  - TF, Robot State Publisher, Joint State Publisher"
echo "  - Gazebo simulation"
echo "  - Python dependencies for Flask integration"
echo ""
read -p "Press ENTER to continue or CTRL+C to cancel..."
echo ""

# Ensure ROS2 Humble is sourced (disable -u temporarily)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    set +u  # Temporarily disable unbound variable checking
    source /opt/ros/humble/setup.bash
    set -u  # Re-enable it
    echo "✓ ROS2 Humble environment sourced"
else
    echo "✗ ERROR: ROS2 Humble not found at /opt/ros/humble"
    echo "  Please run install_ros2_humble.sh first!"
    exit 1
fi

# Update package lists
echo ""
echo "--- Updating package lists ---"
sudo apt update

# ==============================================================================
# CORE ROS2 PACKAGES
# ==============================================================================
echo ""
echo "--- Installing Core ROS2 Packages ---"
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations

echo "  ✓ Core ROS2 packages installed"

# ==============================================================================
# SLAM PACKAGES
# ==============================================================================
echo ""
echo "--- Installing SLAM Packages ---"

# SLAM Toolbox (Recommended - easier than Cartographer)
sudo apt install -y ros-humble-slam-toolbox

# Google Cartographer (Alternative - more complex but powerful)
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros

echo "  ✓ SLAM Toolbox installed"
echo "  ✓ Google Cartographer installed (alternative)"

# ==============================================================================
# NAVIGATION STACK (NAV2) - INCLUDES AMCL
# ==============================================================================
echo ""
echo "--- Installing Navigation2 Stack (includes AMCL localization) ---"

# Nav2 bringup includes AMCL and all core navigation components
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

echo "  ✓ Nav2 complete stack installed (includes AMCL)"

# Additional Nav2 components (most are dependencies, but explicit install helps)
echo "--- Installing additional Nav2 components ---"
sudo apt install -y \
    ros-humble-nav2-map-server \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-localization

echo "  ✓ Map Server installed"
echo "  ✓ Nav2 Simple Commander (Python API) installed"
echo "  ✓ Robot Localization (sensor fusion) installed"

# ==============================================================================
# AUTONOMOUS EXPLORATION
# ==============================================================================
echo ""
echo "--- Installing Autonomous Exploration ---"

# Create workspace if it doesn't exist
if [ ! -d "$HOME/ros2_ws/src" ]; then
    echo "Creating ROS2 workspace..."
    mkdir -p $HOME/ros2_ws/src
fi

# explore_lite needs to be built from source for ROS2 Humble
EXPLORE_INSTALLED=false

if [ -d "$HOME/ros2_ws/src/m-explore" ]; then
    echo "  m-explore already exists in workspace"
    EXPLORE_INSTALLED=true
else
    echo "  Installing explore_lite from source..."
    cd $HOME/ros2_ws/src
    
    # Clone m-explore (contains explore_lite)
    if git clone -b humble https://github.com/robo-friends/m-explore.git; then
        echo "  ✓ m-explore cloned successfully"
        EXPLORE_INSTALLED=true
    else
        echo "  ✗ Warning: Failed to clone m-explore"
        echo "    You may need to install it manually later"
        echo "    Try: cd ~/ros2_ws/src && git clone -b humble https://github.com/robo-friends/m-explore.git"
    fi
fi

# ==============================================================================
# VISUALIZATION
# ==============================================================================
echo ""
echo "--- Installing Visualization Tools ---"

sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-steering

echo "  ✓ RViz2 and RQt tools installed"

# ==============================================================================
# SIMULATION (OPTIONAL)
# ==============================================================================
echo ""
echo "--- Installing Gazebo Simulation ---"
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    gazebo
echo "  ✓ Gazebo installed"

# ==============================================================================
# HARDWARE INTERFACE PACKAGES
# ==============================================================================
echo ""
echo "--- Installing Hardware Interface Packages ---"

# For motor control and sensors
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-controller-manager

# Serial communication for Arduino/motor controllers
sudo apt install -y \
    python3-serial \
    python3-pip

echo "  ✓ Hardware interface packages installed"

# ==============================================================================
# PYTHON DEPENDENCIES
# ==============================================================================
echo ""
echo "--- Installing Python Dependencies ---"

# Flask for REST API
pip3 install --user flask flask-cors

# ROS2 Python utilities
pip3 install --user \
    pyserial \
    pyyaml \
    numpy \
    scipy \
    matplotlib

# Transforms3d for coordinate transformations
pip3 install --user transforms3d

echo "  ✓ Python dependencies installed"

# ==============================================================================
# SENSOR DRIVERS
# ==============================================================================
echo ""
echo "--- Installing Sensor Drivers ---"

# IMU drivers
sudo apt install -y \
    ros-humble-imu-tools \
    ros-humble-imu-filter-madgwick

echo "  ✓ IMU and sensor drivers installed"

# ==============================================================================
# BUILD WORKSPACE
# ==============================================================================
echo ""
echo "--- Building ROS2 Workspace ---"

if [ "$EXPLORE_INSTALLED" = true ]; then
    cd $HOME/ros2_ws
    
    echo "  Installing dependencies..."
    set +e  # Don't exit on rosdep errors
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    set -e
    
    echo "  Building workspace (this may take a few minutes)..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        echo "  ✓ Workspace built successfully"
    else
        echo "  ✗ Workspace build had errors, but continuing..."
    fi
else
    echo "  ⚠ Skipping workspace build (explore_lite not installed)"
fi

# ==============================================================================
# USER PERMISSIONS
# ==============================================================================
echo ""
echo "--- Setting Up User Permissions ---"

# Add user to dialout group for serial devices (lidar, Arduino, etc.)
if groups $USER | grep -q '\bdialout\b'; then
    echo "  ✓ User already in dialout group"
else
    sudo usermod -a -G dialout $USER
    echo "  ✓ Added $USER to dialout group"
    echo "  ⚠ You must LOG OUT and LOG BACK IN for this to take effect!"
fi

# ==============================================================================
# VERIFY INSTALLATIONS
# ==============================================================================
echo ""
echo "--- Verifying Installations ---"
echo ""

set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source $HOME/ros2_ws/install/setup.bash
set -u

echo "Checking installed packages..."
echo ""

check_package() {
    if ros2 pkg list | grep -q "^$1$"; then
        echo "  ✓ $1"
        return 0
    else
        echo "  ✗ $1 NOT FOUND"
        return 1
    fi
}

check_package "slam_toolbox"
check_package "nav2_bringup"
check_package "nav2_simple_commander"
check_package "nav2_map_server"
check_package "robot_localization"

# Check for lidar package if it exists
check_package "sllidar_ros2" || echo "  ℹ sllidar_ros2 not found (install separately if needed)"

if [ "$EXPLORE_INSTALLED" = true ]; then
    check_package "explore_lite" || echo "  ⚠ explore_lite may need manual build"
fi

# ==============================================================================
# SUMMARY
# ==============================================================================
echo ""
echo "======================================================================"
echo "  INSTALLATION COMPLETE!"
echo "======================================================================"
echo ""
echo "Installed packages:"
echo "  ✓ SLAM Toolbox (mapping)"
echo "  ✓ Google Cartographer (alternative SLAM)"
echo "  ✓ Nav2 Stack (navigation)"
echo "  ✓ AMCL (localization - included in Nav2)"
echo "  ✓ Map Server (save/load maps)"
echo "  ✓ Robot Localization (sensor fusion)"
echo "  ✓ RViz2 (visualization)"
echo "  ✓ Hardware interfaces"
echo "  ✓ Python dependencies (Flask, etc.)"

if [ "$EXPLORE_INSTALLED" = true ]; then
    echo "  ✓ Explore Lite (autonomous exploration)"
fi

echo ""
echo "⚠ IMPORTANT: If you were added to dialout group,"
echo "   LOG OUT and LOG BACK IN before using serial devices!"
echo ""
echo "Next steps:"
echo "  1. Open a new terminal (or source ~/.bashrc)"
echo "  2. Test your lidar (if installed):"
echo "     ros2 launch sllidar_ros2 sllidar_c1_launch.py"
echo ""
echo "  3. Start mapping phase:"
echo "     ros2 launch slam_toolbox online_async_launch.py"
echo ""
echo "  4. Visualize in RViz2:"
echo "     rviz2"
echo ""
echo "  5. Check the README.md for full workflow"
echo ""
echo "======================================================================"

exit 0