#!/usr/bin/env bash
# 02_install_slam_navigation.sh
# Install SLAM Toolbox and Navigation2 stack for ROS2 Humble
# Optimized for indoor mapping and localization

set -e
set -o pipefail

echo "=================================================================="
echo "  ROS2 HUMBLE: SLAM + NAVIGATION INSTALLATION"
echo "=================================================================="
echo ""
echo "This script installs:"
echo "  - SLAM Toolbox (mapping & localization)"
echo "  - Navigation2 (path planning & navigation)"
echo "  - Map Server (save/load maps)"
echo "  - Robot Localization (sensor fusion)"
echo "  - RViz2 (visualization)"
echo "  - IMU Tools"
echo "  - Hardware interfaces"
echo ""
read -p "Press ENTER to continue (Ctrl+C to cancel)..."

# Wait for apt locks
echo ""
echo "--- Checking for apt locks ---"
while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
    echo "  ⏳ Waiting for package manager..."
    sleep 3
done
echo "  ✓ Package manager ready"

# Source ROS2
echo ""
echo "--- Sourcing ROS2 environment ---"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "  ✓ ROS2 Humble sourced"
else
    echo "  ✗ ERROR: ROS2 Humble not found at /opt/ros/humble"
    echo "  Please run 01_install_ros2_humble.sh first"
    exit 1
fi

# Update package lists
echo ""
echo "--- Updating package lists ---"
sudo apt update

# Install SLAM Toolbox
echo ""
echo "--- Installing SLAM Toolbox ---"
echo "  SLAM Toolbox is the recommended SLAM solution for ROS2"
echo "  (Better than Cartographer for ROS2 - actively maintained)"
sudo apt install -y ros-humble-slam-toolbox
echo "  ✓ SLAM Toolbox installed"

# Install Navigation2 (full stack)
echo ""
echo "--- Installing Navigation2 Stack ---"
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-simple-commander \
    ros-humble-nav2-msgs
echo "  ✓ Navigation2 installed"

# Install Robot Localization (sensor fusion)
echo ""
echo "--- Installing Robot Localization ---"
sudo apt install -y ros-humble-robot-localization
echo "  ✓ Robot Localization installed (EKF/UKF for IMU+odom fusion)"

# Install visualization tools
echo ""
echo "--- Installing RViz2 & RQt ---"
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-steering
echo "  ✓ Visualization tools installed"

# Install robot description tools
echo ""
echo "--- Installing Robot Description Tools ---"
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros
echo "  ✓ Robot description tools installed"

# Install hardware control
echo ""
echo "--- Installing Hardware Control (ros2_control) ---"
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager
echo "  ✓ Hardware control installed"

# Install IMU tools
echo ""
echo "--- Installing IMU Tools ---"
sudo apt install -y \
    ros-humble-imu-tools \
    ros-humble-imu-filter-madgwick \
    ros-humble-imu-complementary-filter
echo "  ✓ IMU tools installed"

# Install serial communication
echo ""
echo "--- Installing Serial Communication ---"
sudo apt install -y \
    python3-serial \
    python3-pip
echo "  ✓ Serial libraries installed"

# Install Python dependencies
echo ""
echo "--- Installing Python libraries ---"
pip3 install --user \
    pyyaml \
    numpy \
    scipy \
    transforms3d
echo "  ✓ Python dependencies installed"

# Optional: Install Gazebo for simulation
echo ""
read -p "Install Gazebo simulation? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "--- Installing Gazebo Simulation ---"
    sudo apt install -y \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control
    echo "  ✓ Gazebo installed"
else
    echo "  Skipping Gazebo installation"
fi

# Update rosdep
echo ""
echo "--- Updating rosdep ---"
rosdep update
echo "  ✓ rosdep updated"

# Install workspace dependencies
echo ""
echo "--- Installing workspace dependencies ---"
WS="$HOME/ros2_ws"
if [ -d "$WS/src" ]; then
    cd "$WS"
    rosdep install --from-paths src --ignore-src -r -y || true
    echo "  ✓ Workspace dependencies installed"
else
    echo "  Workspace not found, skipping"
fi

# Build workspace
echo ""
echo "--- Building workspace ---"
if [ -d "$WS" ]; then
    cd "$WS"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "  ✓ Workspace built"
else
    echo "  Workspace not found, skipping"
fi

# Add user to dialout group for serial access
echo ""
echo "--- Setting up serial port permissions ---"
if ! groups "$USER" | grep -q "\bdialout\b"; then
    sudo usermod -a -G dialout "$USER"
    echo "  ✓ Added $USER to dialout group"
    echo "  ⚠ IMPORTANT: Log out and log back in for this to take effect"
else
    echo "  dialout group already assigned"
fi

# Create config directory
echo ""
echo "--- Creating configuration directory ---"
CONFIG_DIR="/home/devel/WayfindR-driver/ros2_install_attempt/config"
mkdir -p "$CONFIG_DIR"
echo "  ✓ Created $CONFIG_DIR"

# Summary
echo ""
echo "=================================================================="
echo "  INSTALLATION COMPLETE"
echo "=================================================================="
echo ""
echo "Installed packages:"
echo "  ✓ slam_toolbox - SLAM mapping & localization"
echo "  ✓ navigation2 - Path planning & navigation"
echo "  ✓ nav2_map_server - Map saving/loading"
echo "  ✓ robot_localization - Sensor fusion"
echo "  ✓ rviz2 - 3D visualization"
echo "  ✓ imu_tools - IMU processing"
echo "  ✓ ros2_control - Hardware interfaces"
echo ""
echo "Next steps:"
echo "  1. Log out and log back in (for dialout group)"
echo "  2. Source environment: source ~/.bashrc"
echo "  3. Verify installation: bash 03_verify_installation.sh"
echo ""
echo "Quick test commands:"
echo "  ros2 pkg list | grep slam_toolbox"
echo "  ros2 pkg list | grep navigation2"
echo "  ros2 launch slam_toolbox online_async_launch.py --help"
echo ""
echo "=================================================================="

exit 0
