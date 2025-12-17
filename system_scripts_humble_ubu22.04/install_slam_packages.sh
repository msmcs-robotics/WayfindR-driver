#!/usr/bin/env bash
# install_ros2_humble_full.sh
# ROS2 Humble SLAM + Nav2 + Exploration installer
# Ubuntu 22.04 (Jammy) with Python 3.12

set -e
set -o pipefail

echo ""
echo "======================================================================"
echo "  ROS2 HUMBLE: SLAM + NAVIGATION + EXPLORATION INSTALLER"
echo "======================================================================"
echo ""

read -p "Press ENTER to begin (CTRL+C to cancel)..."

# ------------------------------------------------------------------------------
# WAIT FOR APT LOCK (unattended-upgrades)
# ------------------------------------------------------------------------------
echo ""
echo "--- Checking for APT locks ---"
while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
    echo "  ⏳ Waiting for unattended upgrades..."
    sleep 3
done
echo "  ✓ APT lock free"

# ------------------------------------------------------------------------------
# SOURCE ROS 2
# ------------------------------------------------------------------------------
echo ""
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble environment sourced"
else
    echo "✗ ERROR: ROS2 Humble not found at /opt/ros/humble"
    exit 1
fi

# ------------------------------------------------------------------------------
# UPDATE SYSTEM
# ------------------------------------------------------------------------------
echo ""
echo "--- Updating package lists ---"
sudo apt update

# ------------------------------------------------------------------------------
# INSTALL CORE ROS2 PACKAGES
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing Core ROS2 Packages ---"
sudo apt install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools

# ------------------------------------------------------------------------------
# INSTALL SLAM
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing SLAM Packages ---"
sudo apt install -y ros-humble-slam-toolbox

# ------------------------------------------------------------------------------
# INSTALL NAVIGATION (NAV2)
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing Navigation2 ---"
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-localization

# ------------------------------------------------------------------------------
# INSTALL VISUALIZATION
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing RViz & RQt ---"
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# ------------------------------------------------------------------------------
# INSTALL SIMULATION (Gazebo)
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing Gazebo ---"
sudo apt install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

# ------------------------------------------------------------------------------
# HARDWARE & PYTHON DEPENDENCIES
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing Hardware Interfaces ---"
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-serial \
    python3-pip

echo ""
echo "--- Installing Python libraries ---"
pip3 install --user \
    flask flask-cors \
    pyserial pyyaml numpy scipy matplotlib transforms3d

# ------------------------------------------------------------------------------
# IMU / SENSOR TOOLS
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing IMU Tools ---"
sudo apt install -y \
    ros-humble-imu-tools \
    ros-humble-imu-filter-madgwick

# ------------------------------------------------------------------------------
# ROSDEP INIT & UPDATE (only if needed)
# ------------------------------------------------------------------------------
echo ""
echo "--- Checking rosdep initialization ---"

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo "🧰 rosdep not initialized — running 'sudo rosdep init'"
    sudo rosdep init
    rosdep update
else
    echo "🧰 rosdep already initialized"
    echo "→ Updating rosdep database"
    rosdep update
fi

# ------------------------------------------------------------------------------
# CREATE WORKSPACE
# ------------------------------------------------------------------------------
echo ""
echo "--- Setting up ROS2 workspace ---"
WS="$HOME/ros2_ws"
SRC="$WS/src"
mkdir -p "$SRC"
cd "$SRC"

# ------------------------------------------------------------------------------
# CLONE EXPLORATION PACKAGE (frontier exploration for ROS2 Humble)
# ------------------------------------------------------------------------------
echo ""
echo "--- Cloning exploration package ---"
if [ ! -d "AutoFrontierSearch_ros2-humble" ]; then
    git clone https://github.com/Nyanziba/AutoFrontierSearch_ros2-humble.git
else
    echo "→ Exploration package already cloned"
fi

# ------------------------------------------------------------------------------
# INSTALL ROS DEPENDENCIES for workspace
# ------------------------------------------------------------------------------
echo ""
echo "--- Installing rosdep dependencies for workspace ---"
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || true

# ------------------------------------------------------------------------------
# BUILD WORKSPACE
# ------------------------------------------------------------------------------
echo ""
echo "--- Building ROS2 workspace ---"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ------------------------------------------------------------------------------
# USER PERMISSIONS
# ------------------------------------------------------------------------------
echo ""
echo "--- Setting up user permissions ---"
if ! groups "$USER" | grep -q "\bdialout\b"; then
    sudo usermod -a -G dialout "$USER"
    echo "➕ Added $USER to dialout group (serial access)"
    echo "⚠ Please LOG OUT & LOG BACK IN before using serial devices"
else
    echo "→ dialout group already present"
fi

# ------------------------------------------------------------------------------
# SUMMARY
# ------------------------------------------------------------------------------
echo ""
echo "======================================================================"
echo "  INSTALLATION COMPLETE"
echo "======================================================================"
echo ""
echo "Next steps:"
echo " 1) Open a NEW terminal or source your setup files:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ~/ros2_ws/install/setup.bash"
echo ""
echo " 2) Run SLAM Toolbox:"
echo "    ros2 launch slam_toolbox online_async_launch.py"
echo ""
echo " 3) Run navigation:"
echo "    ros2 launch nav2_bringup navigation_launch.py"
echo ""
echo " 4) Run frontier exploration:"
echo "    ros2 run frontier_exploration exploration_node"
echo ""
echo "======================================================================"

exit 0
