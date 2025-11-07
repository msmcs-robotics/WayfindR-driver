#!/usr/bin/env bash
# install_ros2_humble.sh
# Usage: sudo bash install_ros2_humble.sh
# This script installs ROS 2 Humble on Ubuntu 22.04, with checks and cleanup.

set -e  # exit on error
set -u  # treat unset variables as error
# Optional: set -o pipefail

echo "=== INSTALL ROS 2 HUMBLE ON UBUNTU 22.04 ==="

# 1. Update & upgrade system
echo "--- Step 1: system update/upgrade"
sudo apt update
sudo apt upgrade -y

# 2. Locale setup
echo "--- Step 2: locale setup"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "locale set to:" $(locale | grep LANG)

# 3. Ensure Universe repository
echo "--- Step 3: ensure Universe repository"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# 4. Add ROS 2 apt repository (with cleanup if old)
ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
ROS2_KEY="/usr/share/keyrings/ros-archive-keyring.gpg"

echo "--- Step 4: configure ROS 2 apt repository"
if [ -f "$ROS2_LIST" ]; then
  echo "Found existing $ROS2_LIST — removing to clean old sources"
  sudo rm -f "$ROS2_LIST"
fi
if [ -f "$ROS2_KEY" ]; then
  echo "Found existing ROS 2 keyring — removing"
  sudo rm -f "$ROS2_KEY"
fi

sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o "$ROS2_KEY"
echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS2_KEY}] http://packages.ros.org/ros2/ubuntu \
      $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
     sudo tee "$ROS2_LIST" > /dev/null

# 5. Install ROS 2 desktop
echo "--- Step 5: install ROS 2 Humble desktop"
sudo apt update
sudo apt install -y ros-humble-desktop
# Optional dev tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# 6. Source setup script automatically
echo "--- Step 6: environment setup"
BASHRC="$HOME/.bashrc"
SOURCE_LINE="source /opt/ros/humble/setup.bash"
if grep -Fxq "$SOURCE_LINE" "$BASHRC"; then
  echo "Already sourcing ROS 2 setup in $BASHRC"
else
  echo "$SOURCE_LINE" >> "$BASHRC"
  echo "Added sourcing line to $BASHRC"
fi
# Immediately source for this session
source /opt/ros/humble/setup.bash

# 7. Setup a ROS2 workspace
echo "--- Step 7: create ROS2 workspace"
WS_DIR="$HOME/ros2_ws"
if [ ! -d "$WS_DIR/src" ]; then
  mkdir -p "$WS_DIR/src"
  echo "Created workspace at $WS_DIR"
else
  echo "Workspace $WS_DIR already exists"
fi

cd "$WS_DIR"
colcon build --symlink-install || {
  echo "Workspace build failed or nothing to build – continuing anyway"
}

echo "Add workspace source to bashrc if not present"
WS_SOURCE_LINE="source $WS_DIR/install/setup.bash"
if grep -Fxq "$WS_SOURCE_LINE" "$BASHRC"; then
  echo "Workspace sourcing already in $BASHRC"
else
  echo "$WS_SOURCE_LINE" >> "$BASHRC"
  echo "Added workspace sourcing to $BASHRC"
fi

echo "Installation complete. Please open a new terminal or run `source ~/.bashrc`."

echo "You can test via:"
echo "  ros2 run demo_nodes_cpp talker"
echo "  ros2 run demo_nodes_py listener"

exit 0