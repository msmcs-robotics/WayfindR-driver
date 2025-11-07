#!/usr/bin/env bash
# fix_ros2_apt_sources.sh
# Check for duplicate ROS2 apt source entries and clean them up,
# then continue installation if needed.

set -e
set -u

echo "=== FIXING ROS2 APT SOURCE CONFLICTS ==="

# Path definitions
ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
ROS2_SOURCES="/etc/apt/sources.list.d/ros2.sources"

# If duplicate .sources file exists, move it aside
if [ -f "$ROS2_SOURCES" ]; then
  echo "Found duplicate source file: $ROS2_SOURCES"
  sudo mv "$ROS2_SOURCES" "${ROS2_SOURCES}.backup"
  echo "Moved $ROS2_SOURCES → ${ROS2_SOURCES}.backup"
else
  echo "No duplicate .sources file found; OK"
fi

# Ensure the .list file exists and is correct
if [ -f "$ROS2_LIST" ]; then
  echo "ROS2 list file exists: $ROS2_LIST"
else
  echo "ROS2 list file not found; creating it"
  sudo tee "$ROS2_LIST" > /dev/null <<EOF
deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo \$UBUNTU_CODENAME) main
EOF
  echo "Created $ROS2_LIST"
fi

# Update apt sources
echo "Updating package list..."
sudo apt update

# Optionally install ROS2 if not yet installed
echo "Installing ROS2 Humble desktop (if not already)..."
sudo apt install -y ros-humble-desktop

echo "Fix and installation steps complete. Please open a new terminal."
exit 0