#!/usr/bin/env bash
# fix_ros2_apt_sources.sh
# Safe first-boot ROS2 installation for Raspberry Pi Baker images.

set -euo pipefail

echo "=== ROS2 FIRST-BOOT INSTALLER STARTED ==="

# Load configuration (username, sudo behavior)
USERNAME=$(jq -r '.username' /tmp/baker-config.json)
SUDO_NOPASSWD=$(jq -r '.enable_sudo_nopasswd' /tmp/baker-config.json)

# Helper: always call sudo safely
SUDO="sudo"
if [ "$SUDO_NOPASSWD" = "true" ]; then
    echo "✓ Running with passwordless sudo"
else
    echo "⚠ Using normal sudo (password may be required)"
fi

# File paths
ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
ROS2_SOURCES="/etc/apt/sources.list.d/ros2.sources"
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"

echo "=== Checking for duplicate ROS2 apt entries ==="
if [ -f "$ROS2_SOURCES" ]; then
    echo "Found deprecated: $ROS2_SOURCES"
    $SUDO mv "$ROS2_SOURCES" "${ROS2_SOURCES}.backup"
    echo "→ Moved to ${ROS2_SOURCES}.backup"
else
    echo "✓ No .sources file present — OK"
fi

echo "=== Checking ROS2 GPG keyring ==="
if [ ! -f "$KEYRING" ]; then
    echo "Installing ROS2 GPG key..."
    $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o "$KEYRING"
    $SUDO chmod 644 "$KEYRING"
else
    echo "✓ GPG key already present"
fi

echo "=== Checking ROS2 list file ==="
UBUNTU_CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")

if [ ! -f "$ROS2_LIST" ]; then
    echo "Creating $ROS2_LIST ..."
    $SUDO tee "$ROS2_LIST" > /dev/null <<EOF
deb [arch=$(dpkg --print-architecture) signed-by=$KEYRING] \
  http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main
EOF
    echo "✓ Created ROS2 apt source"
else
    echo "✓ ros2.list already exists"
fi

echo "=== Updating apt sources ==="
$SUDO apt update -y || true

echo "=== Checking for ROS2 installation ==="
if dpkg -l | grep -q "^ii  ros-humble-desktop"; then
    echo "✓ ROS2 Humble already installed"
else
    echo "Installing ROS2 Humble Desktop..."
    $SUDO apt install -y ros-humble-desktop
    echo "✓ ROS2 Installation Complete"
fi

echo "=== FIRST-BOOT ROS2 SETUP FINISHED ==="
exit 0
