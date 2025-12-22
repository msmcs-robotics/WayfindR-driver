#!/bin/bash
#
# Set up LiDAR permissions and udev rules
#
# Usage: ./setup_lidar.sh
#

set -e

echo "============================================"
echo "  LiDAR Setup"
echo "============================================"
echo ""

# Check if running as root (not recommended)
if [ "$EUID" -eq 0 ]; then
    echo "Warning: Running as root. Better to run as normal user with sudo."
fi

# Create udev rule for Slamtec RPLidar
echo "Creating udev rules for RPLidar..."

sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
# Slamtec RPLidar devices
# Creates symlink at /dev/rplidar and sets permissions
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="rplidar"
EOF

# Reload udev rules
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
echo "Adding $USER to dialout group..."
sudo usermod -aG dialout $USER

# Check current device
echo ""
echo "Checking for LiDAR device..."

if ls /dev/ttyUSB* 2>/dev/null; then
    echo "Found USB serial device(s):"
    ls -la /dev/ttyUSB*
else
    echo "No USB serial devices found."
    echo "Make sure LiDAR is connected via USB."
fi

# Check for symlink
if [ -e /dev/rplidar ]; then
    echo ""
    echo "RPLidar symlink exists:"
    ls -la /dev/rplidar
else
    echo ""
    echo "Note: /dev/rplidar symlink will appear after reconnecting LiDAR."
fi

echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo ""
echo "IMPORTANT: Log out and back in for group changes to take effect."
echo ""
echo "To test LiDAR:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0"
echo ""
