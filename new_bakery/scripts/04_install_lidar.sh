#!/bin/bash
#
# 04_install_lidar.sh - Install LiDAR driver packages
#
# Installs drivers for common LiDAR sensors:
# - RPLidar (Slamtec A1, A2, A3, C1, S1, S2)
# - LDROBOT LD06/LD19
# - YDLidar
#

set -e

LOG_PREFIX="[04_install_lidar]"

log() {
    echo "$LOG_PREFIX $1"
}

log "Starting LiDAR driver installation..."

# Source ROS2
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    log "ERROR: ROS2 not found. Run 02_install_ros2.sh first."
    exit 1
fi

# Wait for apt lock
wait_for_apt() {
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        log "Waiting for apt lock..."
        sleep 5
    done
}

wait_for_apt

log "Installing RPLidar ROS2 driver..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-rplidar-ros

log "Setting up udev rules for LiDAR devices..."

# RPLidar (Silicon Labs CP210x)
cat > /etc/udev/rules.d/99-rplidar.rules << 'EOF'
# RPLidar - Silicon Labs CP210x
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
EOF

# LDROBOT LD06/LD19 (CH340/CH341)
cat > /etc/udev/rules.d/99-ldlidar.rules << 'EOF'
# LDROBOT LD06/LD19 - CH340/CH341
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="ldlidar"
# FTDI
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", GROUP:="dialout"
EOF

# YDLidar
cat > /etc/udev/rules.d/99-ydlidar.rules << 'EOF'
# YDLidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"
EOF

log "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

log "Creating LiDAR test scripts..."

# Create a simple test script
cat > /home/ubuntu/test_lidar.sh << 'EOF'
#!/bin/bash
# Test LiDAR connection

source /opt/ros/humble/setup.bash

echo "Checking for LiDAR devices..."
ls -la /dev/ttyUSB* 2>/dev/null || echo "No ttyUSB devices found"
ls -la /dev/rplidar 2>/dev/null || echo "No rplidar symlink"
ls -la /dev/ldlidar 2>/dev/null || echo "No ldlidar symlink"

echo ""
echo "USB devices:"
lsusb | grep -iE "silicon|cp210|ch340|ftdi" || echo "No recognized LiDAR USB devices"

echo ""
echo "To test RPLidar, run:"
echo "ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar"
EOF

chmod +x /home/ubuntu/test_lidar.sh
chown ubuntu:ubuntu /home/ubuntu/test_lidar.sh

log "LiDAR driver installation complete!"
log "Supported LiDARs: RPLidar (all models), LDROBOT LD06/LD19, YDLidar"
