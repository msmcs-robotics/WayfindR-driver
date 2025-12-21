#!/bin/bash
#
# 05_install_extras.sh - Install additional useful packages
#
# Optional extras for robot development and monitoring.
#

set -e

LOG_PREFIX="[05_install_extras]"

log() {
    echo "$LOG_PREFIX $1"
}

log "Starting extras installation..."

# Source ROS2
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

# Wait for apt lock
wait_for_apt() {
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        log "Waiting for apt lock..."
        sleep 5
    done
}

wait_for_apt

log "Installing IMU tools..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-imu-tools \
    ros-humble-imu-filter-madgwick || log "IMU tools not available, skipping"

log "Installing ROS2 control packages..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers || log "ROS2 control not available, skipping"

log "Installing Python packages..."
pip3 install --upgrade \
    numpy \
    scipy \
    pyyaml \
    transforms3d || log "Some Python packages failed"

log "Installing system monitoring tools..."
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    htop \
    iotop \
    nethogs \
    nmap \
    screen \
    tmux

log "Setting up convenience aliases..."
cat >> /home/ubuntu/.bashrc << 'EOF'

# ROS2 Convenience Aliases
alias ros2_nodes='ros2 node list'
alias ros2_topics='ros2 topic list'
alias ros2_services='ros2 service list'
alias ros2_tf='ros2 run tf2_tools view_frames'

# System aliases
alias ll='ls -la'
alias ports='netstat -tuln'
alias diskspace='df -h'
alias meminfo='free -h'
EOF

log "Creating system info script..."
cat > /home/ubuntu/system_info.sh << 'EOF'
#!/bin/bash
# Display system information

echo "=========================================="
echo "  WayfindR System Information"
echo "=========================================="
echo ""
echo "Hostname: $(hostname)"
echo "IP Address: $(hostname -I | awk '{print $1}')"
echo "Uptime: $(uptime -p)"
echo ""
echo "CPU: $(nproc) cores"
echo "Memory: $(free -h | awk '/^Mem:/ {print $2}')"
echo "Disk: $(df -h / | awk 'NR==2 {print $4 " free of " $2}')"
echo ""
echo "ROS2 Version:"
source /opt/ros/humble/setup.bash 2>/dev/null && ros2 --version || echo "  Not installed"
echo ""
echo "LiDAR Devices:"
ls -la /dev/ttyUSB* 2>/dev/null || echo "  None detected"
echo ""
echo "=========================================="
EOF

chmod +x /home/ubuntu/system_info.sh
chown ubuntu:ubuntu /home/ubuntu/system_info.sh

log "Setting up auto-login for serial console (optional)..."
# This helps with headless debugging via serial
mkdir -p /etc/systemd/system/getty@tty1.service.d/
cat > /etc/systemd/system/getty@tty1.service.d/override.conf << 'EOF'
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin ubuntu --noclear %I $TERM
EOF

log "Extras installation complete!"
