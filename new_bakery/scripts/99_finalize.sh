#!/bin/bash
#
# 99_finalize.sh - Final setup and cleanup
#
# This runs last to finalize the installation.
#

set -e

LOG_PREFIX="[99_finalize]"

log() {
    echo "$LOG_PREFIX $1"
}

log "Starting finalization..."

# Clean up apt cache to save space
log "Cleaning apt cache..."
apt-get clean
apt-get autoremove -y

# Update file ownership
log "Fixing file permissions..."
chown -R ubuntu:ubuntu /home/ubuntu/

# Create a marker file with installation info
log "Creating installation marker..."
cat > /home/ubuntu/.wayfinder_installed << EOF
WayfindR Robot Setup
====================
Installation Date: $(date)
Hostname: $(hostname)
Ubuntu Version: $(lsb_release -ds)
Kernel: $(uname -r)
Architecture: $(uname -m)

ROS2 Version: $(source /opt/ros/humble/setup.bash 2>/dev/null && ros2 --version 2>/dev/null || echo "Not verified")

Installed Components:
- ROS2 Humble (ros-base)
- SLAM Toolbox
- Navigation2
- RPLidar Driver
- Robot Localization

Workspace: ~/ros2_ws
Maps Directory: ~/ros2_ws/maps

Quick Commands:
- ros2 topic list
- ros2 node list
- ~/test_lidar.sh
- ~/system_info.sh
EOF

chown ubuntu:ubuntu /home/ubuntu/.wayfinder_installed

# Display completion message
log "=========================================="
log "  WayfindR Setup Complete!"
log "=========================================="
log ""
log "The system is ready for robotics development."
log ""
log "Next steps:"
log "  1. Reboot the system: sudo reboot"
log "  2. Connect your LiDAR"
log "  3. Run: ~/test_lidar.sh"
log "  4. Start mapping!"
log ""
log "For remote access:"
log "  ssh ubuntu@$(hostname).local"
log ""
log "=========================================="

# Optional: reboot after installation
# Uncomment the following line to auto-reboot
# log "Rebooting in 10 seconds..."
# sleep 10
# reboot
