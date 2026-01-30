#!/bin/bash
# =============================================================================
# Setup udev Rules for RPLidar
# =============================================================================
# Creates udev rules for consistent device naming (/dev/rplidar)
# Based on patterns from new_bakery/scripts/04_install_lidar.sh
#
# Usage:
#   sudo ./setup-udev.sh
#
# After running, the RPLidar will always appear as /dev/rplidar
# regardless of which USB port it's plugged into.
# =============================================================================

set -e

RULES_FILE="/etc/udev/rules.d/99-rplidar.rules"

# Check for root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root (sudo)"
    echo "Usage: sudo $0"
    exit 1
fi

echo "Setting up udev rules for RPLidar..."
echo ""

# Create udev rules file
cat > "$RULES_FILE" << 'EOF'
# =============================================================================
# RPLidar udev Rules
# =============================================================================
# Creates /dev/rplidar symlink for Slamtec RPLidar devices
# Supports: RPLidar A1, A2, A3, C1 (C1M1), S1, S2
#
# The Silicon Labs CP210x USB to UART bridge is used by most RPLidar models.
# Vendor ID: 10c4 (Silicon Labs)
# Product ID: ea60 (CP210x UART Bridge)
# =============================================================================

# RPLidar - Silicon Labs CP210x USB to UART Bridge
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"

# Alternative: FTDI-based serial adapters (some LiDAR models)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", GROUP:="dialout"

# Alternative: CH340/CH341 serial adapters
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout"
EOF

echo "Created $RULES_FILE:"
cat "$RULES_FILE"
echo ""

# Reload udev rules
echo "Reloading udev rules..."
udevadm control --reload-rules

# Trigger udev to apply rules to existing devices
echo "Triggering udev for existing devices..."
udevadm trigger

echo ""
echo "Done!"
echo ""

# Check if device exists now
if [ -e /dev/rplidar ]; then
    REAL_PATH=$(readlink -f /dev/rplidar)
    echo "Success: /dev/rplidar -> $REAL_PATH"
else
    echo "Note: /dev/rplidar will be created when LiDAR is plugged in"
fi

echo ""
echo "If the LiDAR is already connected, you may need to:"
echo "  1. Unplug and replug the USB cable, or"
echo "  2. Run: sudo udevadm trigger"
echo ""
echo "Verify with: ls -la /dev/rplidar"
