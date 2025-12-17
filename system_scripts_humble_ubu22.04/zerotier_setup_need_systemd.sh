#!/usr/bin/env bash
# ZeroTier installer + network join
# Works on Ubuntu 22.04+ and WSL
set -e

echo "==== ZeroTier Setup Script ===="

# Prompt for Network ID
read -rp "Enter your ZeroTier Network ID: " ZT_NETWORK

# Install ZeroTier if not installed
if ! command -v zerotier-cli >/dev/null 2>&1; then
    echo "Installing ZeroTier..."
    curl -s https://install.zerotier.com | sudo bash
else
    echo "ZeroTier already installed."
fi

# Detect if systemd is available
if pidof systemd >/dev/null 2>&1; then
    echo "Systemd detected. Enabling and starting zerotier-one service..."
    sudo systemctl enable zerotier-one
    sudo systemctl start zerotier-one
else
    echo "Systemd not detected. Using manual daemon start."
    # Check if daemon is running
    if ! pgrep -x zerotier-one >/dev/null; then
        sudo zerotier-one -d
        echo "ZeroTier daemon started in background."
    else
        echo "ZeroTier daemon already running."
    fi

    # Setup cron job to ensure daemon stays running
    (crontab -l 2>/dev/null | grep -v zerotier-one ; echo "* * * * * pgrep -x zerotier-one >/dev/null || sudo zerotier-one -d") | crontab -
    echo "Cron job added to keep ZeroTier daemon alive."
fi

# Join network if not already joined
if zerotier-cli listnetworks | grep -q "$ZT_NETWORK"; then
    echo "Already joined network $ZT_NETWORK"
else
    echo "Joining ZeroTier network $ZT_NETWORK..."
    sudo zerotier-cli join "$ZT_NETWORK"
fi

echo
echo "==== Setup Complete ===="
echo "Check status with: sudo zerotier-cli info"
echo "Check IP with: ip addr show zt*"
