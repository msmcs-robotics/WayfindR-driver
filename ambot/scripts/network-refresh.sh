#!/bin/bash
# Network Refresh Script for Raspberry Pi
# Run this ON the RPi when network connectivity issues occur
#
# Usage: sudo ./network-refresh.sh
#        sudo ./network-refresh.sh --status   # Just show status
#        sudo ./network-refresh.sh --force    # Force full restart

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================"
echo "  RPi Network Troubleshooter"
echo "========================================"
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root: sudo $0${NC}"
    exit 1
fi

# Show current status
show_status() {
    echo "--- Current Network Status ---"
    echo

    # Get interfaces
    echo "Interfaces:"
    ip -br link show
    echo

    # Get IP addresses
    echo "IP Addresses:"
    ip -br addr show
    echo

    # Get default route
    echo "Default Route:"
    ip route show default 2>/dev/null || echo "  (none)"
    echo

    # Get DNS
    echo "DNS Servers:"
    grep nameserver /etc/resolv.conf 2>/dev/null || echo "  (none)"
    echo

    # Test connectivity
    echo "Connectivity Tests:"
    if ping -c 1 -W 2 8.8.8.8 &>/dev/null; then
        echo -e "  Internet (8.8.8.8): ${GREEN}OK${NC}"
    else
        echo -e "  Internet (8.8.8.8): ${RED}FAIL${NC}"
    fi

    if ping -c 1 -W 2 $(ip route | grep default | awk '{print $3}') &>/dev/null; then
        echo -e "  Gateway: ${GREEN}OK${NC}"
    else
        echo -e "  Gateway: ${RED}FAIL${NC}"
    fi
    echo
}

# Refresh DHCP lease
refresh_dhcp() {
    echo "--- Refreshing DHCP Lease ---"

    # Find the main interface (eth0 or wlan0)
    IFACE=$(ip route | grep default | awk '{print $5}' | head -1)

    if [ -z "$IFACE" ]; then
        # No default route, try common interfaces
        if ip link show eth0 &>/dev/null; then
            IFACE="eth0"
        elif ip link show wlan0 &>/dev/null; then
            IFACE="wlan0"
        else
            echo -e "${RED}Could not determine network interface${NC}"
            return 1
        fi
    fi

    echo "Using interface: $IFACE"

    # Release and renew DHCP
    if command -v dhclient &>/dev/null; then
        echo "Using dhclient..."
        dhclient -r "$IFACE" 2>/dev/null || true
        sleep 1
        dhclient "$IFACE"
    elif command -v dhcpcd &>/dev/null; then
        echo "Using dhcpcd..."
        dhcpcd -k "$IFACE" 2>/dev/null || true
        sleep 1
        dhcpcd "$IFACE"
    else
        echo "No DHCP client found, restarting interface..."
        ip link set "$IFACE" down
        sleep 2
        ip link set "$IFACE" up
        sleep 3
    fi

    echo -e "${GREEN}DHCP refresh complete${NC}"
    echo
}

# Restart networking service
restart_networking() {
    echo "--- Restarting Network Service ---"

    if systemctl is-active --quiet NetworkManager; then
        echo "Restarting NetworkManager..."
        systemctl restart NetworkManager
    elif systemctl is-active --quiet networking; then
        echo "Restarting networking service..."
        systemctl restart networking
    elif systemctl is-active --quiet dhcpcd; then
        echo "Restarting dhcpcd..."
        systemctl restart dhcpcd
    else
        echo "No recognized network service found"
        echo "Manually bringing interface up/down..."

        for iface in eth0 wlan0; do
            if ip link show "$iface" &>/dev/null; then
                ip link set "$iface" down
                sleep 1
                ip link set "$iface" up
            fi
        done
    fi

    sleep 3
    echo -e "${GREEN}Network service restarted${NC}"
    echo
}

# Full network reset
full_reset() {
    echo "--- Full Network Reset ---"
    echo -e "${YELLOW}This will briefly disconnect all network connections${NC}"
    echo

    # Flush routes and addresses
    echo "Flushing network configuration..."

    for iface in eth0 wlan0; do
        if ip link show "$iface" &>/dev/null; then
            ip addr flush dev "$iface" 2>/dev/null || true
            ip route flush dev "$iface" 2>/dev/null || true
        fi
    done

    # Restart service
    restart_networking

    # Wait for DHCP
    echo "Waiting for DHCP..."
    sleep 5

    refresh_dhcp
}

# Parse arguments
case "${1:-}" in
    --status|-s)
        show_status
        ;;
    --force|-f)
        show_status
        full_reset
        show_status
        ;;
    --help|-h)
        echo "Usage: sudo $0 [OPTION]"
        echo
        echo "Options:"
        echo "  --status, -s   Show network status only"
        echo "  --force, -f    Full network reset"
        echo "  --help, -h     Show this help"
        echo
        echo "Default: Refresh DHCP lease and show status"
        ;;
    *)
        show_status
        refresh_dhcp
        show_status
        ;;
esac

echo "========================================"
echo "  Done. Check if SSH works now."
echo "========================================"
