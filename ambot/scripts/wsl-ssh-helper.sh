#!/bin/bash
# WSL SSH Helper Script
#
# WSL2 uses NAT networking and cannot directly reach devices on the
# Windows host's LAN. This script uses Windows' native SSH instead.
#
# Usage:
#   ./wsl-ssh-helper.sh              # SSH to RPi
#   ./wsl-ssh-helper.sh --rsync      # Rsync ambot folder to RPi
#   ./wsl-ssh-helper.sh --check      # Check if known IP is reachable
#   ./wsl-ssh-helper.sh --test       # Test connectivity
#   ./wsl-ssh-helper.sh --ip <IP>    # Use specific IP

set -e

# Configuration - update these as needed
RPI_USER="pi"
RPI_KNOWN_IP="10.33.224.1"
RPI_MAC="b8:27:eb:5f:11:79"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AMBOT_DIR="$(dirname "$SCRIPT_DIR")"
WIN_SSH="/mnt/c/Windows/System32/OpenSSH/ssh.exe"
WIN_SCP="/mnt/c/Windows/System32/OpenSSH/scp.exe"

# Check if running in WSL
is_wsl() {
    if grep -qi microsoft /proc/version 2>/dev/null; then
        return 0
    fi
    return 1
}

# Get Windows path from WSL path
wsl_to_win_path() {
    local wsl_path="$1"
    wslpath -w "$wsl_path" 2>/dev/null || echo "$wsl_path"
}

# Ping test using Windows ping (works across NAT)
win_ping() {
    local ip="$1"
    /mnt/c/Windows/System32/PING.EXE -n 1 -w 1000 "$ip" 2>/dev/null | grep -q "Reply from"
}

# Check known IP (no subnet scanning - just test known address)
check_known_ip() {
    echo -e "${CYAN}Checking known RPi IP...${NC}"
    echo
    echo "Known IP: $RPI_KNOWN_IP"
    echo "MAC: $RPI_MAC"
    echo

    # Test known IP only - no subnet scanning
    echo -n "Testing $RPI_KNOWN_IP... "
    if win_ping "$RPI_KNOWN_IP"; then
        echo -e "${GREEN}REACHABLE${NC}"
        return 0
    else
        echo -e "${RED}NOT RESPONDING${NC}"
        echo
        echo "The RPi is not responding at the known IP."
        echo "Options:"
        echo "  1. Check RPi desktop for current IP address"
        echo "  2. Use --ip <NEW_IP> to specify a different IP"
        echo "  3. Update RPI_KNOWN_IP in this script"
        return 1
    fi
}

# Test connectivity
test_connection() {
    local ip="${1:-$RPI_KNOWN_IP}"

    echo -e "${CYAN}Testing connection to $ip...${NC}"
    echo

    # Test ping
    echo -n "Windows ping: "
    if win_ping "$ip"; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${RED}FAIL${NC}"
        echo
        echo "The RPi is not responding to ping."
        echo "Possible issues:"
        echo "  1. RPi is powered off or not on WiFi"
        echo "  2. IP address has changed (try --scan)"
        echo "  3. Firewall blocking ICMP"
        return 1
    fi

    # Test SSH port
    echo -n "SSH port (22): "
    if /mnt/c/Windows/System32/PING.EXE -n 1 -w 1000 "$ip" >/dev/null 2>&1; then
        # Use PowerShell to test TCP port
        if powershell.exe -Command "Test-NetConnection -ComputerName $ip -Port 22 -InformationLevel Quiet" 2>/dev/null | grep -q "True"; then
            echo -e "${GREEN}OK${NC}"
        else
            echo -e "${YELLOW}Port may be closed${NC}"
        fi
    fi

    echo
    echo -e "${GREEN}Connection test complete!${NC}"
    echo "Run without arguments to SSH into the RPi."
}

# SSH to RPi using Windows SSH
ssh_to_rpi() {
    local ip="${1:-$RPI_KNOWN_IP}"

    if ! is_wsl; then
        echo "Not running in WSL. Using native SSH..."
        ssh "$RPI_USER@$ip"
        return
    fi

    if [ ! -f "$WIN_SSH" ]; then
        echo -e "${RED}Windows OpenSSH not found at $WIN_SSH${NC}"
        echo "Enable OpenSSH Client in Windows Features, or use:"
        echo "  ssh $RPI_USER@$ip"
        exit 1
    fi

    echo -e "${CYAN}Connecting to $RPI_USER@$ip using Windows SSH...${NC}"
    echo "(This bypasses WSL2 NAT networking issues)"
    echo

    "$WIN_SSH" "$RPI_USER@$ip"
}

# Rsync ambot folder to RPi
rsync_to_rpi() {
    local ip="${1:-$RPI_KNOWN_IP}"

    echo -e "${CYAN}Syncing ambot folder to RPi ($ip)...${NC}"
    echo "Source: $AMBOT_DIR"
    echo "Destination: $RPI_USER@$ip:~/ambot/"
    echo

    if is_wsl; then
        # Use Windows SCP for file transfer (rsync not available in Windows)
        # Fall back to rsync if it works (some network configs allow it)

        echo "Attempting rsync (may fail due to WSL2 NAT)..."
        if rsync -avz --progress \
            --exclude '__pycache__' \
            --exclude '*.pyc' \
            --exclude '.git' \
            --exclude 'tests/results/*.png' \
            --exclude 'tests/results/*.jpg' \
            "$AMBOT_DIR/" "$RPI_USER@$ip:~/ambot/" 2>/dev/null; then
            echo -e "${GREEN}Rsync completed successfully!${NC}"
            return 0
        fi

        echo -e "${YELLOW}Rsync failed (expected with WSL2 NAT).${NC}"
        echo
        echo "Alternative options:"
        echo "  1. Run rsync from Windows (Git Bash or WSL1)"
        echo "  2. Use Windows SCP (slower, no delta sync)"
        echo "  3. Run from native Linux or PowerShell"
        echo
        echo "To use from Windows PowerShell:"
        echo "  scp -r $(wsl_to_win_path "$AMBOT_DIR") $RPI_USER@$ip:~/ambot/"
        return 1
    else
        # Native Linux - use rsync directly
        rsync -avz --progress \
            --exclude '__pycache__' \
            --exclude '*.pyc' \
            --exclude '.git' \
            --exclude 'tests/results/*.png' \
            --exclude 'tests/results/*.jpg' \
            "$AMBOT_DIR/" "$RPI_USER@$ip:~/ambot/"
    fi
}

# Show help
show_help() {
    echo "WSL SSH Helper - Connect to RPi from WSL2"
    echo
    echo "Usage: $0 [OPTIONS]"
    echo
    echo "Options:"
    echo "  (none)        SSH to RPi at known IP ($RPI_KNOWN_IP)"
    echo "  --ip <IP>     SSH to RPi at specific IP"
    echo "  --rsync       Sync ambot folder to RPi"
    echo "  --check       Check if known IP is reachable"
    echo "  --test        Test connectivity to RPi (ping + SSH port)"
    echo "  --help        Show this help"
    echo
    echo "Configuration (edit script to change):"
    echo "  RPI_USER=$RPI_USER"
    echo "  RPI_KNOWN_IP=$RPI_KNOWN_IP"
    echo "  RPI_MAC=$RPI_MAC"
    echo
    echo "Why this script exists:"
    echo "  WSL2 uses a virtual NAT network and cannot directly"
    echo "  reach devices on the Windows host's LAN. This script"
    echo "  uses Windows' native SSH client which CAN reach them."
    echo
    echo "Alternative solutions:"
    echo "  1. Enable mirrored networking in .wslconfig (Windows 11 22H2+)"
    echo "  2. Use ssh.exe directly: /mnt/c/Windows/System32/OpenSSH/ssh.exe pi@IP"
    echo "  3. Run SSH from Windows Terminal/PowerShell"
}

# Parse arguments
IP_OVERRIDE=""
ACTION="ssh"

while [[ $# -gt 0 ]]; do
    case $1 in
        --ip)
            IP_OVERRIDE="$2"
            shift 2
            ;;
        --rsync)
            ACTION="rsync"
            shift
            ;;
        --check)
            ACTION="check"
            shift
            ;;
        --test)
            ACTION="test"
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Determine IP to use
TARGET_IP="${IP_OVERRIDE:-$RPI_KNOWN_IP}"

# Execute action
case $ACTION in
    ssh)
        ssh_to_rpi "$TARGET_IP"
        ;;
    rsync)
        rsync_to_rpi "$TARGET_IP"
        ;;
    check)
        check_known_ip
        ;;
    test)
        test_connection "$TARGET_IP"
        ;;
esac
