#!/bin/bash
#
# bake_auto.sh - Fully Automatic Raspberry Pi SD Card Setup
#
# This script is FULLY AUTOMATIC:
# 1. Detects inserted SD cards
# 2. Flashes Ubuntu 22.04 image
# 3. Configures cloud-init for SSH (no blank file needed!)
# 4. Copies ROS2 installation scripts
# 5. Ejects SD card safely
#
# YOU JUST:
# 1. Insert SD card
# 2. Run this script
# 3. Remove SD card, insert in Pi, power on
# 4. Wait 25 minutes
# 5. SSH in: ssh ubuntu@<hostname>.local
#
# Usage:
#   sudo ./bake_auto.sh
#   sudo ./bake_auto.sh --image /path/to/ubuntu.img.xz --hostname myrobot
#

set -e

# ============================================================================
# CONFIGURATION
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default Ubuntu image (will prompt if not found)
DEFAULT_IMAGE_DIR="$HOME/Downloads"
DEFAULT_IMAGE_NAME="ubuntu-22.04*-preinstalled-server-arm64+raspi.img*"

# Default SSH key (will use first found)
DEFAULT_SSH_KEY="$HOME/.ssh/id_rsa.pub"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ============================================================================
# LOGGING
# ============================================================================

log_header() {
    echo ""
    echo -e "${CYAN}${BOLD}=== $1 ===${NC}"
    echo ""
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[!]${NC} $1"
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
}

log_step() {
    echo -e "${BOLD}→${NC} $1"
}

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root"
        echo ""
        echo "Usage: sudo $0 [options]"
        exit 1
    fi
}

check_dependencies() {
    local missing=()

    for cmd in dd lsblk findmnt partprobe mktemp xz; do
        if ! command -v $cmd &> /dev/null; then
            missing+=($cmd)
        fi
    done

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_error "Missing required commands: ${missing[*]}"
        log_info "Install with: sudo apt-get install coreutils util-linux xz-utils"
        exit 1
    fi
}

# Detect removable storage devices (SD cards, USB drives)
detect_removable_devices() {
    log_info "Scanning for removable storage devices..."

    # Get root device to exclude
    local root_dev=$(findmnt -n -o SOURCE / | sed 's/[0-9]*$//' | sed 's/p$//')

    # Find removable devices
    DEVICES=()
    while IFS= read -r line; do
        local dev=$(echo "$line" | awk '{print $1}')
        local size=$(echo "$line" | awk '{print $2}')
        local model=$(echo "$line" | awk '{print $3}')

        # Skip root device
        if [[ "/dev/$dev" == "$root_dev"* ]]; then
            continue
        fi

        # Skip if too small (<4GB) or too large (>256GB, probably not SD)
        local size_bytes=$(lsblk -b -n -o SIZE "/dev/$dev" 2>/dev/null | head -1)
        if [[ -n "$size_bytes" ]]; then
            local size_gb=$((size_bytes / 1024 / 1024 / 1024))
            if [[ $size_gb -lt 4 || $size_gb -gt 256 ]]; then
                continue
            fi
        fi

        DEVICES+=("/dev/$dev|$size|$model")
    done < <(lsblk -d -n -o NAME,SIZE,MODEL | grep -E "^sd|^mmcblk")

    if [[ ${#DEVICES[@]} -eq 0 ]]; then
        return 1
    fi
    return 0
}

# Interactive device selection
select_device() {
    log_header "SELECT SD CARD"

    if ! detect_removable_devices; then
        log_error "No removable storage devices found!"
        log_info "Please insert an SD card and try again."
        exit 1
    fi

    echo "Found ${#DEVICES[@]} removable device(s):"
    echo ""

    local i=1
    for dev_info in "${DEVICES[@]}"; do
        local dev=$(echo "$dev_info" | cut -d'|' -f1)
        local size=$(echo "$dev_info" | cut -d'|' -f2)
        local model=$(echo "$dev_info" | cut -d'|' -f3)

        echo "  $i) $dev - $size - $model"
        ((i++))
    done
    echo ""

    if [[ ${#DEVICES[@]} -eq 1 ]]; then
        DEVICE=$(echo "${DEVICES[0]}" | cut -d'|' -f1)
        log_info "Auto-selecting only device: $DEVICE"
        read -p "Press Enter to continue or Ctrl+C to cancel..."
    else
        read -p "Select device [1-${#DEVICES[@]}]: " selection
        if [[ ! "$selection" =~ ^[0-9]+$ ]] || [[ $selection -lt 1 ]] || [[ $selection -gt ${#DEVICES[@]} ]]; then
            log_error "Invalid selection"
            exit 1
        fi
        DEVICE=$(echo "${DEVICES[$((selection-1))]}" | cut -d'|' -f1)
    fi

    # Final confirmation
    echo ""
    log_warn "WARNING: ALL DATA ON $DEVICE WILL BE DESTROYED!"
    lsblk "$DEVICE"
    echo ""
    read -p "Type 'yes' to confirm: " confirm
    if [[ "$confirm" != "yes" ]]; then
        log_info "Aborted by user."
        exit 0
    fi
}

# Find Ubuntu image
find_image() {
    log_header "LOCATE UBUNTU IMAGE"

    if [[ -n "$IMAGE_PATH" && -f "$IMAGE_PATH" ]]; then
        log_success "Using specified image: $IMAGE_PATH"
        return 0
    fi

    # Search common locations
    local search_paths=(
        "$DEFAULT_IMAGE_DIR"
        "$HOME"
        "/tmp"
        "."
    )

    for path in "${search_paths[@]}"; do
        local found=$(find "$path" -maxdepth 2 -name "$DEFAULT_IMAGE_NAME" 2>/dev/null | head -1)
        if [[ -n "$found" && -f "$found" ]]; then
            IMAGE_PATH="$found"
            log_success "Found image: $IMAGE_PATH"
            return 0
        fi
    done

    # Not found - prompt user
    log_warn "Ubuntu image not found automatically."
    echo ""
    echo "Please download from:"
    echo "  https://ubuntu.com/download/raspberry-pi"
    echo ""
    echo "Or specify with: --image /path/to/ubuntu.img.xz"
    echo ""
    read -p "Enter path to image file: " IMAGE_PATH

    if [[ ! -f "$IMAGE_PATH" ]]; then
        log_error "File not found: $IMAGE_PATH"
        exit 1
    fi
}

# Find SSH key
find_ssh_key() {
    log_header "LOCATE SSH KEY"

    if [[ -n "$SSH_KEY_PATH" && -f "$SSH_KEY_PATH" ]]; then
        log_success "Using specified SSH key: $SSH_KEY_PATH"
        SSH_PUBLIC_KEY=$(cat "$SSH_KEY_PATH")
        return 0
    fi

    # Try common locations
    local key_paths=(
        "$HOME/.ssh/id_rsa.pub"
        "$HOME/.ssh/id_ed25519.pub"
        "$HOME/.ssh/id_ecdsa.pub"
        "/root/.ssh/id_rsa.pub"
    )

    for key in "${key_paths[@]}"; do
        if [[ -f "$key" ]]; then
            SSH_KEY_PATH="$key"
            SSH_PUBLIC_KEY=$(cat "$key")
            log_success "Found SSH key: $SSH_KEY_PATH"
            return 0
        fi
    done

    # Generate one
    log_warn "No SSH key found. Generate one?"
    read -p "Generate new SSH key? [y/N]: " gen
    if [[ "$gen" =~ ^[Yy] ]]; then
        ssh-keygen -t rsa -b 4096 -f "$HOME/.ssh/id_rsa_wayfinder" -N ""
        SSH_KEY_PATH="$HOME/.ssh/id_rsa_wayfinder.pub"
        SSH_PUBLIC_KEY=$(cat "$SSH_KEY_PATH")
        log_success "Generated new key: $SSH_KEY_PATH"
    else
        log_error "SSH key required for access. Aborting."
        exit 1
    fi
}

# Unmount device
unmount_device() {
    log_step "Unmounting $DEVICE..."

    # Unmount all partitions
    for part in $(lsblk -ln -o NAME "$DEVICE" 2>/dev/null | tail -n +2); do
        if findmnt -n "/dev/$part" &>/dev/null; then
            umount "/dev/$part" 2>/dev/null || true
        fi
    done

    sync
    sleep 1
    log_success "Device unmounted"
}

# Flash image to SD card
flash_image() {
    log_header "FLASHING IMAGE"

    log_info "Image: $(basename "$IMAGE_PATH")"
    log_info "Target: $DEVICE"
    log_info "This will take several minutes..."
    echo ""

    # Determine if compressed
    if [[ "$IMAGE_PATH" == *.xz ]]; then
        log_step "Decompressing and flashing .xz image..."
        xz -dc "$IMAGE_PATH" | dd of="$DEVICE" bs=4M status=progress conv=fsync
    elif [[ "$IMAGE_PATH" == *.gz ]]; then
        log_step "Decompressing and flashing .gz image..."
        gunzip -c "$IMAGE_PATH" | dd of="$DEVICE" bs=4M status=progress conv=fsync
    elif [[ "$IMAGE_PATH" == *.zip ]]; then
        log_step "Decompressing and flashing .zip image..."
        unzip -p "$IMAGE_PATH" | dd of="$DEVICE" bs=4M status=progress conv=fsync
    else
        log_step "Flashing raw image..."
        dd if="$IMAGE_PATH" of="$DEVICE" bs=4M status=progress conv=fsync
    fi

    sync
    log_success "Image flashed successfully!"

    # Re-read partition table
    log_step "Re-reading partition table..."
    partprobe "$DEVICE" 2>/dev/null || true
    sleep 3
}

# Find and mount partitions
mount_partitions() {
    log_header "MOUNTING PARTITIONS"

    # Determine partition naming
    if [[ "$DEVICE" == *"mmcblk"* || "$DEVICE" == *"nvme"* ]]; then
        BOOT_PART="${DEVICE}p1"
        ROOT_PART="${DEVICE}p2"
    else
        BOOT_PART="${DEVICE}1"
        ROOT_PART="${DEVICE}2"
    fi

    # Wait for partitions to appear
    local wait_count=0
    while [[ ! -b "$BOOT_PART" && $wait_count -lt 10 ]]; do
        sleep 1
        ((wait_count++))
    done

    if [[ ! -b "$BOOT_PART" ]]; then
        log_error "Boot partition not found: $BOOT_PART"
        log_info "Available partitions:"
        lsblk "$DEVICE"
        exit 1
    fi

    # Mount boot partition
    BOOT_MOUNT=$(mktemp -d)
    log_step "Mounting boot partition ($BOOT_PART)..."
    mount "$BOOT_PART" "$BOOT_MOUNT"
    log_success "Boot mounted at $BOOT_MOUNT"

    # Mount root partition
    ROOT_MOUNT=$(mktemp -d)
    log_step "Mounting root partition ($ROOT_PART)..."
    mount "$ROOT_PART" "$ROOT_MOUNT"
    log_success "Root mounted at $ROOT_MOUNT"
}

# Generate cloud-init configuration
generate_cloud_init() {
    log_header "CONFIGURING CLOUD-INIT"

    log_step "Writing user-data..."

    # Password config
    local password_line="lock_passwd: true"
    if [[ -n "$PASSWORD_HASH" ]]; then
        password_line="passwd: $PASSWORD_HASH"
    fi

    cat > "$BOOT_MOUNT/user-data" << USERDATA
#cloud-config

# ===========================================
# WayfindR Auto-Generated Configuration
# Generated: $(date)
# ===========================================

# Hostname
hostname: ${HOSTNAME}
manage_etc_hosts: true

# Timezone
timezone: ${TIMEZONE:-America/New_York}

# User Configuration
users:
  - name: ${USERNAME:-ubuntu}
    groups: [adm, dialout, cdrom, floppy, sudo, audio, dip, video, plugdev, netdev]
    shell: /bin/bash
    sudo: ALL=(ALL) NOPASSWD:ALL
    ${password_line}
    ssh_authorized_keys:
      - ${SSH_PUBLIC_KEY}

# SSH Configuration
ssh_pwauth: false
disable_root: true

# Package Management
package_update: true
package_upgrade: true
packages:
  - avahi-daemon
  - curl
  - wget
  - git
  - htop
  - vim
  - python3-pip
  - net-tools

# Write first-boot service
write_files:
  - path: /etc/systemd/system/wayfinder-firstboot.service
    permissions: '0644'
    content: |
      [Unit]
      Description=WayfindR First Boot Setup
      After=network-online.target cloud-final.service
      Wants=network-online.target
      ConditionPathExists=!/var/lib/wayfinder-firstboot-complete

      [Service]
      Type=oneshot
      ExecStart=/opt/wayfinder/firstboot.sh
      ExecStartPost=/bin/touch /var/lib/wayfinder-firstboot-complete
      RemainAfterExit=yes
      TimeoutStartSec=3600
      StandardOutput=journal+console
      StandardError=journal+console

      [Install]
      WantedBy=multi-user.target

  - path: /opt/wayfinder/firstboot.sh
    permissions: '0755'
    content: |
      #!/bin/bash
      LOG="/var/log/wayfinder-firstboot.log"
      log() { echo "\$(date '+%Y-%m-%d %H:%M:%S') \$1" | tee -a "\$LOG"; }

      log "========================================"
      log "WayfindR First Boot Starting"
      log "========================================"

      # Wait for package manager
      while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
          log "Waiting for apt..."
          sleep 5
      done

      # Run scripts in order
      for script in /opt/wayfinder/scripts/*.sh; do
          if [[ -f "\$script" ]]; then
              log "Running: \$script"
              bash "\$script" 2>&1 | tee -a "\$LOG"
              log "Completed: \$script"
          fi
      done

      log "========================================"
      log "WayfindR First Boot Complete!"
      log "========================================"

      systemctl disable wayfinder-firstboot.service

# Commands to run
runcmd:
  - systemctl daemon-reload
  - systemctl enable wayfinder-firstboot.service
  - mkdir -p /opt/wayfinder/scripts
  - echo "Cloud-init complete: \$(date)" >> /var/log/wayfinder-cloud-init.log

# Final message
final_message: |
  === WayfindR Cloud-Init Complete ===
  Hostname: ${HOSTNAME}
  User: ${USERNAME:-ubuntu}
  SSH: Enabled with key authentication

  First-boot scripts will run automatically.
  Monitor: tail -f /var/log/wayfinder-firstboot.log
USERDATA

    log_success "user-data written"

    # Network config
    log_step "Writing network-config..."

    if [[ -n "$WIFI_SSID" && -n "$WIFI_PASS" ]]; then
        cat > "$BOOT_MOUNT/network-config" << NETCFG
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "${WIFI_SSID}":
        password: "${WIFI_PASS}"
NETCFG
        log_success "network-config written (Ethernet + WiFi)"
    else
        cat > "$BOOT_MOUNT/network-config" << NETCFG
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
NETCFG
        log_success "network-config written (Ethernet DHCP)"
    fi
}

# Copy installation scripts
copy_scripts() {
    log_header "COPYING INSTALLATION SCRIPTS"

    local scripts_src="$SCRIPT_DIR/scripts"
    local scripts_dst="$ROOT_MOUNT/opt/wayfinder/scripts"

    mkdir -p "$scripts_dst"

    if [[ -d "$scripts_src" ]]; then
        local count=0
        for script in "$scripts_src"/*.sh; do
            if [[ -f "$script" ]]; then
                cp "$script" "$scripts_dst/"
                chmod +x "$scripts_dst/$(basename "$script")"
                log_step "Copied: $(basename "$script")"
                ((count++))
            fi
        done
        log_success "Copied $count installation scripts"
    else
        log_warn "No scripts directory found at $scripts_src"
        log_info "Creating placeholder script..."
        cat > "$scripts_dst/01_placeholder.sh" << 'EOF'
#!/bin/bash
echo "No installation scripts were provided."
echo "Add scripts to new_bakery/scripts/ and re-bake."
EOF
        chmod +x "$scripts_dst/01_placeholder.sh"
    fi
}

# Cleanup
cleanup() {
    log_header "FINISHING UP"

    log_step "Syncing filesystem..."
    sync

    if [[ -n "$BOOT_MOUNT" && -d "$BOOT_MOUNT" ]]; then
        log_step "Unmounting boot partition..."
        umount "$BOOT_MOUNT" 2>/dev/null || true
        rmdir "$BOOT_MOUNT" 2>/dev/null || true
    fi

    if [[ -n "$ROOT_MOUNT" && -d "$ROOT_MOUNT" ]]; then
        log_step "Unmounting root partition..."
        umount "$ROOT_MOUNT" 2>/dev/null || true
        rmdir "$ROOT_MOUNT" 2>/dev/null || true
    fi

    sync
    log_success "Cleanup complete"
}

trap cleanup EXIT

# Parse arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --image|-i)
                IMAGE_PATH="$2"
                shift 2
                ;;
            --device|-d)
                DEVICE="$2"
                shift 2
                ;;
            --ssh-key|-k)
                SSH_KEY_PATH="$2"
                shift 2
                ;;
            --hostname|-n)
                HOSTNAME="$2"
                shift 2
                ;;
            --username|-u)
                USERNAME="$2"
                shift 2
                ;;
            --password|-p)
                PASSWORD_HASH=$(echo "$2" | openssl passwd -6 -stdin)
                shift 2
                ;;
            --timezone|-t)
                TIMEZONE="$2"
                shift 2
                ;;
            --wifi-ssid)
                WIFI_SSID="$2"
                shift 2
                ;;
            --wifi-pass)
                WIFI_PASS="$2"
                shift 2
                ;;
            --help|-h)
                usage
                ;;
            *)
                log_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    # Set default hostname if not specified
    HOSTNAME="${HOSTNAME:-wayfinder-$(date +%s | tail -c 5)}"
}

usage() {
    cat << EOF
WayfindR Pi Bakery - Automatic SD Card Setup

Usage: sudo $(basename "$0") [OPTIONS]

This script automatically:
  1. Detects your SD card
  2. Flashes Ubuntu 22.04
  3. Configures SSH access
  4. Installs ROS2 on first boot

Options:
  --image, -i PATH      Ubuntu image file (.img or .img.xz)
  --device, -d PATH     SD card device (auto-detected if not specified)
  --ssh-key, -k PATH    SSH public key (auto-detected if not specified)
  --hostname, -n NAME   Pi hostname (default: wayfinder-XXXXX)
  --username, -u NAME   Username (default: ubuntu)
  --password, -p PASS   Optional password
  --timezone, -t TZ     Timezone (default: America/New_York)
  --wifi-ssid SSID      WiFi network name
  --wifi-pass PASS      WiFi password
  --help, -h            Show this help

Examples:
  # Fully automatic (will prompt for missing info)
  sudo ./bake_auto.sh

  # Specify everything
  sudo ./bake_auto.sh --image ubuntu.img.xz --hostname robot-01

After baking:
  1. Insert SD card into Raspberry Pi
  2. Connect Ethernet (or use WiFi)
  3. Power on
  4. Wait ~25 minutes for setup
  5. SSH: ssh ubuntu@<hostname>.local

EOF
    exit 0
}

# ============================================================================
# MAIN
# ============================================================================

main() {
    clear
    echo ""
    echo -e "${CYAN}${BOLD}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                                                           ║"
    echo "║   🥧  WayfindR Pi Bakery                                  ║"
    echo "║                                                           ║"
    echo "║   Automatic Raspberry Pi Setup for ROS2 Robotics         ║"
    echo "║                                                           ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"

    check_root
    check_dependencies
    parse_args "$@"

    # Interactive setup if not fully specified
    if [[ -z "$DEVICE" ]]; then
        select_device
    fi

    find_image
    find_ssh_key

    # Show summary
    log_header "CONFIGURATION SUMMARY"
    echo "  Device:    $DEVICE"
    echo "  Image:     $(basename "$IMAGE_PATH")"
    echo "  Hostname:  $HOSTNAME"
    echo "  Username:  ${USERNAME:-ubuntu}"
    echo "  SSH Key:   $SSH_KEY_PATH"
    if [[ -n "$WIFI_SSID" ]]; then
        echo "  WiFi:      $WIFI_SSID"
    fi
    echo ""

    read -p "Press Enter to start baking (Ctrl+C to cancel)..."

    # Do the work
    unmount_device
    flash_image
    mount_partitions
    generate_cloud_init
    copy_scripts

    # Done!
    echo ""
    echo -e "${GREEN}${BOLD}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║                                                           ║"
    echo "║   ✓  SD CARD IS READY!                                    ║"
    echo "║                                                           ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Remove SD card from computer"
    echo "  2. Insert into Raspberry Pi"
    echo "  3. Connect Ethernet cable"
    echo "  4. Power on"
    echo "  5. Wait ~25 minutes for full setup"
    echo ""
    echo "Connect via SSH:"
    echo "  ssh ${USERNAME:-ubuntu}@${HOSTNAME}.local"
    echo ""
    echo "Monitor setup progress:"
    echo "  ssh ${USERNAME:-ubuntu}@${HOSTNAME}.local 'tail -f /var/log/wayfinder-firstboot.log'"
    echo ""
}

main "$@"
