#!/bin/bash
#
# bake.sh - Flash Ubuntu to SD card and configure for Raspberry Pi
#
# This script:
# 1. Flashes Ubuntu 22.04 image to SD card
# 2. Mounts the system-boot partition
# 3. Configures cloud-init for SSH and first-boot scripts
# 4. Copies installation scripts
#
# Usage:
#   sudo ./bake.sh --image <path-to-image> --device /dev/sdX --hostname mypi --ssh-key ~/.ssh/id_rsa.pub
#
# Requirements:
#   - Root privileges (for dd and mount)
#   - Ubuntu 22.04 Raspberry Pi image (xz compressed or raw)
#   - SD card inserted

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOSTNAME="wayfinder-$(date +%s | tail -c 5)"
USERNAME="ubuntu"
PASSWORD_HASH=""  # Empty means use SSH key only
TIMEZONE="America/New_York"
LOCALE="en_US.UTF-8"

# Print usage
usage() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Flash Ubuntu to SD card and configure for Raspberry Pi robotics.

Required:
  --image PATH       Path to Ubuntu image (.img or .img.xz)
  --device PATH      SD card device (e.g., /dev/sdb, /dev/mmcblk0)
  --ssh-key PATH     Path to SSH public key file

Optional:
  --hostname NAME    Hostname for the Pi (default: wayfinder-XXXXX)
  --username NAME    Username (default: ubuntu)
  --password PASS    Password for user (default: SSH key only)
  --timezone TZ      Timezone (default: America/New_York)
  --wifi-ssid SSID   WiFi network name (optional)
  --wifi-pass PASS   WiFi password (optional)
  --skip-flash       Skip flashing, only configure (for testing)
  --help             Show this help

Examples:
  # Basic usage with SSH key
  sudo ./bake.sh --image ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \\
                 --device /dev/sdb \\
                 --ssh-key ~/.ssh/id_rsa.pub

  # With WiFi and custom hostname
  sudo ./bake.sh --image ubuntu.img.xz \\
                 --device /dev/sdb \\
                 --ssh-key ~/.ssh/id_rsa.pub \\
                 --hostname robot-alpha \\
                 --wifi-ssid "MyNetwork" \\
                 --wifi-pass "MyPassword"

EOF
    exit 1
}

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --image)
                IMAGE_PATH="$2"
                shift 2
                ;;
            --device)
                DEVICE="$2"
                shift 2
                ;;
            --ssh-key)
                SSH_KEY_PATH="$2"
                shift 2
                ;;
            --hostname)
                HOSTNAME="$2"
                shift 2
                ;;
            --username)
                USERNAME="$2"
                shift 2
                ;;
            --password)
                # Generate password hash
                PASSWORD_HASH=$(echo "$2" | openssl passwd -6 -stdin)
                shift 2
                ;;
            --timezone)
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
            --skip-flash)
                SKIP_FLASH=true
                shift
                ;;
            --help|-h)
                usage
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                ;;
        esac
    done

    # Validate required arguments
    if [[ -z "$DEVICE" ]]; then
        log_error "Missing required argument: --device"
        usage
    fi

    if [[ -z "$SSH_KEY_PATH" ]]; then
        log_error "Missing required argument: --ssh-key"
        usage
    fi

    if [[ -z "$IMAGE_PATH" && -z "$SKIP_FLASH" ]]; then
        log_error "Missing required argument: --image (or use --skip-flash)"
        usage
    fi

    # Validate files exist
    if [[ ! -f "$SSH_KEY_PATH" ]]; then
        log_error "SSH key file not found: $SSH_KEY_PATH"
        exit 1
    fi

    if [[ -n "$IMAGE_PATH" && ! -f "$IMAGE_PATH" ]]; then
        log_error "Image file not found: $IMAGE_PATH"
        exit 1
    fi

    # Read SSH key
    SSH_PUBLIC_KEY=$(cat "$SSH_KEY_PATH")
}

# Safety check - don't flash the root device
safety_check() {
    local root_device=$(findmnt -n -o SOURCE / | sed 's/[0-9]*$//' | sed 's/p$//')

    if [[ "$DEVICE" == "$root_device"* ]]; then
        log_error "DANGER: $DEVICE appears to be your root device!"
        log_error "Refusing to flash to avoid destroying your system."
        exit 1
    fi

    # Confirm device
    log_warn "About to COMPLETELY ERASE: $DEVICE"
    lsblk "$DEVICE" 2>/dev/null || true
    echo ""
    read -p "Are you sure you want to continue? (yes/no): " confirm
    if [[ "$confirm" != "yes" ]]; then
        log_info "Aborted by user."
        exit 0
    fi
}

# Unmount all partitions on device
unmount_device() {
    log_info "Unmounting all partitions on $DEVICE..."

    # Find and unmount all mounted partitions
    for part in $(lsblk -ln -o NAME "$DEVICE" | tail -n +2); do
        local mount_point=$(findmnt -n -o TARGET "/dev/$part" 2>/dev/null)
        if [[ -n "$mount_point" ]]; then
            log_info "Unmounting /dev/$part from $mount_point"
            umount "/dev/$part" || true
        fi
    done

    # Give kernel time to sync
    sync
    sleep 1
}

# Flash the image to SD card
flash_image() {
    if [[ "$SKIP_FLASH" == "true" ]]; then
        log_info "Skipping flash (--skip-flash specified)"
        return
    fi

    log_info "Flashing image to $DEVICE..."
    log_info "This may take several minutes..."

    # Check if image is compressed
    if [[ "$IMAGE_PATH" == *.xz ]]; then
        log_info "Decompressing and flashing .xz image..."
        xz -dc "$IMAGE_PATH" | dd of="$DEVICE" bs=4M status=progress conv=fsync
    elif [[ "$IMAGE_PATH" == *.gz ]]; then
        log_info "Decompressing and flashing .gz image..."
        gunzip -c "$IMAGE_PATH" | dd of="$DEVICE" bs=4M status=progress conv=fsync
    elif [[ "$IMAGE_PATH" == *.zip ]]; then
        log_info "Decompressing and flashing .zip image..."
        unzip -p "$IMAGE_PATH" | dd of="$DEVICE" bs=4M status=progress conv=fsync
    else
        log_info "Flashing raw image..."
        dd if="$IMAGE_PATH" of="$DEVICE" bs=4M status=progress conv=fsync
    fi

    sync
    log_success "Image flashed successfully!"

    # Tell kernel to re-read partition table
    log_info "Re-reading partition table..."
    partprobe "$DEVICE" 2>/dev/null || true
    sleep 2
}

# Find the system-boot partition
find_boot_partition() {
    # Ubuntu Pi images have:
    # - Partition 1: system-boot (FAT32, ~256MB)
    # - Partition 2: writable (ext4, root filesystem)

    # Try common partition naming schemes
    if [[ "$DEVICE" == *"mmcblk"* || "$DEVICE" == *"nvme"* ]]; then
        # MMC/NVMe style: /dev/mmcblk0p1
        BOOT_PARTITION="${DEVICE}p1"
        ROOT_PARTITION="${DEVICE}p2"
    else
        # Standard style: /dev/sdb1
        BOOT_PARTITION="${DEVICE}1"
        ROOT_PARTITION="${DEVICE}2"
    fi

    if [[ ! -b "$BOOT_PARTITION" ]]; then
        log_error "Boot partition not found: $BOOT_PARTITION"
        log_error "Available partitions:"
        lsblk "$DEVICE"
        exit 1
    fi

    log_info "Boot partition: $BOOT_PARTITION"
    log_info "Root partition: $ROOT_PARTITION"
}

# Mount the boot partition
mount_boot_partition() {
    BOOT_MOUNT=$(mktemp -d)
    log_info "Mounting $BOOT_PARTITION to $BOOT_MOUNT..."
    mount "$BOOT_PARTITION" "$BOOT_MOUNT"
    log_success "Boot partition mounted"
}

# Mount the root partition
mount_root_partition() {
    ROOT_MOUNT=$(mktemp -d)
    log_info "Mounting $ROOT_PARTITION to $ROOT_MOUNT..."
    mount "$ROOT_PARTITION" "$ROOT_MOUNT"
    log_success "Root partition mounted"
}

# Generate cloud-init user-data
generate_user_data() {
    log_info "Generating cloud-init user-data..."

    # Password configuration
    local password_config=""
    if [[ -n "$PASSWORD_HASH" ]]; then
        password_config="passwd: $PASSWORD_HASH"
    else
        password_config="lock_passwd: true"
    fi

    cat > "$BOOT_MOUNT/user-data" << EOF
#cloud-config

# Hostname
hostname: ${HOSTNAME}
manage_etc_hosts: true

# Timezone and locale
timezone: ${TIMEZONE}
locale: ${LOCALE}

# User configuration
users:
  - name: ${USERNAME}
    groups: [adm, dialout, cdrom, floppy, sudo, audio, dip, video, plugdev, netdev, lxd]
    shell: /bin/bash
    sudo: ALL=(ALL) NOPASSWD:ALL
    ${password_config}
    ssh_authorized_keys:
      - ${SSH_PUBLIC_KEY}

# Enable SSH password auth (in addition to keys)
ssh_pwauth: false

# Package management
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

# Write first-boot service file
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
      TimeoutStartSec=1800

      [Install]
      WantedBy=multi-user.target

  - path: /opt/wayfinder/firstboot.sh
    permissions: '0755'
    content: |
      #!/bin/bash
      # First boot script - runs after cloud-init completes

      LOG_FILE="/var/log/wayfinder-firstboot.log"
      SCRIPT_DIR="/opt/wayfinder/scripts"

      log() {
          echo "\$(date '+%Y-%m-%d %H:%M:%S') - \$1" | tee -a "\$LOG_FILE"
      }

      log "=========================================="
      log "WayfindR First Boot Starting"
      log "=========================================="

      # Wait for any package managers to finish
      while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
          log "Waiting for package manager lock..."
          sleep 5
      done

      # Run installation scripts in order
      if [[ -d "\$SCRIPT_DIR" ]]; then
          for script in "\$SCRIPT_DIR"/*.sh; do
              if [[ -f "\$script" ]]; then
                  log "Running: \$script"
                  bash "\$script" 2>&1 | tee -a "\$LOG_FILE"
                  if [[ \$? -eq 0 ]]; then
                      log "Completed: \$script"
                  else
                      log "FAILED: \$script (continuing anyway)"
                  fi
              fi
          done
      fi

      log "=========================================="
      log "WayfindR First Boot Complete"
      log "=========================================="

      # Disable the service after completion
      systemctl disable wayfinder-firstboot.service

# Run commands after cloud-init
runcmd:
  # Enable the first-boot service
  - systemctl daemon-reload
  - systemctl enable wayfinder-firstboot.service
  # Create scripts directory
  - mkdir -p /opt/wayfinder/scripts
  # Log completion
  - echo "Cloud-init configuration complete" >> /var/log/cloud-init-custom.log

# Final message
final_message: |
  Cloud-init has completed!
  Hostname: ${HOSTNAME}
  User: ${USERNAME}
  SSH enabled with key authentication
  First-boot scripts will run on next reboot (or after this boot completes)
EOF

    log_success "user-data generated"
}

# Generate network-config (optional WiFi)
generate_network_config() {
    log_info "Generating network-config..."

    if [[ -n "$WIFI_SSID" && -n "$WIFI_PASS" ]]; then
        # WiFi configuration
        cat > "$BOOT_MOUNT/network-config" << EOF
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
EOF
        log_success "network-config generated with WiFi"
    else
        # Ethernet only (default)
        cat > "$BOOT_MOUNT/network-config" << EOF
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
EOF
        log_success "network-config generated (Ethernet DHCP)"
    fi
}

# Copy installation scripts to root filesystem
copy_scripts() {
    log_info "Copying installation scripts..."

    # Create directory structure
    mkdir -p "$ROOT_MOUNT/opt/wayfinder/scripts"

    # Copy scripts from our scripts directory
    if [[ -d "$SCRIPT_DIR/scripts" ]]; then
        cp -r "$SCRIPT_DIR/scripts"/*.sh "$ROOT_MOUNT/opt/wayfinder/scripts/" 2>/dev/null || true
        chmod +x "$ROOT_MOUNT/opt/wayfinder/scripts"/*.sh 2>/dev/null || true
        log_success "Scripts copied to /opt/wayfinder/scripts/"
    else
        log_warn "No scripts directory found at $SCRIPT_DIR/scripts"
    fi

    # List what was copied
    log_info "Installed scripts:"
    ls -la "$ROOT_MOUNT/opt/wayfinder/scripts/" 2>/dev/null || echo "  (none)"
}

# Cleanup - unmount partitions
cleanup() {
    log_info "Cleaning up..."

    if [[ -n "$BOOT_MOUNT" && -d "$BOOT_MOUNT" ]]; then
        umount "$BOOT_MOUNT" 2>/dev/null || true
        rmdir "$BOOT_MOUNT" 2>/dev/null || true
    fi

    if [[ -n "$ROOT_MOUNT" && -d "$ROOT_MOUNT" ]]; then
        umount "$ROOT_MOUNT" 2>/dev/null || true
        rmdir "$ROOT_MOUNT" 2>/dev/null || true
    fi

    sync
    log_success "Cleanup complete"
}

# Set up trap for cleanup on exit
trap cleanup EXIT

# Main execution
main() {
    echo ""
    echo "==========================================="
    echo "  WayfindR Pi Bakery"
    echo "  Ubuntu 22.04 + ROS2 Automated Setup"
    echo "==========================================="
    echo ""

    check_root
    parse_args "$@"

    log_info "Configuration:"
    log_info "  Image: ${IMAGE_PATH:-'(skip flash)'}"
    log_info "  Device: $DEVICE"
    log_info "  Hostname: $HOSTNAME"
    log_info "  Username: $USERNAME"
    log_info "  SSH Key: $SSH_KEY_PATH"
    if [[ -n "$WIFI_SSID" ]]; then
        log_info "  WiFi: $WIFI_SSID"
    fi
    echo ""

    safety_check
    unmount_device
    flash_image
    find_boot_partition
    mount_boot_partition
    mount_root_partition
    generate_user_data
    generate_network_config
    copy_scripts

    echo ""
    log_success "==========================================="
    log_success "  SD Card is ready!"
    log_success "==========================================="
    echo ""
    log_info "Next steps:"
    log_info "  1. Insert SD card into Raspberry Pi"
    log_info "  2. Connect Ethernet (or wait for WiFi)"
    log_info "  3. Power on the Pi"
    log_info "  4. Wait 2-3 minutes for first boot"
    log_info "  5. Connect: ssh ${USERNAME}@${HOSTNAME}.local"
    echo ""
    log_info "First-boot scripts will install ROS2 automatically."
    log_info "Check progress: ssh ${USERNAME}@${HOSTNAME}.local 'tail -f /var/log/wayfinder-firstboot.log'"
    echo ""
}

main "$@"
