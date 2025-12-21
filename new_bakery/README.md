# New Bakery - Raspberry Pi Automated Setup

**Purpose:** Automate the setup of Raspberry Pi devices with Ubuntu 22.04 for ROS2 robotics applications.

## Overview

This system provides a streamlined way to:
1. Flash Ubuntu 22.04 to an SD card
2. Pre-configure cloud-init for SSH access on first boot
3. Automatically install ROS2 and required packages after first boot
4. Enable headless operation from the start

## Quick Start

### Step 1: Download Ubuntu 22.04 for Raspberry Pi

```bash
# Download from Ubuntu (for Pi 4/5)
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz
```

### Step 2: Flash and Configure

```bash
# Flash the image and configure cloud-init
sudo ./bake.sh --image ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \
               --device /dev/sdX \
               --hostname wayfinder-01 \
               --ssh-key ~/.ssh/id_rsa.pub
```

### Step 3: Insert SD Card and Boot

The Pi will:
1. Boot Ubuntu 22.04
2. Configure itself via cloud-init (SSH enabled)
3. Run first-boot scripts to install ROS2

### Step 4: Connect via SSH

```bash
ssh ubuntu@wayfinder-01.local
# or by IP address
ssh ubuntu@<ip-address>
```

## How It Works

### Cloud-Init (First Boot)

Ubuntu 22.04 for Raspberry Pi uses **cloud-init** for first-boot configuration. We modify two files on the `system-boot` partition:

1. **user-data** - User creation, SSH keys, packages, and scripts
2. **network-config** - Network settings (optional, DHCP by default)

### Two-Stage Installation

**Stage 1 (Cloud-Init):**
- Creates user with SSH access
- Sets hostname
- Runs initial system updates
- Copies installation scripts

**Stage 2 (First-Boot Service):**
- Systemd service runs after cloud-init completes
- Installs ROS2 Humble and dependencies
- Configures SLAM, navigation packages
- Sets up LiDAR drivers

## Files

| File | Purpose |
|------|---------|
| `bake.sh` | Main script - flash image and configure |
| `templates/user-data` | Cloud-init user configuration |
| `templates/network-config` | Network configuration template |
| `scripts/install_ros2.sh` | ROS2 Humble installation |
| `scripts/install_slam.sh` | SLAM and navigation packages |
| `scripts/install_lidar.sh` | LiDAR driver setup |
| `scripts/firstboot.sh` | First-boot orchestrator |

## Requirements

- Linux host machine (for flashing)
- SD card (32GB+ recommended)
- Ubuntu 22.04 Raspberry Pi image
- SSH public key

## Tested On

- Raspberry Pi 4 (4GB/8GB)
- Raspberry Pi 5
- Ubuntu 22.04.5 LTS (arm64)

## Troubleshooting

### Check cloud-init status
```bash
ssh ubuntu@<pi-ip> "cloud-init status"
```

### View cloud-init logs
```bash
ssh ubuntu@<pi-ip> "cat /var/log/cloud-init-output.log"
```

### View first-boot logs
```bash
ssh ubuntu@<pi-ip> "cat /var/log/wayfinder-firstboot.log"
```

### Re-run cloud-init (for testing)
```bash
ssh ubuntu@<pi-ip> "sudo cloud-init clean --logs --reboot"
```

---

**Created:** 2025-12-20
**Based on:** old_bakery learnings + cloud-init best practices
