# Complete Raspberry Pi Baking Workflow

This document explains the complete process of preparing a Raspberry Pi with Ubuntu 22.04 and ROS2 for robotics applications.

## Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    HOST MACHINE                              │
│  1. Download Ubuntu Image                                    │
│  2. Run bake.sh to flash and configure SD card              │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    SD CARD CONTENTS                          │
│  system-boot partition:                                      │
│    - user-data (cloud-init config)                          │
│    - network-config                                          │
│  writable partition:                                         │
│    - /opt/wayfinder/scripts/*.sh                            │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│              RASPBERRY PI FIRST BOOT                         │
│  1. Cloud-init runs (creates user, enables SSH)             │
│  2. System reboots                                           │
│  3. wayfinder-firstboot service runs                        │
│  4. Installation scripts execute in order                    │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│              READY FOR USE                                   │
│  - SSH accessible                                            │
│  - ROS2 Humble installed                                     │
│  - SLAM, Nav2, LiDAR drivers ready                          │
└─────────────────────────────────────────────────────────────┘
```

---

## Step-by-Step Instructions

### Prerequisites

1. **Host Machine**: Linux (Ubuntu recommended)
2. **SD Card**: 32GB or larger
3. **SD Card Reader**: USB or built-in
4. **SSH Key**: Generated public/private key pair

### Step 1: Download Ubuntu Image

```bash
# Create downloads directory
mkdir -p ~/Downloads/pi-images
cd ~/Downloads/pi-images

# Download Ubuntu 22.04 Server for Raspberry Pi (arm64)
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz

# Verify download (optional)
sha256sum ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz
```

### Step 2: Generate SSH Key (if needed)

```bash
# Generate a new SSH key (if you don't have one)
ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa_wayfinder

# Or use existing key
ls ~/.ssh/id_rsa.pub
```

### Step 3: Insert SD Card and Identify Device

```bash
# List block devices BEFORE inserting SD card
lsblk

# Insert SD card

# List block devices AFTER inserting
lsblk

# New device is your SD card (e.g., /dev/sdb or /dev/mmcblk0)
```

**WARNING**: Make absolutely sure you have the correct device. Flashing to the wrong device will destroy data.

### Step 4: Run the Bake Script

```bash
cd /path/to/WayfindR-driver/new_bakery

# Basic usage (Ethernet, SSH key only)
sudo ./bake.sh \
    --image ~/Downloads/pi-images/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \
    --device /dev/sdb \
    --ssh-key ~/.ssh/id_rsa.pub \
    --hostname wayfinder-01

# With WiFi
sudo ./bake.sh \
    --image ~/Downloads/pi-images/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \
    --device /dev/sdb \
    --ssh-key ~/.ssh/id_rsa.pub \
    --hostname wayfinder-01 \
    --wifi-ssid "MyNetwork" \
    --wifi-pass "MyPassword"

# With password authentication (in addition to SSH key)
sudo ./bake.sh \
    --image ~/Downloads/pi-images/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \
    --device /dev/sdb \
    --ssh-key ~/.ssh/id_rsa.pub \
    --hostname wayfinder-01 \
    --password "mysecurepassword"
```

### Step 5: Boot the Raspberry Pi

1. Remove SD card from host machine
2. Insert SD card into Raspberry Pi
3. Connect Ethernet cable (if not using WiFi)
4. Connect power

### Step 6: Wait for First Boot

The first boot takes approximately:
- **Cloud-init**: 2-3 minutes
- **First reboot**: 1 minute
- **ROS2 installation**: 15-30 minutes (depending on network speed)

You can monitor progress:

```bash
# Find the Pi on your network
ping wayfinder-01.local

# Or scan for it
nmap -sn 192.168.1.0/24 | grep -B2 "Raspberry"

# SSH in and watch the logs
ssh ubuntu@wayfinder-01.local
tail -f /var/log/wayfinder-firstboot.log
```

### Step 7: Verify Installation

```bash
# SSH into the Pi
ssh ubuntu@wayfinder-01.local

# Check ROS2
source /opt/ros/humble/setup.bash
ros2 --version

# Check SLAM packages
ros2 pkg list | grep slam

# Run system info
~/system_info.sh

# Test LiDAR (if connected)
~/test_lidar.sh
```

---

## What Gets Installed

### Cloud-Init Phase
- User creation with SSH key
- Hostname configuration
- Basic packages (curl, git, vim, htop)
- SSH enabled
- First-boot service enabled

### First-Boot Phase

| Script | Purpose | Approximate Time |
|--------|---------|------------------|
| 01_system_setup.sh | System update, locale, essentials | 5 min |
| 02_install_ros2.sh | ROS2 Humble base installation | 10-15 min |
| 03_install_slam_nav.sh | SLAM Toolbox, Nav2, localization | 5-10 min |
| 04_install_lidar.sh | LiDAR drivers and udev rules | 2 min |
| 05_install_extras.sh | IMU tools, utilities, aliases | 3 min |
| 99_finalize.sh | Cleanup and marker file | 1 min |

**Total first-boot time**: 25-35 minutes

---

## Troubleshooting

### Can't Find Pi on Network

```bash
# Check if Pi is booting (if you have a monitor)
# Look for cloud-init messages

# Scan network for new devices
nmap -sn 192.168.1.0/24

# Try direct IP if mDNS isn't working
# Check your router's DHCP leases
```

### SSH Connection Refused

```bash
# Cloud-init may still be running
# Wait 3-5 minutes after power on

# Check cloud-init status (if you can connect)
cloud-init status --wait
```

### First-Boot Scripts Failed

```bash
# Check the log file
cat /var/log/wayfinder-firstboot.log

# Check cloud-init logs
cat /var/log/cloud-init-output.log

# Re-run first-boot manually
sudo /opt/wayfinder/firstboot.sh
```

### ROS2 Not Working

```bash
# Ensure it's sourced
source /opt/ros/humble/setup.bash

# Check if packages are installed
dpkg -l | grep ros-humble

# Re-run installation
sudo bash /opt/wayfinder/scripts/02_install_ros2.sh
```

### Wrong Device Flashed (DANGER!)

If you accidentally flashed the wrong device:
- **If it was an external drive**: Data is lost, restore from backup
- **If it was your system drive**: Boot from live USB and attempt recovery
- **Prevention**: Always double-check with `lsblk` before flashing

---

## Customization

### Adding Custom Scripts

Create new scripts in the `scripts/` directory with numbered prefixes:

```bash
# Create a new script
cat > scripts/06_my_custom.sh << 'EOF'
#!/bin/bash
set -e
echo "[06_my_custom] Running custom setup..."
# Your commands here
EOF

chmod +x scripts/06_my_custom.sh
```

Scripts are executed in alphabetical order by filename.

### Modifying Cloud-Init

Edit the `generate_user_data()` function in `bake.sh` to customize:
- Default packages
- User configuration
- Network settings
- Write files

### Changing Default Username

```bash
./bake.sh --username myrobot --ssh-key ~/.ssh/id_rsa.pub ...
```

---

## Security Considerations

1. **SSH Keys**: Use key-based authentication (default)
2. **Passwords**: If using --password, choose a strong one
3. **Network**: Use Ethernet for initial setup when possible
4. **Updates**: Scripts run apt upgrade during setup
5. **Sudo**: User has passwordless sudo (required for robotics)

---

## File Structure

```
new_bakery/
├── bake.sh                    # Main flashing and configuration script
├── README.md                  # Quick start guide
├── WORKFLOW.md               # This detailed documentation
├── scripts/
│   ├── 01_system_setup.sh    # Basic system configuration
│   ├── 02_install_ros2.sh    # ROS2 Humble installation
│   ├── 03_install_slam_nav.sh # SLAM and navigation
│   ├── 04_install_lidar.sh   # LiDAR driver setup
│   ├── 05_install_extras.sh  # Additional tools
│   └── 99_finalize.sh        # Cleanup and completion
├── templates/                 # (Optional) Template files
└── config/                    # (Optional) Configuration files
```

---

## Comparison with Old Bakery

| Feature | Old Bakery | New Bakery |
|---------|------------|------------|
| GUI | PyQt5 application | Command-line only |
| Complexity | High (multiple Python files) | Low (single bash script) |
| First-boot | Custom systemd service | Cloud-init + systemd |
| Script tracking | State file with completion markers | Cloud-init handles once-only |
| Network config | Manual wpa_supplicant | Cloud-init network-config |
| Maintenance | Difficult | Simple bash scripts |

### Why the Change?

1. **Cloud-init is standard**: Ubuntu Pi images have it built-in
2. **Simpler is better**: Bash scripts are easier to debug
3. **No dependencies**: No PyQt5, no complex Python environment
4. **Reproducible**: Same process works across different host systems

---

## References

- [Ubuntu Cloud-Init Documentation](https://cloudinit.readthedocs.io/)
- [Ubuntu Raspberry Pi Images](https://ubuntu.com/download/raspberry-pi)
- [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
- [Cloud-Init Examples](https://cloudinit.readthedocs.io/en/latest/reference/examples.html)

---

**Created**: 2025-12-20
**Author**: Claude Code (Opus 4.5)
**Based on**: Research of old_bakery + Ubuntu cloud-init best practices
