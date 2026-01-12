# New Bakery - Scope Documentation

**Created:** 2026-01-11
**Location:** `/home/devel/Desktop/WayfindR-driver/new_bakery/`

---

## Purpose

The `new_bakery` folder is an automated Raspberry Pi provisioning system designed to streamline the setup of Raspberry Pi devices for ROS2-based robotics applications. It provides a complete, hands-off solution for creating ready-to-use WayfindR robot nodes from a blank SD card to a fully configured ROS2 system with SLAM, navigation, and LiDAR capabilities.

The system leverages Ubuntu 22.04's built-in cloud-init functionality to automate first-boot configuration, eliminating the need for complex GUI tools or manual setup steps.

---

## Relationship to WayfindR Project

The `new_bakery` is a **deployment and provisioning tool** for the WayfindR robotics project. It serves as the foundation layer that:

1. **Prepares Raspberry Pi Hardware**: Transforms bare Raspberry Pi devices into WayfindR-ready robot controllers
2. **Standardizes Environment**: Ensures all WayfindR robots run identical software stacks (Ubuntu 22.04 + ROS2 Humble)
3. **Enables Fleet Deployment**: Makes it easy to provision multiple identical robots for the WayfindR system
4. **Replaces Manual Setup**: Eliminates the error-prone process of manually installing and configuring ROS2 and dependencies

### Integration Points

- **ROS2 Workspaces**: Creates `~/ros2_ws` where WayfindR ROS2 packages would be deployed
- **LiDAR Support**: Pre-installs drivers for common LiDAR sensors used in WayfindR robots
- **SLAM/Navigation**: Installs SLAM Toolbox and Nav2, which WayfindR uses for mapping and autonomous navigation
- **Network Configuration**: Sets up mDNS (Avahi) for easy discovery of WayfindR robots on the network

### Comparison to Legacy System

This replaces the `old_bakery` folder, which contained a complex PyQt5 GUI application (`bakery_old.py`). The new approach is simpler, more maintainable, and leverages standard cloud-init practices instead of custom Python tooling.

---

## Key Files and Their Purposes

### Main Scripts

| File | Purpose | Type |
|------|---------|------|
| **bake.sh** | Primary SD card flashing and configuration script | Bash Script (571 lines) |
| **bake_auto.sh** | Fully automated version with interactive device selection | Bash Script (758 lines) |

**bake.sh** - Command-line focused, requires explicit parameters:
- Flashes Ubuntu 22.04 image to SD card
- Configures cloud-init user-data and network-config
- Mounts partitions and copies installation scripts
- Handles compressed images (.xz, .gz, .zip)
- Includes safety checks to prevent accidental data loss

**bake_auto.sh** - User-friendly automated version:
- Auto-detects removable storage devices
- Auto-locates Ubuntu images and SSH keys
- Interactive prompts for missing information
- More verbose output with progress indicators
- Same core functionality as bake.sh but with better UX

### Documentation

| File | Purpose | Lines |
|------|---------|-------|
| **README.md** | Quick start guide and usage instructions | 121 |
| **WORKFLOW.md** | Detailed step-by-step workflow documentation | 355 |
| **FINDINGS.md** | Research notes and design decisions | 259 |

**README.md**: Quick reference for common use cases
**WORKFLOW.md**: Complete workflow from image download to robot deployment
**FINDINGS.md**: Documents why cloud-init was chosen, lessons from old_bakery, best practices

### Installation Scripts (scripts/ directory)

These scripts run automatically on the Raspberry Pi during first boot:

| Script | Purpose | Runtime | Key Actions |
|--------|---------|---------|-------------|
| **01_system_setup.sh** | System initialization | ~5 min | Updates packages, sets locale/timezone, adds user to dialout group |
| **02_install_ros2.sh** | ROS2 installation | ~15 min | Installs ROS2 Humble base, rosdep, colcon, adds to bashrc |
| **03_install_slam_nav.sh** | SLAM & Navigation | ~10 min | Installs SLAM Toolbox, Nav2, robot_localization, creates workspace |
| **04_install_lidar.sh** | LiDAR drivers | ~2 min | Installs RPLidar driver, creates udev rules, test scripts |
| **05_install_extras.sh** | Additional tools | ~3 min | IMU tools, ROS2 control, monitoring tools, convenience aliases |
| **99_finalize.sh** | Cleanup and completion | ~1 min | Cleans apt cache, fixes permissions, creates installation marker |

**Total automated installation time: ~35 minutes**

### Empty Directories

- **config/**: Reserved for configuration templates (currently unused)
- **templates/**: Reserved for cloud-init templates (currently unused, generated inline in scripts)

---

## Current State of Implementation

### ✅ Fully Implemented

- **SD Card Flashing**: Complete with support for multiple image formats
- **Cloud-Init Configuration**: Automated user creation, SSH key setup, hostname configuration
- **Network Setup**: Ethernet (DHCP) and WiFi configuration support
- **ROS2 Installation**: Automated ROS2 Humble base installation
- **SLAM/Navigation**: SLAM Toolbox and Nav2 packages installed
- **LiDAR Support**: RPLidar driver with udev rules for automatic device detection
- **First-Boot Orchestration**: Systemd service that runs scripts in sequence
- **Safety Features**: Root device detection, user confirmation prompts
- **Logging**: Comprehensive logging to `/var/log/wayfinder-firstboot.log`

### ⚠️ Partially Implemented

- **Multiple LiDAR Drivers**: Only RPLidar is installed from apt; LDROBOT and YDLidar have udev rules but no drivers
- **Error Recovery**: Scripts continue on error but don't have sophisticated rollback mechanisms
- **Testing**: No automated tests for the installation process

### ❌ Not Implemented

- **Fleet Management**: No mechanism for deploying updates to multiple existing robots
- **Web UI**: Command-line only, no graphical interface
- **Validation**: No automated checks that waypoints are in valid locations
- **OTA Updates**: No over-the-air update mechanism for running robots
- **Configuration Templating**: config/ and templates/ directories exist but aren't used (cloud-init config is generated inline)

---

## Dependencies and Requirements

### Host Machine Requirements

**Operating System:**
- Linux (Ubuntu/Debian recommended)
- Root/sudo access required

**Required Commands:**
- `dd` - Disk image writing
- `lsblk` - Block device listing
- `findmnt` - Mount point finding
- `partprobe` - Partition table reloading
- `mktemp` - Temporary directory creation
- `xz` - XZ decompression
- `mount/umount` - Partition mounting
- `openssl` - Password hashing (optional)

**Installation on Ubuntu/Debian:**
```bash
sudo apt-get install coreutils util-linux xz-utils
```

### Target Hardware Requirements

**Tested Devices:**
- Raspberry Pi 4 (4GB/8GB)
- Raspberry Pi 5

**Storage:**
- 32GB+ SD card (recommended)
- Minimum 16GB (tight fit with ROS2 packages)

**Network:**
- Ethernet port OR WiFi capability
- Internet connection during first boot (for package downloads)

### Software Requirements

**Required Download:**
- Ubuntu 22.04.5 LTS Server for Raspberry Pi (arm64)
- Image URL: https://ubuntu.com/download/raspberry-pi
- Filename: `ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz`
- Size: ~1.4GB compressed, ~3.5GB uncompressed

**SSH Key:**
- RSA, Ed25519, or ECDSA public key
- Location: `~/.ssh/id_rsa.pub` (default) or custom path

### Network Requirements During First Boot

**Required Bandwidth:**
- ROS2 packages: ~500MB download
- Additional packages: ~200MB download
- Total: ~700MB minimum

**Required Access:**
- packages.ros.org (ROS2 repository)
- archive.ubuntu.com (Ubuntu packages)
- security.ubuntu.com (Security updates)

---

## Installation Process Flow

### Stage 1: Host Machine (2-5 minutes)

1. **User runs bake.sh or bake_auto.sh**
2. **Image is flashed to SD card** (2-4 minutes depending on SD card speed)
3. **Partitions are mounted**
4. **Cloud-init files are written** to system-boot partition
5. **Installation scripts are copied** to writable partition
6. **SD card is unmounted** and ready for insertion

### Stage 2: First Boot - Cloud-Init (2-3 minutes)

1. **Raspberry Pi boots Ubuntu 22.04**
2. **Cloud-init detects first boot**
3. **User created with SSH key**
4. **Hostname configured**
5. **Basic packages installed** (avahi-daemon, git, vim, htop)
6. **SSH enabled**
7. **First-boot systemd service enabled**
8. **System reboots** (cloud-init complete)

### Stage 3: First-Boot Service (25-35 minutes)

1. **wayfinder-firstboot.service starts**
2. **Scripts execute in numerical order:**
   - 01_system_setup.sh → System configuration
   - 02_install_ros2.sh → ROS2 Humble installation
   - 03_install_slam_nav.sh → SLAM and Nav2 packages
   - 04_install_lidar.sh → LiDAR drivers and udev rules
   - 05_install_extras.sh → Additional tools and utilities
   - 99_finalize.sh → Cleanup and completion marker
3. **Service disables itself** (won't run again)
4. **Robot is ready for use**

### Total Time: ~30-40 minutes from blank SD card to operational robot

---

## Configuration Options

### Hostname

```bash
--hostname wayfinder-01
```
- Default: `wayfinder-XXXXX` (5-digit timestamp)
- Used for mDNS: `wayfinder-01.local`

### Username

```bash
--username ubuntu
```
- Default: `ubuntu`
- Configured with passwordless sudo

### Authentication

```bash
--ssh-key ~/.ssh/id_rsa.pub          # SSH key (required)
--password "mypassword"              # Optional password
```
- SSH key authentication: Required
- Password authentication: Optional, disabled by default

### Network

```bash
--wifi-ssid "MyNetwork"              # WiFi SSID
--wifi-pass "MyPassword"             # WiFi password
```
- Default: Ethernet DHCP
- WiFi: Optional, configured via cloud-init network-config

### Timezone

```bash
--timezone America/New_York
```
- Default: America/New_York
- Standard IANA timezone database format

---

## Usage Examples

### Basic Usage (bake.sh)

```bash
sudo ./bake.sh \
  --image ~/Downloads/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \
  --device /dev/sdb \
  --ssh-key ~/.ssh/id_rsa.pub \
  --hostname wayfinder-alpha
```

### With WiFi Configuration

```bash
sudo ./bake.sh \
  --image ubuntu.img.xz \
  --device /dev/mmcblk0 \
  --ssh-key ~/.ssh/id_rsa.pub \
  --hostname wayfinder-beta \
  --wifi-ssid "RoboticsLab" \
  --wifi-pass "secure_password"
```

### Fully Automated (bake_auto.sh)

```bash
# Interactive - will prompt for missing info
sudo ./bake_auto.sh

# Specify hostname only, auto-detect everything else
sudo ./bake_auto.sh --hostname robot-01
```

---

## Monitoring and Troubleshooting

### Check Installation Progress

```bash
# SSH into the Pi during installation
ssh ubuntu@wayfinder-01.local

# Watch the installation log in real-time
tail -f /var/log/wayfinder-firstboot.log
```

### Verify Installation

```bash
# Check if first-boot completed
ls -la /var/lib/wayfinder-firstboot-complete

# View installation summary
cat ~/.wayfinder_installed

# Run system info script
~/system_info.sh
```

### Common Issues

**Can't find Pi on network:**
- Wait 3-5 minutes after power on
- Try direct IP instead of hostname.local
- Check router DHCP leases

**SSH connection refused:**
- Cloud-init may still be running
- Check status: `cloud-init status --wait`

**First-boot scripts failed:**
- Check logs: `cat /var/log/wayfinder-firstboot.log`
- Re-run manually: `sudo /opt/wayfinder/firstboot.sh`

---

## Security Considerations

### Enabled by Default

- SSH key authentication (required)
- Passwordless sudo for ubuntu user (required for robotics hardware access)
- Firewall: UFW not enabled (robot intended for private networks)

### Optional Security Enhancements

- Set password with `--password` flag
- Modify cloud-init to enable UFW
- Restrict SSH to specific IP ranges
- Add fail2ban for brute force protection

### Sensitive Data Handling

- WiFi passwords passed via command-line (not stored in git)
- SSH keys required (more secure than passwords)
- No secrets stored in configuration files

---

## Future Improvements

Based on FINDINGS.md, potential enhancements include:

1. **Fleet Management**: SSH-based deployment system for updating multiple robots
2. **Web UI**: Browser-based interface for non-technical users
3. **Validation**: Automated checks for robot configuration
4. **OTA Updates**: Push script updates to running robots
5. **Additional LiDAR Support**: Add LDROBOT and YDLidar driver installations
6. **Configuration Templates**: Utilize the empty templates/ directory for cloud-init variants
7. **Automated Testing**: Integration tests for the baking process
8. **Multi-Platform Support**: Support for other SBCs (Orange Pi, Rock Pi, etc.)

---

## Technical Architecture

### Two-Stage Boot Design

**Why two stages?**

1. **Quick SSH Access**: User can connect within 3 minutes via cloud-init
2. **Proper Logging**: Long installations logged to dedicated file
3. **Recovery Options**: If ROS2 install fails, user can SSH in and debug
4. **Monitoring**: User can watch progress in real-time

### Cloud-Init vs. Custom First-Boot

**Why cloud-init?**

- Pre-installed on Ubuntu for Raspberry Pi
- Industry-standard tool (used by AWS, Azure, GCP)
- Well-documented with extensive examples
- Runs exactly once (no state file management needed)
- Simple YAML configuration

**Custom first-boot service still needed for:**

- Long-running installations (ROS2 takes 20+ minutes)
- Proper logging and error handling
- Script ordering and execution tracking

### Script Execution Order

Scripts run in **alphabetical order** by filename:
- Prefix `01-99` ensures deterministic execution
- Each script is independent and idempotent
- Scripts can be added/removed without modifying orchestration code

---

## Comparison: New Bakery vs. Old Bakery

| Feature | Old Bakery | New Bakery |
|---------|------------|------------|
| **Interface** | PyQt5 GUI | Command-line |
| **Complexity** | High (multiple Python files) | Low (single bash script) |
| **Dependencies** | PyQt5, Python packages | Standard Linux tools |
| **First-boot** | Custom systemd + state files | Cloud-init + systemd |
| **Network Config** | Manual wpa_supplicant editing | Cloud-init network-config |
| **Maintenance** | Difficult (GUI framework) | Easy (bash scripts) |
| **Portability** | Requires X11 | Runs on any Linux |
| **Documentation** | Minimal | Comprehensive |

### Why the Redesign?

1. **Cloud-init is standard**: Ubuntu Pi images have it built-in
2. **Simpler is better**: Bash scripts are easier to debug than PyQt5
3. **No GUI dependencies**: Works on headless servers and CI/CD
4. **Reproducible**: Same process across different development machines

---

## Summary

The `new_bakery` folder provides a production-ready, automated provisioning system for WayfindR robots. It transforms the complex, multi-hour process of manually setting up a Raspberry Pi for ROS2 robotics into a simple 30-minute automated workflow. By leveraging Ubuntu's cloud-init and a two-stage boot process, it provides quick SSH access for monitoring while ensuring all required software is properly installed and configured.

This system represents a complete redesign of the older PyQt5-based approach, focusing on simplicity, maintainability, and adherence to industry-standard practices. It serves as the foundational deployment tool for the WayfindR robotics platform, enabling rapid provisioning of standardized robot nodes.
