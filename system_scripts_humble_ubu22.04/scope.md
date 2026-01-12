# System Scripts for ROS2 Humble on Ubuntu 22.04 - Scope Document

## Overview

This folder (`system_scripts_humble_ubu22.04`) contains a collection of system-level setup and configuration scripts designed to prepare an Ubuntu 22.04 system for running the WayfindR autonomous robot navigation platform with ROS2 Humble. These scripts handle everything from base operating system configuration to ROS2 installation and remote access setup.

## Purpose

The scripts in this folder serve as system-level prerequisites for the WayfindR-driver project, which is a driving component for autonomous robot navigation using SLAM (Simultaneous Localization and Mapping) and path planning. These scripts automate the tedious and error-prone process of manually configuring a robotics development environment.

## Available Scripts

### 1. **install_ros2_humble.sh**
- **Purpose**: Core ROS2 Humble installation script for Ubuntu 22.04
- **What it does**:
  - Updates and upgrades the system
  - Configures UTF-8 locale (required for ROS2)
  - Adds Universe repository
  - Sets up ROS2 apt repository with proper GPG keys
  - Installs `ros-humble-desktop` (full desktop installation)
  - Installs development tools (`ros-dev-tools`, `python3-colcon-common-extensions`)
  - Automatically adds ROS2 sourcing to `.bashrc`
  - Creates a default ROS2 workspace at `~/ros2_ws`
  - Performs initial workspace build
- **Usage**: `sudo bash install_ros2_humble.sh`
- **Features**:
  - Idempotent: Safe to run multiple times
  - Includes cleanup of old ROS2 repository configurations
  - Comprehensive error handling with `set -e` and `set -u`
- **Expected outcome**: Fully functional ROS2 Humble installation ready for development

### 2. **install_slam_packages.sh**
- **Purpose**: Comprehensive SLAM, navigation, and exploration package installer
- **What it does**:
  - Waits for APT locks (handles unattended-upgrades gracefully)
  - Installs core ROS2 packages (robot-state-publisher, joint-state-publisher, xacro, tf2-tools)
  - Installs **SLAM Toolbox** for mapping
  - Installs **Navigation2 (Nav2)** stack with map server and simple commander
  - Installs **robot_localization** for sensor fusion
  - Installs visualization tools (RViz2, RQt with plugins)
  - Installs **Gazebo** simulation environment
  - Installs ROS2 Control framework and controllers
  - Installs IMU tools (imu-filter-madgwick)
  - Installs Python dependencies (Flask, Flask-CORS, pyserial, numpy, scipy, matplotlib, transforms3d)
  - Initializes and updates `rosdep`
  - Creates workspace at `~/ros2_ws`
  - Clones **AutoFrontierSearch_ros2-humble** (frontier exploration package)
  - Builds entire workspace with release optimizations
  - Adds user to `dialout` group for serial port access
- **Usage**: Run after `install_ros2_humble.sh` is complete
- **Features**:
  - Interactive confirmation before starting
  - APT lock detection and waiting mechanism
  - Comprehensive error handling
  - Installs both binary packages and builds from source
- **Expected outcome**: Complete robotics navigation stack ready for autonomous exploration

### 3. **fix_apt_sources.sh**
- **Purpose**: Troubleshooting script for ROS2 apt source conflicts
- **What it does**:
  - Detects duplicate ROS2 source files (`ros2.sources` vs `ros2.list`)
  - Backs up conflicting `.sources` file
  - Ensures correct `.list` format source file exists
  - Creates proper ROS2 apt source entry if missing
  - Updates apt cache
  - Optionally installs ROS2 Humble desktop
- **Usage**: `sudo bash fix_apt_sources.sh`
- **When to use**: When encountering apt source errors or duplicate entry warnings
- **Features**:
  - Safe backup mechanism
  - Auto-detects architecture and Ubuntu codename
  - Minimal and focused fix

### 4. **sudo_no_passwd.sh**
- **Purpose**: Configure passwordless sudo for a user
- **What it does**:
  - Validates user exists on the system
  - Creates backup of existing sudoers configuration
  - Writes sudoers rule to `/etc/sudoers.d/[username]`
  - Sets proper permissions (0440, root:root)
  - Validates syntax with `visudo -c`
  - Automatically restores backup if validation fails
- **Usage**: `sudo ./sudo_no_passwd.sh [username]` (defaults to current user)
- **Use case**: Useful for automated systems, Raspberry Pi robots, or development environments
- **Security note**: Disables password requirement for sudo - use with caution
- **Features**:
  - Robust error handling
  - Automatic backup and restore
  - Syntax validation before applying changes
  - Includes instructions for removal

### 5. **xrdp_setup.sh**
- **Purpose**: Idempotent remote desktop (RDP) configuration with XFCE desktop environment
- **What it does**:
  - Installs xRDP, XFCE4, and xorgxrdp packages
  - Installs and configures LightDM display manager
  - Disables Wayland in GDM for RDP compatibility
  - Removes `~/.xsession` to allow local desktop environment choice
  - Creates XFCE session file
  - Configures xRDP to force XFCE for remote sessions (while local sessions can use GNOME)
  - Configures X11 Xwrapper for xRDP compatibility
  - Adds user to `ssl-cert` group
  - Enables and starts xRDP service
- **Usage**: `sudo bash xrdp_setup.sh --install` (or `--check` for status only)
- **Features**:
  - Dual-mode operation: `--install` or `--check`
  - Idempotent: safe to run multiple times
  - Separates local and remote desktop environments
  - Comprehensive status reporting
- **Expected outcome**: Working RDP access using XFCE, while local login can choose GNOME or XFCE

### 6. **zerotier_setup_need_systemd.sh**
- **Purpose**: Install and configure ZeroTier VPN for secure remote access
- **What it does**:
  - Prompts for ZeroTier Network ID
  - Installs ZeroTier from official source if not present
  - Detects systemd availability
  - Enables and starts zerotier-one service (if systemd available)
  - Falls back to manual daemon start for non-systemd systems
  - Sets up cron job to keep daemon alive on non-systemd systems
  - Joins specified ZeroTier network
  - Checks if already joined to avoid duplicate joins
- **Usage**: `sudo bash zerotier_setup_need_systemd.sh`
- **Use case**: Essential for accessing robots remotely over secure virtual network
- **Features**:
  - Systemd detection with fallback
  - Automatic daemon management
  - Prevents duplicate network joins
  - Works on both systemd and non-systemd systems (including WSL)

## Relationship to ROS2 Humble Installation

These scripts form a **layered installation approach**:

```
Layer 1: Base System Configuration
├── fix_apt_sources.sh (if needed)
├── sudo_no_passwd.sh (optional, for convenience)

Layer 2: Core ROS2 Installation
├── install_ros2_humble.sh (REQUIRED FIRST)

Layer 3: Advanced Robotics Packages
├── install_slam_packages.sh (builds on Layer 2)

Layer 4: Remote Access & Networking (optional)
├── xrdp_setup.sh (for remote desktop access)
└── zerotier_setup_need_systemd.sh (for VPN access)
```

### Integration with WayfindR-driver Project

This folder is part of a larger robotics project structure:
- **Related folders**: `ros2_install_attempt/`, `ros2_cartography_attempt/`, `ros2_localization_attempt/`
- **Purpose**: The `ros2_install_attempt/` folder contains similar scripts (e.g., `01_install_ros2_humble.sh`) with more documentation
- **Relationship**: The `system_scripts_humble_ubu22.04/` folder appears to be a refined, production-ready version of installation scripts tested in the `ros2_install_attempt/` directory

## Current State of Implementation

### Completeness
- All 6 scripts are **fully implemented and executable** (permissions: `-rwxr-xr-x`)
- Scripts follow bash best practices with proper error handling
- Last modified: December 19, 2025

### Code Quality
- **Robust error handling**: Uses `set -e` and `set -u` or `set -euo pipefail`
- **Idempotent design**: Most scripts can be run multiple times safely
- **User-friendly output**: Clear progress messages with step indicators
- **Comments**: Well-commented with usage instructions
- **Exit codes**: Proper exit codes for scripting integration

### Testing Status
- Scripts are production versions based on testing in `ros2_install_attempt/`
- The `install_ros2_humble.sh` and `install_slam_packages.sh` have been referenced in multiple project documentation files
- No known bugs or issues reported in the scripts themselves

### Deployment
- Scripts are deployment-ready for Ubuntu 22.04 systems
- Suitable for both physical robot platforms (Raspberry Pi) and development workstations
- Can be run locally or fetched via curl (as shown in main README.md)

## Dependencies and Requirements

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Architecture**: AMD64 or ARM64 (for Raspberry Pi)
- **Python**: Python 3.10+ (Ubuntu 22.04 default is 3.10)
- **Disk Space**: Minimum 10GB free for full installation
- **Memory**: Minimum 4GB RAM (8GB+ recommended for SLAM/navigation)
- **Network**: Internet connection required for package downloads

### Software Dependencies (automatically installed)

#### Core System Packages
- `curl` - For downloading repository keys
- `locales` - For UTF-8 locale configuration
- `software-properties-common` - For repository management

#### ROS2 Packages
- `ros-humble-desktop` - Core ROS2 with GUI tools
- `ros-dev-tools` - ROS2 development utilities
- `python3-colcon-common-extensions` - Workspace build system

#### SLAM & Navigation Packages
- `ros-humble-slam-toolbox` - SLAM implementation
- `ros-humble-navigation2` - Navigation stack
- `ros-humble-nav2-bringup` - Navigation launch files
- `ros-humble-nav2-map-server` - Map serving
- `ros-humble-nav2-simple-commander` - Python navigation API
- `ros-humble-robot-localization` - Sensor fusion/EKF

#### Visualization & Simulation
- `ros-humble-rviz2` - 3D visualization
- `ros-humble-rqt` - GUI plugin framework
- `gazebo` - Robot simulator
- `ros-humble-gazebo-ros-pkgs` - Gazebo-ROS integration

#### Hardware Interfaces
- `ros-humble-ros2-control` - Hardware abstraction
- `ros-humble-ros2-controllers` - Pre-built controllers
- `ros-humble-imu-tools` - IMU sensor processing
- `python3-serial` - Serial port communication

#### Python Libraries
- `flask`, `flask-cors` - Web API framework
- `pyserial` - Serial communication
- `pyyaml` - YAML parsing
- `numpy`, `scipy` - Numerical computing
- `matplotlib` - Plotting
- `transforms3d` - 3D transformations

#### Remote Access
- `xrdp`, `xfce4`, `xorgxrdp` - Remote desktop
- `lightdm` - Display manager
- `zerotier-one` - VPN client

### External Dependencies
- **ROS2 repository**: `http://packages.ros.org/ros2/ubuntu`
- **GitHub**: AutoFrontierSearch_ros2-humble (frontier exploration)
- **ZeroTier**: Official installation script

### User Permissions
- **dialout group**: Required for serial port access (USB devices, Arduino, etc.)
- **ssl-cert group**: Required for xRDP certificate access
- **sudo access**: Required to run all scripts

## Recommended Installation Order

1. **Initial Setup** (if needed):
   ```bash
   sudo bash fix_apt_sources.sh  # Only if you have apt source conflicts
   sudo bash sudo_no_passwd.sh   # Optional, for convenience
   ```

2. **Core ROS2 Installation** (required):
   ```bash
   sudo bash install_ros2_humble.sh
   # Open new terminal or: source ~/.bashrc
   ```

3. **Advanced Robotics Stack** (required for WayfindR):
   ```bash
   bash install_slam_packages.sh
   # Log out and log back in (for dialout group)
   ```

4. **Remote Access** (optional):
   ```bash
   sudo bash zerotier_setup_need_systemd.sh  # For VPN access
   sudo bash xrdp_setup.sh --install         # For remote desktop
   ```

## Notes and Considerations

- **Execution time**: Full installation (steps 2-3) takes 20-45 minutes depending on network speed
- **Reboots**: Not strictly required but recommended after full installation
- **Group membership**: Log out and log back in after installation for group changes to take effect
- **ROS2 workspace**: Default workspace created at `~/ros2_ws`
- **Source scripts**: ROS2 must be sourced in each new terminal unless added to `.bashrc` (scripts do this automatically)
- **Testing**: Test installation with: `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener`

## Related Documentation

- Main project README: `/home/devel/Desktop/WayfindR-driver/README.md`
- ROS2 installation attempts: `/home/devel/Desktop/WayfindR-driver/ros2_install_attempt/`
- Setup documentation: `/home/devel/Desktop/WayfindR-driver/docs/setup/`

## Maintenance and Updates

- Scripts target ROS2 Humble LTS (Long Term Support until 2027)
- Ubuntu 22.04 LTS supported until 2027 (standard support) / 2032 (extended security)
- Scripts may need updates when ROS2 Jazzy or newer distributions are adopted
- ZeroTier and Gazebo versions are installed from official repositories (auto-updated)

## Troubleshooting

Common issues and solutions:

1. **APT lock errors**: `install_slam_packages.sh` includes automatic waiting mechanism
2. **Duplicate source errors**: Run `fix_apt_sources.sh`
3. **ROS2 command not found**: Ensure you've sourced `/opt/ros/humble/setup.bash` or opened a new terminal
4. **Serial port permission denied**: Ensure user is in `dialout` group and has logged out/in
5. **xRDP black screen**: Scripts configure XFCE to prevent this common issue
6. **ZeroTier daemon not starting**: Script detects systemd vs non-systemd and adapts accordingly

---

**Last Updated**: January 11, 2026
**ROS2 Distribution**: Humble Hawksbill
**Target OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
**Project**: WayfindR-driver autonomous navigation platform
