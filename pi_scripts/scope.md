# Pi Scripts Scope Documentation

## Overview

The `pi_scripts` folder contains essential documentation and installation scripts specifically designed to set up a Raspberry Pi as a headless ROS2 Humble robot node for the WayfindR autonomous navigation robot project. This folder provides step-by-step guides and automated installation scripts for configuring the Raspberry Pi operating environment and installing ROS2 Humble on Ubuntu 22.04.

## Purpose

This folder serves as the **initial setup and configuration hub** for the Raspberry Pi hardware platform that powers the WayfindR robot. It bridges the gap between a fresh Ubuntu 22.04 installation on a Raspberry Pi and a fully functional ROS2 Humble robot platform capable of running SLAM, localization, and autonomous navigation.

The scripts and documentation here are designed to:

1. **Initial System Configuration** - Network setup, SSH configuration, and basic system settings
2. **ROS2 Installation** - Complete headless ROS2 Humble installation optimized for Raspberry Pi
3. **Python Environment Setup** - Ensure Python 3.10 compatibility with ROS2 Humble
4. **Robot Node Preparation** - Install SLAM, Nav2, and sensor drivers for autonomous navigation

---

## Files in This Folder

### 1. `initial_setup.md` (527 bytes)

**Purpose:** Basic Raspberry Pi initial configuration guide for network and SSH setup.

**Contents:**
- **Network Configuration** - WiFi setup using netplan for Ubuntu
  - Default network: EagleNet (open network configuration)
  - Configuration file: `/etc/netplan/50-cloud-init.yaml`
  - Commands to apply network settings (`netplan try`, `netplan generate`)

- **SSH Configuration** - Enable both password and public key authentication
  - Configuration file: `/etc/ssh/sshd_config`
  - Settings:
    - `PubkeyAuthentication yes`
    - `PasswordAuthentication yes`
  - Allows flexible remote access during development

**When to Use:**
- First boot of Ubuntu 22.04 on Raspberry Pi
- Before running ROS2 installation scripts
- When setting up headless operation (no monitor/keyboard)

**State:** Basic configuration notes, not a comprehensive automated script.

---

### 2. `install_ros2_humble.md` (15,221 bytes)

**Purpose:** Comprehensive documentation covering ROS2 Humble installation with detailed Python version compatibility information.

**Contents:**

#### Section 1: Basic ROS2 Installation Steps
- Add ROS2 apt repository and GPG keys
- Configure package sources
- Install ROS2 packages
- Environment setup in `.bashrc`

#### Section 2: Python Version Compatibility Analysis
Extensive discussion (from lines 36-245) covering critical Python version considerations:

1. **Why System Python Matters**
   - ROS2 Humble binaries on Ubuntu 22.04 are built against Python 3.10
   - Changing system Python can break apt, system tools, and ROS2
   - Explains risks of overriding `/usr/bin/python3` symlink

2. **Virtual Environment Considerations**
   - ROS2 expects system Python (not virtualenv Python)
   - Running ROS2 nodes in venv causes import errors
   - `ros2 run` bypasses virtualenv's PYTHONPATH

3. **Recommended Approach**
   - Use Ubuntu 22.04's default Python 3.10
   - Do NOT override system Python globally
   - Install ROS2 Humble via apt (binary packages)
   - Use venv/virtualenv only for non-ROS2 Python projects

4. **What Version Works Best**
   - **Python 3.10** is the correct and recommended version
   - This is Ubuntu 22.04's system default
   - ROS2 Humble bindings (`rclpy`) are built against Python 3.10
   - Mismatched Python versions cause `rclpy._rclpy_pybind11` errors

**Key Warnings:**
- Do not force Python 3.10 as default if it's already the system Python
- Building ROS2 from source is only needed for custom Python versions
- Docker/containers recommended if isolation is required

**When to Use:**
- Understanding Python compatibility before installation
- Troubleshooting Python-related ROS2 import errors
- Reference for why system Python must not be changed

**State:** Informational documentation with sourced research (includes links to external references).

---

### 3. `install_ros2-humble_py3-10_ubu-22.04.sh` (5,586 bytes, 197 lines)

**Purpose:** Master automated installation script for a complete headless ROS2 Humble robot node on Ubuntu 22.04 Raspberry Pi.

**Script Features:**

#### Idempotent Design
- Safe to run multiple times without breaking the system
- Checks for existing configurations before adding duplicates
- Uses `append_if_missing()` helper function for bashrc modifications

#### Installation Steps (Detailed Breakdown)

**Step 1: System Update & Locale** (lines 34-42)
- Updates package lists and upgrades system
- Sets locale to `en_US.UTF-8`
- Ensures proper character encoding for ROS2

**Step 2: Universe Repository** (lines 44-49)
- Enables Ubuntu Universe repository
- Required for ROS2 packages

**Step 3: ROS2 APT Repository Configuration** (lines 51-69)
- Cleans up any existing ROS2 sources to prevent conflicts
- Downloads ROS2 GPG key from official repository
- Adds signed ROS2 package source to apt
- Uses idempotent `append_if_missing()` function

**Step 4: Python Package Installation** (lines 71-72)
- Installs Python 3.10 development tools:
  - `python3-venv` - Virtual environment support
  - `python3-dev` - Development headers
  - `python3-distutils` - Distribution utilities
  - `python3-pip` - Package installer
  - `python3-colcon-common-extensions` - ROS2 build system

**Step 5: ROS2 Humble Headless Installation** (lines 74-80)
- Installs `ros-humble-ros-base` (no GUI components)
- Optimized for headless Raspberry Pi operation
- Installs `ros-dev-tools` for development utilities

**Step 6: Environment Configuration** (lines 82-89)
- Auto-sources ROS2 setup in `~/.bashrc`
- Ensures ROS2 commands available in all new terminal sessions

**Step 7: ROS2 Workspace Setup** (lines 91-102)
- Creates `~/ros2_ws/src` directory structure
- Performs initial colcon build (idempotent)
- Adds workspace sourcing to bashrc

**Step 8: SLAM & Navigation Packages** (lines 104-129)
Installs comprehensive robotics packages:

- **Transform & State Publishing:**
  - `ros-humble-xacro` - XML macros for robot descriptions
  - `ros-humble-tf2-tools` - Transform debugging
  - `ros-humble-tf2-ros` - Transform library
  - `ros-humble-tf-transformations` - Coordinate transformations
  - `ros-humble-robot-state-publisher` - URDF to TF publisher

- **SLAM (Simultaneous Localization and Mapping):**
  - `ros-humble-slam-toolbox` - Graph-based SLAM
  - `ros-humble-cartographer` - Google Cartographer SLAM
  - `ros-humble-cartographer-ros` - ROS2 Cartographer interface

- **Navigation (Nav2):**
  - `ros-humble-navigation2` - Full Nav2 stack
  - `ros-humble-nav2-bringup` - Nav2 launch files
  - `ros-humble-nav2-map-server` - Map loading/saving
  - `ros-humble-nav2-simple-commander` - Python navigation API
  - `ros-humble-robot-localization` - Sensor fusion (EKF/UKF)

**Step 9: IMU & Sensor Drivers** (lines 131-138)
- `ros-humble-imu-tools` - IMU calibration and visualization
- `ros-humble-imu-filter-madgwick` - Orientation estimation filter

**Step 10: Python Dependencies** (lines 140-152)
Installs Python packages for robot control and data processing:
- `pyserial` - Serial communication (for LiDAR, IMU)
- `pyyaml` - Configuration file parsing
- `numpy` - Numerical operations
- `scipy` - Scientific computing
- `transforms3d` - 3D coordinate transformations
- `flask` - Web server for remote API
- `flask-cors` - Cross-origin resource sharing

**Step 11: Explore Lite (Autonomous Exploration)** (lines 154-174)
- Clones `m-explore` package from source (Humble branch)
- Provides autonomous frontier exploration for mapping
- Runs `rosdep install` to resolve dependencies
- Builds workspace with Release optimizations

**Completion Output** (lines 176-196)
- Displays installation summary
- Provides testing commands
- Suggests next steps:
  - Add systemd services for auto-start
  - Create URDF robot description
  - Set up RPLIDAR or other sensors
  - Configure TF tree

#### Safety Features
- `set -e` - Exit on any error
- Idempotent operations throughout
- Graceful handling of existing installations
- `|| true` for non-critical operations (allows continuation)

**When to Use:**
- Fresh Ubuntu 22.04 installation on Raspberry Pi
- Setting up multiple identical robot nodes
- Recovering from corrupted ROS2 installation

**Dependencies:**
- Ubuntu 22.04 (Jammy Jellyfish)
- Internet connection for package downloads
- Sudo privileges
- Minimum 4GB RAM, 8GB+ storage recommended

**Estimated Installation Time:** 15-30 minutes (depending on network speed and Pi model)

---

## How These Scripts Support Main Robot Functionality

### Integration with WayfindR System Architecture

The `pi_scripts` folder is the **foundation layer** of the WayfindR robot system. Here's how it fits into the overall architecture:

```
┌─────────────────────────────────────────────────────────────┐
│  WayfindR Robot System Architecture                         │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Layer 5: Application Layer                                  │
│  ├─ PI_API (FastAPI web control)                            │
│  ├─ Navigation commands (waypoint management)               │
│  └─ User interfaces (web dashboard, API clients)            │
│                                                               │
│  Layer 4: ROS2 Application Packages                         │
│  ├─ ros2_comprehensive_attempt (SLAM, localization, Nav2)   │
│  ├─ ros2_cartography_attempt (mapping workflows)            │
│  └─ Custom navigation nodes                                 │
│                                                               │
│  Layer 3: ROS2 System Packages                              │
│  ├─ Nav2 (installed by pi_scripts)                          │
│  ├─ SLAM Toolbox (installed by pi_scripts)                  │
│  ├─ Cartographer (installed by pi_scripts)                  │
│  └─ Sensor drivers (installed by pi_scripts)                │
│                                                               │
│  Layer 2: ROS2 Core Infrastructure                          │
│  ├─ ROS2 Humble Base (installed by pi_scripts)              │
│  ├─ Python 3.10 environment (configured by pi_scripts)      │
│  └─ ROS2 workspace (created by pi_scripts)                  │
│                                                               │
│  Layer 1: Operating System & Hardware                        │
│  ├─ Ubuntu 22.04 (configured by pi_scripts)                 │
│  ├─ Network setup (configured by initial_setup.md)          │
│  ├─ SSH access (configured by initial_setup.md)             │
│  └─ Raspberry Pi hardware                                   │
└─────────────────────────────────────────────────────────────┘
```

### Specific Support for Robot Functions

#### 1. SLAM Mapping Support
The installation script provides:
- **SLAM Toolbox** - For graph-based SLAM mapping
- **Cartographer** - For Google's Cartographer SLAM algorithm
- **TF2 Tools** - For managing coordinate frame transformations
- **Robot State Publisher** - For URDF to TF tree conversion

These packages enable the robot to:
- Build 2D/3D maps of environments using LiDAR data
- Detect loop closures to correct drift
- Save and load maps for later use
- Support the `ros2_cartography_attempt` and `ros2_comprehensive_attempt` workflows

#### 2. Localization Support
The installation script provides:
- **AMCL** (via Nav2 packages) - Particle filter localization
- **Robot Localization** - Extended Kalman Filter for sensor fusion
- **Map Server** - For loading pre-built maps

These packages enable the robot to:
- Determine its position on known maps
- Fuse IMU and wheel odometry data
- Support the `ros2_localization_attempt` workflows

#### 3. Autonomous Navigation Support
The installation script provides:
- **Navigation2 Stack** - Complete navigation framework
- **Nav2 Simple Commander** - Python API for navigation control
- **m-explore** - Autonomous frontier exploration
- **Nav2 Map Server** - Map management

These packages enable the robot to:
- Plan collision-free paths to waypoints
- Execute autonomous navigation missions
- Avoid obstacles dynamically
- Support the PI_API navigation endpoints
- Enable autonomous exploration and mapping

#### 4. Sensor Integration Support
The installation script provides:
- **IMU Tools** - For IMU calibration and filtering
- **Madgwick Filter** - For orientation estimation
- **Serial Communication** (pyserial) - For LiDAR/IMU communication

These packages enable the robot to:
- Integrate MPU-6050, BNO055, or similar IMUs
- Interface with Slamtec C1M1/RPLidar sensors
- Provide orientation data for navigation

#### 5. Remote Control Support
The installation script provides:
- **Flask & Flask-CORS** - Web server framework
- **NumPy & SciPy** - Numerical processing for control algorithms
- **PyYAML** - Configuration file parsing

These packages enable:
- The PI_API web server to run on the Raspberry Pi
- Remote waypoint commands over WiFi
- Configuration file management
- Integration with the web dashboard

---

## Current State of Implementation

### Completed Components

✓ **Network Configuration Documentation** (`initial_setup.md`)
- WiFi setup guide with netplan examples
- SSH configuration for remote access
- Basic system preparation steps

✓ **Python Compatibility Documentation** (`install_ros2_humble.md`)
- Comprehensive Python 3.10 compatibility research
- ROS2 Humble system requirements analysis
- Best practices for Python environment management
- Warnings about common pitfalls

✓ **Automated Installation Script** (`install_ros2-humble_py3-10_ubu-22.04.sh`)
- Complete ROS2 Humble installation
- SLAM and Nav2 package installation
- Python dependency installation
- Workspace setup automation
- Idempotent design (safe to re-run)
- m-explore frontier exploration package

### Missing Components

✗ **Hardware-Specific Setup Scripts**
- No LiDAR setup script (e.g., RPLidar udev rules)
  - Contrast: `ros2_comprehensive_attempt/scripts/setup_lidar.sh` exists
- No IMU setup/calibration script
- No motor controller setup (L298N GPIO configuration)

✗ **Systemd Service Files**
- Script mentions "Add systemd services for LiDAR, SLAM, Nav2"
- No actual service files included for auto-start on boot

✗ **URDF/Robot Description**
- Script mentions "Create URDF + TF tree"
- No example URDF files for WayfindR robot

✗ **Testing/Validation Scripts**
- No automated verification script to test installation
- No diagnostic tool to check ROS2 installation health

✗ **Pi-Specific Optimizations**
- No CPU governor/frequency scaling configuration
- No swap configuration for memory-constrained Pi models
- No thermal throttling management

✗ **Security Hardening**
- No firewall configuration
- No fail2ban setup for SSH protection
- No user permission restrictions

### Implementation Status: ~70% Complete

**What Works:**
- Fresh Ubuntu 22.04 on Raspberry Pi can be transformed into a functional ROS2 robot node
- All necessary ROS2 packages are installed
- Python dependencies are in place
- Basic network and SSH configuration documented

**What's Missing:**
- Hardware-specific configuration automation
- Production deployment features (systemd services)
- Robot-specific configurations (URDF, TF tree)
- Security and optimization

---

## Dependencies and Requirements

### System Requirements

**Hardware:**
- Raspberry Pi 3B+, 4, or 5 (4GB+ RAM recommended)
- MicroSD card (32GB+ recommended, Class 10 or UHS-1)
- Stable 5V power supply (3A minimum for Pi 4/5)
- Keyboard and monitor for initial setup (or headless SSH setup)
- LiDAR sensor (Slamtec C1M1/RPLidar) with USB connection
- Optional: IMU (MPU-6050, BNO055), motor drivers (L298N)

**Operating System:**
- Ubuntu 22.04 LTS Server or Desktop (arm64 for Pi 4/5)
- Fresh installation recommended
- Internet connectivity required

**Network:**
- WiFi or Ethernet connection
- Access to package repositories (ROS, Ubuntu)
- Static IP or DHCP reservation recommended for robot control

### Software Dependencies (Installed by Script)

**ROS2 Packages:**
- ros-humble-ros-base (core ROS2)
- ros-humble-slam-toolbox
- ros-humble-cartographer + cartographer-ros
- ros-humble-navigation2 (complete Nav2 stack)
- ros-humble-nav2-bringup
- ros-humble-nav2-map-server
- ros-humble-nav2-simple-commander
- ros-humble-robot-localization
- ros-humble-imu-tools
- ros-humble-imu-filter-madgwick
- ros-humble-xacro
- ros-humble-tf2-tools + tf2-ros
- ros-humble-tf-transformations
- ros-humble-robot-state-publisher
- ros-humble-rplidar-ros (for RPLidar driver)
- ros-dev-tools
- python3-colcon-common-extensions

**Python Packages:**
- pyserial (serial communication)
- pyyaml (YAML parsing)
- numpy (numerical operations)
- scipy (scientific computing)
- transforms3d (coordinate transformations)
- flask (web server)
- flask-cors (API cross-origin support)

**System Packages:**
- locales
- software-properties-common
- curl, gnupg2
- python3-venv, python3-dev, python3-distutils, python3-pip
- Git (for m-explore clone)

### External Dependencies

**m-explore Package:**
- Source: https://github.com/robo-friends/m-explore.git
- Branch: humble
- Cloned into `~/ros2_ws/src/m-explore`
- Built with colcon

### Storage Requirements

- ROS2 Humble packages: ~2-3 GB
- Build workspace (compiled packages): ~500 MB
- Python packages: ~200 MB
- Total recommended free space: 8+ GB

### Memory Requirements

- Minimum: 4 GB RAM
- Recommended: 8 GB RAM (for SLAM with large maps)
- Swap: 2-4 GB recommended on 4GB Pi models

### Network Requirements

- Bandwidth: ~500 MB download for initial package installation
- Continuous access for rosdep, apt updates
- Low latency preferred for remote control via PI_API

---

## Relationship to Other Project Folders

### Direct Relationship: Foundation Layer

`pi_scripts` → Enables → **All ROS2-based folders**

This folder is the **prerequisite** for:
- `ros2_comprehensive_attempt` - Requires ROS2 Humble, Nav2, SLAM Toolbox
- `ros2_cartography_attempt` - Requires Cartographer, SLAM Toolbox
- `ros2_localization_attempt` - Requires AMCL, Robot Localization
- `ros2_install_attempt` - Alternative installation approach

### Indirect Relationship: Support Layer

`pi_scripts` → Enables → `PI_API`
- Flask and Python dependencies installed by pi_scripts
- Allows PI_API web server to run on Raspberry Pi
- Supports ROS2 integration in PI_API navigation service

`pi_scripts` → Enables → `esp32_api`
- Provides ROS2 environment for potential ESP32-ROS2 bridge
- Supports serial communication libraries

### Complementary Relationship

`pi_scripts` + `system_scripts_humble_ubu22.04`:
- `pi_scripts` focuses on ROS2 installation and basic setup
- `system_scripts_humble_ubu22.04` provides additional system utilities:
  - `fix_apt_sources.sh` - APT repository troubleshooting
  - `xrdp_setup.sh` - Remote desktop setup
  - `zerotier_setup_need_systemd.sh` - VPN networking
  - `sudo_no_passwd.sh` - Passwordless sudo for robot services

Both folders work together to prepare the Raspberry Pi for production robot operation.

---

## Usage Workflow

### Standard Setup Procedure

```bash
# Step 1: Initial System Configuration
# (On fresh Ubuntu 22.04 installation)
sudo nano /etc/netplan/50-cloud-init.yaml
# Configure WiFi as per initial_setup.md
sudo netplan try
sudo nano /etc/ssh/sshd_config
# Enable password and pubkey authentication

# Step 2: Download pi_scripts
git clone https://github.com/msmcs-robotics/WayfindR-driver.git
cd WayfindR-driver/pi_scripts

# Step 3: Review Python compatibility documentation (optional but recommended)
cat install_ros2_humble.md

# Step 4: Run automated installation
chmod +x install_ros2-humble_py3-10_ubu-22.04.sh
./install_ros2-humble_py3-10_ubu-22.04.sh

# Step 5: Open new terminal or source environment
source ~/.bashrc

# Step 6: Verify installation
ros2 topic list
ros2 pkg list | grep navigation
ros2 pkg list | grep slam

# Step 7: Next steps (from other WayfindR folders)
# - Set up LiDAR (see ros2_comprehensive_attempt/scripts/setup_lidar.sh)
# - Configure URDF/TF tree
# - Set up systemd services
# - Install PI_API for web control
```

### Troubleshooting Common Issues

**Issue: ROS2 commands not found**
```bash
# Solution: Source the environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**Issue: Python import errors with rclpy**
```bash
# Solution: Verify Python version
python3 --version  # Should be 3.10.x
which python3      # Should be /usr/bin/python3 (system Python)
# Do NOT use virtualenv for ROS2 commands
```

**Issue: m-explore build fails**
```bash
# Solution: Manually resolve dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

**Issue: Disk space errors during installation**
```bash
# Solution: Clean up apt cache
sudo apt clean
sudo apt autoremove
df -h  # Check available space
```

---

## Future Enhancement Opportunities

### High Priority

1. **LiDAR Setup Automation**
   - Create `setup_lidar.sh` similar to `ros2_comprehensive_attempt`
   - Automated udev rules for `/dev/rplidar` symlink
   - USB permissions configuration
   - LiDAR health check and diagnostics

2. **Systemd Service Files**
   - Auto-start ROS2 nodes on boot
   - Services for: LiDAR driver, SLAM, Nav2, PI_API
   - Proper dependency ordering
   - Logging and restart policies

3. **Robot Description Package**
   - URDF template for WayfindR robot
   - TF tree configuration
   - Visual models for RViz
   - Physical parameters (dimensions, inertia)

4. **Installation Verification**
   - Post-installation test script
   - Automated checks for all packages
   - Diagnostics report generation
   - Version compatibility verification

### Medium Priority

5. **Performance Optimization**
   - CPU governor configuration for performance mode
   - Swap file optimization
   - Process priority tuning for ROS2 nodes
   - Thermal monitoring scripts

6. **Security Hardening**
   - UFW firewall configuration
   - Fail2ban for SSH protection
   - User permission restrictions
   - ROS2 security extensions (SROS2)

7. **Motor Controller Setup**
   - GPIO pin configuration for L298N
   - PWM setup and testing
   - Motor calibration scripts
   - Emergency stop configuration

8. **IMU Calibration Tools**
   - Automated IMU detection
   - Calibration data collection
   - Calibration file generation
   - Integration testing with robot_localization

### Low Priority

9. **Multi-Pi Deployment**
   - Ansible playbooks for fleet deployment
   - Configuration templating
   - SSH key distribution
   - Centralized logging setup

10. **Development Environment**
    - VS Code remote development setup
    - GDB debugging configuration
    - ROS2 launch file templates
    - Docker containerization for simulation

---

## Technical Notes

### Python Version Constraints

**Critical Requirement:** Must use system Python 3.10
- ROS2 Humble's `rclpy` bindings are compiled for Python 3.10
- Ubuntu 22.04 ships with Python 3.10 by default
- **DO NOT** change the system Python version
- **DO NOT** use virtualenv for ROS2 nodes
- Virtualenvs are acceptable for non-ROS2 scripts (e.g., PI_API)

### Workspace Structure

The script creates:
```
~/ros2_ws/
├── src/           # Source packages
│   └── m-explore/ # Autonomous exploration package
├── build/         # Build artifacts (CMake, intermediate files)
├── install/       # Installed packages (setup.bash here)
└── log/           # Build and runtime logs
```

### Environment Sourcing Order

Proper order in `.bashrc`:
```bash
source /opt/ros/humble/setup.bash      # ROS2 core
source ~/ros2_ws/install/setup.bash    # Workspace overlay
```

Workspace setup extends/overlays ROS2 core packages.

### Headless vs Desktop Installation

This script installs `ros-humble-ros-base` (headless):
- No GUI tools (RViz2, rqt)
- Smaller footprint (~1GB less disk space)
- Suitable for remote-controlled robots

For development workstations, use `ros-humble-desktop` instead:
```bash
sudo apt install ros-humble-desktop
```

### Colcon Build Optimization

The script uses:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- `--symlink-install`: Faster rebuilds, links instead of copies
- `-DCMAKE_BUILD_TYPE=Release`: Optimized binaries (no debug symbols)

For debugging, use `Debug` instead of `Release`.

---

## Version Information

- **Script Version:** 1.0.0
- **Target ROS2 Distribution:** Humble Hawksbill
- **Target OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Target Python Version:** 3.10.x
- **Target Platform:** Raspberry Pi (arm64)
- **Creation Date:** December 2024
- **Last Updated:** January 2026

---

## Related Documentation

### Within WayfindR Project
- `/docs/scope.md` - Overall project architecture and goals
- `/docs/setup/ros2_setup_ubu22.04_humble.md` - Alternative ROS2 setup guide with LiDAR
- `/PI_API/scope.md` - Web API control system for robot
- `/ros2_comprehensive_attempt/scope.md` - Complete SLAM/Nav2 implementation
- `/system_scripts_humble_ubu22.04/` - Additional system utility scripts

### External References
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Ubuntu 22.04 on Raspberry Pi](https://ubuntu.com/download/raspberry-pi)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

**Last Updated:** 2026-01-11
**Document Version:** 1.0.0
**Maintainer:** WayfindR Team
