# WayfindR-driver Folder Guide

## Quick Navigation: "I want to do X, which folder do I use?"

This guide helps you quickly identify which folder to work in based on your development goal.

---

## ROS2 Development

### "I want to create maps of my environment (SLAM/Mapping)"
**Use: [ros2_cartography_attempt/](../ros2_cartography_attempt/)**
- Purpose: 2D SLAM using SLAM Toolbox with RP LIDAR C1M1
- Creates `.pgm` and `.yaml` map files
- Includes waypoint management
- Has complete launch files and configuration

### "I want to localize the robot on an existing map"
**Use: [ros2_localization_attempt/](../ros2_localization_attempt/)**
- Purpose: AMCL particle filter localization on pre-built maps
- A* pathfinding for waypoint navigation
- Monitoring and diagnostic tools
- Real-time position estimation

### "I want a complete, all-in-one ROS2 navigation system"
**Use: [ros2_comprehensive_attempt/](../ros2_comprehensive_attempt/)**
- Purpose: Portable, self-contained ROS2 Humble navigation stack
- Combines SLAM + Localization + Navigation
- Doesn't require a ROS2 workspace
- Drag-and-drop deployment ready
- Best for: Production deployments, testing full workflow

### "I need to install/setup ROS2 on a new system"
**Use: [system_scripts_humble_ubu22.04/](../system_scripts_humble_ubu22.04/)** ✅ STABLE
- Purpose: Production-ready installation scripts
- Installs ROS2 Humble, SLAM, Nav2, all dependencies
- System-level prerequisites
- **Status: Complete and working** - Only modify if something breaks

**Alternative: [ros2_install_attempt/](../ros2_install_attempt/)** (archived reference)
- Older installation package with extensive documentation
- Use for reference only

**Alternative: [pi_scripts/](../pi_scripts/)** (foundation layer)
- Basic setup for headless Raspberry Pi
- Use if you need minimal/custom installation

---

## Robot Control & API

### "I want to control the robot via web API (WiFi commands)"
**Use: [PI_API/](../PI_API/)**
- Purpose: FastAPI server for robot control
- REST API endpoints + WebSocket telemetry
- Motor control, navigation commands, waypoint management
- Web dashboard with keyboard controls
- Runs on Raspberry Pi, receives commands over WiFi

### "I need real-time motor control with guaranteed timing"
**Use: [esp32_api/](../esp32_api/)**
- Purpose: Dual-core ESP32 firmware for hardware control
- 100-1000Hz control loop on dedicated core
- Receives high-level commands from Raspberry Pi via WiFi
- Best for: Time-critical motor control, ESC/servo control

---

## Hardware Platform Testing

### "I'm testing with Adeept PiCar-Pro hardware"
**Use: [adeept_car_stuff/](../adeept_car_stuff/)**
- Purpose: Motor control testing for Adeept Robot HAT
- Differential drive validation
- Simple GPIO-based control
- Good for: Algorithm development, quick testing

### "I'm testing with XiaoR ROS Tank hardware"
**Use: [ros_tank_xiaor/](../ros_tank_xiaor/)**
- Purpose: XiaoR GEEK ROS Tank integration
- Commercial ROS-compatible platform
- Includes encoders, IMU, XR-Lidar S1
- Good for: Baseline comparison, known-good hardware

---

## System Deployment & Fleet Management

### "I need to provision/flash new Raspberry Pi SD cards"
**Use: [new_bakery/](../new_bakery/)**
- Purpose: Automated SD card flashing and cloud-init setup
- Creates ready-to-use ROS2 robotics platforms
- CLI-based, simple, production-ready

### "I need to manage multiple Raspberry Pi robots remotely"
**Use: [pi-fleet-manager/](../pi-fleet-manager/)**
- Purpose: Web-based fleet management system
- SSH-based remote operations
- Deploy code/configs to multiple robots
- Interactive terminal, command execution

---

## Demos & Examples

### "I want to see example code or test specific hardware"
**Use: [demos/](../demos/)**
- RC receiver demos (FlySky FS-i6B)
- Voice AI + Raspberry Pi control system
- Basic hardware integration examples

---

## Other Projects (Non-Robotics)

### "Livestock health monitoring with computer vision"
**Use: [sheep_helper/](../sheep_helper/)**
- Completely unrelated to WayfindR robotics
- Agricultural CV application
- Face detection + pixel analysis for health screening

---

## Decision Tree

```
START: What do you want to do?
│
├─ ROS2 Development?
│  ├─ Create maps ───────────────────────► ros2_cartography_attempt/
│  ├─ Localize on map ───────────────────► ros2_localization_attempt/
│  ├─ Complete navigation system ────────► ros2_comprehensive_attempt/
│  └─ Install/setup ROS2 ────────────────► system_scripts_humble_ubu22.04/
│
├─ Robot Control?
│  ├─ WiFi API control ──────────────────► PI_API/
│  └─ Real-time motor control ───────────► esp32_api/
│
├─ Hardware Testing?
│  ├─ Adeept platform ───────────────────► adeept_car_stuff/
│  └─ XiaoR platform ────────────────────► ros_tank_xiaor/
│
├─ Deployment/Management?
│  ├─ Flash SD cards ────────────────────► new_bakery/
│  └─ Manage robot fleet ────────────────► pi-fleet-manager/
│
└─ Examples/Demos ───────────────────────► demos/
```

---

## Development Environment Philosophy

### Consistent Environment Across All Systems
- **OS**: Ubuntu 22.04 LTS (same on dev machine and Raspberry Pi)
- **Python**: System default (Python 3.10 on Ubuntu 22.04) - **DO NOT CHANGE**
- **ROS**: ROS2 Humble (only version supported)
- **LiDAR**: RP LIDAR C1M1 (Slamtec C1M1RP)

### Development Workflow
1. Develop on local Ubuntu 22.04 machine
2. Test with same packages/configuration
3. Deploy directly to Raspberry Pi (identical environment)
4. No surprises, no compatibility issues

### Why System Python 3.10?
- ROS2 Humble is compiled against Ubuntu 22.04's system Python 3.10
- Changing Python version breaks ROS2 package compatibility
- See [pi_scripts/install_ros2_humble.md](../pi_scripts/install_ros2_humble.md) for detailed explanation

---

## Folder Maturity Status

| Folder | Status | Notes |
|--------|--------|-------|
| system_scripts_humble_ubu22.04 | ✅ Stable | Don't touch unless broken |
| ros2_comprehensive_attempt | 🔨 Active Development | Primary development target |
| ros2_cartography_attempt | ✅ Working | Mapping functionality complete |
| ros2_localization_attempt | ✅ Working | Localization functionality complete |
| PI_API | 🔨 Active Development | API framework complete, needs ROS2 integration |
| esp32_api | ✅ Working | Production-ready for ground vehicles |
| new_bakery | ✅ Stable | Provisioning system complete |
| pi-fleet-manager | ✅ Working | Fleet management complete |
| adeept_car_stuff | 🧪 Experimental | Testing platform |
| ros_tank_xiaor | 🧪 Experimental | Evaluation platform (15% complete) |
| demos | 📚 Reference | Examples and tutorials |
| old_bakery | 🗄️ Archived | Superseded by new_bakery |
| old_stuff | 🗄️ Archived | Historical reference |
| ros2_install_attempt | 🗄️ Archived | Reference documentation |

**Legend:**
- ✅ Stable - Production ready, only change if broken
- 🔨 Active Development - Primary focus for new features
- ✅ Working - Functional but may need enhancements
- 🧪 Experimental - Testing/evaluation only
- 📚 Reference - Examples and learning resources
- 🗄️ Archived - Historical reference, don't modify

---

## Current Development Priorities

### Phase 1: ROS2 Navigation Foundation (CURRENT)
**Focus Folders:**
- `ros2_comprehensive_attempt/` - Complete navigation stack
- `ros2_cartography_attempt/` - Mapping improvements
- `ros2_localization_attempt/` - Localization tuning

### Phase 2: API Integration
**Focus Folders:**
- `PI_API/` - Connect to ROS2 topics/actions
- Integration testing with navigation stack

### Phase 3: Production Deployment
**Focus Folders:**
- `new_bakery/` - Ensure provisioning includes all updates
- `pi-fleet-manager/` - Deploy to multiple robots

---

## Quick Reference Commands

### Start Mapping
```bash
cd ros2_cartography_attempt/
./start_all.sh
```

### Start Localization
```bash
cd ros2_localization_attempt/
./launch_amcl_localization.sh
```

### Install ROS2 on New System
```bash
cd system_scripts_humble_ubu22.04/
sudo ./install_ros2_humble.sh
sudo ./install_slam_packages.sh
```

### Start Robot API Server
```bash
cd PI_API/
python3 main.py
```

### Flash Raspberry Pi SD Card
```bash
cd new_bakery/
sudo ./bake_auto.sh
```

---

## Getting Help

Each folder contains:
- **scope.md** - What the folder is for, current state, dependencies
- **roadmap.md** - Development roadmap and next steps (ROS2 folders only)
- **findings/** - Research notes, testing results, development findings
- **README.md** - Quick start guide (where applicable)

See [docs/roadmap.md](roadmap.md) for overall project roadmap and development strategy.
