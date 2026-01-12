# WayfindR-driver Project Scope

## Overview
WayfindR-driver is a comprehensive robotics project aimed at creating an autonomous navigation robot powered by a Raspberry Pi (or potentially Jetson Nano) with LiDAR-based mapping and localization capabilities.

## Project Goals

### Hardware Platform
- **Main Computer**: Raspberry Pi (current) or Jetson Nano (future expansion)
- **LiDAR Sensor**: RP LIDAR C1M1 connected via USB serial
- **Motor System**: Differential drive with motor drivers controlled by the Raspberry Pi
- **Future Expansion**: IMU/MPU-6050 for sensor fusion and improved localization

### Core Functionality

#### 1. Autonomous Mapping (Cartography)
- Use ROS2 cartography packages to autonomously create maps of the environment
- Save generated maps for later use
- Support map editing on remote servers to add waypoints with names and locations

#### 2. Localization
- Implement ROS2 localization strategies to determine robot position within known maps
- Use LiDAR data for position estimation
- Future: Integrate IMU data for sensor fusion

#### 3. Autonomous Navigation
- Navigate to named waypoints on pre-generated maps
- Obstacle avoidance using LiDAR data
- Path planning and execution

#### 4. Remote Control & API
- Wi-Fi enabled communication
- Web server running on the Raspberry Pi
- API endpoints for:
  - Waypoint navigation commands
  - Manual motor control (forward, backward, turn left, turn right, etc.)
  - Map upload/download
  - Status monitoring

### Technology Stack
- **ROS2**: Primary framework for robotics functionality (provides robust tools for mapping, localization, and navigation)
- **Python**: Primary programming language for API and control scripts
- **Web Server**: For remote command reception over Wi-Fi
- **USB Serial**: Communication protocol for LiDAR

## Workflow

1. **Mapping Phase**: Robot autonomously explores and maps the environment using cartography
2. **Map Processing**: Human operator downloads map, adds named waypoints on remote server
3. **Navigation Phase**: Upload edited map to robot, command it to navigate to specific waypoints via API
4. **Operation Modes**:
   - Autonomous navigation to waypoints
   - Manual control via API commands
   - Monitoring and status reporting

## Project Structure
This repository contains multiple folders for different aspects of the project:
- ROS2 localization experiments and implementations
- ROS2 cartography/mapping implementations
- API servers for Raspberry Pi control
- Hardware integration scripts
- Testing and demonstration code
- Installation and setup scripts

Each folder contains its own scope.md file detailing its specific purpose and implementation.

## Future Enhancements
- Integration of IMU/MPU-6050 for improved localization through sensor fusion
- Migration to Jetson Nano for increased computational power
- Advanced navigation algorithms
- Multi-robot coordination
