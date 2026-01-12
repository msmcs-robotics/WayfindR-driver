# Adeept Car Stuff - Scope Documentation

## Overview

The `adeept_car_stuff` folder contains testing and integration code for the **Adeept PiCar-Pro** robot platform with the **Adeept Robot HAT** motor driver board. This serves as one of the hardware testing platforms for the WayfindR autonomous navigation project.

## What This Folder Is For

This folder provides motor control testing utilities for the Adeept PiCar-Pro platform, which is being evaluated as a potential hardware base for the WayfindR indoor navigation robot. It contains code to test differential drive motor control before integrating with the full ROS2 navigation stack.

## Hardware Platform: Adeept PiCar-Pro

### Core Components
- **Adeept Robot HAT**: Motor driver expansion board for Raspberry Pi
- **Differential Drive**: Two DC motors (Motor A and Motor B) for skid-steer control
- **Power**: Dual 18650 battery configuration
- **Raspberry Pi**: Compatible with Pi 3B+, Pi 4, Pi 5

### Motor Driver Specifications (from code analysis)
- **GPIO Control**: Direct PWM control via RPi.GPIO library
- **PWM Frequency**: 1000 Hz
- **Speed Range**: 0-100% duty cycle (default: 70%)
- **Motor Pins**:
  - Motor A (rear right): EN=GPIO4, DIR1=GPIO26, DIR2=GPIO21
  - Motor B (rear left): EN=GPIO17, DIR1=GPIO27, DIR2=GPIO18

### What's NOT Included (Yet)
The current implementation is **motor-only** and deliberately excludes:
- I2C servo control (pan/tilt camera mount)
- Sensors (ultrasonic, line following, etc.)
- Camera integration
- Adeept's default software suite

This simplified approach focuses on validating core differential drive functionality before adding complexity.

## Relationship to Main WayfindR Project

The Adeept platform serves as a **development and testing baseline** for the WayfindR project:

### Role in Project
1. **Hardware Validation**: Test differential drive algorithms on known-good hardware
2. **Motor Control Testing**: Validate GPIO-based motor control before ROS2 integration
3. **Alternative Platform**: Comparison point vs. XiaoR tank (in `/ros_tank_xiaor/`)
4. **Educational Platform**: Simpler than custom builds, good for algorithm testing

### Integration Points with WayfindR
- **Motor Control**: Patterns here inform `/PI_API/services/motor_driver.py` implementation
- **Hardware Abstraction**: GPIO pin mappings feed into ROS2 hardware interface
- **Differential Drive**: Movement primitives (forward, backward, turn) used in navigation stack
- **LiDAR Mount**: Platform can accommodate RP LIDAR C1M1 for SLAM testing

### Differences from Main Project Hardware
| Aspect | Adeept Platform | WayfindR Target |
|--------|-----------------|-----------------|
| Chassis | Commercial kit | Custom/modular |
| Motor Drivers | Integrated Robot HAT | L298N or custom |
| LiDAR | Not included | RP LIDAR C1M1 (primary) |
| Purpose | Testing platform | Production robot |
| ROS2 Integration | Not yet implemented | Full Nav2 stack |

## Key Files

### 1. `motors_adeept_picar_pro_test.py` (304 lines)

**Purpose**: Standalone motor control testing tool for Adeept Robot HAT

**Capabilities**:
- Individual motor testing (Motor A and B separately)
- Basic movement primitives (forward, backward, turn left/right, stop)
- Pre-programmed patterns (square driving pattern)
- Interactive WASD keyboard control
- Menu-driven demo system

**Key Functions**:
- `setup()` - Initialize GPIO pins and PWM controllers
- `motor_forward/backward/turn_left/turn_right(speed)` - Movement primitives
- `motor_stop()` - Emergency stop
- `demo_motor_test()` - Individual motor validation
- `demo_basic_movements()` - Test all movement directions
- `demo_square_pattern()` - Autonomous square driving
- `interactive_control()` - Manual WASD control

**Usage**:
```bash
# With virtual environment (recommended)
python3 -m venv --system-site-packages venv
source venv/bin/activate
python motors_adeept_picar_pro_test.py

# Or direct execution (requires sudo for GPIO)
sudo python3 motors_adeept_picar_pro_test.py
```

**Safety Features**:
- Proper GPIO cleanup on exit
- Keyboard interrupt handling (Ctrl+C)
- Motors stop before GPIO cleanup

### 2. `511343Tutorial.pdf` (4.8 MB)

**Purpose**: Official Adeept PiCar-Pro assembly and programming tutorial

**Contents** (likely):
- Hardware assembly instructions
- Wiring diagrams for Robot HAT
- Adeept's original Python codebase documentation
- I2C servo control examples
- Full feature demonstrations

**Note**: This is reference material for understanding the complete platform capabilities beyond just motor control.

### 3. `Introduction Robot HAT.pdf` (1.3 MB)

**Purpose**: Technical documentation for Adeept Robot HAT expansion board

**Contents** (likely):
- Pinout diagrams
- Electrical specifications
- Motor driver capabilities
- Power requirements
- I2C interface documentation for servos
- GPIO mapping reference

**Value**: Critical reference for understanding the hardware interface used by the Python script.

## Current Implementation State

### Fully Implemented (100%)
- GPIO-based motor control with PWM
- All basic movement primitives
- Interactive control interface
- Demo patterns
- Safety and cleanup

### Not Yet Implemented (0%)
- ROS2 integration (no launch files, nodes, or topics)
- Hardware interface layer for Nav2
- Odometry estimation (no encoders integrated)
- LiDAR mounting and integration
- Sensor fusion with IMU
- Navigation stack integration
- API endpoint integration (FastAPI)
- Waypoint navigation

### Out of Scope (Deliberately Excluded)
- Servo control (pan/tilt camera)
- Adeept's proprietary software suite
- Web interface (using WayfindR's PI_API instead)
- Line following sensors
- Ultrasonic sensors

## Dependencies

### Hardware Requirements
- Adeept PiCar-Pro chassis kit
- Adeept Robot HAT expansion board
- Raspberry Pi (3B+, 4, or 5)
- Two 18650 batteries (7.4V nominal)
- DC motors (included with Adeept kit)

### Software Requirements
- **OS**: Raspbian/Raspberry Pi OS or Ubuntu 22.04
- **Python**: 3.7+
- **Libraries**:
  - `RPi.GPIO` (motor control)
- **Permissions**: GPIO access (run with sudo or add user to gpio group)

### Optional (for future integration)
- ROS2 Humble
- Nav2 packages
- RP LIDAR C1M1 drivers
- WayfindR PI_API server

## Integration Workflow

### Current State: Standalone Testing
```
[Python Script] → [RPi.GPIO] → [Robot HAT GPIO] → [Motors]
```

### Future State: ROS2 Integration
```
[Nav2 Controller] → [WayfindR Hardware Interface] → [Motor Service] → [Robot HAT] → [Motors]
                ↓
         [RP LIDAR C1M1] → [rplidar_ros] → [SLAM/Localization]
```

## Next Steps for Full Integration

1. **Create ROS2 Hardware Interface**
   - Implement `hardware_interface::SystemInterface` for Robot HAT
   - Map velocity commands to motor PWM
   - Publish joint states

2. **Add Odometry**
   - Integrate wheel encoders (if available)
   - Implement dead reckoning
   - Publish to `/odom` topic

3. **Mount and Configure LiDAR**
   - Physical mounting on chassis
   - Launch file for `rplidar_ros` node
   - TF transforms for sensor position

4. **Create URDF Model**
   - Robot dimensions and geometry
   - Wheel parameters for differential drive
   - LiDAR sensor frame

5. **Integrate with PI_API**
   - Expose motor control via FastAPI endpoints
   - Connect to WayfindR command system

6. **Nav2 Configuration**
   - Controller parameters for differential drive
   - Costmap configuration
   - Recovery behaviors

## Comparison with Other Platforms

### vs. XiaoR Tank (`/ros_tank_xiaor/`)
- **Adeept**: Simpler Robot HAT, easier GPIO control, better for beginners
- **XiaoR**: More advanced, includes encoders and IMU, better for production

### vs. Custom Build (target for WayfindR)
- **Adeept**: Commercial kit, faster to test, good for algorithm development
- **Custom**: More flexible, optimized for specific use case, production-ready

### vs. ESP32 Platform (`/esp32_api/`)
- **Adeept**: Raspberry Pi-based, full Linux, direct ROS2 integration
- **ESP32**: Real-time control, lower power, complementary (not replacement)

## Recommendations

### Keep This Platform If:
- You need a quick testing platform for algorithms
- You want to validate differential drive control patterns
- You need a baseline for comparison
- Educational/demonstration purposes

### Consider Alternatives If:
- You need encoders for accurate odometry (→ XiaoR tank)
- You need production-level reliability (→ custom build)
- You need real-time motor control (→ ESP32)
- Budget is very constrained (→ custom build may be cheaper)

## Technical Notes

### Motor Control Characteristics
- **Default Speed**: 70% duty cycle (conservative, safe for testing)
- **Turn Behavior**: Skid-steer (one motor forward, one reverse)
- **Response Time**: ~100ms for direction changes
- **Power Draw**: Depends on load, typically 1-2A per motor under load

### Known Limitations
- No closed-loop speed control (open-loop PWM only)
- No odometry feedback (unless encoders added separately)
- GPIO-based control has higher latency than dedicated motor controller
- Battery voltage affects speed (no voltage compensation)

### Safety Considerations
- Always test on a raised surface first (wheels off ground)
- Keep emergency stop accessible (Ctrl+C or 'x' command)
- Ensure proper battery connections (reverse polarity protection recommended)
- Monitor battery voltage (LiPo/Li-ion cells should not drop below 3.0V per cell)

## Conclusion

The Adeept platform serves as a valuable testing and development tool for the WayfindR project. While not the final production hardware, it provides a quick way to validate motor control algorithms and differential drive navigation strategies before committing to a custom build or more expensive commercial platform.

**Current Status**: Motor control validated, ready for ROS2 integration experiments.
**Next Priority**: Create ROS2 hardware interface layer for Nav2 compatibility.
