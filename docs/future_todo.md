# WayfindR Robot Platform - Future Development Roadmap

## Project Status Summary

The WayfindR-driver project is a **well-structured but partially integrated robotics platform**. Individual components are mature:
- **ESP32 API**: Dual-core motor control with REST/WebSocket - WORKING
- **PI_API**: FastAPI web server with dashboard - WORKING
- **ROS2 SLAM**: Cartography with SLAM Toolbox - WORKING
- **ROS2 Localization**: AMCL particle filter - WORKING
- **Waypoint Navigation**: A* pathfinding - IMPLEMENTED

**The main gap**: These components exist as separate subsystems, not a unified robot.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         TARGET ARCHITECTURE                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────┐     ┌──────────────────────────────────────────────────┐  │
│   │   LLM API   │────▶│              RASPBERRY PI                        │  │
│   │  (Remote)   │     │  ┌─────────────────────────────────────────────┐ │  │
│   └─────────────┘     │  │            PI_API (FastAPI)                 │ │  │
│                       │  │  - Command parsing                          │ │  │
│                       │  │  - Navigation planning                      │ │  │
│                       │  │  - State management                         │ │  │
│                       │  └──────────────┬──────────────────────────────┘ │  │
│                       │                 │                                │  │
│                       │  ┌──────────────┴──────────────────────────────┐ │  │
│                       │  │            ROS2 Humble                      │ │  │
│                       │  │  ┌─────────┐ ┌─────────┐ ┌───────────────┐  │ │  │
│                       │  │  │  SLAM   │ │  AMCL   │ │    Nav2       │  │ │  │
│                       │  │  │ Toolbox │ │  Locl.  │ │  Planner      │  │ │  │
│                       │  │  └────┬────┘ └────┬────┘ └───────┬───────┘  │ │  │
│                       │  │       │           │              │          │ │  │
│                       │  │  ┌────┴───────────┴──────────────┴───────┐  │ │  │
│                       │  │  │           /cmd_vel topic              │  │ │  │
│                       │  │  └───────────────────┬───────────────────┘  │ │  │
│                       │  └──────────────────────┼──────────────────────┘ │  │
│                       │                         │                        │  │
│                       │  ┌──────────────────────┴──────────────────────┐ │  │
│                       │  │        Serial/WiFi Bridge                   │ │  │
│                       │  │  (converts cmd_vel to motor commands)       │ │  │
│                       │  └──────────────────────┬──────────────────────┘ │  │
│                       └─────────────────────────┼────────────────────────┘  │
│                                                 │                           │
│   ┌─────────────────────────────────────────────┴─────────────────────────┐ │
│   │                           ESP32                                       │ │
│   │  ┌─────────────────────┐    ┌──────────────────────────────────────┐  │ │
│   │  │      Core 0         │    │            Core 1                    │  │ │
│   │  │  Control Loop       │    │         Web Server                   │  │ │
│   │  │  - PID control      │◄───│  - REST API                          │  │ │
│   │  │  - Motor mixing     │    │  - WebSocket                         │  │ │
│   │  │  - Safety watchdog  │    │  - Telemetry broadcast               │  │ │
│   │  └─────────┬───────────┘    └──────────────────────────────────────┘  │ │
│   │            │                                                          │ │
│   │  ┌─────────┴───────────┐                                              │ │
│   │  │   Output Channels   │                                              │ │
│   │  │  - 4x PWM Motors    │                                              │ │
│   │  │  - H-Bridge Control │                                              │ │
│   │  └─────────┬───────────┘                                              │ │
│   └────────────┼──────────────────────────────────────────────────────────┘ │
│                │                                                            │
│   ┌────────────┴────────────────────────────────────────────────────────┐   │
│   │                        HARDWARE                                      │   │
│   │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌───────────┐  │   │
│   │  │  LiDAR  │  │  IMU    │  │ Encoders│  │ Motors  │  │  Battery  │  │   │
│   │  │ C1M1RP  │  │ MPU6050 │  │ (TODO)  │  │  4x DC  │  │  Monitor  │  │   │
│   │  └─────────┘  └─────────┘  └─────────┘  └─────────┘  └───────────┘  │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Critical Integration (Weeks 1-2)

### 1.1 Pi ↔ ESP32 Communication Bridge
**Status**: NOT STARTED
**Priority**: CRITICAL

Currently PI_API and ESP32 are independent. Need to connect them.

**Tasks**:
- [ ] Define communication protocol (Serial UART or WiFi HTTP)
- [ ] Create `esp32_bridge.py` service in PI_API
- [ ] Implement command translation: `cmd_vel` → motor values
- [ ] Add telemetry feedback: ESP32 → Pi
- [ ] Handle connection loss gracefully

**Recommended Approach**:
```
Option A: Serial (simpler, more reliable)
  Pi GPIO TX/RX ←→ ESP32 Serial2
  Baud: 115200
  Protocol: JSON lines {"throttle": 0.5, "steering": 0.2}

Option B: WiFi (more flexible)
  Pi sends HTTP POST to ESP32 /api/move
  ESP32 broadcasts telemetry via WebSocket
  Requires stable WiFi connection
```

**Files to create**:
- `PI_API/services/esp32_bridge.py`
- `PI_API/config/esp32_config.yaml`

### 1.2 ROS2 ↔ PI_API Integration
**Status**: NOT STARTED
**Priority**: CRITICAL

ROS2 Nav2 outputs `cmd_vel` but PI_API doesn't subscribe to it.

**Tasks**:
- [ ] Create ROS2 node that bridges `cmd_vel` → PI_API
- [ ] Or: Make PI_API a ROS2 node directly
- [ ] Subscribe to `/cmd_vel` (geometry_msgs/Twist)
- [ ] Publish robot state to `/odom` (nav_msgs/Odometry)

**Files to create**:
- `PI_API/ros2_bridge/cmd_vel_subscriber.py`
- `PI_API/ros2_bridge/odom_publisher.py`
- `ros2_comprehensive_attempt/launch/full_robot.launch.py`

### 1.3 Unified Launch System
**Status**: PARTIAL
**Priority**: HIGH

**Tasks**:
- [ ] Create single launch file that starts everything
- [ ] Implement health checks for all components
- [ ] Add systemd services for auto-start on boot
- [ ] Create status dashboard showing all component states

**Files to create**:
- `ros2_comprehensive_attempt/launch/wayfinder_bringup.launch.py`
- `scripts/start_robot.sh`
- `scripts/stop_robot.sh`
- `scripts/status_robot.sh`

---

## Phase 2: Sensor Integration (Weeks 3-4)

### 2.1 MPU6050 IMU Integration
**Status**: CODE EXISTS (in esp32_api/examples), NOT INTEGRATED
**Priority**: HIGH

The IMU is essential for:
- Heading estimation (yaw drift correction)
- Tilt detection (safety)
- Sensor fusion with LiDAR

**Tasks**:
- [ ] Wire MPU6050 to ESP32 (I2C: SDA=21, SCL=22)
- [ ] Enable IMU in ESP32 firmware
- [ ] Publish IMU data to `/imu` topic
- [ ] Configure robot_localization for sensor fusion
- [ ] Calibrate gyro offsets at startup

**Configuration needed**:
```yaml
# robot_localization config
ekf_node:
  ros__parameters:
    odom0: wheel_odom
    imu0: /imu
    imu0_config: [false, false, false,  # x, y, z
                  false, false, true,   # roll, pitch, yaw
                  false, false, false,  # vx, vy, vz
                  false, false, true,   # vroll, vpitch, vyaw
                  false, false, false]  # ax, ay, az
```

**Files to modify**:
- `esp32_api/include/config.h` - Enable IMU
- `esp32_api/src/main.cpp` - Initialize IMU

**Files to create**:
- `ros2_comprehensive_attempt/config/ekf.yaml`

### 2.2 Wheel Encoders (Odometry)
**Status**: NOT IMPLEMENTED
**Priority**: HIGH

Without encoders, the robot has no idea how far it's traveled.

**Options**:
1. **Hall effect encoders** on motor shafts (cheap, moderate resolution)
2. **Optical encoders** (more accurate, more expensive)
3. **Visual odometry** from LiDAR scan matching (no hardware needed)

**Tasks**:
- [ ] Decide on encoder type
- [ ] Wire encoders to ESP32 (interrupt-capable pins)
- [ ] Implement encoder counting in control loop
- [ ] Calculate wheel odometry (distance, velocity)
- [ ] Publish to `/wheel_odom` topic

**Code structure for ESP32**:
```cpp
// Encoder interrupt handler
void IRAM_ATTR encoderISR() {
    encoderCount++;
}

// In control loop
float distance = (encoderCount / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
float velocity = (encoderCount - lastCount) / dt * WHEEL_CIRCUMFERENCE;
```

**Files to create**:
- `esp32_api/include/encoders.h`
- `esp32_api/src/encoders.cpp`

### 2.3 Battery Monitoring
**Status**: NOT IMPLEMENTED
**Priority**: MEDIUM

**Tasks**:
- [ ] Add voltage divider to ESP32 ADC pin
- [ ] Implement battery voltage reading
- [ ] Add low battery warning/auto-dock behavior
- [ ] Display battery level in dashboard

---

## Phase 3: Navigation Enhancement (Weeks 5-6)

### 3.1 Transform Tree (TF2) Setup
**Status**: NOT CONFIGURED
**Priority**: CRITICAL for Nav2

Nav2 requires proper coordinate transforms between frames.

**Required transforms**:
```
map → odom → base_link → [sensors]

map
 └── odom (from localization)
      └── base_footprint
           └── base_link
                ├── laser (LiDAR)
                ├── imu_link
                └── camera_link (future)
```

**Tasks**:
- [ ] Create URDF/Xacro robot description
- [ ] Configure static transforms for sensors
- [ ] Verify transforms in RViz

**Files to create**:
- `ros2_comprehensive_attempt/urdf/wayfinder.urdf.xacro`
- `ros2_comprehensive_attempt/config/robot_description.yaml`

### 3.2 Nav2 Parameter Tuning
**Status**: DEFAULT PARAMS
**Priority**: HIGH

Current Nav2 uses default parameters. Need to tune for this specific robot.

**Parameters to tune**:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Match control loop
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5           # Adjust for robot speed
      max_vel_theta: 1.0
      min_vel_x: 0.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2

costmap:
  robot_radius: 0.15           # Actual robot footprint
  inflation_radius: 0.3
```

**Files to create/modify**:
- `ros2_comprehensive_attempt/config/nav2_params.yaml`

### 3.3 Collision Avoidance
**Status**: NOT IMPLEMENTED
**Priority**: HIGH

**Tasks**:
- [ ] Configure costmap inflation layer
- [ ] Add obstacle detection layer
- [ ] Implement emergency stop on imminent collision
- [ ] Test with obstacles in path

### 3.4 Path Smoothing
**Status**: BASIC A* ONLY
**Priority**: MEDIUM

Current A* produces jagged paths.

**Tasks**:
- [ ] Implement path smoothing (cubic spline or bezier)
- [ ] Add turning radius constraints
- [ ] Optimize for differential drive kinematics

---

## Phase 4: Autonomy Features (Weeks 7-8)

### 4.1 Waypoint Mission System
**Status**: BASIC IMPLEMENTATION
**Priority**: MEDIUM

**Tasks**:
- [ ] Create mission file format (YAML)
- [ ] Implement mission sequencer
- [ ] Add pause/resume/cancel commands
- [ ] Support conditional waypoints (wait for signal, repeat)
- [ ] Add waypoint actions (take photo, announce, etc.)

**Mission file format**:
```yaml
mission:
  name: "Patrol Route A"
  waypoints:
    - name: "entrance"
      x: 1.0
      y: 2.0
      theta: 0.0
      action: "announce arrival"
    - name: "checkpoint_1"
      x: 3.0
      y: 4.0
      action: "wait 5s"
    - name: "return"
      x: 0.0
      y: 0.0
      action: "dock"
```

**Files to create**:
- `PI_API/services/mission_executor.py`
- `PI_API/models/mission.py`
- `missions/patrol_a.yaml`

### 4.2 Auto-Docking
**Status**: NOT IMPLEMENTED
**Priority**: LOW (nice to have)

**Tasks**:
- [ ] Design docking station with IR beacons
- [ ] Implement dock detection
- [ ] Create precision approach behavior
- [ ] Add charging status monitoring

### 4.3 Recovery Behaviors
**Status**: NOT IMPLEMENTED
**Priority**: MEDIUM

What happens when the robot gets stuck?

**Tasks**:
- [ ] Implement "back up and retry" behavior
- [ ] Add "spin in place" recovery
- [ ] Create "return to last known good position"
- [ ] Add human intervention request

---

## Phase 5: LLM Integration (Weeks 9-10)

### 5.1 Command Parser
**Status**: BASIC IN PI_API
**Priority**: HIGH

**Tasks**:
- [ ] Define command vocabulary
- [ ] Create intent classifier
- [ ] Map intents to robot actions
- [ ] Handle ambiguous commands gracefully

**Command examples**:
```
"go to the kitchen" → navigate_to_waypoint("kitchen")
"move forward 2 meters" → move_relative(2.0, 0.0)
"turn around" → rotate(180)
"stop" → emergency_stop()
"where are you?" → get_location()
"what do you see?" → describe_surroundings()
```

**Files to create**:
- `PI_API/services/command_parser.py`
- `PI_API/config/command_vocabulary.yaml`

### 5.2 State Feedback to LLM
**Status**: NOT IMPLEMENTED
**Priority**: HIGH

LLM needs to know robot state to make decisions.

**State to report**:
```json
{
  "position": {"x": 1.5, "y": 2.3, "theta": 45.0},
  "velocity": {"linear": 0.3, "angular": 0.0},
  "battery": 75,
  "status": "navigating",
  "current_goal": "kitchen",
  "distance_to_goal": 3.2,
  "obstacles_detected": false,
  "localization_quality": "good"
}
```

**Files to create**:
- `PI_API/routers/llm_interface.py`
- `PI_API/models/robot_state.py`

### 5.3 Conversation Context
**Status**: NOT IMPLEMENTED
**Priority**: MEDIUM

**Tasks**:
- [ ] Track conversation history
- [ ] Maintain spatial memory ("the chair I mentioned earlier")
- [ ] Support follow-up questions
- [ ] Handle clarification requests

---

## Phase 6: Production Readiness (Weeks 11-12)

### 6.1 Testing Framework
**Status**: NO TESTS
**Priority**: HIGH

**Tasks**:
- [ ] Unit tests for PI_API services
- [ ] Integration tests for Pi ↔ ESP32
- [ ] Simulation environment (Gazebo)
- [ ] Hardware-in-loop testing

**Files to create**:
- `PI_API/tests/test_motor_driver.py`
- `PI_API/tests/test_navigation.py`
- `esp32_api/test/test_mixer.cpp`

### 6.2 Monitoring & Logging
**Status**: BASIC SERIAL OUTPUT
**Priority**: MEDIUM

**Tasks**:
- [ ] Centralized logging (to file and remote)
- [ ] Metrics collection (Prometheus format)
- [ ] Alert system for errors
- [ ] Performance monitoring dashboard

### 6.3 Documentation
**Status**: PARTIAL
**Priority**: MEDIUM

**Missing documentation**:
- [ ] Complete system integration guide
- [ ] Hardware assembly instructions
- [ ] Troubleshooting guide
- [ ] API reference (OpenAPI spec)

### 6.4 Deployment Automation
**Status**: PARTIAL (new_bakery)
**Priority**: MEDIUM

**Tasks**:
- [ ] Docker containers for each component
- [ ] Ansible playbooks for fleet deployment
- [ ] OTA firmware updates for ESP32
- [ ] Configuration management system

---

## Hardware Shopping List

For a complete robot build, you'll need:

| Component | Quantity | Est. Cost | Notes |
|-----------|----------|-----------|-------|
| Raspberry Pi 4/5 | 1 | $55-80 | 4GB+ RAM recommended |
| ESP32 DevKit | 1 | $8-15 | Any ESP32 variant |
| Slamtec C1M1RP LiDAR | 1 | $99 | Already owned |
| MPU6050 IMU | 1 | $3-5 | 6-axis accelerometer/gyro |
| L298N Motor Driver | 2 | $6-10 | Dual H-bridge |
| DC Gear Motors | 4 | $20-40 | With encoder preferred |
| Wheel Encoders | 4 | $15-30 | If motors don't have built-in |
| 18650 Battery Pack | 1 | $20-40 | 3S or 4S Li-ion |
| BMS Board | 1 | $5-10 | Battery management |
| Voltage Regulator | 2 | $5-10 | 5V for Pi, 3.3V for ESP32 |
| Robot Chassis | 1 | $30-60 | 4WD platform |
| Jumper Wires | 1 set | $5-10 | Various lengths |
| **TOTAL** | | **~$270-400** | |

---

## Code Quality Improvements

### Immediate Fixes Needed

1. **ESP32: Flash usage is 88%** on standard ESP32
   - Solution: Use ESP32-S3 (more flash) or optimize
   - Remove unused features, compress web assets

2. **PI_API: No error handling for ESP32 disconnect**
   - Add connection state machine
   - Implement automatic reconnection

3. **ROS2: Missing robot_state_publisher**
   - Create URDF and add to launch files
   - Essential for TF tree

4. **Security: WiFi credentials in source code**
   - Move to environment variables or config file
   - Add `.env` file support

### Code Smell Fixes

1. **Duplicate motor mixing code** in PI_API and ESP32
   - Define mixing in one place (ESP32)
   - PI_API should send high-level commands

2. **Magic numbers** throughout codebase
   - Extract to config files
   - Document units and valid ranges

3. **No input validation** on API endpoints
   - Add Pydantic validators
   - Clamp values to safe ranges

---

## Suggested Development Order

```
Week 1-2: Integration
├── Pi ↔ ESP32 serial bridge
├── cmd_vel subscriber
└── Unified launch file

Week 3-4: Sensors
├── MPU6050 integration
├── TF tree setup
└── Basic odometry (even without encoders)

Week 5-6: Navigation
├── Nav2 parameter tuning
├── Collision avoidance
└── End-to-end navigation test

Week 7-8: Autonomy
├── Waypoint missions
├── Recovery behaviors
└── Path smoothing

Week 9-10: LLM
├── Command parser
├── State feedback
└── Basic conversations

Week 11-12: Polish
├── Testing
├── Documentation
└── Deployment automation
```

---

## Quick Wins (Can Do Today)

1. **Test ESP32 → Motor connection**
   - Flash ESP32, connect to WiFi
   - Open dashboard, test motor response
   - Verify watchdog stops motors on disconnect

2. **Run full SLAM mapping session**
   - Launch SLAM Toolbox
   - Drive robot manually (teleop)
   - Save map
   - Test localization on saved map

3. **Wire IMU and test**
   - Connect MPU6050 to ESP32
   - Run example code
   - Verify gyro readings

4. **Create integration test script**
   ```bash
   # scripts/test_integration.sh
   echo "Testing ESP32 connection..."
   curl http://esp32.local/api/status

   echo "Testing PI_API..."
   curl http://localhost:8000/api/status

   echo "Testing ROS2..."
   ros2 topic list
   ```

---

## Resources & References

### Documentation
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/)
- [FastAPI](https://fastapi.tiangolo.com/)

### Similar Projects
- [Linorobot2](https://github.com/linorobot/linorobot2) - Full ROS2 robot
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/) - Reference design
- [ROSbot](https://husarion.com/manuals/rosbot/) - Commercial example

### Community
- [ROS Discourse](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
- [ESP32 Forum](https://esp32.com/)

---

---

## LiDAR Denoising & Native Code Port (C/Rust)

> **Added 2026-02-19** — Key future direction for the entire WayfindR platform.

### Core Idea

Video game development techniques for particle systems, point-cloud rendering, and spatial clustering translate directly to LiDAR scan processing. C (or Rust) implementations of these techniques can run 10-100x faster than Python for tight inner loops, which matters on memory/CPU-constrained platforms like RPi (906MB, Cortex-A53) and Jetson.

### Why This Matters

- Current Python LiDAR processing works but is slow for real-time SLAM
- Don't need all ~467 raw scan points — can reduce via density filtering and still navigate effectively
- Game engines solve the same fundamental problem: processing thousands of spatial points per frame efficiently
- C is proven on both RPi and Jetson (aarch64) with excellent hardware access (GPIO, I2C, SPI)

### Video Game Techniques → LiDAR

> **RESEARCH NEEDED**: Dedicated web research session required before implementation.

| Game Dev Technique | LiDAR Application |
|---|---|
| **Density-based clustering** (particle LOD) | Group nearby points into objects, discard isolated noise |
| **Spatial hashing** (collision detection) | O(1) neighbor lookup for point clustering |
| **Level-of-detail (LOD)** | Reduce point density at long range, full res close up |
| **Temporal accumulation** (TAA) | Average across multiple scans to reduce noise |
| **Frustum culling** | Only process points in direction of travel |
| **GPU particle systems** | CUDA-accelerated point processing on Jetson |
| **KD-tree / octree** | Efficient nearest-neighbor for ICP scan matching |

### Object Detection via Point Density

- Cluster scan points by proximity → discrete objects
- Filter clusters below minimum point count (noise, dust, wires)
- Classify by shape: wall (long/thin), person (blob), furniture (irregular)
- Replace naive "nearest point in sector" with actual object awareness

### C vs Rust vs Python+C Hybrid

| Approach | Pros | Cons |
|---|---|---|
| **Pure C** | Max speed, full GPIO/I2C access, proven on ARM | Manual memory management, no safety guarantees |
| **Pure Rust** | Memory safety, modern tooling | More complex cross-compilation, less GPIO library support |
| **Python + C extensions** | Best of both — Python orchestration + C hot loops | IPC overhead, two languages to maintain |

**Recommended**: Start with **Python + C extensions** (ctypes/cffi) for scan processing hot loops. Expand to more C if the performance gains justify the complexity.

**Challenge**: If LiDAR processing moves to C but face detection (OpenCV) stays in Python, need clean interprocess communication. If face detection also moves to C++ (OpenCV C++ API), the entire sensor pipeline could be native — but adds significant build complexity and GPIO interop work on both platforms.

### Research Tasks

- [ ] Web research: Video game particle techniques → 2D LiDAR denoising
- [ ] Web research: DBSCAN density clustering on embedded ARM
- [ ] Web research: C vs Rust for real-time robotics on aarch64 (RPi + Jetson)
- [ ] Web research: GPU-accelerated point cloud processing on Jetson Orin Nano
- [ ] Benchmark: Python vs C for ICP scan matching on RPi (currently ~32ms Python)
- [ ] Prototype: C extension for LiDAR filtering/clustering (callable from Python)
- [ ] Evaluate: Full C/Rust port vs hybrid for entire sensor pipeline (LiDAR + camera + GPIO)

> Cross-reference: See `ambot/docs/roadmap.md` (Milestone 5), `ambot-slam/docs/roadmap.md`, and `docs/roadmap.md` for project-specific details.

---

## Summary

**What works today**:
- ESP32 motor control with web dashboard
- PI_API with keyboard control
- SLAM mapping with Slamtec LiDAR
- AMCL localization on saved maps
- A* waypoint pathfinding
- AMBOT: LiDAR wandering, face tracking, IMU driver, RAG stack on Jetson

**Critical gaps**:
- No Pi ↔ ESP32 communication
- No ROS2 ↔ PI_API bridge
- No IMU or encoder feedback
- No unified launch/deployment
- LiDAR denoising and C/Rust port (research needed)

**Recommended first steps**:
1. Wire and test ESP32 → motors
2. Add serial bridge Pi → ESP32
3. Integrate IMU for heading
4. Create unified launch file
5. Run end-to-end navigation test

The foundation is solid. Focus on integration, then iterate.
