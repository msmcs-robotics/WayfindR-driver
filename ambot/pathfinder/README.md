# Pathfinder - LiDAR Wandering & Obstacle Avoidance

## Overview

Pathfinder is the LiDAR-based wandering component for the Ambot platform. It provides:
- Real-time obstacle detection using RPLidar C1M1
- **Simple wandering behaviors** for demo robots (NO SLAM required)
- Reactive navigation algorithms that make the robot explore a room

> **Note**: This is NOT a SLAM/mapping system. Pathfinder uses simple reactive algorithms to make a demo robot wander around avoiding obstacles.

This code is ported from various WayfindR-driver attempts, consolidating working patterns from:
- `ros_tank_xiaor/lidar_youyeetoo_visual_plotter.py`
- `old_stuff/rplidar_setup/` scripts
- `ros2_comprehensive_attempt/config/lidar_params.yaml`
- `new_bakery/scripts/04_install_lidar.sh`

## Hardware

### Primary Supported LiDAR: Slamtec RPLidar C1M1

| Parameter | Value |
|-----------|-------|
| Model | RPLidar C1 (C1M1RP) |
| Interface | USB (CP210x UART Bridge) |
| Baud Rate | 460800 |
| Range | 0.1m - 12m (indoor), up to 40m (Sensitivity mode) |
| Scan Rate | 10 Hz |
| Angular Resolution | 0.225 degrees |
| Power | 5V DC via USB |

### Scan Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| Standard | Balanced range/speed | General use |
| DenseBoost | High sample rate | Indoor mapping (recommended) |
| Sensitivity | Maximum range (40m) | Long-range detection |

## Deployment Target

Pathfinder runs on the **Raspberry Pi** (separate from the Jetson "bootylicious" system). The Pi handles LiDAR data acquisition and obstacle detection, communicating results to other system components.

## Quick Start

```bash
# Initial setup (run once)
./deploy.sh setup

# Start the service
./deploy.sh start

# Check status
./deploy.sh status

# Test LiDAR
python3 test_lidar.py --check
python3 test_lidar.py --scan
```

## Directory Structure

```
pathfinder/
├── README.md           # This file
├── deploy.sh           # Deployment and management script
├── config.py           # Configuration settings
├── lidar.py            # Core LiDAR communication module
├── obstacle_detector.py # Obstacle detection algorithms
├── behaviors.py        # Wandering behavior algorithms
├── test_lidar.py       # Testing and diagnostics
├── requirements.txt    # Python dependencies
├── __init__.py         # Package exports
└── scripts/
    └── setup-udev.sh   # udev rules setup for /dev/rplidar
```

## Configuration

Edit `config.py` to adjust settings:

```python
SERIAL_PORT = "/dev/rplidar"  # Primary device path
BAUD_RATE = 460800            # C1M1 baud rate
SCAN_MODE = "DenseBoost"      # Recommended for indoor use
```

## Safety Zones

The obstacle detector uses configurable safety zones:

- **STOP_DISTANCE** (0.3m): Immediate halt required
- **SLOW_DISTANCE** (0.8m): Reduce speed zone
- **WARN_DISTANCE** (1.5m): Awareness zone

## Wandering Behaviors

Pathfinder includes several reactive navigation algorithms in `behaviors.py`:

### MaxClearanceBehavior (Recommended)
Moves toward the direction with the most open space.

```python
from ambot.pathfinder import MaxClearanceBehavior, BehaviorRunner

behavior = MaxClearanceBehavior(forward_speed=0.3, turn_speed=0.5)
runner = BehaviorRunner(lidar, detector, behavior, robot)
runner.run()  # Robot wanders toward open space
```

### WallFollowerBehavior
Follows walls at a set distance (left-hand or right-hand rule).

```python
from ambot.pathfinder import WallFollowerBehavior, WallSide

behavior = WallFollowerBehavior(wall_side=WallSide.LEFT, target_distance=0.5)
```

### RandomWanderBehavior
Random exploration with obstacle avoidance - moves forward, turns randomly when blocked.

```python
from ambot.pathfinder import RandomWanderBehavior

behavior = RandomWanderBehavior(turn_probability=0.05)
```

### AvoidAndGoBehavior
Simple priority-based: stop if too close, otherwise go forward.

```python
from ambot.pathfinder import AvoidAndGoBehavior

behavior = AvoidAndGoBehavior(stop_threshold=0.3, slow_threshold=0.8)
```

### Dynamic Obstacle Detection (People Walking By)

For environments with people, use `SafetyWrapper` or `create_safe_wanderer()` to automatically detect and react to sudden obstacles:

```python
from ambot.pathfinder import create_safe_wanderer, BehaviorRunner

# Recommended: Safe wanderer that reacts to people walking close
behavior = create_safe_wanderer(
    forward_speed=0.4,
    emergency_distance=0.25,  # Emergency stop if something gets this close
    approach_rate_threshold=0.5  # React if something approaches at 0.5 m/s
)

runner = BehaviorRunner(lidar, detector, behavior, robot)
runner.run()  # Will emergency stop if someone walks near the robot
```

Or wrap any behavior manually:

```python
from ambot.pathfinder import MaxClearanceBehavior, SafetyWrapper, DynamicObstacleMonitor

# Create custom monitor
monitor = DynamicObstacleMonitor(
    emergency_distance=0.3,
    approach_rate_threshold=0.5,
    time_to_collision_threshold=1.5
)

# Wrap any behavior with safety
inner = MaxClearanceBehavior()
safe_behavior = SafetyWrapper(inner, monitor)
```

### Integration with Locomotion

```python
from ambot.pathfinder import RPLidar, SectorBasedDetector, create_safe_wanderer
from ambot.locomotion.yahboomg1 import create_robot

# Create components
lidar = RPLidar()
detector = SectorBasedDetector()
behavior = create_safe_wanderer()  # Safe for environments with people
robot = create_robot()

# Run wandering loop
lidar.connect()
robot.enable()

while True:
    scan = lidar.get_scan()
    detection = detector.detect(scan)
    command = behavior.step(detection)
    robot.drive(command.left_speed * 100, command.right_speed * 100)
```

## Troubleshooting

### LiDAR not detected

```bash
./deploy.sh diagnose
```

### Permission denied

```bash
sudo ./scripts/setup-udev.sh
# Then logout and login again
```

### No data received

1. Check baud rate matches (C1M1 uses 460800)
2. Verify motor is spinning
3. Clean the LiDAR lens

## License

Part of the WayfindR-driver project.
