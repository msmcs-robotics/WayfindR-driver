# LiDAR Navigation System

## LD19 LiDAR Sensor

The AMBOT uses an LD19 LiDAR sensor (YOUYEETOO/LDRobot) for obstacle detection and navigation.

### Specifications
- Baud rate: 230400
- Protocol: One-way auto-stream (no commands needed)
- Scan rate: ~467 points per scan at 360 degrees
- Connection: USB serial (/dev/ttyUSB0)
- Range: Approximately 12 meters

### Key Difference from RPLidar
The LD19 is NOT an RPLidar C1M1 as originally planned. It uses a different protocol (one-way auto-stream vs bidirectional command protocol). The driver is at pathfinder/lidar_ld19.py.

## Safety Zones

The pathfinder uses three concentric safety zones for obstacle avoidance:

| Zone | Distance | Action |
|------|----------|--------|
| STOP | < 200mm | Emergency stop, reverse |
| SLOW | 200-500mm | Reduce speed to 30% |
| WARN | 500-1000mm | Reduce speed to 60% |
| CLEAR | > 1000mm | Full speed allowed |

## Wandering Algorithms

### NaturalWanderBehavior (Default)
The primary wandering algorithm creates natural-looking exploration:
1. Bins raw LiDAR scan into 36 angular buckets (10 degrees each)
2. Identifies top 10 clearance peaks with minimum 30 degree angular separation
3. Cycles through targets sequentially rather than always chasing the max
4. Time-based target switching (5 seconds per target)
5. Safety zone override always takes priority

### Other Available Behaviors
- MaxClearance: Always heads toward longest distance
- WallFollower: Follows walls at a set distance (left or right)
- RandomWander: Random direction changes at intervals
- AvoidAndGo: Basic obstacle avoidance with forward bias

## Front Calibration

The LiDAR sensor may be mounted at an angle. The gui_lidar_nav.py tool supports front calibration:
1. Place an object directly in front of the robot
2. Press 'c' to start calibration
3. The tool finds the nearest cluster and sets it as the front direction
4. Calibration is saved to tests/results/lidar_calibration.json

## MPU6050 IMU Integration

An optional MPU6050 IMU provides gyroscope heading data for closed-loop turns. The IMU driver is at pathfinder/imu.py and supports:
- Gyro heading integration (Z-axis)
- Startup calibration (bias averaging)
- Complementary filter for pitch/roll
- Graceful degradation (returns None if sensor not found)

Wiring: VCC to Pin 1 (3.3V), GND to Pin 9, SCL to Pin 5 (GPIO3), SDA to Pin 3 (GPIO2).
