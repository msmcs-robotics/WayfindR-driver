# Ambot - Raspberry Pi Subsystem

The "body" of the Ambot platform. Handles LiDAR sensing, obstacle avoidance, and basic algorithmic movement.

## Components

- **LiDAR**: Direct serial reading (no ROS2)
- **Obstacle Avoidance**: Sector-based reactive avoidance algorithm
- **Movement**: GPIO motor control (L298N driver)

## System Access

```
Host: TBD
User: TBD
SSH:  TBD
```

## Design Decisions

- **No ROS2**: Too resource-heavy and steep learning curve for this use case
- **Python first**: Rapid prototyping, C port planned for future optimization
- **No ML**: Pure algorithmic approach for movement and avoidance

## Directory Structure

```
raspberry_pi/
├── README.md       # This file
├── lidar/          # LiDAR reading and obstacle detection (future)
└── movement/       # Motor control and movement patterns (future)
```

## References

- LiDAR code: `~/WayfindR-driver/ros_tank_xiaor/lidar_youyeetoo_visual_plotter.py`
- Motor control: `~/WayfindR-driver/PI_API/services/motor_driver.py`
- Robot controller: `~/WayfindR-driver/PI_API/services/robot_controller.py`
