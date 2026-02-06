"""
Pathfinder - LiDAR Obstacle Avoidance System

This package provides LiDAR-based obstacle detection for the Ambot platform
using the RPLidar C1M1 sensor.

Components:
    - config: Configuration settings
    - lidar: Core LiDAR communication (RPLidar class)
    - obstacle_detector: Sector-based obstacle detection
    - behaviors: Navigation and wandering behaviors

Usage:
    from pathfinder.lidar import RPLidar
    from pathfinder.obstacle_detector import SectorBasedDetector
    from pathfinder.behaviors import MaxClearanceBehavior, BehaviorRunner

    with RPLidar() as lidar:
        detector = SectorBasedDetector()

        # Simple obstacle detection
        for scan in lidar.iter_scans():
            result = detector.process_scan(scan)
            if result.overall_safety.value == "stop":
                print("OBSTACLE DETECTED!")

    # Or use behaviors for autonomous navigation
    behavior = MaxClearanceBehavior()
    runner = BehaviorRunner(lidar, detector, behavior, robot)
    runner.run()
"""

from . import config
from .lidar import RPLidar, RPLidarException, ScanPoint
from .obstacle_detector import (
    SectorBasedDetector,
    ContinuousDetector,
    SafetyLevel,
    SectorReading,
    DetectionResult,
)
from .behaviors import (
    # Motor command
    MotorCommand,
    # Base class
    Behavior,
    # Behaviors
    MaxClearanceBehavior,
    WallFollowerBehavior,
    WallSide,
    RandomWanderBehavior,
    AvoidAndGoBehavior,
    # Dynamic obstacle detection (for people walking by)
    DynamicObstacle,
    DynamicObstacleMonitor,
    SafetyWrapper,
    create_safe_wanderer,
    # Runner
    BehaviorRunner,
    BehaviorSelector,
)
from .imu import IMU

__all__ = [
    # Configuration
    "config",
    # LiDAR
    "RPLidar",
    "RPLidarException",
    "ScanPoint",
    # Obstacle Detection
    "SectorBasedDetector",
    "ContinuousDetector",
    "SafetyLevel",
    "SectorReading",
    "DetectionResult",
    # Behaviors
    "MotorCommand",
    "Behavior",
    "MaxClearanceBehavior",
    "WallFollowerBehavior",
    "WallSide",
    "RandomWanderBehavior",
    "AvoidAndGoBehavior",
    # Dynamic obstacle detection
    "DynamicObstacle",
    "DynamicObstacleMonitor",
    "SafetyWrapper",
    "create_safe_wanderer",
    # Runner
    "BehaviorRunner",
    "BehaviorSelector",
    # IMU
    "IMU",
]

__version__ = "0.1.0"
