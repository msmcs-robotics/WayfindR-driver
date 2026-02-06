"""
Demos Common - Shared components for wandering demos.

Provides reusable sensor wrappers and utilities so demo files
stay small and focused on high-level logic.

Components:
    - RobotAdapter: Float-to-int motor speed bridge
    - create_lidar: Auto-detect and create LiDAR instance
    - create_behavior: Create wandering behavior by name
    - CameraFaceThread / FaceData: Background face detection
    - setup_imu: Connect IMU with graceful degradation
"""

from .robot import RobotAdapter
from .sensors import (
    create_lidar,
    CameraFaceThread,
    FaceData,
    setup_imu,
)
from .behaviors import create_behavior

__all__ = [
    "RobotAdapter",
    "create_lidar",
    "CameraFaceThread",
    "FaceData",
    "setup_imu",
    "create_behavior",
]
