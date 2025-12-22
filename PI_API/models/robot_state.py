"""
Robot State Models

Defines the data structures for robot state, position, and telemetry.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List
from enum import Enum
import time


class RobotMode(str, Enum):
    """Robot operating modes."""
    MANUAL = "manual"           # Direct keyboard/joystick control
    AUTONOMOUS = "autonomous"   # Following waypoints/path
    PATTERN = "pattern"         # Executing a movement pattern
    IDLE = "idle"               # Stopped, waiting for commands
    EMERGENCY = "emergency"     # Emergency stop activated


class MovementState(str, Enum):
    """Current movement state."""
    STOPPED = "stopped"
    FORWARD = "forward"
    BACKWARD = "backward"
    TURNING_LEFT = "turning_left"
    TURNING_RIGHT = "turning_right"
    ROTATING_LEFT = "rotating_left"
    ROTATING_RIGHT = "rotating_right"
    CURVE_FORWARD_LEFT = "curve_forward_left"
    CURVE_FORWARD_RIGHT = "curve_forward_right"
    CURVE_BACKWARD_LEFT = "curve_backward_left"
    CURVE_BACKWARD_RIGHT = "curve_backward_right"


@dataclass
class Position:
    """Robot position in 2D space."""
    x: float = 0.0          # meters
    y: float = 0.0          # meters
    theta: float = 0.0      # radians (heading)

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "theta": self.theta}


@dataclass
class Velocity:
    """Robot velocity."""
    linear: float = 0.0     # m/s
    angular: float = 0.0    # rad/s

    def to_dict(self) -> Dict[str, float]:
        return {"linear": self.linear, "angular": self.angular}


@dataclass
class MotorState:
    """Individual motor state."""
    speed: int = 0          # -255 to 255
    direction: str = "stopped"  # "forward", "backward", "stopped"
    pwm: int = 0            # 0-255

    def to_dict(self) -> Dict[str, Any]:
        return {
            "speed": self.speed,
            "direction": self.direction,
            "pwm": self.pwm
        }


@dataclass
class DriveState:
    """Skid-steer drive state."""
    left_front: MotorState = field(default_factory=MotorState)
    left_rear: MotorState = field(default_factory=MotorState)
    right_front: MotorState = field(default_factory=MotorState)
    right_rear: MotorState = field(default_factory=MotorState)

    # Normalized control inputs (-1.0 to 1.0)
    throttle: float = 0.0   # Forward/backward
    steering: float = 0.0   # Left/right

    def to_dict(self) -> Dict[str, Any]:
        return {
            "left_front": self.left_front.to_dict(),
            "left_rear": self.left_rear.to_dict(),
            "right_front": self.right_front.to_dict(),
            "right_rear": self.right_rear.to_dict(),
            "throttle": self.throttle,
            "steering": self.steering
        }


@dataclass
class NavigationState:
    """Navigation/waypoint state."""
    current_waypoint: Optional[str] = None
    target_waypoint: Optional[str] = None
    waypoints_remaining: int = 0
    distance_to_target: float = 0.0
    eta_seconds: float = 0.0
    route_name: Optional[str] = None
    route_progress: float = 0.0  # 0-100%

    def to_dict(self) -> Dict[str, Any]:
        return {
            "current_waypoint": self.current_waypoint,
            "target_waypoint": self.target_waypoint,
            "waypoints_remaining": self.waypoints_remaining,
            "distance_to_target": self.distance_to_target,
            "eta_seconds": self.eta_seconds,
            "route_name": self.route_name,
            "route_progress": self.route_progress
        }


@dataclass
class SensorData:
    """Sensor readings."""
    battery_voltage: float = 0.0
    battery_percent: int = 100
    temperature: float = 0.0
    imu_connected: bool = False
    lidar_connected: bool = False
    gps_connected: bool = False

    # IMU data (if available)
    imu_roll: float = 0.0
    imu_pitch: float = 0.0
    imu_yaw: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "battery_voltage": self.battery_voltage,
            "battery_percent": self.battery_percent,
            "temperature": self.temperature,
            "imu_connected": self.imu_connected,
            "lidar_connected": self.lidar_connected,
            "gps_connected": self.gps_connected,
            "imu": {
                "roll": self.imu_roll,
                "pitch": self.imu_pitch,
                "yaw": self.imu_yaw
            }
        }


@dataclass
class RobotState:
    """Complete robot state."""
    # Status
    mode: RobotMode = RobotMode.IDLE
    movement_state: MovementState = MovementState.STOPPED
    is_connected: bool = True
    error_message: Optional[str] = None

    # Position and velocity
    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)

    # Drive system
    drive: DriveState = field(default_factory=DriveState)

    # Navigation
    navigation: NavigationState = field(default_factory=NavigationState)

    # Sensors
    sensors: SensorData = field(default_factory=SensorData)

    # Timing
    last_command_time: float = 0.0
    start_time: float = field(default_factory=time.time)

    # Command history (last N commands)
    command_history: List[str] = field(default_factory=list)

    @property
    def uptime(self) -> float:
        """Seconds since robot started."""
        return time.time() - self.start_time

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "mode": self.mode.value,
            "movement_state": self.movement_state.value,
            "is_connected": self.is_connected,
            "error_message": self.error_message,
            "position": self.position.to_dict(),
            "velocity": self.velocity.to_dict(),
            "drive": self.drive.to_dict(),
            "navigation": self.navigation.to_dict(),
            "sensors": self.sensors.to_dict(),
            "uptime": self.uptime,
            "last_command_time": self.last_command_time,
            "command_history": self.command_history[-10:]  # Last 10 commands
        }

    def add_command(self, command: str):
        """Add command to history."""
        self.command_history.append(f"{time.time():.2f}: {command}")
        if len(self.command_history) > 100:
            self.command_history = self.command_history[-100:]
        self.last_command_time = time.time()
