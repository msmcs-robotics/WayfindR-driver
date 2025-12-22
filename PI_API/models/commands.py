"""
Command Models

Pydantic models for API request/response validation.
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from enum import Enum


class MovementCommand(BaseModel):
    """Direct movement control command."""
    throttle: float = Field(
        default=0.0,
        ge=-1.0,
        le=1.0,
        description="Forward/backward throttle (-1.0 to 1.0)"
    )
    steering: float = Field(
        default=0.0,
        ge=-1.0,
        le=1.0,
        description="Left/right steering (-1.0 to 1.0)"
    )
    duration: Optional[float] = Field(
        default=None,
        ge=0,
        le=60,
        description="Duration in seconds (None = until next command)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "throttle": 0.5,
                "steering": 0.0,
                "duration": 2.0
            }
        }


class RotateCommand(BaseModel):
    """Rotate in place command."""
    direction: str = Field(
        default="left",
        description="Rotation direction: 'left' or 'right'"
    )
    speed: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Rotation speed (0.0 to 1.0)"
    )
    angle: Optional[float] = Field(
        default=None,
        description="Target angle in degrees (None = continuous)"
    )


class StopCommand(BaseModel):
    """Stop command with optional mode."""
    mode: str = Field(
        default="normal",
        description="Stop mode: 'normal', 'emergency', or 'brake'"
    )


class NavigationCommand(BaseModel):
    """Navigation to a position."""
    x: float = Field(..., description="Target X coordinate (meters)")
    y: float = Field(..., description="Target Y coordinate (meters)")
    theta: Optional[float] = Field(
        default=None,
        description="Target heading (radians)"
    )
    speed: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Navigation speed (0.0 to 1.0)"
    )


class WaypointCommand(BaseModel):
    """Navigate to a named waypoint."""
    waypoint_name: str = Field(..., description="Name of target waypoint")
    speed: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Navigation speed"
    )


class RouteCommand(BaseModel):
    """Execute a named route."""
    route_name: str = Field(..., description="Name of route to execute")
    loop: bool = Field(
        default=False,
        description="Loop the route continuously"
    )
    speed: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Navigation speed"
    )


class PatternType(str, Enum):
    """Available movement patterns."""
    CIRCLE = "circle"
    SQUARE = "square"
    FIGURE_EIGHT = "figure_eight"
    ZIGZAG = "zigzag"
    SPIRAL = "spiral"
    LINE = "line"
    SCAN = "scan"


class NavigationGoal(BaseModel):
    """Navigation goal coordinates."""
    x: float = Field(..., description="Target X coordinate (meters)")
    y: float = Field(..., description="Target Y coordinate (meters)")
    yaw: Optional[float] = Field(
        default=None,
        description="Target heading in degrees"
    )


class PatternCommand(BaseModel):
    """Execute a movement pattern."""
    pattern: PatternType = Field(..., description="Pattern type to execute")
    size: float = Field(
        default=1.0,
        ge=0.1,
        le=10.0,
        description="Pattern size in meters"
    )
    speed: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Movement speed"
    )
    repetitions: int = Field(
        default=1,
        ge=1,
        le=100,
        description="Number of times to repeat pattern"
    )
    clockwise: bool = Field(
        default=True,
        description="Direction for circular patterns"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "pattern": "circle",
                "size": 2.0,
                "speed": 0.3,
                "repetitions": 1,
                "clockwise": True
            }
        }


class KeyboardInput(BaseModel):
    """Keyboard input from dashboard."""
    keys: List[str] = Field(
        default_factory=list,
        description="List of currently pressed keys"
    )


class CommandResponse(BaseModel):
    """Standard API response."""
    success: bool
    message: str
    data: Optional[dict] = None


class TelemetryResponse(BaseModel):
    """Telemetry data response."""
    timestamp: float
    position: dict
    velocity: dict
    drive: dict
    sensors: dict
    mode: str
    movement_state: str
