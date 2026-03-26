"""
AMBOT Ability Server — FastMCP Server

Exposes robot control tools that an LLM can call:
  - Motor control (forward, backward, turn, stop)
  - Sensor queries (future: LiDAR, camera, IMU)
  - System status
"""

import logging
from fastmcp import FastMCP
from .motor_interface import MotorInterface

logger = logging.getLogger(__name__)

mcp = FastMCP("AMBOT Ability Server")

# Singleton motor interface — set by run.py before server starts
_motors: MotorInterface | None = None


def init_motors(simulate: bool = True, auto_stop_timeout: float = 30.0):
    """Initialize the global motor interface."""
    global _motors
    _motors = MotorInterface(simulate=simulate, auto_stop_timeout=auto_stop_timeout)
    mode = "SIMULATE" if simulate else "HARDWARE"
    logger.info("Motor interface ready (%s)", mode)


def _get_motors() -> MotorInterface:
    """Get the motor interface, auto-init in simulate mode if needed."""
    global _motors
    if _motors is None:
        init_motors(simulate=True)
    return _motors


# ------------------------------------------------------------------
# MCP Tools
# ------------------------------------------------------------------

@mcp.tool()
async def move_forward(speed: int = 50, duration: float = 0) -> dict:
    """Move the robot forward. Speed 0-100. Duration in seconds (0 = continuous until stop)."""
    speed = max(0, min(100, speed))
    return _get_motors().drive(speed, speed, duration=duration, label="move_forward")


@mcp.tool()
async def move_backward(speed: int = 50, duration: float = 0) -> dict:
    """Move the robot backward. Speed 0-100. Duration in seconds (0 = continuous until stop)."""
    speed = max(0, min(100, speed))
    return _get_motors().drive(-speed, -speed, duration=duration, label="move_backward")


@mcp.tool()
async def turn_left(speed: int = 50, duration: float = 0) -> dict:
    """Spin the robot left (counterclockwise). Speed 0-100. Duration in seconds (0 = continuous until stop)."""
    speed = max(0, min(100, speed))
    return _get_motors().drive(-speed, speed, duration=duration, label="turn_left")


@mcp.tool()
async def turn_right(speed: int = 50, duration: float = 0) -> dict:
    """Spin the robot right (clockwise). Speed 0-100. Duration in seconds (0 = continuous until stop)."""
    speed = max(0, min(100, speed))
    return _get_motors().drive(speed, -speed, duration=duration, label="turn_right")


@mcp.tool()
async def stop_motors() -> dict:
    """Stop all motors immediately."""
    return _get_motors().stop(reason="tool_call")


@mcp.tool()
async def get_motor_status() -> dict:
    """Get current motor state including speeds, direction, and uptime."""
    return _get_motors().get_status()
