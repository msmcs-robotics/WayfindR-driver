"""
Control Router

API endpoints for direct robot control.
Used by LLM and dashboard for movement commands.
"""

from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel, Field
from typing import Optional

from models.commands import (
    MovementCommand,
    RotateCommand,
    StopCommand,
    CommandResponse
)

router = APIRouter()


@router.post("/move", response_model=CommandResponse)
async def move_robot(request: Request, command: MovementCommand):
    """
    Move robot with throttle and steering.

    - **throttle**: -1.0 (backward) to 1.0 (forward)
    - **steering**: -1.0 (left) to 1.0 (right)
    - **duration**: Optional seconds, None = until next command

    Examples:
        - Forward: throttle=0.5, steering=0
        - Turn right while moving: throttle=0.5, steering=0.5
        - Pivot right: throttle=0, steering=0.5
    """
    robot = request.app.state.robot

    if command.duration:
        await robot.move_for_duration(
            command.throttle,
            command.steering,
            command.duration
        )
        return CommandResponse(
            success=True,
            message=f"Moved for {command.duration}s",
            data={"throttle": command.throttle, "steering": command.steering}
        )
    else:
        await robot.set_movement(command.throttle, command.steering)
        return CommandResponse(
            success=True,
            message="Movement set",
            data={"throttle": command.throttle, "steering": command.steering}
        )


@router.post("/forward", response_model=CommandResponse)
async def move_forward(
    request: Request,
    speed: float = 0.5,
    duration: Optional[float] = None
):
    """Move robot forward."""
    robot = request.app.state.robot

    if duration:
        await robot.move_for_duration(speed, 0, duration)
        return CommandResponse(
            success=True,
            message=f"Moved forward for {duration}s"
        )
    else:
        await robot.set_movement(speed, 0)
        return CommandResponse(
            success=True,
            message="Moving forward"
        )


@router.post("/backward", response_model=CommandResponse)
async def move_backward(
    request: Request,
    speed: float = 0.5,
    duration: Optional[float] = None
):
    """Move robot backward."""
    robot = request.app.state.robot

    if duration:
        await robot.move_for_duration(-speed, 0, duration)
        return CommandResponse(
            success=True,
            message=f"Moved backward for {duration}s"
        )
    else:
        await robot.set_movement(-speed, 0)
        return CommandResponse(
            success=True,
            message="Moving backward"
        )


@router.post("/rotate", response_model=CommandResponse)
async def rotate_robot(request: Request, command: RotateCommand):
    """
    Rotate robot in place.

    - **direction**: 'left' or 'right'
    - **speed**: 0.0 to 1.0
    - **angle**: Optional degrees, None = continuous

    Examples:
        - Rotate 90° right: direction='right', angle=90
        - Spin left continuously: direction='left', angle=None
    """
    robot = request.app.state.robot

    direction_mult = 1.0 if command.direction == "right" else -1.0

    if command.angle:
        await robot.rotate_angle(command.angle * direction_mult, command.speed)
        return CommandResponse(
            success=True,
            message=f"Rotated {command.angle}° {command.direction}"
        )
    else:
        await robot.rotate(command.speed * direction_mult)
        return CommandResponse(
            success=True,
            message=f"Rotating {command.direction}"
        )


@router.post("/turn_left", response_model=CommandResponse)
async def turn_left(
    request: Request,
    angle: Optional[float] = None,
    speed: float = 0.5
):
    """Turn left (rotate counter-clockwise)."""
    robot = request.app.state.robot

    if angle:
        await robot.rotate_angle(-angle, speed)
        return CommandResponse(
            success=True,
            message=f"Turned left {angle}°"
        )
    else:
        await robot.rotate(-speed)
        return CommandResponse(
            success=True,
            message="Turning left"
        )


@router.post("/turn_right", response_model=CommandResponse)
async def turn_right(
    request: Request,
    angle: Optional[float] = None,
    speed: float = 0.5
):
    """Turn right (rotate clockwise)."""
    robot = request.app.state.robot

    if angle:
        await robot.rotate_angle(angle, speed)
        return CommandResponse(
            success=True,
            message=f"Turned right {angle}°"
        )
    else:
        await robot.rotate(speed)
        return CommandResponse(
            success=True,
            message="Turning right"
        )


@router.post("/stop", response_model=CommandResponse)
async def stop_robot(request: Request, command: StopCommand = StopCommand()):
    """
    Stop robot movement.

    - **mode**: 'normal', 'emergency', or 'brake'
    """
    robot = request.app.state.robot

    if command.mode == "emergency":
        await robot.emergency_stop()
        return CommandResponse(
            success=True,
            message="Emergency stop activated"
        )
    else:
        await robot.stop()
        return CommandResponse(
            success=True,
            message="Robot stopped"
        )


@router.get("/state")
async def get_robot_state(request: Request):
    """Get current robot state."""
    robot = request.app.state.robot
    return robot.get_state().to_dict()


# Convenience endpoints for LLM integration
@router.post("/command", response_model=CommandResponse)
async def natural_command(request: Request, command: str):
    """
    Natural language command interface for LLM.

    Parses simple commands like:
    - "go forward"
    - "turn left 90 degrees"
    - "stop"
    - "move forward for 2 seconds"

    Note: For production, use the specific endpoints above.
    This is provided for LLM convenience.
    """
    robot = request.app.state.robot
    cmd = command.lower().strip()

    # Parse command
    if "stop" in cmd or "halt" in cmd:
        if "emergency" in cmd:
            await robot.emergency_stop()
            return CommandResponse(success=True, message="Emergency stop")
        await robot.stop()
        return CommandResponse(success=True, message="Stopped")

    elif "forward" in cmd:
        duration = _extract_duration(cmd)
        if duration:
            await robot.move_for_duration(0.5, 0, duration)
            return CommandResponse(success=True, message=f"Moved forward {duration}s")
        await robot.set_movement(0.5, 0)
        return CommandResponse(success=True, message="Moving forward")

    elif "backward" in cmd or "back" in cmd or "reverse" in cmd:
        duration = _extract_duration(cmd)
        if duration:
            await robot.move_for_duration(-0.5, 0, duration)
            return CommandResponse(success=True, message=f"Moved backward {duration}s")
        await robot.set_movement(-0.5, 0)
        return CommandResponse(success=True, message="Moving backward")

    elif "left" in cmd:
        angle = _extract_angle(cmd)
        if angle:
            await robot.rotate_angle(-angle, 0.5)
            return CommandResponse(success=True, message=f"Turned left {angle}°")
        await robot.rotate(-0.5)
        return CommandResponse(success=True, message="Turning left")

    elif "right" in cmd:
        angle = _extract_angle(cmd)
        if angle:
            await robot.rotate_angle(angle, 0.5)
            return CommandResponse(success=True, message=f"Turned right {angle}°")
        await robot.rotate(0.5)
        return CommandResponse(success=True, message="Turning right")

    else:
        return CommandResponse(
            success=False,
            message=f"Unknown command: {command}"
        )


def _extract_duration(text: str) -> Optional[float]:
    """Extract duration from text like 'for 2 seconds'."""
    import re
    match = re.search(r'(\d+(?:\.\d+)?)\s*(?:s|sec|seconds?)', text)
    if match:
        return float(match.group(1))
    return None


def _extract_angle(text: str) -> Optional[float]:
    """Extract angle from text like '90 degrees'."""
    import re
    match = re.search(r'(\d+(?:\.\d+)?)\s*(?:°|deg|degrees?)', text)
    if match:
        return float(match.group(1))
    return None
