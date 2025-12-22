"""
Patterns Router

API endpoints for executing movement patterns.
Useful for testing, calibration, and demonstrations.
"""

import asyncio
import math
from fastapi import APIRouter, Request, HTTPException
from typing import Optional

from models.commands import PatternCommand, PatternType, CommandResponse

router = APIRouter()

# Track active pattern task
active_pattern_task = None


@router.post("/execute", response_model=CommandResponse)
async def execute_pattern(request: Request, command: PatternCommand):
    """
    Execute a movement pattern.

    - **pattern**: circle, square, figure_eight, zigzag, spiral, scan
    - **size**: Pattern size in meters (default 1.0)
    - **speed**: Movement speed 0.0-1.0 (default 0.5)
    - **repetitions**: Number of times to repeat (default 1)
    """
    global active_pattern_task

    robot = request.app.state.robot

    # Cancel any existing pattern
    if active_pattern_task and not active_pattern_task.done():
        active_pattern_task.cancel()
        await robot.stop()

    # Start pattern execution
    active_pattern_task = asyncio.create_task(
        _execute_pattern(robot, command)
    )

    return CommandResponse(
        success=True,
        message=f"Starting pattern: {command.pattern.value}",
        data={
            "pattern": command.pattern.value,
            "size": command.size,
            "speed": command.speed,
            "repetitions": command.repetitions
        }
    )


@router.post("/circle", response_model=CommandResponse)
async def pattern_circle(
    request: Request,
    radius: float = 1.0,
    speed: float = 0.5,
    clockwise: bool = True
):
    """Execute a circle pattern."""
    command = PatternCommand(
        pattern=PatternType.CIRCLE,
        size=radius,
        speed=speed
    )
    command.clockwise = clockwise
    return await execute_pattern(request, command)


@router.post("/square", response_model=CommandResponse)
async def pattern_square(
    request: Request,
    side_length: float = 1.0,
    speed: float = 0.5
):
    """Execute a square pattern."""
    command = PatternCommand(
        pattern=PatternType.SQUARE,
        size=side_length,
        speed=speed
    )
    return await execute_pattern(request, command)


@router.post("/figure_eight", response_model=CommandResponse)
async def pattern_figure_eight(
    request: Request,
    size: float = 1.0,
    speed: float = 0.5
):
    """Execute a figure-eight pattern."""
    command = PatternCommand(
        pattern=PatternType.FIGURE_EIGHT,
        size=size,
        speed=speed
    )
    return await execute_pattern(request, command)


@router.post("/zigzag", response_model=CommandResponse)
async def pattern_zigzag(
    request: Request,
    width: float = 2.0,
    legs: int = 4,
    speed: float = 0.5
):
    """Execute a zigzag pattern."""
    command = PatternCommand(
        pattern=PatternType.ZIGZAG,
        size=width,
        speed=speed,
        repetitions=legs
    )
    return await execute_pattern(request, command)


@router.post("/spiral", response_model=CommandResponse)
async def pattern_spiral(
    request: Request,
    max_radius: float = 2.0,
    speed: float = 0.5,
    outward: bool = True
):
    """Execute a spiral pattern."""
    command = PatternCommand(
        pattern=PatternType.SPIRAL,
        size=max_radius,
        speed=speed
    )
    command.outward = outward
    return await execute_pattern(request, command)


@router.post("/scan", response_model=CommandResponse)
async def pattern_scan(
    request: Request,
    width: float = 3.0,
    height: float = 2.0,
    speed: float = 0.3
):
    """Execute a lawn-mower scan pattern."""
    command = PatternCommand(
        pattern=PatternType.SCAN,
        size=width,
        speed=speed
    )
    command.height = height
    return await execute_pattern(request, command)


@router.post("/stop", response_model=CommandResponse)
async def stop_pattern(request: Request):
    """Stop the current pattern."""
    global active_pattern_task

    robot = request.app.state.robot

    if active_pattern_task and not active_pattern_task.done():
        active_pattern_task.cancel()
        try:
            await active_pattern_task
        except asyncio.CancelledError:
            pass

    await robot.stop()

    return CommandResponse(
        success=True,
        message="Pattern stopped"
    )


@router.get("/status")
async def pattern_status():
    """Get current pattern execution status."""
    global active_pattern_task

    if active_pattern_task is None:
        return {"running": False, "pattern": None}

    return {
        "running": not active_pattern_task.done(),
        "cancelled": active_pattern_task.cancelled() if active_pattern_task.done() else False
    }


@router.get("/list")
async def list_patterns():
    """List all available patterns."""
    return {
        "patterns": [
            {
                "name": "circle",
                "description": "Drive in a circle",
                "parameters": ["radius", "speed", "clockwise"]
            },
            {
                "name": "square",
                "description": "Drive in a square with 90° turns",
                "parameters": ["side_length", "speed"]
            },
            {
                "name": "figure_eight",
                "description": "Drive in a figure-eight pattern",
                "parameters": ["size", "speed"]
            },
            {
                "name": "zigzag",
                "description": "Drive in a zigzag pattern",
                "parameters": ["width", "legs", "speed"]
            },
            {
                "name": "spiral",
                "description": "Drive in an expanding/contracting spiral",
                "parameters": ["max_radius", "speed", "outward"]
            },
            {
                "name": "scan",
                "description": "Lawn-mower pattern for area coverage",
                "parameters": ["width", "height", "speed"]
            }
        ]
    }


async def _execute_pattern(robot, command: PatternCommand):
    """Execute the specified pattern."""
    try:
        pattern = command.pattern
        size = command.size
        speed = command.speed
        reps = command.repetitions

        for _ in range(reps):
            if pattern == PatternType.CIRCLE:
                await _pattern_circle(robot, size, speed, getattr(command, 'clockwise', True))
            elif pattern == PatternType.SQUARE:
                await _pattern_square(robot, size, speed)
            elif pattern == PatternType.FIGURE_EIGHT:
                await _pattern_figure_eight(robot, size, speed)
            elif pattern == PatternType.ZIGZAG:
                await _pattern_zigzag(robot, size, speed)
            elif pattern == PatternType.SPIRAL:
                await _pattern_spiral(robot, size, speed, getattr(command, 'outward', True))
            elif pattern == PatternType.SCAN:
                await _pattern_scan(robot, size, getattr(command, 'height', size), speed)

        await robot.stop()

    except asyncio.CancelledError:
        await robot.stop()
        raise


async def _pattern_circle(robot, radius: float, speed: float, clockwise: bool = True):
    """Drive in a circle."""
    # Calculate steering based on radius
    # Tighter radius = more steering
    steering = min(1.0, 1.0 / radius) * (1 if clockwise else -1)

    # Estimate time for full circle
    circumference = 2 * math.pi * radius
    estimated_speed_mps = speed * 0.5  # Rough m/s estimate
    duration = circumference / estimated_speed_mps if estimated_speed_mps > 0 else 10

    await robot.set_movement(speed, steering)
    await asyncio.sleep(duration)
    await robot.stop()


async def _pattern_square(robot, side_length: float, speed: float):
    """Drive in a square."""
    # Estimate time for one side
    estimated_speed_mps = speed * 0.5
    side_duration = side_length / estimated_speed_mps if estimated_speed_mps > 0 else 2

    for i in range(4):
        # Drive forward
        await robot.set_movement(speed, 0)
        await asyncio.sleep(side_duration)
        await robot.stop()
        await asyncio.sleep(0.2)

        # Turn 90 degrees right
        await robot.rotate_angle(90, speed)
        await asyncio.sleep(0.2)

    await robot.stop()


async def _pattern_figure_eight(robot, size: float, speed: float):
    """Drive in a figure-eight."""
    # Right circle
    await _pattern_circle(robot, size / 2, speed, clockwise=True)
    # Left circle
    await _pattern_circle(robot, size / 2, speed, clockwise=False)


async def _pattern_zigzag(robot, width: float, speed: float, legs: int = 4):
    """Drive in a zigzag pattern."""
    estimated_speed_mps = speed * 0.5
    leg_duration = width / estimated_speed_mps if estimated_speed_mps > 0 else 2

    for i in range(legs):
        # Drive diagonal
        steering = 0.3 if i % 2 == 0 else -0.3
        await robot.set_movement(speed, steering)
        await asyncio.sleep(leg_duration)

    await robot.stop()


async def _pattern_spiral(robot, max_radius: float, speed: float, outward: bool = True):
    """Drive in a spiral."""
    steps = 20
    for i in range(steps):
        progress = i / steps if outward else (steps - i) / steps
        radius = 0.3 + (max_radius - 0.3) * progress
        steering = min(1.0, 1.0 / radius)

        await robot.set_movement(speed, steering)
        await asyncio.sleep(0.5)

    await robot.stop()


async def _pattern_scan(robot, width: float, height: float, speed: float):
    """Lawn-mower scan pattern."""
    estimated_speed_mps = speed * 0.5
    row_duration = width / estimated_speed_mps if estimated_speed_mps > 0 else 3
    rows = int(height / 0.3) + 1  # ~30cm spacing

    for i in range(rows):
        # Drive across
        await robot.set_movement(speed, 0)
        await asyncio.sleep(row_duration)
        await robot.stop()
        await asyncio.sleep(0.2)

        if i < rows - 1:
            # Turn and move to next row
            turn_dir = 90 if i % 2 == 0 else -90
            await robot.rotate_angle(turn_dir, speed)

            # Move forward to next row
            await robot.set_movement(speed * 0.5, 0)
            await asyncio.sleep(0.6)
            await robot.stop()

            # Turn to face opposite direction
            await robot.rotate_angle(turn_dir, speed)

    await robot.stop()
