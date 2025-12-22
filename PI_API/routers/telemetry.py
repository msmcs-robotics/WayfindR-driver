"""
Telemetry Router

API endpoints for robot telemetry and sensor data.
"""

from fastapi import APIRouter, Request
from typing import Optional, List
from datetime import datetime
import time

router = APIRouter()

# Store telemetry history for analysis
telemetry_history: List[dict] = []
MAX_HISTORY = 1000


@router.get("/current")
async def get_current_telemetry(request: Request):
    """
    Get current robot telemetry.

    Returns all sensor data and state information.
    """
    robot = request.app.state.robot
    state = robot.get_state()

    telemetry = {
        "timestamp": time.time(),
        "datetime": datetime.now().isoformat(),
        "mode": state.mode.value,
        "movement_state": state.movement_state.value,
        "position": {
            "x": state.position.x,
            "y": state.position.y,
            "z": state.position.z,
            "yaw": state.position.yaw,
            "pitch": state.position.pitch,
            "roll": state.position.roll
        },
        "velocity": {
            "linear_x": state.velocity.linear_x,
            "linear_y": state.velocity.linear_y,
            "angular_z": state.velocity.angular_z
        },
        "drive": {
            "throttle": state.drive.throttle,
            "steering": state.drive.steering,
            "motors": {
                "left_front": {"speed": state.drive.left_front.speed, "current": state.drive.left_front.current},
                "left_rear": {"speed": state.drive.left_rear.speed, "current": state.drive.left_rear.current},
                "right_front": {"speed": state.drive.right_front.speed, "current": state.drive.right_front.current},
                "right_rear": {"speed": state.drive.right_rear.speed, "current": state.drive.right_rear.current}
            }
        },
        "sensors": {
            "battery_voltage": state.sensors.battery_voltage,
            "battery_percentage": state.sensors.battery_percentage,
            "imu": {
                "accel": state.sensors.imu_accel,
                "gyro": state.sensors.imu_gyro,
                "mag": state.sensors.imu_mag
            },
            "temperature": state.sensors.temperature,
            "obstacles": state.sensors.obstacle_distances
        },
        "robot": {
            "connected": robot.is_connected,
            "uptime": robot.uptime
        }
    }

    # Add to history
    _add_to_history(telemetry)

    return telemetry


@router.get("/position")
async def get_position(request: Request):
    """Get current position and orientation."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "x": state.position.x,
        "y": state.position.y,
        "z": state.position.z,
        "yaw": state.position.yaw,
        "pitch": state.position.pitch,
        "roll": state.position.roll,
        "timestamp": time.time()
    }


@router.get("/velocity")
async def get_velocity(request: Request):
    """Get current velocity."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "linear_x": state.velocity.linear_x,
        "linear_y": state.velocity.linear_y,
        "angular_z": state.velocity.angular_z,
        "timestamp": time.time()
    }


@router.get("/battery")
async def get_battery(request: Request):
    """Get battery status."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "voltage": state.sensors.battery_voltage,
        "percentage": state.sensors.battery_percentage,
        "timestamp": time.time()
    }


@router.get("/motors")
async def get_motor_status(request: Request):
    """Get motor status for all wheels."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "left_front": {
            "speed": state.drive.left_front.speed,
            "current": state.drive.left_front.current,
            "temperature": state.drive.left_front.temperature,
            "enabled": state.drive.left_front.enabled
        },
        "left_rear": {
            "speed": state.drive.left_rear.speed,
            "current": state.drive.left_rear.current,
            "temperature": state.drive.left_rear.temperature,
            "enabled": state.drive.left_rear.enabled
        },
        "right_front": {
            "speed": state.drive.right_front.speed,
            "current": state.drive.right_front.current,
            "temperature": state.drive.right_front.temperature,
            "enabled": state.drive.right_front.enabled
        },
        "right_rear": {
            "speed": state.drive.right_rear.speed,
            "current": state.drive.right_rear.current,
            "temperature": state.drive.right_rear.temperature,
            "enabled": state.drive.right_rear.enabled
        },
        "timestamp": time.time()
    }


@router.get("/imu")
async def get_imu_data(request: Request):
    """Get IMU sensor data."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "accelerometer": state.sensors.imu_accel,
        "gyroscope": state.sensors.imu_gyro,
        "magnetometer": state.sensors.imu_mag,
        "timestamp": time.time()
    }


@router.get("/obstacles")
async def get_obstacle_data(request: Request):
    """Get obstacle detection data."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "distances": state.sensors.obstacle_distances,
        "timestamp": time.time()
    }


@router.get("/history")
async def get_telemetry_history(
    limit: int = 100,
    offset: int = 0
):
    """
    Get historical telemetry data.

    - **limit**: Number of records to return (max 1000)
    - **offset**: Starting offset
    """
    limit = min(limit, MAX_HISTORY)
    start = max(0, len(telemetry_history) - offset - limit)
    end = len(telemetry_history) - offset

    return {
        "history": telemetry_history[start:end],
        "total": len(telemetry_history),
        "returned": end - start
    }


@router.delete("/history")
async def clear_telemetry_history():
    """Clear telemetry history."""
    global telemetry_history
    count = len(telemetry_history)
    telemetry_history = []
    return {"message": f"Cleared {count} records"}


@router.get("/summary")
async def get_telemetry_summary(request: Request):
    """Get a summary of robot status for dashboards."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "status": "online" if robot.is_connected else "offline",
        "mode": state.mode.value,
        "movement": state.movement_state.value,
        "battery": f"{state.sensors.battery_percentage:.0f}%",
        "position": f"({state.position.x:.2f}, {state.position.y:.2f})",
        "heading": f"{state.position.yaw:.1f}°",
        "speed": f"{abs(state.velocity.linear_x):.2f} m/s",
        "uptime": _format_uptime(robot.uptime),
        "navigating": state.navigation.is_navigating,
        "timestamp": time.time()
    }


def _add_to_history(telemetry: dict):
    """Add telemetry to history, maintaining max size."""
    global telemetry_history
    telemetry_history.append(telemetry)
    if len(telemetry_history) > MAX_HISTORY:
        telemetry_history = telemetry_history[-MAX_HISTORY:]


def _format_uptime(seconds: float) -> str:
    """Format uptime as human-readable string."""
    if seconds < 60:
        return f"{int(seconds)}s"
    elif seconds < 3600:
        return f"{int(seconds / 60)}m {int(seconds % 60)}s"
    else:
        hours = int(seconds / 3600)
        minutes = int((seconds % 3600) / 60)
        return f"{hours}h {minutes}m"
