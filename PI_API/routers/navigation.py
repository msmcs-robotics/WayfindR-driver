"""
Navigation Router

API endpoints for waypoint-based navigation.
Integrates with ROS2 Nav2 when available.
"""

from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, List, Dict

from models.commands import (
    NavigationGoal,
    WaypointCommand,
    CommandResponse
)

router = APIRouter()


class Waypoint(BaseModel):
    """Waypoint definition."""
    name: str
    x: float
    y: float
    yaw: float = 0.0
    description: Optional[str] = None


class Route(BaseModel):
    """Named route through multiple waypoints."""
    name: str
    waypoints: List[str]


# In-memory waypoint storage (would be persisted in production)
waypoints_db: Dict[str, Waypoint] = {}
routes_db: Dict[str, Route] = {}


@router.post("/goto", response_model=CommandResponse)
async def navigate_to_goal(request: Request, goal: NavigationGoal):
    """
    Navigate to coordinates.

    - **x**: Target X position in meters
    - **y**: Target Y position in meters
    - **yaw**: Target orientation in degrees (optional)
    """
    robot = request.app.state.robot
    nav_service = getattr(request.app.state, 'navigation', None)

    if nav_service:
        success = await nav_service.navigate_to(goal.x, goal.y, goal.yaw)
        if success:
            return CommandResponse(
                success=True,
                message=f"Navigating to ({goal.x:.2f}, {goal.y:.2f})",
                data={"x": goal.x, "y": goal.y, "yaw": goal.yaw}
            )
        else:
            return CommandResponse(
                success=False,
                message="Navigation failed or not available"
            )
    else:
        # Fallback: simple dead reckoning movement
        return CommandResponse(
            success=False,
            message="Navigation service not available. Use /api/control endpoints."
        )


@router.post("/waypoint/goto", response_model=CommandResponse)
async def navigate_to_waypoint(request: Request, command: WaypointCommand):
    """
    Navigate to a named waypoint.

    - **waypoint_name**: Name of the waypoint to navigate to
    """
    if command.waypoint_name not in waypoints_db:
        raise HTTPException(
            status_code=404,
            detail=f"Waypoint not found: {command.waypoint_name}"
        )

    wp = waypoints_db[command.waypoint_name]
    nav_service = getattr(request.app.state, 'navigation', None)

    if nav_service:
        success = await nav_service.navigate_to(wp.x, wp.y, wp.yaw)
        return CommandResponse(
            success=success,
            message=f"Navigating to waypoint: {command.waypoint_name}",
            data={"waypoint": command.waypoint_name, "x": wp.x, "y": wp.y}
        )

    return CommandResponse(
        success=False,
        message="Navigation service not available"
    )


@router.get("/waypoints")
async def list_waypoints():
    """Get all defined waypoints."""
    return {
        "waypoints": [
            {
                "name": wp.name,
                "x": wp.x,
                "y": wp.y,
                "yaw": wp.yaw,
                "description": wp.description
            }
            for wp in waypoints_db.values()
        ],
        "count": len(waypoints_db)
    }


@router.post("/waypoints", response_model=CommandResponse)
async def add_waypoint(waypoint: Waypoint):
    """Add a new waypoint."""
    waypoints_db[waypoint.name] = waypoint
    return CommandResponse(
        success=True,
        message=f"Waypoint added: {waypoint.name}",
        data={"name": waypoint.name, "x": waypoint.x, "y": waypoint.y}
    )


@router.delete("/waypoints/{name}", response_model=CommandResponse)
async def delete_waypoint(name: str):
    """Delete a waypoint."""
    if name not in waypoints_db:
        raise HTTPException(status_code=404, detail=f"Waypoint not found: {name}")

    del waypoints_db[name]
    return CommandResponse(
        success=True,
        message=f"Waypoint deleted: {name}"
    )


@router.post("/waypoints/save_current", response_model=CommandResponse)
async def save_current_position(request: Request, name: str, description: str = ""):
    """Save current robot position as a waypoint."""
    robot = request.app.state.robot
    state = robot.get_state()

    waypoint = Waypoint(
        name=name,
        x=state.position.x,
        y=state.position.y,
        yaw=state.position.yaw,
        description=description
    )
    waypoints_db[name] = waypoint

    return CommandResponse(
        success=True,
        message=f"Saved current position as: {name}",
        data={"name": name, "x": waypoint.x, "y": waypoint.y}
    )


@router.get("/routes")
async def list_routes():
    """Get all defined routes."""
    return {
        "routes": [
            {"name": r.name, "waypoints": r.waypoints}
            for r in routes_db.values()
        ],
        "count": len(routes_db)
    }


@router.post("/routes", response_model=CommandResponse)
async def create_route(route: Route):
    """Create a new route."""
    # Validate all waypoints exist
    missing = [wp for wp in route.waypoints if wp not in waypoints_db]
    if missing:
        raise HTTPException(
            status_code=400,
            detail=f"Unknown waypoints: {missing}"
        )

    routes_db[route.name] = route
    return CommandResponse(
        success=True,
        message=f"Route created: {route.name}",
        data={"name": route.name, "waypoints": route.waypoints}
    )


@router.post("/routes/{name}/execute", response_model=CommandResponse)
async def execute_route(request: Request, name: str):
    """Execute a named route."""
    if name not in routes_db:
        raise HTTPException(status_code=404, detail=f"Route not found: {name}")

    route = routes_db[name]
    nav_service = getattr(request.app.state, 'navigation', None)

    if not nav_service:
        return CommandResponse(
            success=False,
            message="Navigation service not available"
        )

    # Queue waypoints for navigation
    for wp_name in route.waypoints:
        wp = waypoints_db[wp_name]
        await nav_service.add_goal(wp.x, wp.y, wp.yaw)

    return CommandResponse(
        success=True,
        message=f"Executing route: {name}",
        data={"route": name, "waypoints": route.waypoints}
    )


@router.post("/cancel", response_model=CommandResponse)
async def cancel_navigation(request: Request):
    """Cancel current navigation."""
    robot = request.app.state.robot
    await robot.stop()

    nav_service = getattr(request.app.state, 'navigation', None)
    if nav_service:
        await nav_service.cancel()

    return CommandResponse(
        success=True,
        message="Navigation cancelled"
    )


@router.get("/status")
async def navigation_status(request: Request):
    """Get current navigation status."""
    robot = request.app.state.robot
    state = robot.get_state()

    return {
        "navigating": state.navigation.is_navigating,
        "current_goal": state.navigation.current_goal,
        "distance_to_goal": state.navigation.distance_to_goal,
        "position": {
            "x": state.position.x,
            "y": state.position.y,
            "yaw": state.position.yaw
        }
    }
