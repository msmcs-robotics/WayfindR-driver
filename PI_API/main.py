#!/usr/bin/env python3
"""
WayfindR Robot Control API

A FastAPI-based web application for controlling a skid-steer robot.
Provides both REST API endpoints for LLM/programmatic control and
a web dashboard for manual control via keyboard/mouse.

Features:
- Manual control via keyboard (WASD, arrow keys, combinations)
- API endpoints for navigation commands
- Waypoint-based navigation integration
- Real-time telemetry via WebSocket
- Pattern movements (circle, square, etc.)

Usage:
    uvicorn main:app --host 0.0.0.0 --port 8000 --reload

Or run directly:
    python3 main.py
"""

import asyncio
import json
import time
from contextlib import asynccontextmanager
from datetime import datetime
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from routers import control, navigation, telemetry, patterns
from services.robot_controller import RobotController
from services.connection_manager import ConnectionManager
from models.robot_state import RobotState

# Global instances
robot = RobotController()
manager = ConnectionManager()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events."""
    # Startup
    print("=" * 50)
    print("  WayfindR Robot Control API")
    print("=" * 50)
    print(f"Starting at {datetime.now().isoformat()}")

    # Initialize robot controller
    await robot.initialize()

    # Start telemetry broadcast task
    asyncio.create_task(broadcast_telemetry())

    yield

    # Shutdown
    print("Shutting down...")
    await robot.stop()
    await robot.shutdown()


app = FastAPI(
    title="WayfindR Robot Control API",
    description="REST API and Dashboard for robot control",
    version="1.0.0",
    lifespan=lifespan
)

# Mount static files and templates
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

# Include routers
app.include_router(control.router, prefix="/api/control", tags=["Control"])
app.include_router(navigation.router, prefix="/api/navigation", tags=["Navigation"])
app.include_router(telemetry.router, prefix="/api/telemetry", tags=["Telemetry"])
app.include_router(patterns.router, prefix="/api/patterns", tags=["Patterns"])


# Store robot and manager in app state for access from routers
@app.on_event("startup")
async def store_globals():
    app.state.robot = robot
    app.state.manager = manager


async def broadcast_telemetry():
    """Broadcast robot telemetry to all connected WebSocket clients."""
    while True:
        if manager.active_connections:
            state = robot.get_state()
            await manager.broadcast({
                "type": "telemetry",
                "data": state.to_dict(),
                "timestamp": time.time()
            })
        await asyncio.sleep(0.1)  # 10 Hz update rate


# ============================================
# Dashboard Routes
# ============================================

@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):
    """Main dashboard page with robot controls."""
    return templates.TemplateResponse("index.html", {
        "request": request,
        "title": "WayfindR Control Dashboard",
        "robot_state": robot.get_state().to_dict()
    })


@app.get("/status", response_class=HTMLResponse)
async def status_page(request: Request):
    """Status/telemetry page."""
    return templates.TemplateResponse("status.html", {
        "request": request,
        "title": "Robot Status",
        "robot_state": robot.get_state().to_dict()
    })


# ============================================
# WebSocket for Real-time Control
# ============================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for real-time bidirectional communication.

    Receives:
        - Key press/release events
        - Direct movement commands

    Sends:
        - Telemetry updates
        - Command acknowledgments
        - Error messages
    """
    await manager.connect(websocket)

    try:
        while True:
            data = await websocket.receive_json()

            msg_type = data.get("type", "")

            if msg_type == "keydown":
                key = data.get("key", "").lower()
                await handle_key_press(key, pressed=True)

            elif msg_type == "keyup":
                key = data.get("key", "").lower()
                await handle_key_press(key, pressed=False)

            elif msg_type == "move":
                # Direct movement command with throttle and steering
                throttle = data.get("throttle", 0)
                steering = data.get("steering", 0)
                await robot.set_movement(throttle, steering)

            elif msg_type == "stop":
                await robot.stop()

            elif msg_type == "get_state":
                state = robot.get_state()
                await websocket.send_json({
                    "type": "state",
                    "data": state.to_dict()
                })

            # Send acknowledgment
            await websocket.send_json({
                "type": "ack",
                "command": msg_type,
                "status": "ok"
            })

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        manager.disconnect(websocket)


# Track pressed keys for combination handling
pressed_keys = set()


async def handle_key_press(key: str, pressed: bool):
    """
    Handle keyboard input with support for key combinations.

    Supported keys:
        - w, ArrowUp: Forward
        - s, ArrowDown: Backward
        - a, ArrowLeft: Turn left
        - d, ArrowRight: Turn right
        - q: Rotate left (in place)
        - e: Rotate right (in place)
        - Space: Stop

    Combinations:
        - w+a, Up+Left: Forward-left curve
        - w+d, Up+Right: Forward-right curve
        - s+a, Down+Left: Backward-left curve
        - s+d, Down+Right: Backward-right curve
    """
    global pressed_keys

    # Normalize key names
    key_map = {
        "arrowup": "w",
        "arrowdown": "s",
        "arrowleft": "a",
        "arrowright": "d",
        " ": "space"
    }
    key = key_map.get(key, key)

    if pressed:
        pressed_keys.add(key)
    else:
        pressed_keys.discard(key)

    # Calculate movement based on all pressed keys
    throttle = 0.0
    steering = 0.0

    # Forward/backward
    if "w" in pressed_keys:
        throttle = 1.0
    elif "s" in pressed_keys:
        throttle = -1.0

    # Left/right steering
    if "a" in pressed_keys:
        steering = -1.0
    elif "d" in pressed_keys:
        steering = 1.0

    # Rotate in place (q/e)
    if "q" in pressed_keys and throttle == 0:
        # Rotate left in place
        await robot.rotate(-1.0)
        return
    elif "e" in pressed_keys and throttle == 0:
        # Rotate right in place
        await robot.rotate(1.0)
        return

    # Space = emergency stop
    if "space" in pressed_keys:
        await robot.emergency_stop()
        pressed_keys.clear()
        return

    # Apply movement
    if throttle == 0 and steering == 0:
        await robot.stop()
    else:
        await robot.set_movement(throttle, steering)


# ============================================
# Quick API Endpoints (non-router)
# ============================================

@app.get("/api/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "robot_connected": robot.is_connected,
        "uptime": robot.uptime
    }


@app.post("/api/emergency_stop")
async def emergency_stop():
    """Emergency stop - immediately halt all motors."""
    await robot.emergency_stop()
    return {"status": "stopped", "message": "Emergency stop activated"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
