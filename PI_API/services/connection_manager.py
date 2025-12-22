"""
WebSocket Connection Manager

Manages WebSocket connections for real-time communication
with dashboard and other clients.
"""

import asyncio
import json
from typing import List, Dict, Any
from fastapi import WebSocket


class ConnectionManager:
    """
    Manages WebSocket connections.

    Features:
    - Track active connections
    - Broadcast to all clients
    - Send to specific client
    - Handle disconnections gracefully
    """

    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        """Accept and register a new WebSocket connection."""
        await websocket.accept()
        async with self._lock:
            self.active_connections.append(websocket)
        print(f"WebSocket connected. Total connections: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        """Remove a WebSocket connection."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        print(f"WebSocket disconnected. Total connections: {len(self.active_connections)}")

    async def send_personal_message(self, message: Dict[str, Any], websocket: WebSocket):
        """Send a message to a specific client."""
        try:
            await websocket.send_json(message)
        except Exception as e:
            print(f"Error sending personal message: {e}")
            self.disconnect(websocket)

    async def broadcast(self, message: Dict[str, Any]):
        """Broadcast a message to all connected clients."""
        if not self.active_connections:
            return

        # Create tasks for all sends
        disconnected = []

        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                print(f"Error broadcasting to client: {e}")
                disconnected.append(connection)

        # Remove disconnected clients
        for conn in disconnected:
            self.disconnect(conn)

    async def broadcast_text(self, message: str):
        """Broadcast a text message to all clients."""
        await self.broadcast({"type": "message", "data": message})

    @property
    def connection_count(self) -> int:
        """Get number of active connections."""
        return len(self.active_connections)
