"""SocketIO event registration."""

from .motor_events import register_motor_events
from .lidar_stream import register_lidar_stream
from .telemetry import register_telemetry_stream
from .chat_stream import register_chat_stream


def register_events(socketio, app):
    """Register all SocketIO event handlers."""
    register_motor_events(socketio, app)
    register_lidar_stream(socketio, app)
    register_telemetry_stream(socketio, app)
    register_chat_stream(socketio, app)
