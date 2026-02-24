"""Background thread that broadcasts system telemetry via SocketIO."""

import logging
import time

logger = logging.getLogger(__name__)


def register_telemetry_stream(socketio, app):
    """Start a background thread that emits telemetry events."""
    from web_control.config import TELEMETRY_HZ

    def broadcast():
        interval = 1.0 / TELEMETRY_HZ
        while True:
            time.sleep(interval)
            with app.app_context():
                hw = app.hardware
                data = hw.get_telemetry()
                socketio.emit('telemetry', data)

    socketio.start_background_task(broadcast)
    logger.info('Telemetry stream started at %d Hz', TELEMETRY_HZ)
