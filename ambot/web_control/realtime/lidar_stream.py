"""Background thread that broadcasts LiDAR scans via SocketIO."""

import logging
import time

logger = logging.getLogger(__name__)


def register_lidar_stream(socketio, app):
    """Start a background thread that emits lidar_scan events."""
    from web_control.config import LIDAR_HZ

    def broadcast():
        interval = 1.0 / LIDAR_HZ
        while True:
            time.sleep(interval)
            with app.app_context():
                hw = app.hardware
                scan = hw.get_scan()
                if scan:
                    socketio.emit('lidar_scan', {
                        'scan': scan,
                        'count': len(scan),
                    })

    socketio.start_background_task(broadcast)
    logger.info('LiDAR stream started at %d Hz', LIDAR_HZ)
