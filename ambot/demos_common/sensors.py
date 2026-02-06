"""
Sensor wrappers - LiDAR auto-detect, camera face detection, IMU setup.
"""

import logging
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)


# =============================================================================
# LiDAR
# =============================================================================

def create_lidar():
    """Create LiDAR instance based on available driver."""
    try:
        from pathfinder.lidar_ld19 import LD19Lidar
        lidar = LD19Lidar()
        logger.info("Using LD19 LiDAR driver")
        return lidar
    except ImportError:
        pass

    try:
        from pathfinder.lidar import RPLidar
        lidar = RPLidar()
        logger.info("Using RPLidar driver")
        return lidar
    except ImportError:
        pass

    raise RuntimeError("No LiDAR driver available")


# =============================================================================
# Camera Face Detection
# =============================================================================

class FaceData:
    """Thread-safe container for face detection results."""

    def __init__(self):
        self._lock = threading.Lock()
        self._faces = []  # list of (center_x, center_y, width, height)
        self._timestamp = 0.0
        self._frame_count = 0

    def update(self, faces, timestamp):
        with self._lock:
            self._faces = list(faces)
            self._timestamp = timestamp
            self._frame_count += 1

    def get(self):
        """Returns (faces, timestamp, frame_count)."""
        with self._lock:
            return list(self._faces), self._timestamp, self._frame_count


class CameraFaceThread(threading.Thread):
    """Background thread for camera face detection."""

    def __init__(self, face_data: FaceData, device: int = 0):
        super().__init__(daemon=True)
        self.face_data = face_data
        self.device = device
        self.running = False
        self.connected = False
        self.frame_width = 640
        self.frame_height = 480

    def run(self):
        self.running = True
        try:
            import cv2
        except ImportError:
            logger.error("OpenCV not available - camera disabled")
            return

        # Open camera
        cap = cv2.VideoCapture(self.device)
        if not cap.isOpened():
            logger.error(f"Failed to open camera device {self.device}")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.connected = True
        logger.info(f"Camera opened: {self.frame_width}x{self.frame_height}")

        # Load face cascade
        face_cascade = self._load_cascade(cv2)
        if face_cascade is None:
            logger.error("Failed to load face cascade - camera disabled")
            cap.release()
            return

        logger.info("Face detection ready")

        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                # Detect faces
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = face_cascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=(30, 30),
                )

                faces = []
                if len(detections) > 0:
                    for (x, y, w, h) in detections:
                        center_x = int(x + w // 2)
                        center_y = int(y + h // 2)
                        faces.append((center_x, center_y, int(w), int(h)))

                self.face_data.update(faces, time.time())

                # ~10 fps for face detection
                time.sleep(0.1)

        finally:
            cap.release()
            logger.info("Camera released")

    def stop(self):
        self.running = False

    def _load_cascade(self, cv2):
        """Load Haar cascade for face detection."""
        search_paths = [
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ]

        # Try OpenCV package data
        try:
            data_dir = getattr(cv2, 'data', None)
            if data_dir and hasattr(data_dir, 'haarcascades'):
                search_paths.insert(0, data_dir.haarcascades + "haarcascade_frontalface_default.xml")
        except Exception:
            pass

        for path in search_paths:
            if Path(path).exists():
                cascade = cv2.CascadeClassifier(path)
                if not cascade.empty():
                    logger.info(f"Loaded face cascade: {path}")
                    return cascade

        return None


# =============================================================================
# IMU
# =============================================================================

def setup_imu(bus=1, address=0x68, calibrate=True):
    """
    Connect and optionally calibrate the MPU6050 IMU.

    Returns the IMU instance if connected, or None if not available.
    Designed for graceful degradation - demos work without the IMU.

    Args:
        bus: I2C bus number (default 1)
        address: I2C address (default 0x68)
        calibrate: Run gyro calibration at startup (default True)

    Returns:
        IMU instance or None
    """
    try:
        from pathfinder.imu import IMU

        imu = IMU(bus=bus, address=address)
        if imu.connect():
            logger.info("IMU connected")
            if calibrate:
                logger.info("Calibrating IMU (keep robot still)...")
                if imu.calibrate():
                    logger.info("IMU calibrated")
                else:
                    logger.warning("IMU calibration failed, continuing without calibration")
            return imu
        else:
            logger.info("IMU not found, continuing without heading")
            return None
    except ImportError:
        logger.info("IMU driver not available")
        return None
    except Exception as e:
        logger.warning(f"IMU setup failed: {e}")
        return None
