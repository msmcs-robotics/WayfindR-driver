"""Hardware abstraction layer — manages motors, LiDAR, camera, IMU.

Graceful degradation: each sensor is optional. If hardware is
unavailable, the manager falls back to simulation/mock data.
"""

import logging
import threading
import time

logger = logging.getLogger(__name__)


class HardwareManager:
    """Singleton manager for all robot hardware."""

    def __init__(self, simulate=False):
        self.simulate = simulate
        self.robot = None
        self.lidar = None
        self.camera = None
        self.face_data = None
        self.imu = None

        self._last_motor_cmd = 0.0
        self._motor_left = 0
        self._motor_right = 0
        self._motor_lock = threading.Lock()
        self._running = False
        self._watchdog_thread = None
        self._start_time = time.time()
        self._motor_cmd_count = 0

        # LiDAR background thread
        self._lidar_thread = None
        self._latest_scan = None
        self._scan_lock = threading.Lock()
        self._scan_rate = 0.0

        # Camera background
        self._latest_jpeg = None
        self._jpeg_lock = threading.Lock()
        self._camera_fps = 0.0

    def initialize(self):
        """Initialize all hardware. Safe to call multiple times."""
        self._running = True
        self._init_motors()
        self._init_lidar()
        self._init_camera()
        self._init_imu()
        self._start_watchdog()
        logger.info('Hardware initialized (simulate=%s)', self.simulate)

    # -- Motors ----------------------------------------------------------------

    def _init_motors(self):
        if self.simulate:
            logger.info('Motors: simulation mode')
            return

        try:
            from locomotion.rpi_motors.factory import create_robot
            from locomotion.rpi_motors.drivers import DriverType
            real_robot = create_robot(driver_type=DriverType.L298N)
            real_robot.setup()
            real_robot.stop()

            from demos_common.robot import RobotAdapter
            self.robot = RobotAdapter(robot=real_robot, simulate=False)
            logger.info('Motors: L298N initialized')
        except Exception as e:
            logger.warning('Motors unavailable: %s', e)

    def motor_drive(self, left, right):
        """Send motor command. left/right are -100 to 100."""
        with self._motor_lock:
            self._motor_left = int(max(-100, min(100, left)))
            self._motor_right = int(max(-100, min(100, right)))
            self._last_motor_cmd = time.time()
            self._motor_cmd_count += 1

        if self.robot:
            self.robot.set_motors(self._motor_left / 100.0,
                                  self._motor_right / 100.0)
        elif self.simulate:
            logger.debug('Motor: L=%+d R=%+d', self._motor_left,
                         self._motor_right)

    def emergency_stop(self):
        """Immediate motor stop."""
        with self._motor_lock:
            self._motor_left = 0
            self._motor_right = 0
            self._last_motor_cmd = time.time()

        if self.robot:
            self.robot.stop()
        logger.warning('EMERGENCY STOP')

    def get_motor_status(self):
        """Return current motor state."""
        with self._motor_lock:
            return {
                'left': self._motor_left,
                'right': self._motor_right,
                'cmd_count': self._motor_cmd_count,
            }

    def _start_watchdog(self):
        """Background thread that stops motors if no command received."""
        from web_control.config import MOTOR_WATCHDOG_TIMEOUT

        def watchdog():
            while self._running:
                time.sleep(0.25)
                with self._motor_lock:
                    if (self._motor_left != 0 or self._motor_right != 0):
                        elapsed = time.time() - self._last_motor_cmd
                        if elapsed > MOTOR_WATCHDOG_TIMEOUT:
                            self._motor_left = 0
                            self._motor_right = 0
                            if self.robot:
                                self.robot.stop()
                            logger.info('Watchdog: motors stopped (%.1fs idle)',
                                        elapsed)

        self._watchdog_thread = threading.Thread(target=watchdog, daemon=True,
                                                 name='motor-watchdog')
        self._watchdog_thread.start()

    # -- LiDAR -----------------------------------------------------------------

    def _init_lidar(self):
        if self.simulate:
            logger.info('LiDAR: simulation mode (mock data)')
            return

        try:
            from pathfinder.lidar_ld19 import LD19Lidar
            self.lidar = LD19Lidar()
            if self.lidar.connect():
                logger.info('LiDAR: LD19 connected')
                self._start_lidar_thread()
            else:
                logger.warning('LiDAR: failed to connect')
                self.lidar = None
        except Exception as e:
            logger.warning('LiDAR unavailable: %s', e)
            self.lidar = None

    def _start_lidar_thread(self):
        """Background thread that reads LiDAR scans."""
        def reader():
            scan_times = []
            try:
                for scan in self.lidar.iter_scans(min_points=200):
                    if not self._running:
                        break
                    now = time.time()
                    points = [{'angle': p.angle, 'distance': p.distance}
                              for p in scan]
                    with self._scan_lock:
                        self._latest_scan = points
                    scan_times.append(now)
                    # Calculate scan rate over last 10 scans
                    scan_times = scan_times[-10:]
                    if len(scan_times) > 1:
                        dt = scan_times[-1] - scan_times[0]
                        self._scan_rate = (len(scan_times) - 1) / dt if dt > 0 else 0
            except Exception as e:
                logger.error('LiDAR reader error: %s', e)

        self._lidar_thread = threading.Thread(target=reader, daemon=True,
                                              name='lidar-reader')
        self._lidar_thread.start()

    def get_scan(self):
        """Get latest LiDAR scan as list of {angle, distance} dicts."""
        with self._scan_lock:
            if self._latest_scan is not None:
                return self._latest_scan
        return []

    # -- Camera ----------------------------------------------------------------

    def _init_camera(self):
        if self.simulate:
            logger.info('Camera: simulation mode (no feed)')
            return

        try:
            import cv2
            from web_control.config import CAMERA_DEVICE
            from demos_common.sensors import FaceData
            self.face_data = FaceData()
            self._start_camera_thread(CAMERA_DEVICE)
        except Exception as e:
            logger.warning('Camera unavailable: %s', e)

    def _start_camera_thread(self, device):
        """Background thread: captures frames, detects faces, stores JPEG."""
        import cv2
        from web_control.config import CAMERA_JPEG_QUALITY

        def reader():
            cap = cv2.VideoCapture(device)
            if not cap.isOpened():
                logger.warning('Camera: could not open device %d', device)
                return

            self.camera = cap
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            face_cascade = cv2.CascadeClassifier(cascade_path)
            frame_count = 0
            fps_times = []

            logger.info('Camera: device %d opened (%dx%d)', device,
                        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

            while self._running:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                frame_count += 1
                now = time.time()

                # Encode JPEG
                _, jpeg = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY,
                                        CAMERA_JPEG_QUALITY])
                with self._jpeg_lock:
                    self._latest_jpeg = jpeg.tobytes()

                # Face detection every 3rd frame
                if frame_count % 3 == 0 and self.face_data is not None:
                    small = cv2.resize(frame, None, fx=0.5, fy=0.5)
                    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                    faces = face_cascade.detectMultiScale(
                        gray, scaleFactor=1.1, minNeighbors=5,
                        minSize=(30, 30))
                    # Scale back to full resolution
                    scaled = [(int(x * 2), int(y * 2), int(w * 2), int(h * 2))
                              for (x, y, w, h) in faces]
                    centers = [(x + w // 2, y + h // 2, w, h)
                               for (x, y, w, h) in scaled]
                    self.face_data.update(centers, now)

                # FPS tracking
                fps_times.append(now)
                fps_times = fps_times[-30:]
                if len(fps_times) > 1:
                    dt = fps_times[-1] - fps_times[0]
                    self._camera_fps = (len(fps_times) - 1) / dt if dt > 0 else 0

                time.sleep(0.03)  # ~30fps cap

            cap.release()

        t = threading.Thread(target=reader, daemon=True, name='camera-reader')
        t.start()

    def get_camera_jpeg(self):
        """Get latest camera frame as JPEG bytes, or None."""
        with self._jpeg_lock:
            return self._latest_jpeg

    def get_face_data(self):
        """Get latest face detection results."""
        if self.face_data is None:
            return {'faces': [], 'count': 0, 'connected': False}
        faces, ts, count = self.face_data.get()
        return {
            'faces': [{'cx': f[0], 'cy': f[1], 'w': f[2], 'h': f[3]}
                       for f in faces] if faces else [],
            'count': len(faces) if faces else 0,
            'connected': self.camera is not None,
        }

    # -- IMU -------------------------------------------------------------------

    def _init_imu(self):
        if self.simulate:
            logger.info('IMU: simulation mode')
            return

        try:
            from demos_common.sensors import setup_imu
            self.imu = setup_imu()
            if self.imu:
                logger.info('IMU: MPU6050 connected')
            else:
                logger.info('IMU: not found (optional)')
        except Exception as e:
            logger.warning('IMU unavailable: %s', e)

    # -- Telemetry -------------------------------------------------------------

    def get_telemetry(self):
        """Get full system telemetry dict."""
        import os
        uptime = time.time() - self._start_time

        # CPU load
        try:
            load1, load5, load15 = os.getloadavg()
        except OSError:
            load1 = load5 = load15 = 0.0

        # Memory
        try:
            with open('/proc/meminfo') as f:
                lines = f.readlines()
            mem = {}
            for line in lines[:5]:
                parts = line.split()
                mem[parts[0].rstrip(':')] = int(parts[1])
            total = mem.get('MemTotal', 1)
            avail = mem.get('MemAvailable', total)
            mem_pct = 100.0 * (1 - avail / total)
        except Exception:
            mem_pct = 0.0

        motor = self.get_motor_status()
        return {
            'uptime': uptime,
            'cpu_load': round(load1, 2),
            'mem_pct': round(mem_pct, 1),
            'motor': motor,
            'lidar': {
                'connected': self.lidar is not None,
                'scan_rate': round(self._scan_rate, 1),
            },
            'camera': {
                'connected': self.camera is not None,
                'fps': round(self._camera_fps, 1),
            },
            'imu': {
                'connected': self.imu is not None,
            },
            'sensors': {
                'lidar': self.lidar is not None,
                'camera': self.camera is not None,
                'imu': self.imu is not None,
            },
        }

    # -- Cleanup ---------------------------------------------------------------

    def cleanup(self):
        """Shutdown all hardware."""
        logger.info('Cleaning up hardware...')
        self._running = False

        # Stop motors first
        if self.robot:
            try:
                self.robot.stop()
            except Exception:
                pass

        # Disconnect LiDAR
        if self.lidar:
            try:
                self.lidar.disconnect()
            except Exception:
                pass

        # Release camera
        if self.camera:
            try:
                self.camera.release()
            except Exception:
                pass

        logger.info('Hardware cleanup complete')
