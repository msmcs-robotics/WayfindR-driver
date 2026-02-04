#!/usr/bin/env python3
"""
AMBOT Live Monitor - Terminal-Based Sensor Monitoring

Continuously monitors LiDAR, camera, and system resources with terminal output.
No GUI required - runs over SSH without X11 forwarding.

Features:
    - LiDAR status: connection, scan rate, nearest/furthest points, sectors
    - Camera status: connection, frame rate, face detection with centers
    - System status: CPU usage, RAM usage, GPU usage (Jetson), network status

Usage:
    python3 live_monitor.py                    # Monitor both sensors + system
    python3 live_monitor.py --lidar-only       # LiDAR only
    python3 live_monitor.py --camera-only      # Camera only
    python3 live_monitor.py --log events.log   # Save events to log file
    python3 live_monitor.py --json             # JSON output for scripts

Controls:
    Ctrl+C - Stop monitoring
"""

import argparse
import json
import logging
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

# Add parent for imports
sys.path.insert(0, str(Path(__file__).parent))

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class Event:
    """Represents a monitoring event."""

    def __init__(self, event_type: str, source: str, data: dict = None):
        self.timestamp = datetime.now()
        self.event_type = event_type
        self.source = source
        self.data = data or {}

    def __str__(self):
        data_str = ", ".join(f"{k}={v}" for k, v in self.data.items())
        return f"{self.timestamp.strftime('%H:%M:%S')} [{self.source}] {self.event_type}: {data_str}"

    def to_dict(self):
        return {
            "timestamp": self.timestamp.isoformat(),
            "type": self.event_type,
            "source": self.source,
            "data": self.data,
        }


class EventBus:
    """Simple event queue for inter-thread communication."""

    def __init__(self, max_events: int = 100):
        self.events = deque(maxlen=max_events)
        self.lock = threading.Lock()
        self.callbacks = []

    def publish(self, event: Event):
        with self.lock:
            self.events.append(event)
        for callback in self.callbacks:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Event callback error: {e}")

    def subscribe(self, callback):
        self.callbacks.append(callback)

    def get_recent(self, count: int = 10):
        with self.lock:
            return list(self.events)[-count:]


class SystemMonitor:
    """Monitors system resources (CPU, RAM, GPU, network)."""

    def __init__(self):
        self._cpu_count = os.cpu_count() or 1
        self._last_cpu_times = None
        self._last_cpu_time = 0

    def get_cpu_usage(self) -> float:
        """Get CPU usage percentage (0-100) from /proc/stat."""
        try:
            with open('/proc/stat', 'r') as f:
                line = f.readline()
                parts = line.split()
                # user, nice, system, idle, iowait, irq, softirq
                if len(parts) >= 5:
                    user = int(parts[1])
                    nice = int(parts[2])
                    system = int(parts[3])
                    idle = int(parts[4])

                    total = user + nice + system + idle
                    active = user + nice + system

                    if self._last_cpu_times:
                        last_total, last_active = self._last_cpu_times
                        diff_total = total - last_total
                        diff_active = active - last_active
                        if diff_total > 0:
                            usage = (diff_active / diff_total) * 100
                        else:
                            usage = 0
                    else:
                        usage = 0

                    self._last_cpu_times = (total, active)
                    return usage
        except:
            pass
        return 0.0

    def get_memory_usage(self) -> dict:
        """Get memory usage from /proc/meminfo."""
        mem = {"total_mb": 0, "used_mb": 0, "percent": 0}
        try:
            with open('/proc/meminfo', 'r') as f:
                meminfo = {}
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        key = parts[0].rstrip(':')
                        meminfo[key] = int(parts[1])  # in KB

                total = meminfo.get('MemTotal', 0)
                available = meminfo.get('MemAvailable', meminfo.get('MemFree', 0))
                used = total - available

                mem['total_mb'] = total // 1024
                mem['used_mb'] = used // 1024
                mem['percent'] = (used / total * 100) if total > 0 else 0
        except:
            pass
        return mem

    def get_gpu_usage(self) -> dict:
        """Get GPU usage (Jetson-specific via tegrastats or sysfs)."""
        gpu = {"available": False, "percent": 0, "temp_c": 0}

        # Try Jetson GPU load from sysfs
        jetson_gpu_paths = [
            "/sys/devices/gpu.0/load",  # Older Jetsons
            "/sys/devices/platform/gpu.0/load",
            "/sys/devices/17000000.ga10b/load",  # Orin
            "/sys/devices/17000000.gv11b/load",
        ]

        for path in jetson_gpu_paths:
            try:
                with open(path, 'r') as f:
                    load = int(f.read().strip())
                    gpu['available'] = True
                    gpu['percent'] = load / 10.0  # Value is in 0.1% units
                    break
            except:
                continue

        # Try to get GPU temperature
        temp_paths = [
            "/sys/class/thermal/thermal_zone1/temp",
            "/sys/class/thermal/thermal_zone2/temp",
        ]

        for path in temp_paths:
            try:
                with open(path, 'r') as f:
                    temp = int(f.read().strip())
                    if temp > 1000:  # millidegrees
                        temp = temp // 1000
                    gpu['temp_c'] = temp
                    break
            except:
                continue

        return gpu

    def get_network_status(self) -> dict:
        """Check network interface status."""
        net = {"connected": False, "interface": None, "ip": None}

        try:
            import socket
            # Try to get the IP by connecting (doesn't actually send data)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            try:
                s.connect(('8.8.8.8', 80))
                net['ip'] = s.getsockname()[0]
                net['connected'] = True
            except:
                pass
            finally:
                s.close()
        except:
            pass

        # Find active interface
        try:
            with open('/proc/net/route', 'r') as f:
                for line in f.readlines()[1:]:  # Skip header
                    parts = line.split()
                    if len(parts) >= 2 and parts[1] == '00000000':  # Default route
                        net['interface'] = parts[0]
                        break
        except:
            pass

        return net

    def get_status(self) -> dict:
        """Get all system status."""
        return {
            "cpu_percent": self.get_cpu_usage(),
            "memory": self.get_memory_usage(),
            "gpu": self.get_gpu_usage(),
            "network": self.get_network_status(),
        }


class LidarMonitor(threading.Thread):
    """Monitors LiDAR sensor in background thread."""

    def __init__(self, event_bus: EventBus, port: str = "/dev/ttyUSB0"):
        super().__init__(daemon=True)
        self.event_bus = event_bus
        self.port = port
        self.running = False

        # State
        self.connected = False
        self.scan_count = 0
        self.last_scan_time = 0
        self.scan_rate = 0
        self.sectors = {}
        self.nearest_distance = float('inf')
        self.nearest_angle = 0
        self.furthest_distance = 0
        self.furthest_angle = 0

    def run(self):
        self.running = True

        try:
            from pathfinder.lidar_ld19 import LD19Lidar
            from pathfinder.obstacle_detector import SectorBasedDetector

            lidar = LD19Lidar(port=self.port)
            detector = SectorBasedDetector()

            if not lidar.connect():
                self.event_bus.publish(Event("error", "lidar", {"message": "Connection failed"}))
                return

            self.connected = True
            self.event_bus.publish(Event("connected", "lidar", {"port": self.port}))

            scan_times = deque(maxlen=10)

            for scan in lidar.iter_scans(min_points=100):
                if not self.running:
                    break

                current_time = time.time()
                self.scan_count += 1

                # Calculate scan rate
                scan_times.append(current_time)
                if len(scan_times) > 1:
                    self.scan_rate = len(scan_times) / (scan_times[-1] - scan_times[0])

                # Find nearest and furthest points
                nearest_dist = float('inf')
                nearest_ang = 0
                furthest_dist = 0
                furthest_ang = 0

                for point in scan:
                    if point.distance > 50:  # Filter noise
                        if point.distance < nearest_dist:
                            nearest_dist = point.distance
                            nearest_ang = point.angle
                        if point.distance > furthest_dist:
                            furthest_dist = point.distance
                            furthest_ang = point.angle

                self.nearest_distance = nearest_dist
                self.nearest_angle = nearest_ang
                self.furthest_distance = furthest_dist
                self.furthest_angle = furthest_ang

                # Process through detector
                detection = detector.process_scan(scan)
                self.sectors = {
                    name: {"distance": s.min_distance, "safety": s.safety_level.name}
                    for name, s in detection.sectors.items()
                }

                # Emit events based on safety level
                if detection.overall_safety.name == "STOP":
                    self.event_bus.publish(Event("obstacle_stop", "lidar", {
                        "sector": detection.closest_sector,
                        "distance": f"{detection.closest_distance:.2f}m",
                    }))
                elif detection.overall_safety.name == "SLOW":
                    self.event_bus.publish(Event("obstacle_slow", "lidar", {
                        "sector": detection.closest_sector,
                        "distance": f"{detection.closest_distance:.2f}m",
                    }))

                self.last_scan_time = current_time

            lidar.disconnect()

        except ImportError as e:
            self.event_bus.publish(Event("error", "lidar", {"message": f"Import error: {e}"}))
        except Exception as e:
            self.event_bus.publish(Event("error", "lidar", {"message": str(e)}))
        finally:
            self.connected = False
            self.running = False

    def stop(self):
        self.running = False

    def get_status(self) -> dict:
        return {
            "connected": self.connected,
            "scans": self.scan_count,
            "rate_hz": f"{self.scan_rate:.1f}",
            "nearest": f"{self.nearest_distance:.0f}mm @ {self.nearest_angle:.0f}°",
            "furthest": f"{self.furthest_distance:.0f}mm @ {self.furthest_angle:.0f}°",
            "sectors": self.sectors,
        }


class CameraMonitor(threading.Thread):
    """Monitors camera for face detection in background thread."""

    def __init__(self, event_bus: EventBus, device: int = 0):
        super().__init__(daemon=True)
        self.event_bus = event_bus
        self.device = device
        self.running = False

        # State
        self.connected = False
        self.frame_count = 0
        self.face_count = 0
        self.last_face_count = 0
        self.face_centers = []
        self.fps = 0

    def run(self):
        self.running = True

        try:
            import cv2

            cap = cv2.VideoCapture(self.device)
            if not cap.isOpened():
                self.event_bus.publish(Event("error", "camera", {"message": "Failed to open camera"}))
                return

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            self.connected = True
            self.event_bus.publish(Event("connected", "camera", {"device": self.device}))

            # Load face cascade
            cascade_paths = [
                "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
                "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
            ]

            # Try cv2 data path
            try:
                cv2_path = Path(cv2.__file__).parent / "data" / "haarcascade_frontalface_default.xml"
                cascade_paths.insert(0, str(cv2_path))
            except:
                pass

            face_cascade = None
            for path in cascade_paths:
                if Path(path).exists():
                    face_cascade = cv2.CascadeClassifier(path)
                    if not face_cascade.empty():
                        break

            if face_cascade is None or face_cascade.empty():
                self.event_bus.publish(Event("warning", "camera", {"message": "Face cascade not found"}))

            frame_times = deque(maxlen=30)

            while self.running:
                ret, frame = cap.read()
                if not ret:
                    continue

                current_time = time.time()
                self.frame_count += 1

                # Calculate FPS
                frame_times.append(current_time)
                if len(frame_times) > 1:
                    self.fps = len(frame_times) / (frame_times[-1] - frame_times[0])

                # Face detection
                if face_cascade is not None and not face_cascade.empty():
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    faces = face_cascade.detectMultiScale(
                        gray,
                        scaleFactor=1.1,
                        minNeighbors=5,
                        minSize=(30, 30)
                    )

                    self.face_count = len(faces)
                    self.face_centers = []

                    for (x, y, w, h) in faces:
                        center_x = x + w // 2
                        center_y = y + h // 2
                        self.face_centers.append((center_x, center_y))

                    # Emit face events
                    if self.face_count > 0 and self.last_face_count == 0:
                        self.event_bus.publish(Event("face_detected", "camera", {
                            "count": self.face_count,
                            "centers": self.face_centers,
                        }))
                    elif self.face_count == 0 and self.last_face_count > 0:
                        self.event_bus.publish(Event("face_lost", "camera", {}))
                    elif self.face_count != self.last_face_count:
                        self.event_bus.publish(Event("face_count_changed", "camera", {
                            "count": self.face_count,
                            "prev": self.last_face_count,
                        }))

                    self.last_face_count = self.face_count

                # Rate limit to ~10 FPS for detection
                time.sleep(0.1)

            cap.release()

        except ImportError as e:
            self.event_bus.publish(Event("error", "camera", {"message": f"Import error: {e}"}))
        except Exception as e:
            self.event_bus.publish(Event("error", "camera", {"message": str(e)}))
        finally:
            self.connected = False
            self.running = False

    def stop(self):
        self.running = False

    def get_status(self) -> dict:
        return {
            "connected": self.connected,
            "frames": self.frame_count,
            "fps": f"{self.fps:.1f}",
            "faces": self.face_count,
            "centers": self.face_centers,
        }


class TerminalDisplay:
    """Terminal-based display with ANSI colors and formatting."""

    CLEAR_SCREEN = "\033[2J\033[H"
    BOLD = "\033[1m"
    RESET = "\033[0m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    RED = "\033[31m"
    BLUE = "\033[34m"
    CYAN = "\033[36m"
    MAGENTA = "\033[35m"

    def __init__(self, event_bus: EventBus, lidar: LidarMonitor = None,
                 camera: CameraMonitor = None, system: SystemMonitor = None):
        self.event_bus = event_bus
        self.lidar = lidar
        self.camera = camera
        self.system = system or SystemMonitor()
        self.start_time = time.time()
        self.running = False

    def format_safety(self, level: str) -> str:
        colors = {
            "CLEAR": self.GREEN,
            "WARN": self.YELLOW,
            "SLOW": self.YELLOW,
            "STOP": self.RED,
        }
        color = colors.get(level, self.RESET)
        return f"{color}{level:5s}{self.RESET}"

    def format_status(self, connected: bool) -> str:
        if connected:
            return f"{self.GREEN}✓{self.RESET}"
        return f"{self.RED}✗{self.RESET}"

    def render(self) -> str:
        elapsed = time.time() - self.start_time
        elapsed_str = f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d}"

        lines = []
        lines.append(f"{self.BOLD}┌{'─' * 60}┐{self.RESET}")
        lines.append(f"{self.BOLD}│{self.CYAN} AMBOT LIVE MONITOR{self.RESET}{' ' * 23}{self.BOLD}[{elapsed_str}] │{self.RESET}")
        lines.append(f"{self.BOLD}├{'─' * 60}┤{self.RESET}")

        # LiDAR status
        if self.lidar:
            status = self.lidar.get_status()
            conn = self.format_status(status['connected'])
            lines.append(f"{self.BOLD}│{self.RESET} LiDAR: {conn}  Scans: {status['scans']:5d}  Rate: {status['rate_hz']} Hz{' ' * 15}{self.BOLD}│{self.RESET}")
            lines.append(f"{self.BOLD}│{self.RESET}   Nearest:  {status['nearest']:25s}{' ' * 18}{self.BOLD}│{self.RESET}")
            lines.append(f"{self.BOLD}│{self.RESET}   Furthest: {status['furthest']:25s}{' ' * 18}{self.BOLD}│{self.RESET}")

            # Sectors
            if status['sectors']:
                sectors_line = "   "
                for name, data in status['sectors'].items():
                    dist = data['distance']
                    safety = self.format_safety(data['safety'])
                    if dist == float('inf'):
                        dist_str = "∞"
                    else:
                        dist_str = f"{dist:.1f}m"
                    sectors_line += f"{name[0].upper()}:{dist_str}[{safety}] "

                # Pad to fit box
                sectors_line = sectors_line[:58]
                lines.append(f"{self.BOLD}│{self.RESET}{sectors_line}{' ' * (59 - len(sectors_line))}{self.BOLD}│{self.RESET}")

        # Camera status
        if self.camera:
            status = self.camera.get_status()
            conn = self.format_status(status['connected'])
            face_color = self.GREEN if status['faces'] > 0 else self.RESET
            lines.append(f"{self.BOLD}│{self.RESET} Camera: {conn}  Frames: {status['frames']:5d}  FPS: {status['fps']}{' ' * 16}{self.BOLD}│{self.RESET}")
            lines.append(f"{self.BOLD}│{self.RESET}   Faces: {face_color}{status['faces']}{self.RESET}  Centers: {str(status['centers'])[:35]:35s}{self.BOLD}│{self.RESET}")

        lines.append(f"{self.BOLD}├{'─' * 60}┤{self.RESET}")

        # System status
        if self.system:
            sys_status = self.system.get_status()

            # CPU & Memory
            cpu = sys_status['cpu_percent']
            mem = sys_status['memory']
            cpu_color = self.RED if cpu > 80 else self.YELLOW if cpu > 50 else self.GREEN
            mem_color = self.RED if mem['percent'] > 80 else self.YELLOW if mem['percent'] > 50 else self.GREEN

            lines.append(f"{self.BOLD}│{self.CYAN} SYSTEM{self.RESET}{' ' * 53}{self.BOLD}│{self.RESET}")
            lines.append(f"{self.BOLD}│{self.RESET}   CPU: {cpu_color}{cpu:5.1f}%{self.RESET}  RAM: {mem_color}{mem['used_mb']:4d}/{mem['total_mb']}MB ({mem['percent']:.0f}%){self.RESET}{' ' * 14}{self.BOLD}│{self.RESET}")

            # GPU (if available)
            gpu = sys_status['gpu']
            if gpu['available']:
                gpu_color = self.RED if gpu['percent'] > 80 else self.YELLOW if gpu['percent'] > 50 else self.GREEN
                lines.append(f"{self.BOLD}│{self.RESET}   GPU: {gpu_color}{gpu['percent']:5.1f}%{self.RESET}  Temp: {gpu['temp_c']}°C{' ' * 33}{self.BOLD}│{self.RESET}")

            # Network
            net = sys_status['network']
            net_status = self.format_status(net['connected'])
            net_info = f"{net['interface'] or 'N/A'}: {net['ip'] or 'No IP'}"
            lines.append(f"{self.BOLD}│{self.RESET}   Network: {net_status}  {net_info[:40]:40s}{self.BOLD}│{self.RESET}")

        lines.append(f"{self.BOLD}├{'─' * 60}┤{self.RESET}")

        # Recent events
        lines.append(f"{self.BOLD}│{self.RESET} EVENTS (last 5){' ' * 43}{self.BOLD}│{self.RESET}")
        events = self.event_bus.get_recent(5)
        for event in events[-5:]:
            event_str = str(event)[:58]
            lines.append(f"{self.BOLD}│{self.RESET}   {event_str}{' ' * (57 - len(event_str))}{self.BOLD}│{self.RESET}")

        # Pad if fewer events
        for _ in range(5 - len(events)):
            lines.append(f"{self.BOLD}│{self.RESET}{' ' * 60}{self.BOLD}│{self.RESET}")

        lines.append(f"{self.BOLD}└{'─' * 60}┘{self.RESET}")
        lines.append(f" Press {self.BOLD}Ctrl+C{self.RESET} to stop")

        return "\n".join(lines)

    def run(self, refresh_rate: float = 0.5):
        self.running = True
        try:
            while self.running:
                # Clear and redraw
                print(self.CLEAR_SCREEN + self.render(), end="", flush=True)
                time.sleep(refresh_rate)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            print()  # New line after display


class JsonOutput:
    """JSON output mode for programmatic consumption."""

    def __init__(self, event_bus: EventBus, lidar: LidarMonitor = None,
                 camera: CameraMonitor = None, system: SystemMonitor = None):
        self.event_bus = event_bus
        self.lidar = lidar
        self.camera = camera
        self.system = system or SystemMonitor()
        self.running = False

        # Subscribe to events
        event_bus.subscribe(self.on_event)

    def on_event(self, event: Event):
        if self.running:
            print(json.dumps(event.to_dict()))

    def run(self, refresh_rate: float = 1.0):
        self.running = True
        try:
            while self.running:
                # Periodic status output
                status = {"type": "status", "timestamp": datetime.now().isoformat()}
                if self.lidar:
                    status["lidar"] = self.lidar.get_status()
                if self.camera:
                    status["camera"] = self.camera.get_status()
                if self.system:
                    status["system"] = self.system.get_status()
                print(json.dumps(status))
                time.sleep(refresh_rate)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False


def main():
    parser = argparse.ArgumentParser(
        description="AMBOT Live Monitor - Terminal-based sensor monitoring",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 live_monitor.py                 # Monitor both sensors
  python3 live_monitor.py --lidar-only    # LiDAR only
  python3 live_monitor.py --camera-only   # Camera only
  python3 live_monitor.py --json          # JSON output for scripts
        """
    )

    parser.add_argument("--lidar-only", action="store_true", help="Monitor LiDAR only")
    parser.add_argument("--camera-only", action="store_true", help="Monitor camera only")
    parser.add_argument("--lidar-port", default="/dev/ttyUSB0", help="LiDAR serial port")
    parser.add_argument("--camera-device", type=int, default=0, help="Camera device number")
    parser.add_argument("--json", action="store_true", help="Output JSON format")
    parser.add_argument("--refresh", type=float, default=0.5, help="Display refresh rate (seconds)")
    parser.add_argument("--log", type=str, help="Log events to file")

    args = parser.parse_args()

    # Setup file logging if requested
    if args.log:
        file_handler = logging.FileHandler(args.log)
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
        logging.getLogger().addHandler(file_handler)

    # Determine what to monitor
    monitor_lidar = not args.camera_only
    monitor_camera = not args.lidar_only

    if not args.json:
        print("Starting AMBOT Live Monitor...")
        print(f"  LiDAR: {'enabled' if monitor_lidar else 'disabled'}")
        print(f"  Camera: {'enabled' if monitor_camera else 'disabled'}")
        print()

    # Create event bus
    event_bus = EventBus()

    # Create monitors
    lidar_monitor = None
    camera_monitor = None

    if monitor_lidar:
        lidar_monitor = LidarMonitor(event_bus, port=args.lidar_port)
        lidar_monitor.start()

    if monitor_camera:
        camera_monitor = CameraMonitor(event_bus, device=args.camera_device)
        camera_monitor.start()

    # Wait for initial connections
    time.sleep(1)

    # Create system monitor
    system_monitor = SystemMonitor()

    # Create display
    if args.json:
        display = JsonOutput(event_bus, lidar_monitor, camera_monitor, system_monitor)
    else:
        display = TerminalDisplay(event_bus, lidar_monitor, camera_monitor, system_monitor)

    try:
        display.run(refresh_rate=args.refresh)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop monitors
        if lidar_monitor:
            lidar_monitor.stop()
        if camera_monitor:
            camera_monitor.stop()

        if not args.json:
            print("\nStopping monitors...")

            # Wait for threads to finish
            if lidar_monitor:
                lidar_monitor.join(timeout=2)
            if camera_monitor:
                camera_monitor.join(timeout=2)

            print("Monitor stopped.")


if __name__ == "__main__":
    main()
