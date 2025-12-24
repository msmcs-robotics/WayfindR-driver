"""
Camera Module
Handles webcam capture with cross-platform support and comprehensive debugging
"""

import cv2
import sys
import platform
import subprocess
import os


def get_system_info():
    """
    Get comprehensive system information for debugging

    Returns:
        dict with system details
    """
    info = {
        "platform": sys.platform,
        "platform_detail": platform.platform(),
        "python_version": platform.python_version(),
        "opencv_version": cv2.__version__,
        "machine": platform.machine(),
        "processor": platform.processor(),
        "is_wsl": False,
        "is_wsl2": False,
        "wsl_version": None,
    }

    # Detect WSL
    if sys.platform == "linux":
        try:
            with open("/proc/version", "r") as f:
                version_info = f.read().lower()
                if "microsoft" in version_info or "wsl" in version_info:
                    info["is_wsl"] = True
                    if "wsl2" in version_info:
                        info["is_wsl2"] = True
                        info["wsl_version"] = "WSL2"
                    else:
                        info["wsl_version"] = "WSL1"
        except:
            pass

        # Alternative WSL detection
        if os.path.exists("/proc/sys/fs/binfmt_misc/WSLInterop"):
            info["is_wsl"] = True
            if not info["wsl_version"]:
                info["wsl_version"] = "WSL (version unknown)"

    return info


def detect_available_cameras(max_cameras=10):
    """
    Probe for available camera devices

    Args:
        max_cameras: Maximum number of camera indices to check

    Returns:
        List of (index, info_dict) tuples for working cameras
    """
    available = []

    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # Get camera properties
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            backend = cap.getBackendName()

            # Try to read a frame to confirm it actually works
            ret, frame = cap.read()

            available.append((i, {
                "width": width,
                "height": height,
                "fps": fps,
                "backend": backend,
                "can_read": ret,
                "frame_shape": frame.shape if ret else None
            }))
            cap.release()

    return available


def get_video_devices_linux():
    """
    Get video devices on Linux using v4l2

    Returns:
        List of device info dicts
    """
    devices = []

    # Check /dev/video* devices
    try:
        import glob
        video_devices = sorted(glob.glob("/dev/video*"))

        for dev in video_devices:
            device_info = {"path": dev, "name": None, "driver": None}

            # Try to get device info using v4l2-ctl
            try:
                result = subprocess.run(
                    ["v4l2-ctl", "-d", dev, "--info"],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if result.returncode == 0:
                    for line in result.stdout.split("\n"):
                        if "Card type" in line:
                            device_info["name"] = line.split(":")[-1].strip()
                        if "Driver name" in line:
                            device_info["driver"] = line.split(":")[-1].strip()
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass

            devices.append(device_info)
    except:
        pass

    return devices


def get_video_devices_windows():
    """
    Get video devices on Windows using DirectShow enumeration

    Returns:
        List of device names (limited info available without extra libs)
    """
    devices = []

    # Try using PowerShell to enumerate devices
    try:
        ps_command = """
        Get-PnpDevice -Class Camera -Status OK | Select-Object -ExpandProperty FriendlyName
        Get-PnpDevice -Class Image -Status OK | Select-Object -ExpandProperty FriendlyName
        """
        result = subprocess.run(
            ["powershell", "-Command", ps_command],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            for line in result.stdout.strip().split("\n"):
                line = line.strip()
                if line:
                    devices.append({"name": line})
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return devices


def get_video_devices_macos():
    """
    Get video devices on macOS using system_profiler

    Returns:
        List of device info dicts
    """
    devices = []

    try:
        result = subprocess.run(
            ["system_profiler", "SPCameraDataType"],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            current_device = {}
            for line in result.stdout.split("\n"):
                line = line.strip()
                if line.endswith(":") and not line.startswith("Camera"):
                    if current_device:
                        devices.append(current_device)
                    current_device = {"name": line[:-1]}
                elif "Model ID:" in line:
                    current_device["model_id"] = line.split(":")[-1].strip()
                elif "Unique ID:" in line:
                    current_device["unique_id"] = line.split(":")[-1].strip()
            if current_device:
                devices.append(current_device)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return devices


def print_camera_debug_info():
    """
    Print comprehensive camera debugging information
    """
    print("=" * 60)
    print("CAMERA DEBUG INFORMATION")
    print("=" * 60)
    print()

    # System Info
    print("[SYSTEM INFO]")
    print("-" * 40)
    sys_info = get_system_info()
    for key, value in sys_info.items():
        print(f"  {key}: {value}")
    print()

    # WSL Warning
    if sys_info["is_wsl"]:
        print("[WARNING - WSL DETECTED]")
        print("-" * 40)
        print("  You are running in Windows Subsystem for Linux (WSL).")
        print("  WSL does NOT have direct access to USB webcams!")
        print()
        print("  SOLUTIONS:")
        print("  1. Run this script directly on Windows (recommended)")
        print("  2. Use USB/IP to forward webcam (complex)")
        print("  3. Use a virtual camera with network streaming")
        print()
        print("  To run on Windows:")
        print("    1. Open PowerShell or CMD")
        print("    2. cd to this folder")
        print("    3. python main.py")
        print()

    # OS-specific device enumeration
    print("[VIDEO DEVICES - OS LEVEL]")
    print("-" * 40)

    if sys.platform == "linux":
        devices = get_video_devices_linux()
        if devices:
            for dev in devices:
                print(f"  {dev['path']}")
                if dev.get("name"):
                    print(f"    Name: {dev['name']}")
                if dev.get("driver"):
                    print(f"    Driver: {dev['driver']}")
        else:
            print("  No /dev/video* devices found")
            if sys_info["is_wsl"]:
                print("  (This is expected in WSL - webcams are on Windows)")

    elif sys.platform == "win32":
        devices = get_video_devices_windows()
        if devices:
            for dev in devices:
                print(f"  - {dev['name']}")
        else:
            print("  Could not enumerate Windows devices")

    elif sys.platform == "darwin":
        devices = get_video_devices_macos()
        if devices:
            for dev in devices:
                print(f"  - {dev.get('name', 'Unknown')}")
                if dev.get("model_id"):
                    print(f"    Model: {dev['model_id']}")
        else:
            print("  No cameras found via system_profiler")

    else:
        print(f"  Unknown platform: {sys.platform}")

    print()

    # OpenCV camera probing
    print("[OPENCV CAMERA PROBE]")
    print("-" * 40)
    print("  Probing camera indices 0-9...")
    print()

    cameras = detect_available_cameras(max_cameras=10)

    if cameras:
        for idx, info in cameras:
            status = "OK" if info["can_read"] else "OPEN but NO FRAMES"
            print(f"  Camera {idx}: {status}")
            print(f"    Resolution: {info['width']}x{info['height']}")
            print(f"    FPS: {info['fps']}")
            print(f"    Backend: {info['backend']}")
            if info["frame_shape"]:
                print(f"    Frame shape: {info['frame_shape']}")
            print()
    else:
        print("  No cameras detected by OpenCV!")
        print()
        if sys_info["is_wsl"]:
            print("  This is expected in WSL. Run on Windows instead.")
        else:
            print("  Possible causes:")
            print("    - No webcam connected")
            print("    - Webcam in use by another application")
            print("    - Missing camera drivers")
            print("    - Permission denied (try running as admin/root)")

    print()
    print("=" * 60)
    print("END DEBUG INFO")
    print("=" * 60)
    print()

    return sys_info, cameras


class Camera:
    def __init__(self, camera_index=0, width=640, height=480):
        """
        Initialize camera capture

        Args:
            camera_index: Camera device index (0 = default webcam)
            width: Desired frame width
            height: Desired frame height
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None
        self.backend_name = None

    def open(self, verbose=True):
        """
        Open the camera

        Args:
            verbose: Print debug info on failure

        Returns:
            True if successful, False otherwise
        """
        # Use DirectShow backend on Windows for better compatibility
        if sys.platform == 'win32':
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            if verbose:
                print(f"Failed to open camera {self.camera_index}")
            return False

        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Store backend name
        self.backend_name = self.cap.getBackendName()

        # Verify we can actually read frames
        ret, frame = self.cap.read()
        if not ret:
            if verbose:
                print(f"Camera {self.camera_index} opened but cannot read frames")
            self.cap.release()
            self.cap = None
            return False

        return True

    def read(self):
        """
        Read a frame from the camera

        Returns:
            (success, frame) tuple
        """
        if self.cap is None:
            return False, None

        return self.cap.read()

    def get_dimensions(self):
        """
        Get actual frame dimensions

        Returns:
            (width, height) tuple
        """
        if self.cap is None:
            return self.width, self.height

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def get_info(self):
        """
        Get camera information

        Returns:
            dict with camera properties
        """
        if self.cap is None:
            return {"status": "not opened"}

        return {
            "status": "opened",
            "index": self.camera_index,
            "width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": self.cap.get(cv2.CAP_PROP_FPS),
            "backend": self.backend_name,
            "brightness": self.cap.get(cv2.CAP_PROP_BRIGHTNESS),
            "contrast": self.cap.get(cv2.CAP_PROP_CONTRAST),
            "saturation": self.cap.get(cv2.CAP_PROP_SATURATION),
            "exposure": self.cap.get(cv2.CAP_PROP_EXPOSURE),
            "autofocus": self.cap.get(cv2.CAP_PROP_AUTOFOCUS),
        }

    def release(self):
        """
        Release the camera
        """
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# Allow running this module directly for debugging
if __name__ == "__main__":
    print()
    print("Running camera diagnostics...")
    print()
    sys_info, cameras = print_camera_debug_info()

    if cameras and not sys_info["is_wsl"]:
        print("Attempting to open first available camera...")
        cam = Camera(camera_index=cameras[0][0])
        if cam.open():
            print("SUCCESS! Camera opened.")
            print(f"Camera info: {cam.get_info()}")
            cam.release()
        else:
            print("Failed to open camera.")
