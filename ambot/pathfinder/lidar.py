"""
Pathfinder LiDAR Module

Core LiDAR communication class for RPLidar C1M1 via direct serial communication.
Based on patterns from:
- ros_tank_xiaor/lidar_youyeetoo_visual_plotter.py
- old_stuff/rplidar_setup/ scripts

This module provides low-level LiDAR control without requiring ROS or the
adafruit_rplidar library, using direct serial protocol communication.
"""

import serial
import struct
import time
import os
from typing import Optional, List, Dict, Tuple, Generator
from dataclasses import dataclass

from . import config


@dataclass
class ScanPoint:
    """Single LiDAR scan point."""
    angle: float      # Angle in degrees (0-360)
    distance: float   # Distance in millimeters
    quality: int      # Signal quality (0-255)

    @property
    def distance_m(self) -> float:
        """Distance in meters."""
        return self.distance / 1000.0

    @property
    def angle_rad(self) -> float:
        """Angle in radians."""
        import math
        return math.radians(self.angle)


class RPLidarException(Exception):
    """Exception for RPLidar communication errors."""
    pass


class RPLidar:
    """
    RPLidar direct serial communication class.

    Supports RPLidar C1M1 and compatible models via USB serial interface.

    Usage:
        lidar = RPLidar()
        lidar.connect()
        for scan in lidar.iter_scans():
            for point in scan:
                print(f"Angle: {point.angle:.1f}, Distance: {point.distance:.0f}mm")
        lidar.disconnect()
    """

    # Protocol constants
    SYNC_BYTE = 0xA5
    SYNC_BYTE2 = 0x5A

    # Commands
    CMD_STOP = 0x25
    CMD_RESET = 0x40
    CMD_SCAN = 0x20
    CMD_EXPRESS_SCAN = 0x82
    CMD_FORCE_SCAN = 0x21
    CMD_GET_INFO = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_GET_SAMPLERATE = 0x59
    CMD_SET_MOTOR_PWM = 0xF0

    # Response descriptors
    RESP_INFO = 0x04
    RESP_HEALTH = 0x06
    RESP_SCAN = 0x81

    # Health status
    HEALTH_GOOD = 0
    HEALTH_WARNING = 1
    HEALTH_ERROR = 2

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: Optional[int] = None,
        timeout: Optional[float] = None
    ):
        """
        Initialize RPLidar.

        Args:
            port: Serial port path. Defaults to config.SERIAL_PORT
            baudrate: Baud rate. Defaults to config.BAUD_RATE
            timeout: Serial timeout. Defaults to config.SERIAL_TIMEOUT
        """
        self.port = port or self._find_port()
        self.baudrate = baudrate or config.BAUD_RATE
        self.timeout = timeout or config.SERIAL_TIMEOUT

        self._serial: Optional[serial.Serial] = None
        self._motor_running = False
        self._scanning = False

    def _find_port(self) -> str:
        """Find the LiDAR serial port."""
        # Try primary port first
        if os.path.exists(config.SERIAL_PORT):
            return config.SERIAL_PORT

        # Try fallback
        if os.path.exists(config.SERIAL_PORT_FALLBACK):
            return config.SERIAL_PORT_FALLBACK

        # Search for ttyUSB devices
        for i in range(10):
            path = f"/dev/ttyUSB{i}"
            if os.path.exists(path):
                return path

        return config.SERIAL_PORT  # Return default even if not found

    @property
    def is_connected(self) -> bool:
        """Check if connected to LiDAR."""
        return self._serial is not None and self._serial.is_open

    @property
    def motor_running(self) -> bool:
        """Check if motor is running."""
        return self._motor_running

    def connect(self) -> bool:
        """
        Connect to the LiDAR.

        Returns:
            True if connection successful

        Raises:
            RPLidarException: If connection fails
        """
        if self.is_connected:
            return True

        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            # Clear any pending data
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            # Give device time to initialize
            time.sleep(0.1)

            if config.DEBUG:
                print(f"Connected to LiDAR on {self.port} @ {self.baudrate} baud")

            return True

        except serial.SerialException as e:
            raise RPLidarException(f"Failed to connect to {self.port}: {e}")

    def disconnect(self):
        """Disconnect from the LiDAR."""
        if self._scanning:
            self.stop_scan()

        if self._motor_running:
            self.stop_motor()

        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

        if config.DEBUG:
            print("Disconnected from LiDAR")

    def _send_command(self, cmd: int, payload: bytes = b""):
        """Send a command to the LiDAR."""
        if not self.is_connected:
            raise RPLidarException("Not connected")

        # Build command packet
        packet = bytes([self.SYNC_BYTE, cmd])

        if payload:
            packet += bytes([len(payload)]) + payload
            # Calculate checksum
            checksum = 0
            for b in packet:
                checksum ^= b
            packet += bytes([checksum])

        self._serial.write(packet)

    def _read_response_descriptor(self) -> Tuple[int, int, int]:
        """
        Read response descriptor.

        Returns:
            Tuple of (data_length, send_mode, data_type)
        """
        descriptor = self._serial.read(7)

        if len(descriptor) != 7:
            raise RPLidarException("Timeout reading response descriptor")

        if descriptor[0] != self.SYNC_BYTE or descriptor[1] != self.SYNC_BYTE2:
            raise RPLidarException("Invalid response descriptor sync bytes")

        data_length = struct.unpack("<I", descriptor[2:6])[0] & 0x3FFFFFFF
        send_mode = (descriptor[5] >> 6) & 0x03
        data_type = descriptor[6]

        return data_length, send_mode, data_type

    def _read_response(self, length: int) -> bytes:
        """Read response data of specified length."""
        data = self._serial.read(length)
        if len(data) != length:
            raise RPLidarException(f"Timeout reading response: got {len(data)}/{length} bytes")
        return data

    def get_info(self) -> Dict:
        """
        Get device information.

        Returns:
            Dictionary with model, firmware, hardware, and serial number
        """
        self._send_command(self.CMD_GET_INFO)

        _, _, dtype = self._read_response_descriptor()
        if dtype != self.RESP_INFO:
            raise RPLidarException(f"Unexpected response type: {dtype}")

        data = self._read_response(20)

        return {
            "model": data[0],
            "firmware_minor": data[1],
            "firmware_major": data[2],
            "hardware": data[3],
            "serial": "".join(f"{b:02X}" for b in data[4:20])
        }

    def get_health(self) -> Tuple[str, int]:
        """
        Get device health status.

        Returns:
            Tuple of (status_string, error_code)
        """
        self._send_command(self.CMD_GET_HEALTH)

        _, _, dtype = self._read_response_descriptor()
        if dtype != self.RESP_HEALTH:
            raise RPLidarException(f"Unexpected response type: {dtype}")

        data = self._read_response(3)

        status_code = data[0]
        error_code = struct.unpack("<H", data[1:3])[0]

        status_map = {
            self.HEALTH_GOOD: "Good",
            self.HEALTH_WARNING: "Warning",
            self.HEALTH_ERROR: "Error"
        }

        return status_map.get(status_code, "Unknown"), error_code

    def start_motor(self):
        """Start the LiDAR motor."""
        if not self.is_connected:
            raise RPLidarException("Not connected")

        # DTR controls motor on most RPLidars
        self._serial.dtr = False
        time.sleep(0.1)
        self._motor_running = True

        if config.DEBUG:
            print("Motor started")

    def stop_motor(self):
        """Stop the LiDAR motor."""
        if not self.is_connected:
            return

        self._serial.dtr = True
        time.sleep(0.1)
        self._motor_running = False

        if config.DEBUG:
            print("Motor stopped")

    def start_scan(self):
        """Start scanning."""
        if not self.is_connected:
            raise RPLidarException("Not connected")

        if not self._motor_running:
            self.start_motor()

        self._send_command(self.CMD_SCAN)

        # Read and verify response descriptor
        _, _, dtype = self._read_response_descriptor()
        if dtype != self.RESP_SCAN:
            raise RPLidarException(f"Unexpected scan response type: {dtype}")

        self._scanning = True

        if config.DEBUG:
            print("Scanning started")

    def stop_scan(self):
        """Stop scanning."""
        if not self.is_connected:
            return

        self._send_command(self.CMD_STOP)
        time.sleep(0.01)
        self._serial.reset_input_buffer()
        self._scanning = False

        if config.DEBUG:
            print("Scanning stopped")

    def reset(self):
        """Reset the LiDAR."""
        if self.is_connected:
            self._send_command(self.CMD_RESET)
            time.sleep(0.5)
            self._serial.reset_input_buffer()
            self._motor_running = False
            self._scanning = False

    def _parse_scan_response(self) -> Optional[ScanPoint]:
        """Parse a single scan response packet."""
        data = self._serial.read(5)

        if len(data) != 5:
            return None

        # Check sync bits
        if not ((data[0] & 0x01) and (data[1] & 0x01)):
            return None

        # Parse data
        quality = data[0] >> 2
        angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0
        distance = (data[3] | (data[4] << 8)) / 4.0

        # Validate
        if distance < config.MIN_DISTANCE_MM or distance > config.MAX_DISTANCE_MM:
            if distance != 0:  # Zero distance means no reflection
                return None

        return ScanPoint(angle=angle, distance=distance, quality=quality)

    def iter_measurements(self, max_buf_meas: int = 500) -> Generator[ScanPoint, None, None]:
        """
        Iterate over individual measurements.

        Args:
            max_buf_meas: Maximum buffer size

        Yields:
            ScanPoint for each valid measurement
        """
        if not self._scanning:
            self.start_scan()

        while self._scanning:
            try:
                point = self._parse_scan_response()
                if point is not None:
                    yield point
            except serial.SerialException:
                break

    def iter_scans(self, max_buf_meas: int = 500) -> Generator[List[ScanPoint], None, None]:
        """
        Iterate over complete 360-degree scans.

        A scan is complete when angle wraps from high to low.

        Args:
            max_buf_meas: Maximum measurements per scan

        Yields:
            List of ScanPoints for each complete scan
        """
        scan: List[ScanPoint] = []
        prev_angle = 0.0

        for point in self.iter_measurements(max_buf_meas):
            # Detect scan boundary (angle wrap-around)
            if point.angle < prev_angle - 180:
                if scan:
                    yield scan
                    scan = []

            scan.append(point)
            prev_angle = point.angle

            # Prevent memory issues
            if len(scan) >= max_buf_meas:
                yield scan
                scan = []

    def get_scan(self, timeout: float = 2.0) -> List[ScanPoint]:
        """
        Get a single complete scan.

        Args:
            timeout: Maximum time to wait for scan

        Returns:
            List of ScanPoints
        """
        start_time = time.time()

        for scan in self.iter_scans():
            return scan

        return []

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


def check_device_exists() -> Tuple[bool, str]:
    """
    Check if LiDAR device exists.

    Returns:
        Tuple of (exists, device_path)
    """
    for path in [config.SERIAL_PORT, config.SERIAL_PORT_FALLBACK]:
        if os.path.exists(path):
            return True, path

    # Check all ttyUSB devices
    for i in range(10):
        path = f"/dev/ttyUSB{i}"
        if os.path.exists(path):
            return True, path

    return False, ""


def check_permissions(path: str) -> bool:
    """Check if we have read/write permissions on device."""
    return os.access(path, os.R_OK | os.W_OK)
