"""
LD19 LiDAR Driver

Driver for YOUYEETOO LD19 / LDRobot LD19 LiDAR sensor.

Protocol: LDRobot proprietary (NOT RPLidar)
Baud rate: 230400
Packet size: 47 bytes
Points per packet: 12

This driver is separate from lidar.py (RPLidar) but provides a compatible interface.
"""

import serial
import struct
import math
import time
from typing import Optional, List, Generator, Tuple
from dataclasses import dataclass

# CRC8 lookup table (polynomial 0x4D)
CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
]

# Protocol constants
PACKET_HEADER = 0x54
PACKET_VER_LEN = 0x2C
PACKET_SIZE = 47
POINTS_PER_PACKET = 12
DEFAULT_BAUD_RATE = 230400


@dataclass
class ScanPoint:
    """Single LiDAR scan point (compatible with RPLidar ScanPoint)."""
    angle: float       # Angle in degrees (0-360)
    distance: float    # Distance in millimeters
    quality: int       # Signal intensity (0-255)

    @property
    def distance_m(self) -> float:
        """Distance in meters."""
        return self.distance / 1000.0

    @property
    def angle_rad(self) -> float:
        """Angle in radians."""
        return math.radians(self.angle)

    @property
    def x(self) -> float:
        """X coordinate in meters."""
        return self.distance_m * math.cos(self.angle_rad)

    @property
    def y(self) -> float:
        """Y coordinate in meters."""
        return self.distance_m * math.sin(self.angle_rad)


class LD19LidarException(Exception):
    """Exception for LD19 LiDAR errors."""
    pass


def calculate_crc8(data: bytes) -> int:
    """Calculate CRC8 checksum using lookup table."""
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def verify_packet(data: bytes) -> bool:
    """Verify packet header, length, and CRC."""
    if len(data) != PACKET_SIZE:
        return False
    if data[0] != PACKET_HEADER:
        return False
    if data[1] != PACKET_VER_LEN:
        return False
    # CRC is calculated on all bytes except the last one
    calculated_crc = calculate_crc8(data[:-1])
    return calculated_crc == data[-1]


def parse_packet(data: bytes) -> Optional[List[ScanPoint]]:
    """
    Parse a 47-byte LiDAR packet.

    Returns list of 12 ScanPoint objects, or None if packet is invalid.
    """
    if not verify_packet(data):
        return None

    # Parse header fields
    start_angle_raw = struct.unpack('<H', data[4:6])[0]
    end_angle_raw = struct.unpack('<H', data[42:44])[0]

    # Convert angles (stored as 0.01 degree units)
    start_angle = start_angle_raw / 100.0
    end_angle = end_angle_raw / 100.0

    # Calculate angle step (handle wraparound)
    angle_diff = end_angle - start_angle
    if angle_diff < 0:
        angle_diff += 360.0
    angle_step = angle_diff / (POINTS_PER_PACKET - 1) if POINTS_PER_PACKET > 1 else 0

    # Parse measurement points
    points = []
    for i in range(POINTS_PER_PACKET):
        offset = 6 + (i * 3)
        distance = struct.unpack('<H', data[offset:offset+2])[0]
        intensity = data[offset + 2]

        # Calculate interpolated angle
        angle = start_angle + (i * angle_step)
        if angle >= 360.0:
            angle -= 360.0

        points.append(ScanPoint(
            angle=angle,
            distance=float(distance),
            quality=intensity
        ))

    return points


class LD19Lidar:
    """
    LD19 LiDAR driver class.

    Usage:
        lidar = LD19Lidar('/dev/ttyUSB0')
        lidar.connect()
        for scan in lidar.iter_scans():
            for point in scan:
                print(f"Angle: {point.angle:.1f}, Distance: {point.distance:.0f}mm")
        lidar.disconnect()

    Or using context manager:
        with LD19Lidar('/dev/ttyUSB0') as lidar:
            for scan in lidar.iter_scans():
                process(scan)
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = DEFAULT_BAUD_RATE):
        self.port = port
        self.baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._debug = False

    @property
    def is_connected(self) -> bool:
        """Check if connected to LiDAR."""
        return self._serial is not None and self._serial.is_open

    def connect(self) -> bool:
        """Connect to the LiDAR."""
        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self._serial.reset_input_buffer()
            self._buffer.clear()

            if self._debug:
                print(f"Connected to LD19 on {self.port} @ {self.baudrate} baud")

            return True
        except serial.SerialException as e:
            raise LD19LidarException(f"Connection failed: {e}")

    def disconnect(self):
        """Disconnect from the LiDAR."""
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._buffer.clear()

        if self._debug:
            print("Disconnected from LD19")

    def start_motor(self):
        """Start motor (no-op for LD19 - motor runs continuously)."""
        # LD19 motor starts automatically on power-up
        pass

    def stop_motor(self):
        """Stop motor (no-op for LD19 - control via power/PWM)."""
        # LD19 motor cannot be stopped via serial
        pass

    def _sync_to_header(self) -> bool:
        """Synchronize to packet header in the data stream."""
        while self._serial and self._serial.is_open:
            if self._serial.in_waiting == 0:
                return False

            byte = self._serial.read(1)
            if len(byte) == 0:
                return False

            if byte[0] == PACKET_HEADER:
                self._buffer.clear()
                self._buffer.append(byte[0])
                return True

        return False

    def read_packet(self) -> Optional[List[ScanPoint]]:
        """Read and parse a single packet (returns 12 points)."""
        if not self.is_connected:
            return None

        # If buffer is empty, sync to header
        if len(self._buffer) == 0:
            if not self._sync_to_header():
                return None

        # Read remaining bytes for complete packet
        remaining = PACKET_SIZE - len(self._buffer)
        if remaining > 0:
            data = self._serial.read(remaining)
            if len(data) < remaining:
                return None
            self._buffer.extend(data)

        # Verify and parse packet
        if len(self._buffer) == PACKET_SIZE:
            if self._buffer[1] == PACKET_VER_LEN:
                points = parse_packet(bytes(self._buffer))
                self._buffer.clear()
                return points
            else:
                # Invalid packet, resync
                self._buffer.clear()
                return None

        return None

    def iter_measurements(self, max_buf_meas: int = 500) -> Generator[ScanPoint, None, None]:
        """
        Iterate over individual measurements.

        Yields:
            ScanPoint for each valid measurement
        """
        while self.is_connected:
            points = self.read_packet()
            if points:
                for point in points:
                    if point.distance > 0:  # Filter out zero distances
                        yield point

    def iter_scans(self, max_buf_meas: int = 500, min_points: int = 300) -> Generator[List[ScanPoint], None, None]:
        """
        Iterate over complete 360-degree scans.

        A scan is complete when angle wraps from high to low.

        Args:
            max_buf_meas: Maximum measurements per scan buffer
            min_points: Minimum points required for a valid scan

        Yields:
            List of ScanPoints for each complete scan
        """
        scan: List[ScanPoint] = []
        prev_angle = 0.0

        for point in self.iter_measurements(max_buf_meas):
            # Detect scan boundary (angle wraparound)
            if point.angle < prev_angle - 180:
                if len(scan) >= min_points:
                    yield scan
                scan = []

            # Filter out invalid measurements
            if 100 < point.distance < 12000:  # 0.1m to 12m range
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

    def get_info(self) -> dict:
        """Get device information (limited for LD19)."""
        return {
            "model": "LD19",
            "firmware": "N/A",
            "hardware": "N/A",
            "serial": "N/A",
            "port": self.port,
            "baudrate": self.baudrate,
        }

    def get_health(self) -> Tuple[str, int]:
        """Get device health (always returns Good for LD19)."""
        # LD19 doesn't have health command
        return "Good", 0

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


def check_device_exists(port: str = '/dev/ttyUSB0') -> Tuple[bool, str]:
    """Check if LiDAR device exists at port."""
    import os
    if os.path.exists(port):
        return True, port
    return False, ""


# Test code
if __name__ == '__main__':
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

    print(f"Testing LD19 LiDAR on {port}")
    print(f"Baud rate: {DEFAULT_BAUD_RATE}")
    print()

    try:
        with LD19Lidar(port) as lidar:
            print("Connected! Reading scans...")

            scan_count = 0
            for scan in lidar.iter_scans():
                scan_count += 1

                # Print every 10th scan
                if scan_count % 10 == 0:
                    print(f"Scan {scan_count}: {len(scan)} points")

                    # Find closest point
                    if scan:
                        closest = min(scan, key=lambda p: p.distance)
                        print(f"  Closest: {closest.distance:.0f}mm at {closest.angle:.1f}°")

                # Stop after 50 scans for demo
                if scan_count >= 50:
                    break

            print(f"\nReceived {scan_count} complete scans")

    except LD19LidarException as e:
        print(f"Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted")
