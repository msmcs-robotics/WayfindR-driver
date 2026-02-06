"""
MPU6050 IMU Driver for Gyro Heading

Provides gyroscope-based heading tracking for closed-loop turns.
Uses I2C to communicate with the MPU6050 (GY-521 breakout board).

Key features:
- Gyro Z-axis heading integration (tracks how much the robot turned)
- Startup calibration (averages gyro bias while robot is still)
- Pitch/roll from accelerometer (complementary filter)
- Graceful degradation (connect() returns False if sensor missing)

Wiring: See docs/findings/mpu6050-wiring.md

Usage:
    from pathfinder.imu import IMU

    imu = IMU()
    if imu.connect():
        # In your main loop (~100Hz):
        imu.update()
        print(f"Heading: {imu.heading:.1f}°")
    else:
        print("IMU not found, continuing without heading")
"""

import logging
import math
import struct
import time
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

# MPU6050 Register Map
_WHO_AM_I = 0x75
_PWR_MGMT_1 = 0x6B
_PWR_MGMT_2 = 0x6C
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_ACCEL_XOUT_H = 0x3B
_TEMP_OUT_H = 0x41
_GYRO_XOUT_H = 0x43

# Gyro sensitivity (LSB per degree/second) by range setting
_GYRO_SENSITIVITY = {
    250: 131.0,
    500: 65.5,
    1000: 32.8,
    2000: 16.4,
}

# Gyro config register values by range
_GYRO_RANGE_CONFIG = {
    250: 0x00,
    500: 0x08,
    1000: 0x10,
    2000: 0x18,
}

# Accel sensitivity (LSB per g) by range setting
_ACCEL_SENSITIVITY = {
    2: 16384.0,
    4: 8192.0,
    8: 4096.0,
    16: 2048.0,
}

# Accel config register values by range
_ACCEL_RANGE_CONFIG = {
    2: 0x00,
    4: 0x08,
    8: 0x10,
    16: 0x18,
}


class IMU:
    """
    MPU6050 IMU driver for gyro heading and tilt detection.

    Connects via I2C bus 1 (SDA=Pin 3, SCL=Pin 5 on RPi).
    Default address 0x68 (AD0 pin floating or LOW).

    Parameters:
        bus: I2C bus number (default 1 for RPi)
        address: I2C address (0x68 default, 0x69 if AD0=HIGH)
        gyro_range: Gyro full-scale range in dps (250, 500, 1000, 2000)
        accel_range: Accelerometer full-scale range in g (2, 4, 8, 16)
    """

    def __init__(
        self,
        bus: int = 1,
        address: int = 0x68,
        gyro_range: int = 250,
        accel_range: int = 2,
    ):
        self._bus_num = bus
        self._address = address
        self._bus = None
        self._connected = False

        # Range settings
        self._gyro_range = gyro_range
        self._accel_range = accel_range
        self._gyro_sensitivity = _GYRO_SENSITIVITY[gyro_range]
        self._accel_sensitivity = _ACCEL_SENSITIVITY[accel_range]

        # Calibration offsets (determined at startup)
        self._gyro_bias = [0.0, 0.0, 0.0]  # x, y, z bias in raw LSB
        self._calibrated = False

        # Heading integration state
        self._heading = 0.0  # Accumulated heading in degrees
        self._heading_rate = 0.0  # Current yaw rate in dps
        self._last_update_time = 0.0

        # Tilt (complementary filter)
        self._pitch = 0.0
        self._roll = 0.0
        self._comp_alpha = 0.98  # Complementary filter weight (gyro vs accel)

    def connect(self) -> bool:
        """
        Connect to the MPU6050 and initialize.

        Returns True if sensor found and initialized.
        Returns False if sensor not found (graceful degradation).
        """
        try:
            # Try smbus2 first, fall back to smbus
            try:
                import smbus2 as smbus
            except ImportError:
                import smbus

            self._bus = smbus.SMBus(self._bus_num)

            # Check WHO_AM_I register
            who_am_i = self._bus.read_byte_data(self._address, _WHO_AM_I)
            if who_am_i not in (0x68, 0x72):
                # 0x68 = MPU6050, 0x72 = MPU6500 (compatible)
                logger.warning(f"Unexpected WHO_AM_I: 0x{who_am_i:02x} (expected 0x68)")
                self._bus.close()
                self._bus = None
                return False

            # Wake from sleep
            self._bus.write_byte_data(self._address, _PWR_MGMT_1, 0x00)
            time.sleep(0.1)

            # Configure gyro range
            gyro_config = _GYRO_RANGE_CONFIG[self._gyro_range]
            self._bus.write_byte_data(self._address, _GYRO_CONFIG, gyro_config)

            # Configure accel range
            accel_config = _ACCEL_RANGE_CONFIG[self._accel_range]
            self._bus.write_byte_data(self._address, _ACCEL_CONFIG, accel_config)

            self._connected = True
            self._last_update_time = time.time()

            logger.info(
                f"MPU6050 connected at 0x{self._address:02x} "
                f"(gyro ±{self._gyro_range}°/s, accel ±{self._accel_range}g)"
            )
            return True

        except (OSError, IOError, ImportError) as e:
            logger.info(f"IMU not available: {e}")
            if self._bus:
                try:
                    self._bus.close()
                except Exception:
                    pass
                self._bus = None
            return False

    @property
    def connected(self) -> bool:
        """Whether the IMU is connected and ready."""
        return self._connected

    @property
    def calibrated(self) -> bool:
        """Whether the gyro has been calibrated."""
        return self._calibrated

    def calibrate(self, samples: int = 200, duration: float = 2.0) -> bool:
        """
        Calibrate gyro bias by averaging readings while stationary.

        The robot must be STILL during calibration (2 seconds by default).

        Args:
            samples: Number of readings to average
            duration: Maximum calibration time in seconds

        Returns:
            True if calibration succeeded
        """
        if not self._connected:
            return False

        logger.info(f"Calibrating IMU gyro ({samples} samples, keep robot still)...")

        gyro_sum = [0.0, 0.0, 0.0]
        count = 0
        start_time = time.time()
        interval = duration / samples

        for _ in range(samples):
            if time.time() - start_time > duration:
                break
            try:
                raw = self._read_gyro_raw()
                gyro_sum[0] += raw[0]
                gyro_sum[1] += raw[1]
                gyro_sum[2] += raw[2]
                count += 1
                time.sleep(interval)
            except (OSError, IOError):
                continue

        if count < 10:
            logger.warning(f"Calibration failed: only {count} samples")
            return False

        self._gyro_bias = [s / count for s in gyro_sum]
        self._calibrated = True
        self._heading = 0.0
        self._last_update_time = time.time()

        bias_dps = [b / self._gyro_sensitivity for b in self._gyro_bias]
        logger.info(
            f"IMU calibrated ({count} samples): "
            f"bias = ({bias_dps[0]:.2f}, {bias_dps[1]:.2f}, {bias_dps[2]:.2f}) °/s"
        )
        return True

    def update(self) -> None:
        """
        Read sensor and update heading integration.

        Call this at ~100Hz (or at your behavior loop rate) for accurate heading.
        """
        if not self._connected:
            return

        current_time = time.time()
        dt = current_time - self._last_update_time
        self._last_update_time = current_time

        # Clamp dt to avoid jumps from pauses
        if dt <= 0 or dt > 0.5:
            return

        try:
            # Read gyro (bias-corrected, in dps)
            gx, gy, gz = self.get_gyro()
            self._heading_rate = gz

            # Integrate yaw (Z-axis) for heading
            self._heading += gz * dt
            # Normalize to 0-360
            self._heading = self._heading % 360

            # Update pitch/roll with complementary filter
            ax, ay, az = self.get_accel()
            self._update_tilt(gx, gy, ax, ay, az, dt)

        except (OSError, IOError) as e:
            logger.debug(f"IMU read error: {e}")

    @property
    def heading(self) -> float:
        """Current heading in degrees (0-360, relative to start/reset)."""
        return self._heading

    @property
    def heading_rate(self) -> float:
        """Current yaw rate in degrees/second (positive = clockwise)."""
        return self._heading_rate

    @property
    def pitch(self) -> float:
        """Current pitch in degrees (nose up = positive)."""
        return self._pitch

    @property
    def roll(self) -> float:
        """Current roll in degrees (right side down = positive)."""
        return self._roll

    def get_gyro(self) -> Tuple[float, float, float]:
        """
        Get bias-corrected gyro rates.

        Returns:
            (x, y, z) rotation rates in degrees/second
        """
        if not self._connected:
            return (0.0, 0.0, 0.0)

        raw = self._read_gyro_raw()
        return (
            (raw[0] - self._gyro_bias[0]) / self._gyro_sensitivity,
            (raw[1] - self._gyro_bias[1]) / self._gyro_sensitivity,
            (raw[2] - self._gyro_bias[2]) / self._gyro_sensitivity,
        )

    def get_accel(self) -> Tuple[float, float, float]:
        """
        Get accelerometer readings.

        Returns:
            (x, y, z) acceleration in g (1g = 9.81 m/s^2)
        """
        if not self._connected:
            return (0.0, 0.0, 0.0)

        raw = self._read_accel_raw()
        return (
            raw[0] / self._accel_sensitivity,
            raw[1] / self._accel_sensitivity,
            raw[2] / self._accel_sensitivity,
        )

    def get_tilt(self) -> Tuple[float, float]:
        """
        Get current tilt angles from complementary filter.

        Returns:
            (pitch, roll) in degrees
        """
        return (self._pitch, self._roll)

    def get_temperature(self) -> float:
        """
        Get chip temperature.

        Returns:
            Temperature in degrees Celsius
        """
        if not self._connected:
            return 0.0

        try:
            data = self._bus.read_i2c_block_data(self._address, _TEMP_OUT_H, 2)
            raw = struct.unpack(">h", bytes(data))[0]
            return raw / 340.0 + 36.53
        except (OSError, IOError):
            return 0.0

    def reset_heading(self) -> None:
        """Reset heading to 0 degrees."""
        self._heading = 0.0

    def cleanup(self) -> None:
        """Close I2C bus."""
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None
        self._connected = False

    # =========================================================================
    # Internal Methods
    # =========================================================================

    def _read_gyro_raw(self) -> Tuple[int, int, int]:
        """Read raw 16-bit signed gyro values (X, Y, Z)."""
        data = self._bus.read_i2c_block_data(self._address, _GYRO_XOUT_H, 6)
        x, y, z = struct.unpack(">hhh", bytes(data))
        return (x, y, z)

    def _read_accel_raw(self) -> Tuple[int, int, int]:
        """Read raw 16-bit signed accel values (X, Y, Z)."""
        data = self._bus.read_i2c_block_data(self._address, _ACCEL_XOUT_H, 6)
        x, y, z = struct.unpack(">hhh", bytes(data))
        return (x, y, z)

    def _update_tilt(
        self,
        gx: float, gy: float,
        ax: float, ay: float, az: float,
        dt: float,
    ) -> None:
        """Update pitch/roll using complementary filter."""
        # Accel-based angles (only valid when not accelerating)
        accel_pitch = math.atan2(ax, math.sqrt(ay * ay + az * az)) * 180 / math.pi
        accel_roll = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180 / math.pi

        # Complementary filter: trust gyro short-term, accel long-term
        alpha = self._comp_alpha
        self._pitch = alpha * (self._pitch + gx * dt) + (1 - alpha) * accel_pitch
        self._roll = alpha * (self._roll + gy * dt) + (1 - alpha) * accel_roll
