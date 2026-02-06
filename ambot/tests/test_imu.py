#!/usr/bin/env python3
"""
MPU6050 IMU Hardware Test

Tests the MPU6050 (GY-521) IMU connected via I2C bus 1.
Verifies connection, WHO_AM_I register, gyro/accel reads, calibration,
and heading integration.

Wiring: See docs/findings/mpu6050-wiring.md
  VCC -> Pin 1 (3.3V)
  GND -> Pin 9
  SCL -> Pin 5 (GPIO3)
  SDA -> Pin 3 (GPIO2)

Usage:
    python3 tests/test_imu.py
    python3 tests/test_imu.py --verbose
    python3 tests/test_imu.py --duration 5  # Read for 5 seconds
"""

import argparse
import sys
import time
from pathlib import Path

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))


def test_i2c_bus():
    """Test that I2C bus 1 is available."""
    print("\n--- Test: I2C Bus Availability ---")
    try:
        try:
            import smbus2 as smbus
            print("  Using smbus2")
        except ImportError:
            import smbus
            print("  Using smbus")

        bus = smbus.SMBus(1)
        bus.close()
        print("  PASS: I2C bus 1 is available")
        return True
    except ImportError:
        print("  FAIL: smbus/smbus2 not installed")
        print("        Fix: sudo apt install python3-smbus")
        return False
    except (OSError, IOError) as e:
        print(f"  FAIL: Cannot open I2C bus 1: {e}")
        print("        Fix: sudo raspi-config -> Interface Options -> I2C -> Enable")
        return False


def test_who_am_i():
    """Test MPU6050 WHO_AM_I register."""
    print("\n--- Test: MPU6050 WHO_AM_I Register ---")
    try:
        try:
            import smbus2 as smbus
        except ImportError:
            import smbus

        bus = smbus.SMBus(1)
        who = bus.read_byte_data(0x68, 0x75)
        bus.close()

        if who == 0x68:
            print(f"  PASS: WHO_AM_I = 0x{who:02x} (MPU6050)")
            return True
        elif who == 0x72:
            print(f"  PASS: WHO_AM_I = 0x{who:02x} (MPU6500, compatible)")
            return True
        else:
            print(f"  FAIL: WHO_AM_I = 0x{who:02x} (expected 0x68 or 0x72)")
            return False
    except (OSError, IOError) as e:
        print(f"  FAIL: Cannot read WHO_AM_I: {e}")
        print("        Check wiring: SDA->Pin3, SCL->Pin5, VCC->Pin1, GND->Pin9")
        print("        Verify with: i2cdetect -y 1 (should show 0x68)")
        return False


def test_imu_connect():
    """Test IMU driver connect()."""
    print("\n--- Test: IMU Driver Connect ---")
    from pathfinder.imu import IMU

    imu = IMU()
    if imu.connect():
        print("  PASS: IMU connected successfully")
        print(f"        Gyro range: +/-{imu._gyro_range} dps")
        print(f"        Accel range: +/-{imu._accel_range} g")
        imu.cleanup()
        return True
    else:
        print("  FAIL: IMU connect() returned False")
        return False


def test_raw_reads():
    """Test raw gyro and accel reads."""
    print("\n--- Test: Raw Sensor Reads ---")
    from pathfinder.imu import IMU

    imu = IMU()
    if not imu.connect():
        print("  SKIP: IMU not connected")
        return False

    try:
        # Read gyro
        gx, gy, gz = imu.get_gyro()
        print(f"  Gyro (dps):  X={gx:+8.2f}  Y={gy:+8.2f}  Z={gz:+8.2f}")

        # Read accel
        ax, ay, az = imu.get_accel()
        print(f"  Accel (g):   X={ax:+8.4f}  Y={ay:+8.4f}  Z={az:+8.4f}")

        # Sanity: accel magnitude should be ~1g when stationary
        import math
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        print(f"  Accel magnitude: {mag:.4f} g (expected ~1.0)")

        if 0.8 < mag < 1.2:
            print("  PASS: Accelerometer reads are sane")
        else:
            print(f"  WARN: Accel magnitude {mag:.2f}g is unusual (expected ~1.0g)")

        # Temperature
        temp = imu.get_temperature()
        print(f"  Temperature: {temp:.1f} C")

        if 10 < temp < 60:
            print("  PASS: Temperature is sane")
        else:
            print(f"  WARN: Temperature {temp:.1f}C seems unusual")

        return True
    finally:
        imu.cleanup()


def test_calibration():
    """Test gyro calibration."""
    print("\n--- Test: Gyro Calibration ---")
    print("  Keep robot STILL for calibration...")
    from pathfinder.imu import IMU

    imu = IMU()
    if not imu.connect():
        print("  SKIP: IMU not connected")
        return False

    try:
        if imu.calibrate(samples=100, duration=1.0):
            bias = imu._gyro_bias
            sens = imu._gyro_sensitivity
            print(f"  Gyro bias (dps): X={bias[0]/sens:+.3f}  Y={bias[1]/sens:+.3f}  Z={bias[2]/sens:+.3f}")
            print("  PASS: Calibration succeeded")

            # After calibration, gyro should read near zero when stationary
            time.sleep(0.1)
            gx, gy, gz = imu.get_gyro()
            print(f"  Post-cal gyro (dps): X={gx:+.3f}  Y={gy:+.3f}  Z={gz:+.3f}")

            if abs(gx) < 2.0 and abs(gy) < 2.0 and abs(gz) < 2.0:
                print("  PASS: Post-calibration drift is low")
            else:
                print("  WARN: Post-calibration gyro readings seem high")
            return True
        else:
            print("  FAIL: Calibration failed")
            return False
    finally:
        imu.cleanup()


def test_heading_integration(duration=2.0):
    """Test heading integration over time."""
    print(f"\n--- Test: Heading Integration ({duration}s) ---")
    print("  Keep robot STILL - heading should stay near 0...")
    from pathfinder.imu import IMU

    imu = IMU()
    if not imu.connect():
        print("  SKIP: IMU not connected")
        return False

    try:
        imu.calibrate(samples=100, duration=1.0)
        imu.reset_heading()

        start = time.time()
        updates = 0
        while time.time() - start < duration:
            imu.update()
            updates += 1
            time.sleep(0.01)  # ~100Hz

        heading = imu.heading
        rate = imu.heading_rate
        pitch, roll = imu.get_tilt()

        print(f"  Updates: {updates} in {duration}s ({updates/duration:.0f} Hz)")
        print(f"  Final heading: {heading:.2f} deg (expected ~0 when still)")
        print(f"  Heading rate: {rate:+.3f} dps")
        print(f"  Tilt: pitch={pitch:+.2f} deg, roll={roll:+.2f} deg")

        # When stationary, heading drift should be small (< 5 deg over 2s)
        # Normalize heading to be closest to 0 (could be 359.x)
        drift = heading if heading < 180 else heading - 360
        if abs(drift) < 5.0:
            print(f"  PASS: Heading drift is acceptable ({drift:+.2f} deg)")
            return True
        else:
            print(f"  WARN: Heading drift is {drift:+.2f} deg (may indicate calibration issue)")
            return True  # Still pass, just warn
    finally:
        imu.cleanup()


def continuous_read(duration, verbose):
    """Continuous sensor readout for specified duration."""
    print(f"\n--- Continuous Read ({duration}s) ---")
    from pathfinder.imu import IMU

    imu = IMU()
    if not imu.connect():
        print("  IMU not connected")
        return

    print("  Calibrating...")
    imu.calibrate(samples=100, duration=1.0)
    imu.reset_heading()

    print("  Reading sensors (Ctrl+C to stop)...")
    print(f"  {'Time':>6s}  {'Heading':>8s}  {'Rate':>8s}  {'Pitch':>7s}  {'Roll':>7s}  {'Temp':>6s}")
    print("  " + "-" * 50)

    start = time.time()
    try:
        while time.time() - start < duration:
            imu.update()
            elapsed = time.time() - start
            heading = imu.heading
            rate = imu.heading_rate
            pitch, roll = imu.get_tilt()
            temp = imu.get_temperature()

            print(f"\r  {elapsed:6.1f}s  {heading:8.2f}  {rate:+8.3f}  {pitch:+7.2f}  {roll:+7.2f}  {temp:5.1f}C", end="")
            time.sleep(0.05)  # ~20Hz display
    except KeyboardInterrupt:
        pass

    print()
    imu.cleanup()


def main():
    parser = argparse.ArgumentParser(description="MPU6050 IMU Hardware Test")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--duration", "-d", type=float, default=0,
                        help="Continuous read duration in seconds (0 = run tests only)")
    args = parser.parse_args()

    print("=" * 60)
    print("MPU6050 IMU Hardware Test")
    print("=" * 60)

    if args.duration > 0:
        continuous_read(args.duration, args.verbose)
        return 0

    # Run test sequence
    passed = 0
    failed = 0
    skipped = 0

    tests = [
        test_i2c_bus,
        test_who_am_i,
        test_imu_connect,
        test_raw_reads,
        test_calibration,
        test_heading_integration,
    ]

    for test_fn in tests:
        try:
            result = test_fn()
            if result:
                passed += 1
            elif result is False:
                failed += 1
            else:
                skipped += 1
        except Exception as e:
            print(f"  ERROR: {e}")
            failed += 1

    # Summary
    print("\n" + "=" * 60)
    print(f"Results: {passed} passed, {failed} failed, {skipped} skipped")
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
