#!/usr/bin/env python3
"""
IMU Calibration & Axis Orientation Tool

Interactive tool to determine and save MPU6050 axis orientations:
1. Gyro bias calibration (keep robot still)
2. Gravity axis detection (which axis points up/down)
3. Heading axis detection (rotate robot, find which gyro axis responds)
4. Save calibration to tests/results/imu_calibration.json

The calibration file stores:
- Gyro bias (raw LSB offsets for X, Y, Z)
- Gravity axis (which accel axis reads ~1g when level)
- Heading axis (which gyro axis reads highest rate when turning in place)
- Axis signs (positive rotation direction)

Usage:
    python3 tests/test_imu_calibrate.py                  # Full interactive calibration
    python3 tests/test_imu_calibrate.py --quick           # Quick: bias + gravity only
    python3 tests/test_imu_calibrate.py --stream 10       # Stream raw data for 10s
    python3 tests/test_imu_calibrate.py --load             # Load and display saved calibration

This can be run without a display (SSH-friendly, no GUI needed).
"""

import sys
import os
import time
import json
import math
import argparse
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(exist_ok=True)
CALIBRATION_FILE = RESULTS_DIR / "imu_calibration.json"

sys.path.insert(0, str(SCRIPT_DIR.parent))


def connect_imu():
    """Connect to IMU, return instance or None."""
    try:
        from pathfinder.imu import IMU
        imu = IMU(bus=1, address=0x68)
        if imu.connect():
            return imu
        print("ERROR: IMU not found at 0x68 on I2C bus 1")
        print("Check wiring: VCC→Pin1(3.3V), GND→Pin9, SCL→Pin5, SDA→Pin3")
        return None
    except ImportError:
        print("ERROR: smbus2 not installed (run: pip3 install smbus2)")
        return None


def calibrate_gyro_bias(imu, samples=500, duration=3.0):
    """
    Calibrate gyro bias. Robot must be STILL.

    Returns dict with bias values.
    """
    print()
    print("=" * 50)
    print("STEP 1: Gyro Bias Calibration")
    print("=" * 50)
    print()
    print("Keep the robot COMPLETELY STILL for 3 seconds...")
    print("Starting in 2 seconds...")
    time.sleep(2.0)
    print("Calibrating...")

    success = imu.calibrate(samples=samples, duration=duration)
    if not success:
        print("ERROR: Calibration failed")
        return None

    bias = imu._gyro_bias
    sensitivity = imu._gyro_sensitivity
    bias_dps = [b / sensitivity for b in bias]

    print(f"\nGyro bias (raw LSB):   X={bias[0]:.1f}  Y={bias[1]:.1f}  Z={bias[2]:.1f}")
    print(f"Gyro bias (deg/s):     X={bias_dps[0]:.3f}  Y={bias_dps[1]:.3f}  Z={bias_dps[2]:.3f}")

    return {
        "bias_raw": {"x": bias[0], "y": bias[1], "z": bias[2]},
        "bias_dps": {"x": bias_dps[0], "y": bias_dps[1], "z": bias_dps[2]},
        "samples": samples,
    }


def detect_gravity_axis(imu, samples=50):
    """
    Detect which accelerometer axis points up/down (reads ~1g).

    The robot should be level on a flat surface.
    Returns dict with gravity axis info.
    """
    print()
    print("=" * 50)
    print("STEP 2: Gravity Axis Detection")
    print("=" * 50)
    print()
    print("Robot should be on a FLAT, LEVEL surface.")
    print("Reading accelerometer...")

    ax_sum, ay_sum, az_sum = 0.0, 0.0, 0.0
    for _ in range(samples):
        ax, ay, az = imu.get_accel()
        ax_sum += ax
        ay_sum += ay
        az_sum += az
        time.sleep(0.02)

    ax_avg = ax_sum / samples
    ay_avg = ay_sum / samples
    az_avg = az_sum / samples

    print(f"\nAccel averages (g):  X={ax_avg:+.3f}  Y={ay_avg:+.3f}  Z={az_avg:+.3f}")

    # Find which axis has the largest magnitude (~1g)
    axes = {"X": ax_avg, "Y": ay_avg, "Z": az_avg}
    gravity_axis = max(axes, key=lambda k: abs(axes[k]))
    gravity_sign = "+" if axes[gravity_axis] > 0 else "-"
    gravity_value = axes[gravity_axis]

    print(f"\nGravity axis: {gravity_sign}{gravity_axis} (reading {gravity_value:+.3f}g)")

    if abs(gravity_value) < 0.8:
        print("WARNING: Gravity reading is low — sensor may be tilted or noisy")
    elif abs(gravity_value) > 1.2:
        print("WARNING: Gravity reading is high — check accel range setting")
    else:
        print("Gravity reading looks good (expected ~1.0g)")

    # Determine orientation description
    if gravity_axis == "Z":
        if gravity_value > 0:
            orientation = "Chip face UP (standard mounting, Z points up)"
        else:
            orientation = "Chip face DOWN (inverted, Z points down)"
    elif gravity_axis == "X":
        orientation = f"Tilted on X axis ({'front' if gravity_value > 0 else 'back'} edge up)"
    else:
        orientation = f"Tilted on Y axis ({'right' if gravity_value > 0 else 'left'} edge up)"

    print(f"Orientation: {orientation}")

    return {
        "accel_avg": {"x": ax_avg, "y": ay_avg, "z": az_avg},
        "gravity_axis": gravity_axis,
        "gravity_sign": gravity_sign,
        "gravity_value": gravity_value,
        "orientation": orientation,
    }


def detect_heading_axis(imu, duration=5.0):
    """
    Detect which gyro axis responds to yaw (turning in place).

    User must rotate the robot LEFT and RIGHT during this test.
    Returns dict with heading axis info.
    """
    print()
    print("=" * 50)
    print("STEP 3: Heading Axis Detection")
    print("=" * 50)
    print()
    print(f"ROTATE the robot LEFT and RIGHT for {duration:.0f} seconds.")
    print("Keep it level (don't tilt). Just spin/turn in place.")
    print()
    print("Starting in 3 seconds...")
    time.sleep(3.0)
    print("GO! Rotate now!")
    print()

    gx_max, gy_max, gz_max = 0.0, 0.0, 0.0
    gx_samples, gy_samples, gz_samples = [], [], []
    start = time.time()
    count = 0

    while time.time() - start < duration:
        gx, gy, gz = imu.get_gyro()
        gx_samples.append(abs(gx))
        gy_samples.append(abs(gy))
        gz_samples.append(abs(gz))

        gx_max = max(gx_max, abs(gx))
        gy_max = max(gy_max, abs(gy))
        gz_max = max(gz_max, abs(gz))

        count += 1

        # Live feedback
        elapsed = time.time() - start
        remaining = duration - elapsed
        if count % 20 == 0:
            print(f"  [{remaining:.1f}s left] Gyro: X={gx:+7.1f}  Y={gy:+7.1f}  Z={gz:+7.1f} deg/s", end='\r')

        time.sleep(0.01)

    print()
    print(f"\nCollected {count} samples")

    # Average absolute values (higher = more responsive to rotation)
    gx_avg = sum(gx_samples) / len(gx_samples) if gx_samples else 0
    gy_avg = sum(gy_samples) / len(gy_samples) if gy_samples else 0
    gz_avg = sum(gz_samples) / len(gz_samples) if gz_samples else 0

    print(f"\nPeak rates (deg/s):     X={gx_max:.1f}  Y={gy_max:.1f}  Z={gz_max:.1f}")
    print(f"Avg abs rates (deg/s):  X={gx_avg:.1f}  Y={gy_avg:.1f}  Z={gz_avg:.1f}")

    # Heading axis = the one with highest response
    axes = {"X": gx_avg, "Y": gy_avg, "Z": gz_avg}
    heading_axis = max(axes, key=lambda k: axes[k])

    peaks = {"X": gx_max, "Y": gy_max, "Z": gz_max}

    # Check if the detection is clear (dominant axis should be >3x the others)
    others = [v for k, v in axes.items() if k != heading_axis]
    ratio = axes[heading_axis] / max(others) if max(others) > 0 else float('inf')

    print(f"\nHeading axis: {heading_axis} (dominance ratio: {ratio:.1f}x)")

    if ratio < 2.0:
        print("WARNING: Axis detection is ambiguous (ratio < 2x)")
        print("Try rotating more aggressively and keeping the robot level")
    elif ratio < 3.0:
        print("Heading axis detection is OK but could be clearer")
    else:
        print("Heading axis detection is clear and confident")

    return {
        "heading_axis": heading_axis,
        "peak_rates": {"x": gx_max, "y": gy_max, "z": gz_max},
        "avg_rates": {"x": gx_avg, "y": gy_avg, "z": gz_avg},
        "dominance_ratio": ratio,
        "samples": count,
    }


def stream_raw_data(imu, duration=10.0):
    """Stream raw sensor data for manual inspection."""
    print()
    print("=" * 50)
    print(f"Streaming raw IMU data for {duration:.0f}s")
    print("=" * 50)
    print()
    print(f"{'Time':>6}  {'Gx':>8} {'Gy':>8} {'Gz':>8}  {'Ax':>7} {'Ay':>7} {'Az':>7}  {'Temp':>6}")
    print("-" * 75)

    start = time.time()
    count = 0
    try:
        while time.time() - start < duration:
            gx, gy, gz = imu.get_gyro()
            ax, ay, az = imu.get_accel()
            temp = imu.get_temperature()
            elapsed = time.time() - start

            if count % 5 == 0:  # Print every 5th sample
                print(f"{elapsed:6.2f}  {gx:+8.2f} {gy:+8.2f} {gz:+8.2f}  "
                      f"{ax:+7.3f} {ay:+7.3f} {az:+7.3f}  {temp:5.1f}C")

            count += 1
            time.sleep(0.02)  # ~50Hz
    except KeyboardInterrupt:
        print("\nStopped")

    print(f"\n{count} samples in {time.time()-start:.1f}s ({count/(time.time()-start):.0f} Hz)")


def save_calibration(data):
    """Save calibration data to JSON file."""
    data["calibrated_at"] = datetime.now().isoformat()
    data["note"] = "IMU calibration data. Axis names are MPU6050 chip-relative."
    CALIBRATION_FILE.write_text(json.dumps(data, indent=2))
    print(f"\nCalibration saved to: {CALIBRATION_FILE}")


def load_and_display_calibration():
    """Load and display saved calibration."""
    if not CALIBRATION_FILE.exists():
        print(f"No calibration file found at: {CALIBRATION_FILE}")
        print("Run calibration first: python3 tests/test_imu_calibrate.py")
        return

    data = json.loads(CALIBRATION_FILE.read_text())
    print()
    print("=" * 50)
    print("Saved IMU Calibration")
    print("=" * 50)
    print(f"\nCalibrated at: {data.get('calibrated_at', 'unknown')}")

    if "gyro_bias" in data:
        bias = data["gyro_bias"]
        print(f"\nGyro bias (raw):  X={bias['bias_raw']['x']:.1f}  Y={bias['bias_raw']['y']:.1f}  Z={bias['bias_raw']['z']:.1f}")
        print(f"Gyro bias (dps):  X={bias['bias_dps']['x']:.3f}  Y={bias['bias_dps']['y']:.3f}  Z={bias['bias_dps']['z']:.3f}")

    if "gravity" in data:
        grav = data["gravity"]
        print(f"\nGravity axis: {grav['gravity_sign']}{grav['gravity_axis']} ({grav['gravity_value']:+.3f}g)")
        print(f"Orientation: {grav['orientation']}")

    if "heading" in data:
        head = data["heading"]
        print(f"\nHeading axis: {head['heading_axis']} (dominance: {head['dominance_ratio']:.1f}x)")
        print(f"Peak rates: X={head['peak_rates']['x']:.1f}  Y={head['peak_rates']['y']:.1f}  Z={head['peak_rates']['z']:.1f} deg/s")

    print()


def main():
    parser = argparse.ArgumentParser(
        description="IMU Calibration & Axis Orientation Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Steps:
  1. Gyro bias: Keep robot still for 3s (auto-averages noise)
  2. Gravity axis: Keep robot level, detect which axis = "up"
  3. Heading axis: Rotate robot in place, detect yaw axis

Results saved to tests/results/imu_calibration.json
        """
    )
    parser.add_argument("--quick", action="store_true",
                        help="Quick calibration (bias + gravity only, no rotation)")
    parser.add_argument("--stream", type=float, metavar="SECONDS",
                        help="Stream raw IMU data for N seconds")
    parser.add_argument("--load", action="store_true",
                        help="Load and display saved calibration")

    args = parser.parse_args()

    print("=" * 50)
    print("AMBOT IMU Calibration Tool")
    print("=" * 50)

    if args.load:
        load_and_display_calibration()
        return 0

    # Connect
    imu = connect_imu()
    if imu is None:
        return 1

    print(f"IMU connected (gyro ±{imu._gyro_range}°/s, accel ±{imu._accel_range}g)")

    try:
        if args.stream:
            stream_raw_data(imu, duration=args.stream)
            return 0

        calibration_data = {}

        # Step 1: Gyro bias
        bias_data = calibrate_gyro_bias(imu)
        if bias_data is None:
            return 1
        calibration_data["gyro_bias"] = bias_data

        # Step 2: Gravity axis
        gravity_data = detect_gravity_axis(imu)
        calibration_data["gravity"] = gravity_data

        if not args.quick:
            # Step 3: Heading axis (requires user interaction)
            heading_data = detect_heading_axis(imu)
            calibration_data["heading"] = heading_data

        # Summary
        print()
        print("=" * 50)
        print("CALIBRATION SUMMARY")
        print("=" * 50)
        print(f"  Gravity axis:  {gravity_data['gravity_sign']}{gravity_data['gravity_axis']} ({gravity_data['orientation']})")
        if "heading" in calibration_data:
            hd = calibration_data["heading"]
            print(f"  Heading axis:  {hd['heading_axis']} (confidence: {hd['dominance_ratio']:.1f}x)")
        print(f"  Gyro bias:     X={bias_data['bias_dps']['x']:.3f}  Y={bias_data['bias_dps']['y']:.3f}  Z={bias_data['bias_dps']['z']:.3f} °/s")

        # Save
        save_calibration(calibration_data)

        print()
        print("Next steps:")
        print("  - If heading axis is NOT Z, update pathfinder/imu.py to use correct axis")
        print("  - Load calibration in code: json.loads(Path('tests/results/imu_calibration.json').read_text())")
        print("  - Test heading tracking: python3 tests/test_imu.py --duration 10")

    finally:
        imu.cleanup()

    return 0


if __name__ == "__main__":
    sys.exit(main())
