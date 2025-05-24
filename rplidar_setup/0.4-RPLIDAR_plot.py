import platform
import math
import numpy as np
import matplotlib.pyplot as plt
from adafruit_rplidar import RPLidar, RPLidarException
import time

# Configuration
BAUDRATE = 115200
TIMEOUT = 3
MAX_DISTANCE = 4000  # mm
PLOT_REFRESH_INTERVAL = 0.25  # seconds between updates

# Device paths
DEVICE_PATHS = {
    'Windows': 'COM7',
    'Darwin': '/dev/cu.usbserial-0001',
    'Linux': '/dev/ttyUSB0'
}

def setup_plot():
    """Initialize polar plot and return figure, axis, and scatter object."""
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, MAX_DISTANCE)
    ax.set_title('LiDAR Scan - Blue Dots Visualization', pad=20)
    scatter = ax.scatter([], [], s=5, c='blue')  # Empty data initially
    plt.tight_layout()
    plt.show(block=False)
    return fig, ax, scatter

def update_plot(scatter, angles, distances):
    """Update scatter data instead of redrawing everything."""
    if len(angles) == 0:
        scatter.set_offsets([])
    else:
        theta = np.deg2rad(angles)
        r = distances
        points = np.column_stack((theta, r))
        scatter.set_offsets(points)
    plt.pause(0.001)

def get_device_path():
    """Get platform-specific device path."""
    return DEVICE_PATHS.get(platform.system(), '')

def process_scan(scan):
    """Efficiently process scan to NumPy arrays."""
    angles = []
    distances = []
    for _, angle, distance in scan:
        if distance > 0:
            angles.append(angle)
            distances.append(min(distance, MAX_DISTANCE))
    return np.array(angles), np.array(distances)

def main():
    fig, ax, scatter = setup_plot()
    device_path = get_device_path()
    if not device_path:
        print("Unsupported OS.")
        return

    lidar = None
    try:
        print(f"Connecting to LiDAR on {device_path}...")
        lidar = RPLidar(None, device_path, baudrate=BAUDRATE, timeout=TIMEOUT)
        
        print(f"LiDAR Health: {lidar.health[0]} (Error Code: {lidar.health[1]})")
        print("\nLiDAR Info:")
        for k, v in lidar.info.items():
            print(f"{k:>15}: {v}")

        print("\nStarting scan... Press Ctrl+C to stop.")
        last_plot_time = time.time()

        for scan in lidar.iter_scans(max_buf_meas=1000):
            try:
                angles, distances = process_scan(scan)
                now = time.time()

                if now - last_plot_time >= PLOT_REFRESH_INTERVAL:
                    update_plot(scatter, angles, distances)
                    last_plot_time = now

            except RPLidarException as e:
                print(f"Scan error: {e}")
                continue

    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    except RPLidarException as e:
        print(f"LiDAR error: {e}")
    finally:
        if lidar:
            lidar.stop()
            lidar.disconnect()
            print("LiDAR disconnected.")
        plt.show(block=True)

if __name__ == "__main__":
    main()
