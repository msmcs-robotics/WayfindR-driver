import platform
import math
import numpy as np
import matplotlib.pyplot as plt
from adafruit_rplidar import RPLidar, RPLidarException

# Configuration
BAUDRATE = 115200
TIMEOUT = 3  # Increased timeout
MAX_DISTANCE = 4000  # mm (4 meters)
PLOT_REFRESH_RATE = 5  # Update plot every N scans

# Device paths
DEVICE_PATHS = {
    'Windows': 'COM7',
    'Darwin': '/dev/cu.usbserial-0001',
    'Linux': '/dev/ttyUSB0'
}

def setup_plot():
    """Initialize the polar plot."""
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, MAX_DISTANCE)
    ax.set_title('LiDAR Scan - Blue Dots Visualization', pad=20)
    plt.tight_layout()
    return fig, ax

def update_plot(ax, angles, distances):
    """Update the plot with new scan data."""
    ax.clear()
    if angles and distances:
        ax.scatter(np.deg2rad(angles), distances, s=5, c='blue')
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, MAX_DISTANCE)
    ax.set_title('LiDAR Scan - Blue Dots Visualization', pad=20)
    plt.draw()
    plt.pause(0.001)

def get_device_path():
    """Get the correct serial port path for the current OS."""
    system = platform.system()
    return DEVICE_PATHS.get(system, '')

def process_scan(scan):
    """Extract angles and distances from a scan."""
    angles = []
    distances = []
    for _, angle, distance in scan:
        if distance > 0:  # Filter invalid measurements
            angles.append(min(359, math.floor(angle)))
            distances.append(min(MAX_DISTANCE, distance))
    return angles, distances

def main():
    # Initialize plot
    fig, ax = setup_plot()
    plt.show(block=False)
    
    # Initialize LiDAR
    device_path = get_device_path()
    if not device_path:
        print("Error: Unsupported operating system")
        return
    
    lidar = None
    try:
        print(f"Connecting to LiDAR on {device_path}...")
        lidar = RPLidar(None, device_path, baudrate=BAUDRATE, timeout=TIMEOUT)
        
        # Check device health
        health, error_code = lidar.health
        print(f"LiDAR Health: {health} (Error Code: {error_code})")
        if health != "Good":
            print("Warning: LiDAR health status is not optimal")
        
        # Get device info
        info = lidar.info
        print("\nLiDAR Information:")
        for key, value in info.items():
            print(f"{key:>15}: {value}")
        
        print("\nStarting LiDAR scan... Press Ctrl+C to stop.")
        scan_count = 0
        
        for scan in lidar.iter_scans(max_buf_meas=1000):  # Increased buffer size
            try:
                angles, distances = process_scan(scan)
                
                # Update plot periodically
                scan_count += 1
                if scan_count % PLOT_REFRESH_RATE == 0:
                    update_plot(ax, angles, distances)
                    scan_count = 0
                    
            except RPLidarException as e:
                print(f"Scan processing error: {e}")
                continue
                
    except KeyboardInterrupt:
        print("\nStopping LiDAR scan...")
    except RPLidarException as e:
        print(f"\nLiDAR Error: {e}")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        if lidar:
            lidar.stop()
            lidar.disconnect()
            print("LiDAR disconnected.")
        plt.show(block=True)  # Keep plot open after scanning

if __name__ == "__main__":
    main()