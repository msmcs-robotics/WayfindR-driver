import platform
from adafruit_rplidar import RPLidar

# Configuration
BAUDRATE: int = 115200
TIMEOUT: int = 1

# Device paths for different operating systems
MAC_DEVICE_PATH: str = '/dev/cu.usbserial-0001'
LINUX_DEVICE_PATH: str = '/dev/ttyUSB0'
WINDOWS_DEVICE_PATH: str = 'COM7'

def get_device_path() -> str:
    """Determine the correct device path based on the operating system."""
    system = platform.system()
    if system == 'Windows':
        return WINDOWS_DEVICE_PATH
    elif system == 'Darwin':
        return MAC_DEVICE_PATH
    elif system == 'Linux':
        return LINUX_DEVICE_PATH
    else:
        raise OSError(f"Unsupported operating system: {system}")

def print_lidar_info(lidar: RPLidar) -> None:
    """Print all available information about the LiDAR."""
    try:
        # Device information
        print("\n=== Device Information ===")
        info = lidar.info
        for key, value in info.items():
            print(f"{key:>15}: {value}")
        
        # Health status
        print("\n=== Health Status ===")
        health_status, error_code = lidar.health
        print(f"{'Status':>15}: {health_status}")
        print(f"{'Error Code':>15}: {error_code}")
        
        # Basic properties
        print("\n=== Connection Details ===")
        print(f"{'Port':>15}: {lidar.port}")
        print(f"{'Baudrate':>15}: {lidar.baudrate}")
        print(f"{'Timeout':>15}: {lidar.timeout}")
        print(f"{'Motor Running':>15}: {lidar.motor_running}")
        
    except Exception as e:
        print(f"\nError while getting LiDAR info: {e}")

def main():
    try:
        # Initialize the RPLidar
        device_path = get_device_path()
        print(f"Connecting to LiDAR on {device_path}...")
        
        # Note: Passing None for motor_pin since we're just getting info
        lidar = RPLidar(motor_pin=None, port=device_path, baudrate=BAUDRATE, timeout=TIMEOUT)
        
        # Print all available information
        print_lidar_info(lidar)
        
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        try:
            if 'lidar' in locals():
                print("\nDisconnecting from LiDAR...")
                lidar.stop()
                lidar.disconnect()
        except Exception as e:
            print(f"Error while disconnecting: {e}")

if __name__ == "__main__":
    main()
    print("\nDone.")