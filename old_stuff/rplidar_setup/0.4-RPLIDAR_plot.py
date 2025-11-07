import platform
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from adafruit_rplidar import RPLidar, RPLidarException
import time
from collections import deque
import threading

# Configuration
BAUDRATE = 115200
TIMEOUT = 3
MAX_DISTANCE = 4000  # mm
BUFFER_SIZE = 500
FPS = 30  # Back to higher FPS
MAX_PLOT_POINTS = 2000  # Limit total points for performance
POINT_LIFETIME = 1.5  # seconds to keep points visible

# Device paths
DEVICE_PATHS = {
    'Windows': 'COM7',
    'Darwin': '/dev/cu.usbserial-0001',
    'Linux': '/dev/ttyUSB0'
}

# Global variables
latest_scan = {'angles': np.array([]), 'distances': np.array([]), 'timestamp': 0}
accumulated_points = deque(maxlen=MAX_PLOT_POINTS)
data_lock = threading.Lock()
scan_active = True

def get_device_path():
    """Get platform-specific device path."""
    return DEVICE_PATHS.get(platform.system(), '')

def lidar_thread(device_path):
    """Background thread for continuous lidar data collection."""
    global latest_scan, scan_active
    
    lidar = None
    try:
        print(f"Connecting to LiDAR on {device_path}...")
        lidar = RPLidar(None, device_path, baudrate=BAUDRATE, timeout=TIMEOUT)
        
        print(f"LiDAR Health: {lidar.health[0]} (Error Code: {lidar.health[1]})")
        print("\nLiDAR Info:")
        for k, v in lidar.info.items():
            print(f"{k:>15}: {v}")

        print("\nStarting scan...")
        print("Controls:")
        print("- Mouse wheel: Zoom in/out")
        print("- Click and drag: Pan around")
        print("- 'c' key: Clear points")
        print("- Ctrl+C: Exit")
        
        # Pre-allocate arrays
        temp_angles = np.zeros(BUFFER_SIZE, dtype=np.float32)
        temp_distances = np.zeros(BUFFER_SIZE, dtype=np.float32)
        
        for scan in lidar.iter_scans(max_buf_meas=300):  # Reduced buffer for speed
            if not scan_active:
                break
                
            try:
                count = 0
                for _, angle, distance in scan:
                    if distance > 0 and count < BUFFER_SIZE:
                        temp_angles[count] = angle
                        temp_distances[count] = min(distance, MAX_DISTANCE)
                        count += 1
                
                if count > 0:
                    with data_lock:
                        latest_scan = {
                            'angles': temp_angles[:count].copy(),
                            'distances': temp_distances[:count].copy(),
                            'timestamp': time.time()
                        }
                        
            except RPLidarException as e:
                print(f"Scan error: {e}")
                continue
                
    except RPLidarException as e:
        print(f"LiDAR error: {e}")
    finally:
        if lidar:
            lidar.stop()
            lidar.disconnect()
            print("LiDAR disconnected.")

def setup_plot():
    """Initialize fast polar plot."""
    plt.style.use('fast')
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(10, 10))
    
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, MAX_DISTANCE)
    ax.set_title('High-Speed LiDAR Scanner', pad=20)
    ax.grid(True, alpha=0.2)
    
    # Create empty scatter plots for different point types
    current_scatter = ax.scatter([], [], s=8, c='red', alpha=1.0, label='Current', edgecolors='none')
    old_scatter = ax.scatter([], [], s=3, c='cyan', alpha=0.6, label='Previous', edgecolors='none')
    
    # Enable toolbar for zoom/pan
    fig.canvas.toolbar_visible = True
    
    return fig, ax, current_scatter, old_scatter

def on_key_press(event):
    """Handle key press events."""
    global accumulated_points
    
    if event.key == 'c':
        with data_lock:
            accumulated_points.clear()
        print("Points cleared")

def update_accumulated_points(new_angles, new_distances, current_time):
    """Add new points and remove old ones efficiently."""
    global accumulated_points
    
    # Add new points with timestamp
    theta_rad = new_angles * np.pi / 180.0
    for i in range(len(new_angles)):
        point = {
            'theta': theta_rad[i],
            'r': new_distances[i],
            'timestamp': current_time
        }
        accumulated_points.append(point)
    
    # Remove old points (keep recent ones for performance)
    cutoff_time = current_time - POINT_LIFETIME
    # Only check oldest points to avoid full scan
    while accumulated_points and accumulated_points[0]['timestamp'] < cutoff_time:
        accumulated_points.popleft()

def animate(frame, ax, current_scatter, old_scatter):
    """Fast animation with incremental updates."""
    global latest_scan, accumulated_points
    
    current_time = time.time()
    
    # Get latest scan data
    with data_lock:
        if len(latest_scan['angles']) == 0:
            return current_scatter, old_scatter
        
        angles = latest_scan['angles'].copy()
        distances = latest_scan['distances'].copy()
        scan_time = latest_scan['timestamp']
    
    # Update accumulated points
    update_accumulated_points(angles, distances, current_time)
    
    # Prepare current scan points
    theta_current = angles * np.pi / 180.0
    if len(theta_current) > 0:
        current_points = np.column_stack((theta_current, distances))
        current_scatter.set_offsets(current_points)
    else:
        current_scatter.set_offsets(np.empty((0, 2)))
    
    # Prepare accumulated points (excluding very recent ones to avoid overlap)
    old_theta = []
    old_r = []
    cutoff_time = current_time - 0.1  # 100ms ago
    
    for point in accumulated_points:
        if point['timestamp'] < cutoff_time:
            old_theta.append(point['theta'])
            old_r.append(point['r'])
    
    if old_theta:
        old_points = np.column_stack((old_theta, old_r))
        old_scatter.set_offsets(old_points)
    else:
        old_scatter.set_offsets(np.empty((0, 2)))
    
    # Update title with stats (less frequently for performance)
    if frame % 30 == 0:  # Update every 30 frames
        total_points = len(accumulated_points)
        current_points = len(angles)
        ax.set_title(f'High-Speed LiDAR - Current: {current_points} | Total: {total_points}', 
                    pad=20)
    
    return current_scatter, old_scatter

def main():
    global scan_active
    
    device_path = get_device_path()
    if not device_path:
        print("Unsupported OS.")
        return

    # Setup plot
    fig, ax, current_scatter, old_scatter = setup_plot()
    
    # Connect key handler
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    # Start lidar thread
    lidar_thread_obj = threading.Thread(target=lidar_thread, args=(device_path,), daemon=True)
    lidar_thread_obj.start()
    
    time.sleep(2)  # Give lidar time to start
    
    try:
        # High-performance animation with blitting
        ani = FuncAnimation(
            fig, animate, 
            fargs=(ax, current_scatter, old_scatter),
            interval=1000//FPS,
            blit=True,  # Enable blitting for speed
            cache_frame_data=False
        )
        
        plt.show()
        
    except KeyboardInterrupt:
        print("\nVisualization stopped.")
    finally:
        scan_active = False
        if lidar_thread_obj.is_alive():
            lidar_thread_obj.join(timeout=2)

if __name__ == "__main__":
    main()