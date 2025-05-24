import serial
import math
import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar

# === Config ===
LIDAR_PORT = '/dev/ttyUSB0'
IMU_PORT = '/dev/ttyACM0'
MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10

class TinySLAM:
    def __init__(self, map_size_pixels, map_size_meters):
        self.map = np.zeros((map_size_pixels, map_size_pixels), dtype=np.uint8)
        self.origin = map_size_pixels // 2
        self.px_per_meter = map_size_pixels / map_size_meters
        self.x, self.y = 0.0, 0.0
        self.theta = 0.0

    def update_pose(self, yaw_deg):
        self.theta = math.radians(yaw_deg)

    def update_map(self, scan):
        for dist, angle in scan:
            if dist == 0.0:
                continue
            global_angle = self.theta + angle
            x = self.x + dist * math.cos(global_angle)
            y = self.y + dist * math.sin(global_angle)

            ix = int(self.origin + x * self.px_per_meter)
            iy = int(self.origin + y * self.px_per_meter)
            if 0 <= ix < self.map.shape[0] and 0 <= iy < self.map.shape[1]:
                self.map[iy][ix] = 255

    def save_map(self, filename='slam_map.png'):
        plt.imshow(self.map, cmap='gray')
        plt.title("TinySLAM 2D Map")
        plt.savefig(filename)
        plt.close()

def get_yaw(serial_imu):
    line = serial_imu.readline().decode().strip()
    try:
        return float(line)
    except:
        return 0.0

def run():
    lidar = RPLidar(LIDAR_PORT)
    imu = serial.Serial(IMU_PORT, 9600, timeout=1)
    slam = TinySLAM(MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    print("Starting SLAM mapping. Press Ctrl+C to stop.")
    try:
        for scan in lidar.iter_scans():
            yaw = get_yaw(imu)
            slam.update_pose(yaw)

            formatted_scan = [(d / 1000.0, math.radians(a)) for (_, a, d) in scan if d > 0]
            slam.update_map(formatted_scan)

    except KeyboardInterrupt:
        print("Saving and exiting...")
        slam.save_map()
        np.save("map_matrix.npy", slam.map)

    finally:
        lidar.stop()
        lidar.disconnect()
        imu.close()

if __name__ == '__main__':
    run()
