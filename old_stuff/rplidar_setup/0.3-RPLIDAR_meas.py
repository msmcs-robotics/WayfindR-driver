import platform
from math import floor
from adafruit_rplidar import RPLidar


raw = True

BAUDRATE: int = 115200
TIMEOUT: int = 1

MAC_DEVICE_PATH: str = '/dev/cu.usbserial-0001'
LINUX_DEVICE_PATH: str = '/dev/ttyUSB0'
WINDOWS_DEVICE_PATH: str = 'COM7'

# Automatically detect OS and select device path
system = platform.system()
if system == 'Windows':
    DEVICE_PATH: str = WINDOWS_DEVICE_PATH
elif system == 'Darwin':
    DEVICE_PATH: str = MAC_DEVICE_PATH
elif system == 'Linux':
    DEVICE_PATH: str = LINUX_DEVICE_PATH
else:
    DEVICE_PATH: str = ''


# Setup the RPLidar
PORT_NAME = DEVICE_PATH
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT


lidar = RPLidar(None, PORT_NAME, timeout=3)

# used to scale data to fit on the screen
max_distance = 0


def process_data(data):
    print(data)


scan_data = [0] * 360

try:
    #    print(lidar.get_info())
    for scan in lidar.iter_scans():
        for _, angle, distance in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)

except KeyboardInterrupt:
    print("Stopping.")
lidar.stop()
lidar.disconnect()