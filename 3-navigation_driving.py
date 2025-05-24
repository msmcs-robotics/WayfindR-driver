import numpy as np
import serial
import time
import math
import json
from collections import deque

# === Config ===
MAP_PATH = "map_matrix.npy"
WAYPOINTS_PATH = "waypoints.json"
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

# Map scale - pixels per meter
PX_PER_METER = 100  
# Assuming origin at center of map grid
# Map must be square for this example:
origin_px = None  

# === Serial Setup ===
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # wait for serial connection
except Exception as e:
    print("Error opening serial port:", e)
    arduino = None

# === Load Map and Waypoints ===
map_grid = np.load(MAP_PATH)
origin_px = map_grid.shape[0] // 2

with open(WAYPOINTS_PATH, 'r') as f:
    waypoints = json.load(f)

# === Helper Functions ===

def send_command(cmd):
    if arduino:
        arduino.write((cmd + '\n').encode())
        print(f"Sent command: {cmd}")
    else:
        print(f"(Simulated) Command: {cmd}")

def meters_to_pixels(x, y):
    px = int(origin_px + x * PX_PER_METER)
    py = int(origin_px - y * PX_PER_METER)  # y inverted for numpy row indexing
    return (py, px)  # row, col

def pixels_to_meters(row, col):
    x = (col - origin_px) / PX_PER_METER
    y = (origin_px - row) / PX_PER_METER
    return (x, y)

def angle_diff(a, b):
    """Shortest difference between two angles in degrees."""
    d = (a - b + 180) % 360 - 180
    return d

def bfs_path(grid, start, goal):
    rows, cols = grid.shape
    queue = deque([start])
    visited = set([start])
    parent = {}

    directions = [(-1,0),(1,0),(0,-1),(0,1)]  # N, S, W, E

    while queue:
        current = queue.popleft()
        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            return path
        for dr, dc in directions:
            nr, nc = current[0] + dr, current[1] + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr, nc] == 0 and (nr,nc) not in visited:
                    visited.add((nr,nc))
                    parent[(nr,nc)] = current
                    queue.append((nr,nc))
    return []

def get_waypoint_by_name(name):
    for wp in waypoints:
        if wp['name'] == name:
            return wp
    return None

# === Placeholder for Localization ===
# Replace with your TinySLAM + IMU fusion code!
robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0  # meters, radians

def update_robot_pose():
    global robot_x, robot_y, robot_theta
    # TODO: Implement localization update here
    # For now, we just keep last known pose
    pass

def get_robot_grid_pos():
    return meters_to_pixels(robot_x, robot_y)

def move_to_waypoint(wp_name):
    global robot_x, robot_y, robot_theta

    wp = get_waypoint_by_name(wp_name)
    if wp is None:
        print(f"Waypoint '{wp_name}' not found!")
        return

    goal_px = (wp['y'], wp['x'])  # waypoint pixels stored as x,y, but grid is row,col (y,x)
    start_px = get_robot_grid_pos()

    print(f"Current grid pos: {start_px}, Goal grid pos: {goal_px}")

    path = bfs_path(map_grid, start_px, goal_px)
    if not path:
        print("No path found!")
        return

    for next_cell in path[1:]:
        update_robot_pose()
        dr = next_cell[0] - start_px[0]
        dc = next_cell[1] - start_px[1]

        # Determine desired heading in degrees (N=0°, E=90°, S=180°, W=270°)
        if dr == -1:
            desired_deg = 0
        elif dr == 1:
            desired_deg = 180
        elif dc == 1:
            desired_deg = 90
        elif dc == -1:
            desired_deg = 270
        else:
            print("Invalid move detected.")
            return

        current_deg = math.degrees(robot_theta) % 360
        turn_angle = angle_diff(desired_deg, current_deg)

        # Turn first
        if turn_angle < 0:
            send_command(f"L{int(abs(turn_angle))}")
        elif turn_angle > 0:
            send_command(f"R{int(turn_angle)}")
        time.sleep(0.5)  # pause after turn

        # Move forward one grid cell (assumes 1 cell = 1 meter approx)
        send_command("F")
        time.sleep(1)  # move forward time (adjust to your speed)

        # Stop after move
        send_command("S")
        time.sleep(0.2)

        # Update internal pose estimate for demo only
        robot_theta = math.radians(desired_deg)
        robot_x += dc * (1 / PX_PER_METER)
        robot_y += -dr * (1 / PX_PER_METER)

        start_px = next_cell

    print(f"Arrived at waypoint '{wp_name}'")

# === Main ===
if __name__ == "__main__":
    print("Starting navigation script...")
    while True:
        target_wp = input("Enter waypoint name to navigate to (or 'exit'): ").strip()
        if target_wp.lower() == "exit":
            break
        move_to_waypoint(target_wp)
