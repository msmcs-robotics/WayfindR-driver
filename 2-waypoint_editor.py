import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import tkinter as tk
from tkinter import simpledialog, messagebox, filedialog
import json
import os

# === Load map ===
MAP_IMAGE = "slam_map.png"
MAP_MATRIX = "map_matrix.npy"
WAYPOINT_FILE = "waypoints.json"

waypoints = []

# === Helper GUI popup ===
def get_name_popup():
    root = tk.Tk()
    root.withdraw()
    name = simpledialog.askstring("Waypoint Name", "Enter a name for the waypoint:")
    root.destroy()
    return name

def confirm_popup(action="Add"):
    root = tk.Tk()
    root.withdraw()
    result = messagebox.askyesno(f"{action} Waypoint", f"Do you want to {action.lower()} this waypoint?")
    root.destroy()
    return result

# === Waypoint Storage ===
def save_waypoints():
    with open(WAYPOINT_FILE, "w") as f:
        json.dump(waypoints, f)
    print(f"Saved {len(waypoints)} waypoints.")

def load_waypoints():
    if os.path.exists(WAYPOINT_FILE):
        with open(WAYPOINT_FILE, "r") as f:
            return json.load(f)
    return []

# === Plot setup ===
fig, ax = plt.subplots()
map_img = mpimg.imread(MAP_IMAGE)
ax.imshow(map_img, cmap='gray')
waypoints = load_waypoints()

waypoint_plot = []

def draw_waypoints():
    global waypoint_plot
    for plot in waypoint_plot:
        plot.remove()
    waypoint_plot = []
    for wp in waypoints:
        p = ax.plot(wp["x"], wp["y"], "ro")[0]
        ax.text(wp["x"], wp["y"], wp["name"], color='red', fontsize=8)
        waypoint_plot.append(p)
    fig.canvas.draw()

def on_click(event):
    if event.xdata is None or event.ydata is None:
        return
    x, y = int(event.xdata), int(event.ydata)
    name = get_name_popup()
    if not name:
        return
    if confirm_popup("Add"):
        waypoints.append({"x": x, "y": y, "name": name})
        draw_waypoints()

def on_key(event):
    if event.key == 's':
        save_waypoints()
    elif event.key == 'd':
        delete_waypoint()
    elif event.key == 'm':
        move_waypoint()

def delete_waypoint():
    name = get_name_popup()
    global waypoints
    if not name:
        return
    if confirm_popup("Delete"):
        waypoints = [wp for wp in waypoints if wp["name"] != name]
        draw_waypoints()

def move_waypoint():
    name = get_name_popup()
    if not name:
        return
    match = [wp for wp in waypoints if wp["name"] == name]
    if not match:
        messagebox.showerror("Not found", "Waypoint not found.")
        return

    print("Click new location for waypoint:", name)

    def move_click(event):
        x, y = int(event.xdata), int(event.ydata)
        if confirm_popup("Move"):
            match[0]["x"] = x
            match[0]["y"] = y
            draw_waypoints()
        fig.canvas.mpl_disconnect(cid_move)

    cid_move = fig.canvas.mpl_connect("button_press_event", move_click)

# === Init ===
draw_waypoints()
fig.canvas.mpl_connect("button_press_event", on_click)
fig.canvas.mpl_connect("key_press_event", on_key)

plt.title("Click to Add Waypoint | Press 's' to Save | 'd' to Delete | 'm' to Move")
plt.show()
