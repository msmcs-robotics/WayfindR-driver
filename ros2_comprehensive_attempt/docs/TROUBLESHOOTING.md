# Troubleshooting Guide

Common problems and solutions for the ROS2 Navigation System.

---

## LiDAR Issues

### Problem: LiDAR not detected

**Symptoms:**
- No `/dev/ttyUSB*` device
- "Device not found" error

**Solutions:**

1. Check USB connection:
```bash
lsusb | grep -i silicon
# Should show: Silicon Labs CP210x UART Bridge
```

2. Check device exists:
```bash
ls /dev/ttyUSB*
```

3. Try different USB port

4. Check udev rules:
```bash
cat /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Problem: Permission denied on serial port

**Symptoms:**
- "Permission denied: '/dev/ttyUSB0'"
- LiDAR motor spins but no data

**Solutions:**

1. Quick fix:
```bash
sudo chmod 666 /dev/ttyUSB0
```

2. Permanent fix:
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

3. Check group:
```bash
groups
# Should include 'dialout'
```

### Problem: No scan data

**Symptoms:**
- LiDAR running but `/scan` topic empty
- No data in RViz

**Solutions:**

1. Check topic is publishing:
```bash
ros2 topic hz /scan
```

2. Check baud rate matches LiDAR model:
   - C1, A2, A3: 460800
   - A1: 115200

3. Try different scan mode:
```bash
ros2 run rplidar_ros rplidar_node --ros-args -p scan_mode:=Standard
```

---

## SLAM Issues

### Problem: Map not building

**Symptoms:**
- RViz shows empty map
- `/map` topic not publishing

**Solutions:**

1. Check TF tree:
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```
Should show: map → odom → base_link → laser

2. Check scan topic:
```bash
ros2 topic echo /scan --once
```

3. Increase movement - SLAM needs motion to build map

### Problem: Map has ghosting/double walls

**Causes:**
- Moving too fast
- Poor odometry
- LiDAR vibration

**Solutions:**
- Move slower (0.2-0.3 m/s)
- Mount LiDAR more rigidly
- Reduce `minimum_travel_distance` in slam_params.yaml

### Problem: Loop closure not working

**Symptoms:**
- Map drifts over time
- Revisited areas don't align

**Solutions:**

1. Enable loop closure:
```yaml
# slam_params.yaml
do_loop_closing: true
```

2. Revisit start position more often

3. Lower loop closure thresholds:
```yaml
loop_match_minimum_response_coarse: 0.3
loop_match_minimum_response_fine: 0.4
```

---

## Localization Issues

### Problem: Particles scattered everywhere

**Symptoms:**
- AMCL particles spread across entire map
- Position estimate jumping

**Solutions:**

1. Set initial pose:
   - RViz → "2D Pose Estimate" tool
   - Click on map where robot is
   - Drag to set orientation

2. Increase particles:
```yaml
# amcl_params.yaml
min_particles: 1000
max_particles: 5000
```

3. Check map matches environment

### Problem: Localization drifts

**Symptoms:**
- Position slowly becomes wrong
- Particles cluster in wrong location

**Solutions:**

1. Improve odometry (if available)

2. Tune laser model:
```yaml
z_hit: 0.9
sigma_hit: 0.15
```

3. Lower update thresholds:
```yaml
update_min_d: 0.05
update_min_a: 0.1
```

### Problem: "Transform from odom to base_link unavailable"

**Symptoms:**
- AMCL won't start
- TF errors in console

**Solutions:**

1. Add static transform:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

2. Check launch file includes static TF publishers

---

## Pathfinding Issues

### Problem: "No path found"

**Symptoms:**
- Pathfinder returns None
- Navigator can't reach waypoint

**Solutions:**

1. Check waypoint is in free space:
```bash
python3 scripts/navigator.py --waypoints <file> --list
```

2. Lower obstacle threshold in pathfinder:
```python
# pathfinder.py, is_free()
if self.grid[ny][nx] < 50:  # Was 100
```

3. Check map quality - waypoint may be in unknown/occupied area

### Problem: Path goes through walls

**Causes:**
- Inflation radius too small
- Map not accurate

**Solutions:**

1. Increase inflation:
```python
# pathfinder.py
def is_free(self, gx, gy, inflation=4):  # Was 2
```

2. Regenerate map with better coverage

---

## RViz Issues

### Problem: "could not connect to display"

**Symptoms:**
- RViz crashes immediately
- Qt platform plugin error

**Causes:**
- Running over SSH without X11

**Solutions:**

1. Use physical display on robot

2. Use X11 forwarding:
```bash
ssh -X user@robot
```

3. Skip RViz:
```bash
./scripts/start_mapping.sh /dev/ttyUSB0 false
```

### Problem: Map not showing in RViz

**Solutions:**

1. Set Fixed Frame to "map"

2. Add Map display:
   - Click "Add"
   - By topic → /map → Map

3. Wait for map update (up to 5 seconds)

4. Check map server is running:
```bash
ros2 node list | grep map_server
```

---

## General ROS2 Issues

### Problem: "ros2: command not found"

**Solution:**
```bash
source /opt/ros/humble/setup.bash
```

Add to ~/.bashrc for persistence.

### Problem: "Package not found"

**Solution:**
```bash
sudo apt update
sudo apt install ros-humble-<package-name>
```

### Problem: Nodes not finding each other

**Solutions:**

1. Check ROS_DOMAIN_ID matches:
```bash
echo $ROS_DOMAIN_ID
```

2. Check network connectivity:
```bash
ros2 node list
ros2 topic list
```

3. Disable firewall temporarily:
```bash
sudo ufw disable
```

---

## Still stuck?

1. Check ROS2 logs:
```bash
~/.ros/log/
```

2. Enable debug logging:
```yaml
# In params file
debug_logging: true
```

3. Check system resources:
```bash
htop  # CPU/memory
df -h # Disk space
```

4. Restart ROS2 daemon:
```bash
ros2 daemon stop
ros2 daemon start
```
