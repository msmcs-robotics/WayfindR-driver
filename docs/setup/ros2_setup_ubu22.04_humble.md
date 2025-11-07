> Ubuntu 22.04, ROS2 Humble, Slamtec C1M1


# ROS2 Humble + Slamtec C1M1 Lidar Setup Guide

Complete setup guide for ROS2 Humble on Ubuntu 22.04 with Slamtec C1M1 lidar module.

---

## Table of Contents
1. [Initial ROS2 Installation](#initial-ros2-installation)
2. [Fixing APT Source Conflicts](#fixing-apt-source-conflicts)
3. [Installing Slamtec Driver](#installing-slamtec-driver)
4. [Reinstalling Slamtec Driver](#reinstalling-slamtec-driver)
5. [Testing the Lidar](#testing-the-lidar)
6. [Visualizing in RViz2](#visualizing-in-rviz2)

---

## Initial ROS2 Installation

### Script: `install_ros.sh`

```bash
#!/usr/bin/env bash
# install_ros2_humble.sh
# Usage: bash install_ros2_humble.sh
# This script installs ROS 2 Humble on Ubuntu 22.04, with checks and cleanup.

set -e  # exit on error
set -u  # treat unset variables as error

echo "=== INSTALL ROS 2 HUMBLE ON UBUNTU 22.04 ==="

# 1. Update & upgrade system
echo "--- Step 1: system update/upgrade"
sudo apt update
sudo apt upgrade -y

# 2. Locale setup
echo "--- Step 2: locale setup"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "locale set to:" $(locale | grep LANG)

# 3. Ensure Universe repository
echo "--- Step 3: ensure Universe repository"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# 4. Add ROS 2 apt repository (with cleanup if old)
ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
ROS2_KEY="/usr/share/keyrings/ros-archive-keyring.gpg"

echo "--- Step 4: configure ROS 2 apt repository"
if [ -f "$ROS2_LIST" ]; then
  echo "Found existing $ROS2_LIST — removing to clean old sources"
  sudo rm -f "$ROS2_LIST"
fi
if [ -f "$ROS2_KEY" ]; then
  echo "Found existing ROS 2 keyring — removing"
  sudo rm -f "$ROS2_KEY"
fi

sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o "$ROS2_KEY"
echo "deb [arch=$(dpkg --print-architecture) signed-by=${ROS2_KEY}] http://packages.ros.org/ros2/ubuntu \
      $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
     sudo tee "$ROS2_LIST" > /dev/null

# 5. Install ROS 2 desktop
echo "--- Step 5: install ROS 2 Humble desktop"
sudo apt update
sudo apt install -y ros-humble-desktop
# Optional dev tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# 6. Source setup script automatically
echo "--- Step 6: environment setup"
BASHRC="$HOME/.bashrc"
SOURCE_LINE="source /opt/ros/humble/setup.bash"
if grep -Fxq "$SOURCE_LINE" "$BASHRC"; then
  echo "Already sourcing ROS 2 setup in $BASHRC"
else
  echo "$SOURCE_LINE" >> "$BASHRC"
  echo "Added sourcing line to $BASHRC"
fi
# Immediately source for this session
source /opt/ros/humble/setup.bash

# 7. Setup a ROS2 workspace
echo "--- Step 7: create ROS2 workspace"
WS_DIR="$HOME/ros2_ws"
if [ ! -d "$WS_DIR/src" ]; then
  mkdir -p "$WS_DIR/src"
  echo "Created workspace at $WS_DIR"
else
  echo "Workspace $WS_DIR already exists"
fi

cd "$WS_DIR"
colcon build --symlink-install || {
  echo "Workspace build failed or nothing to build – continuing anyway"
}

echo "Add workspace source to bashrc if not present"
WS_SOURCE_LINE="source $WS_DIR/install/setup.bash"
if grep -Fxq "$WS_SOURCE_LINE" "$BASHRC"; then
  echo "Workspace sourcing already in $BASHRC"
else
  echo "$WS_SOURCE_LINE" >> "$BASHRC"
  echo "Added workspace sourcing to $BASHRC"
fi

echo "Installation complete. Please open a new terminal or run 'source ~/.bashrc'."

echo "You can test via:"
echo "  ros2 run demo_nodes_cpp talker"
echo "  ros2 run demo_nodes_py listener"

exit 0
```

**Run it:**
```bash
bash install_ros.sh
```

---

## Fixing APT Source Conflicts

If you encounter duplicate source errors, use this script.

### Script: `ros_fix_apt_source.sh`

```bash
#!/usr/bin/env bash
# fix_ros2_apt_sources.sh
# Check for duplicate ROS2 apt source entries and clean them up,
# then continue installation if needed.

set -e
set -u

echo "=== FIXING ROS2 APT SOURCE CONFLICTS ==="

# Path definitions
ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
ROS2_SOURCES="/etc/apt/sources.list.d/ros2.sources"

# If duplicate .sources file exists, move it aside
if [ -f "$ROS2_SOURCES" ]; then
  echo "Found duplicate source file: $ROS2_SOURCES"
  sudo mv "$ROS2_SOURCES" "${ROS2_SOURCES}.backup"
  echo "Moved $ROS2_SOURCES → ${ROS2_SOURCES}.backup"
else
  echo "No duplicate .sources file found; OK"
fi

# Ensure the .list file exists and is correct
if [ -f "$ROS2_LIST" ]; then
  echo "ROS2 list file exists: $ROS2_LIST"
else
  echo "ROS2 list file not found; creating it"
  sudo tee "$ROS2_LIST" > /dev/null <<EOF
deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo \$UBUNTU_CODENAME) main
EOF
  echo "Created $ROS2_LIST"
fi

# Update apt sources
echo "Updating package list..."
sudo apt update

# Optionally install ROS2 if not yet installed
echo "Installing ROS2 Humble desktop (if not already)..."
sudo apt install -y ros-humble-desktop

echo "Fix and installation steps complete. Please open a new terminal."
exit 0
```

**Run it:**
```bash
bash ros_fix_apt_source.sh
```

---

## Installing Slamtec Driver

### First-Time Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Reinstalling Slamtec Driver

If you need to reinstall the driver, choose one of these options:

### Option 1: Clean Just the Slamtec Package (Recommended)

```bash
cd ~/ros2_ws

# Remove build artifacts for just sllidar_ros2
rm -rf build/sllidar_ros2 install/sllidar_ros2 log/

# Remove the source if you want to re-clone it
rm -rf src/sllidar_ros2

# Re-clone
cd src
git clone https://github.com/Slamtec/sllidar_ros2.git

# Rebuild just that package
cd ~/ros2_ws
colcon build --packages-select sllidar_ros2 --symlink-install

# Source the workspace
source install/setup.bash
```

### Option 2: Clean Entire Workspace Build (Safer)

```bash
cd ~/ros2_ws

# Remove all build artifacts but keep your source code
rm -rf build/ install/ log/

# Rebuild everything
colcon build --symlink-install

source install/setup.bash
```

### Option 3: Nuclear Option (Start Fresh)

```bash
# Delete the whole workspace
rm -rf ~/ros2_ws

# Recreate it from scratch
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Testing the Lidar

### Launch the Lidar Node

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

### Expected Output

You should see:
```
[INFO] [sllidar_node]: SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:1.0.1
[INFO] [sllidar_node]: SLLidar S/N: C117E0F6C0E292CDB5E099F044C7400A
[INFO] [sllidar_node]: Firmware Ver: 1.02
[INFO] [sllidar_node]: Hardware Rev: 18
[INFO] [sllidar_node]: SLLidar health status : OK.
[INFO] [sllidar_node]: current scan mode: Standard, sample rate: 5 Khz, max_distance: 16.0 m, scan frequency:10.0 Hz
```

### Troubleshooting

If you see `Package 'sllidar_ros2' not found`, make sure you've sourced your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

Or open a new terminal (if you added the source line to `.bashrc`).

### Verify Data is Publishing

In a **new terminal**:
```bash
# Check if scan data is being published
ros2 topic list

# Should see /scan in the list

# Check publishing rate (should be ~10 Hz)
ros2 topic hz /scan

# View raw scan data
ros2 topic echo /scan
```

---

## Visualizing in RViz2

### Step-by-Step Guide

1. **Keep the lidar node running** in Terminal 1:
   ```bash
   ros2 launch sllidar_ros2 sllidar_c1_launch.py
   ```

2. **Open RViz2** in Terminal 2:
   ```bash
   rviz2
   ```

3. **Add LaserScan Display:**
   - Click **Add** button (bottom left)
   - Select **By topic** tab
   - Find `/scan` → **LaserScan**
   - Click **OK**

4. **Change Fixed Frame:**
   - In the left panel under **Global Options**
   - Find **Fixed Frame** dropdown (currently shows `map`)
   - Change it to `laser_frame` or `laser`

5. **You should now see:**
   - Red dots showing obstacles detected by the lidar
   - The dots update in real-time as the lidar spins
   - Objects closer to the sensor appear as denser point clouds

### Saving Your RViz Configuration

Once you have it set up:
- **File** → **Save Config As**
- Save as `~/ros2_ws/lidar_view.rviz`

Next time, load it directly:
```bash
rviz2 -d ~/ros2_ws/lidar_view.rviz
```

---

## Next Steps

Now that your lidar is working and visualized, you're ready for:

1. **SLAM (Mapping)**: Install and run SLAM Toolbox
   ```bash
   sudo apt install ros-humble-slam-toolbox
   ros2 launch slam_toolbox online_async_launch.py
   ```

2. **Navigation**: Install Nav2 stack
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

3. **Robot Integration**: Connect motors and create a robot description (URDF)

---

## Common Issues

### Permission Denied on /dev/ttyUSB0

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### Package Not Found

Always source your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### Lidar Not Spinning

Check power and USB connection. The C1M1 should have an LED indicator when powered.

---

## Hardware Info

**Slamtec C1M1 Lidar Specs:**
- Max Range: 16 meters
- Scan Rate: 10 Hz
- Sample Rate: 5 kHz
- Interface: USB (appears as /dev/ttyUSB0)
- Baudrate: 460800
