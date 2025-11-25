#!/usr/bin/env bash
# install_robot_node.sh
# Master installer for a headless ROS2 Humble robot node on Ubuntu 22.04 (Raspberry Pi)
# Idempotent — safe to run multiple times.

set -e

echo "============================================================"
echo "   HEADLESS ROS2 HUMBLE — ROBOT NODE INSTALLER"
echo "============================================================"

##############################
# Helper: idempotent append #
##############################
append_if_missing() {
    local file="$1"
    local line="$2"

    if [ ! -f "$file" ]; then
        sudo touch "$file"
    fi

    if grep -Fxq "$line" "$file"; then
        echo "[OK] Already present: $line"
    else
        echo "[ADD] $line → $file"
        echo "$line" | sudo tee -a "$file" >/dev/null
    fi
}

###################################
# Step 1 — System update & locale #
###################################
echo "--- Updating system ---"
sudo apt update
sudo apt upgrade -y

echo "--- Setting locale ---"
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

##########################################
# Step 2 — Ensure Universe repo enabled  #
##########################################
echo "--- Ensuring universe repo ---"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe || true

#############################################
# Step 3 — Configure ROS2 apt repo cleanly  #
#############################################
echo "--- Configuring ROS2 repository ---"

ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
ROS2_KEY="/usr/share/keyrings/ros-archive-keyring.gpg"

sudo rm -f "$ROS2_LIST" "$ROS2_KEY"

sudo apt install -y curl gnupg2

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o "$ROS2_KEY"

append_if_missing "$ROS2_LIST" \
"deb [arch=$(dpkg --print-architecture) signed-by=$ROS2_KEY] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main"

sudo apt update

echo "--- Installing Python Packages ---"
sudo apt install -y python3-venv python3-dev python3-distutils python3-pip python3-colcon-common-extensions

####################################################
# Step 4 — Install ROS2 Humble HEADLESS (ros-base) #
####################################################
echo "--- Installing ROS 2 Humble (headless) ---"

sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools

########################################
# Step 5 — Auto-source ROS environment #
########################################
echo "--- Configuring ~/.bashrc sourcing ---"

append_if_missing "$HOME/.bashrc" "source /opt/ros/humble/setup.bash"

source /opt/ros/humble/setup.bash

##################################
# Step 6 — Setup ROS2 workspace  #
##################################
echo "--- Setting up ROS2 workspace ---"

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# idempotent build
colcon build --symlink-install || true

append_if_missing "$HOME/.bashrc" "source ~/ros2_ws/install/setup.bash"

###################################################
# Step 7 — Install headless SLAM / Nav2 packages  #
###################################################
echo "--- Installing SLAM + Nav2 (headless) ---"

sudo apt update

# Core TF + robot state
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    ros-humble-robot-state-publisher

# SLAM
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros

# Nav2
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-localization

########################################
# Step 8 — IMU + sensor driver support #
########################################
echo "--- Installing IMU & sensor drivers ---"

sudo apt install -y \
    ros-humble-imu-tools \
    ros-humble-imu-filter-madgwick

###################################
# Step 9 — Python dependencies    #
###################################
echo "--- Installing Python dependencies ---"

pip3 install --user \
    pyserial \
    pyyaml \
    numpy \
    scipy \
    transforms3d \
    flask \
    flask-cors

#############################################
# Step 10 — Install Explore Lite (from src) #
#############################################
echo "--- Installing Explore Lite (source) ---"

cd ~/ros2_ws/src

if [ ! -d "m-explore" ]; then
    git clone -b humble https://github.com/robo-friends/m-explore.git
else
    echo "[OK] m-explore already present"
fi

cd ~/ros2_ws

echo "--- rosdep install ---"
rosdep update
rosdep install --from-paths src --ignore-src -y || true

echo "--- Building workspace ---"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || true

################################
# DONE                         #
################################
echo ""
echo "============================================================"
echo "   INSTALLATION COMPLETE — Robot Node Ready"
echo "============================================================"
echo ""
echo "Run:"
echo "   source ~/.bashrc"
echo ""
echo "Test ROS communication:"
echo "   ros2 topic list"
echo ""
echo "Next steps:"
echo "   - Add systemd services for LiDAR, SLAM, Nav2, etc."
echo "   - Create URDF + TF tree"
echo "   - Set up RPLIDAR or other sensors"
echo ""
echo "This system is now ready for remote SLAM, Nav2, and mapping."
echo ""
