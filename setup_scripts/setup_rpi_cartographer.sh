#!/bin/bash

# Raspberry Pi Cartographer Setup Script
# For Ubuntu 18.04 with ROS Melodic

# Check if script is run as root
if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

# Update system
echo "Updating system packages..."
apt-get update && apt-get upgrade -y

# Install required ROS packages
echo "Installing ROS Cartographer and RPLidar packages..."
apt-get install -y ros-melodic-cartographer-ros ros-melodic-rplidar-ros ros-melodic-robot-state-publisher

# Create catkin workspace
echo "Setting up catkin workspace..."
source /opt/ros/melodic/setup.bash
mkdir -p ~/cartographer_ws/src
cd ~/cartographer_ws/src
catkin_init_workspace

# Clone gbot_core repository
echo "Cloning gbot_core repository..."
git clone https://github.com/Andrew-rw/gbot_core.git

# Build the workspace
echo "Building workspace..."
cd ~/cartographer_ws
catkin_make
source devel/setup.bash

# Create udev rule for RPLidar
echo "Creating udev rule for RPLidar..."
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"' > /etc/udev/rules.d/rplidar.rules
udevadm control --reload-rules
udevadm trigger

# Get WiFi IP address
WIFI_IP=$(ip a | grep -E 'wlan[0-9]' -A 2 | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)

if [ -z "$WIFI_IP" ]; then
    echo "Warning: Could not detect WiFi IP address. Using first available IP."
    WIFI_IP=$(hostname -I | awk '{print $1}')
fi

# Setup ROS environment variables
echo "Setting up ROS environment variables..."
echo "export ROS_MASTER_URI=http://$WIFI_IP:11311" >> ~/.bashrc
echo "export ROS_IP=$WIFI_IP" >> ~/.bashrc
source ~/.bashrc

echo ""
echo "Raspberry Pi Cartographer setup complete!"
echo "Detected WiFi IP: $WIFI_IP"
echo "To start mapping, run:"
echo "  roslaunch gbot_core gbot.launch"
echo ""
echo "Make sure to configure ROS_MASTER_URI on the desktop to point to this IP."