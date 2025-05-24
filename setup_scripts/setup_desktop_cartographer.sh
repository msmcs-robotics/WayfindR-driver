#!/bin/bash

# Desktop/Laptop Cartographer Visualization Setup Script
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
echo "Installing ROS Cartographer visualization packages..."
apt-get install -y ros-melodic-cartographer-rviz ros-melodic-rviz

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

# Get WiFi IP address
WIFI_IP=$(ip a | grep -E 'wlan[0-9]' -A 2 | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)

if [ -z "$WIFI_IP" ]; then
    echo "Warning: Could not detect WiFi IP address. Using first available IP."
    WIFI_IP=$(hostname -I | awk '{print $1}')
fi

# Setup ROS environment variables
read -p "Enter Raspberry Pi's IP address: " RPI_IP
echo "export ROS_MASTER_URI=http://$RPI_IP:11311" >> ~/.bashrc
echo "export ROS_IP=$WIFI_IP" >> ~/.bashrc
source ~/.bashrc

echo ""
echo "Desktop Cartographer visualization setup complete!"
echo "Detected WiFi IP: $WIFI_IP"
echo "Configured to connect to Raspberry Pi at: $RPI_IP"
echo ""
echo "To start visualization:"
echo "1. First run on Raspberry Pi: roslaunch gbot_core gbot.launch"
echo "2. Then on this machine run: roslaunch gbot_core visualization.launch"
echo ""
echo "Make sure both devices are on the same network and can communicate."