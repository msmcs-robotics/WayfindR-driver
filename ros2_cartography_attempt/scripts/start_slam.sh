#!/bin/bash
# Start SLAM Toolbox for mapping
# Run this AFTER start_lidar.sh is running

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "  Starting SLAM Toolbox (Mapping Mode)"
echo "=========================================="

# Check if /scan topic is available
echo "Checking for /scan topic..."
SCAN_CHECK=$(ros2 topic list 2>/dev/null | grep "/scan")
if [ -z "$SCAN_CHECK" ]; then
    echo "WARNING: /scan topic not found!"
    echo "Make sure start_lidar.sh is running first"
    echo ""
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/../config"

# Check for config file
if [ -f "$CONFIG_DIR/slam_toolbox_params.yaml" ]; then
    echo "Using config: $CONFIG_DIR/slam_toolbox_params.yaml"
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
        --params-file "$CONFIG_DIR/slam_toolbox_params.yaml"
else
    echo "No custom config found, using defaults"
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
        -p use_sim_time:=false \
        -p base_frame:=base_link \
        -p odom_frame:=odom \
        -p map_frame:=map \
        -p scan_topic:=/scan \
        -p mode:=mapping
fi
