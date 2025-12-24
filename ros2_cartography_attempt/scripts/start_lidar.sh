#!/bin/bash
# Start RPLidar node only
# Run this first, then check that /scan topic is publishing

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "  Starting RPLidar Node"
echo "  Device: /dev/rplidar (/dev/ttyUSB0)"
echo "=========================================="

# Check if device exists
if [ ! -e /dev/rplidar ] && [ ! -e /dev/ttyUSB0 ]; then
    echo "ERROR: LiDAR not found at /dev/rplidar or /dev/ttyUSB0"
    echo "Check USB connection"
    exit 1
fi

DEVICE="/dev/rplidar"
if [ ! -e /dev/rplidar ]; then
    DEVICE="/dev/ttyUSB0"
fi

echo "Using device: $DEVICE"
echo ""

ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=$DEVICE \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser \
    -p angle_compensate:=true \
    -p scan_mode:=DenseBoost
