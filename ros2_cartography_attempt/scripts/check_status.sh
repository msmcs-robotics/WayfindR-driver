#!/bin/bash
# Check status of all SLAM components

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "  WayfindR SLAM Status Check"
echo "  $(date)"
echo "=========================================="
echo ""

# Check LiDAR device
echo "=== LiDAR Device ==="
if [ -e /dev/rplidar ]; then
    echo "✓ /dev/rplidar exists -> $(readlink -f /dev/rplidar)"
elif [ -e /dev/ttyUSB0 ]; then
    echo "⚠ /dev/rplidar not found, but /dev/ttyUSB0 exists"
else
    echo "✗ No LiDAR device found"
fi
echo ""

# Check ROS2 nodes
echo "=== Active ROS2 Nodes ==="
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    echo "  (no nodes running)"
else
    echo "$NODES" | sed 's/^/  /'
fi
echo ""

# Check topics
echo "=== Key Topics ==="
TOPICS=$(ros2 topic list 2>/dev/null)

check_topic() {
    if echo "$TOPICS" | grep -q "^$1$"; then
        HZ=$(timeout 2 ros2 topic hz $1 2>/dev/null | head -1 | grep -oP 'average rate: \K[0-9.]+' || echo "?")
        echo "✓ $1 (${HZ} Hz)"
    else
        echo "✗ $1 (not publishing)"
    fi
}

check_topic "/scan"
check_topic "/map"
check_topic "/tf"
echo ""

# Check TF frames
echo "=== TF Frames ==="
TF_FRAMES=$(ros2 run tf2_ros tf2_echo map laser 2>&1 | head -3)
if echo "$TF_FRAMES" | grep -q "Translation"; then
    echo "✓ map -> laser transform available"
else
    echo "✗ map -> laser transform NOT available"
    echo "  (run start_tf.sh if needed)"
fi
echo ""

# Summary
echo "=== Quick Commands ==="
echo "  Check /scan data: ros2 topic echo /scan --once"
echo "  Check /map info:  ros2 topic info /map"
echo "  View TF tree:     ros2 run tf2_tools view_frames"
echo ""
