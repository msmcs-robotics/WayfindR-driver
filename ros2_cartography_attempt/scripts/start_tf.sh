#!/bin/bash
# Start static transforms for testing
# These provide the TF tree: map -> odom -> base_link -> laser

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "  Starting Static Transform Publishers"
echo "=========================================="
echo ""
echo "TF Tree:"
echo "  map -> odom -> base_link -> laser"
echo ""
echo "NOTE: For a real robot, odom->base_link should come"
echo "      from wheel encoders, not a static transform."
echo ""

# odom -> base_link (static for testing - would be dynamic on real robot)
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id odom --child-frame-id base_link &

# base_link -> laser (position of LiDAR on robot)
# Adjust these values for your actual mounting position
# x=0 y=0 z=0.1 means LiDAR is 10cm above base_link center
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.1 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id laser &

echo "Static transforms started."
echo "Press Ctrl+C to stop."
wait
