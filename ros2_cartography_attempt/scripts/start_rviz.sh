#!/bin/bash
# Start RViz2 for visualization
# Run this on a machine with a display

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "  Starting RViz2"
echo "=========================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/../config/slam_view.rviz"

if [ -f "$RVIZ_CONFIG" ]; then
    echo "Using config: $RVIZ_CONFIG"
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG"
else
    echo "No RViz config found, starting with defaults"
    echo ""
    echo "In RViz, add these displays:"
    echo "  1. Add -> By Topic -> /scan -> LaserScan"
    echo "  2. Add -> By Topic -> /map -> Map"
    echo "  3. Set Fixed Frame to 'map'"
    echo ""
    ros2 run rviz2 rviz2
fi
