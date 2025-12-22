#!/bin/bash
#
# Start AMCL localization with a saved map
#
# Usage:
#   ./start_localization.sh /path/to/map.yaml
#   ./start_localization.sh /path/to/map.yaml /dev/rplidar
#   ./start_localization.sh /path/to/map.yaml /dev/ttyUSB0 false
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$(dirname "$SCRIPT_DIR")/launch"
MAPS_DIR="$(dirname "$SCRIPT_DIR")/maps"

# Arguments
MAP_FILE="${1:-}"
SERIAL_PORT="${2:-/dev/ttyUSB0}"
USE_RVIZ="${3:-true}"

# Check map file
if [ -z "$MAP_FILE" ]; then
    echo "Usage: $0 <map.yaml> [serial_port] [use_rviz]"
    echo ""
    echo "Available maps in $MAPS_DIR:"
    ls -1 "$MAPS_DIR"/*.yaml 2>/dev/null || echo "  (none found)"
    exit 1
fi

if [ ! -f "$MAP_FILE" ]; then
    # Try maps directory
    if [ -f "$MAPS_DIR/$MAP_FILE" ]; then
        MAP_FILE="$MAPS_DIR/$MAP_FILE"
    elif [ -f "$MAPS_DIR/${MAP_FILE}.yaml" ]; then
        MAP_FILE="$MAPS_DIR/${MAP_FILE}.yaml"
    else
        echo "Error: Map file not found: $MAP_FILE"
        exit 1
    fi
fi

echo "============================================"
echo "  Starting Localization"
echo "============================================"
echo ""
echo "Map:   $MAP_FILE"
echo "LiDAR: $SERIAL_PORT"
echo "RViz:  $USE_RVIZ"
echo ""
echo "Instructions:"
echo "  1. Wait for system to start"
echo "  2. In RViz, click '2D Pose Estimate'"
echo "  3. Click on map where robot is located"
echo "  4. Drag to set orientation"
echo "  5. Watch particles converge!"
echo ""
echo "Press Ctrl+C to stop"
echo "============================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Run launch file
ros2 launch "$LAUNCH_DIR/localization.launch.py" \
    map:="$MAP_FILE" \
    serial_port:="$SERIAL_PORT" \
    use_rviz:="$USE_RVIZ"
