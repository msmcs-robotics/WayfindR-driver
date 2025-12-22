#!/bin/bash
#
# start_localization.sh - Start AMCL localization with LiDAR
#
# Usage:
#   ./start_localization.sh                    # Use default map
#   ./start_localization.sh /path/to/map.yaml  # Use specific map
#

set -e

# Default map path
DEFAULT_MAP="/home/devel/ros2_ws/maps/first_map.yaml"

# Get map path from argument or use default
MAP_PATH="${1:-$DEFAULT_MAP}"

# Check if map exists
if [ ! -f "$MAP_PATH" ]; then
    echo "Error: Map file not found: $MAP_PATH"
    echo ""
    echo "Available maps in ~/ros2_ws/maps/:"
    ls -la ~/ros2_ws/maps/*.yaml 2>/dev/null || echo "  (none found)"
    exit 1
fi

echo "============================================"
echo "  Starting AMCL Localization"
echo "============================================"
echo ""
echo "Map: $MAP_PATH"
echo ""
echo "This will launch:"
echo "  - RPLidar driver"
echo "  - Map server"
echo "  - AMCL localization"
echo "  - RViz visualization"
echo ""
echo "============================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$(dirname "$SCRIPT_DIR")/launch"

# Check if launch file exists
LAUNCH_FILE="$LAUNCH_DIR/localization_with_lidar.launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "Error: Launch file not found: $LAUNCH_FILE"
    exit 1
fi

echo "Starting localization..."
echo "Press Ctrl+C to stop"
echo ""

# Run the launch file
python3 "$LAUNCH_FILE" 2>/dev/null || ros2 launch "$LAUNCH_FILE" map:="$MAP_PATH"
