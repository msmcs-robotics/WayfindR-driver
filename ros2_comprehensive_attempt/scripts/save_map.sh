#!/bin/bash
#
# Save current SLAM map
#
# Usage:
#   ./save_map.sh my_map           # Saves to maps/my_map.yaml
#   ./save_map.sh /path/to/my_map  # Saves to specified path
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="$(dirname "$SCRIPT_DIR")/maps"

# Check argument
if [ -z "$1" ]; then
    echo "Usage: $0 <map_name>"
    echo ""
    echo "Example: $0 office_floor1"
    exit 1
fi

MAP_NAME="$1"

# Determine full path
if [[ "$MAP_NAME" == /* ]]; then
    # Absolute path
    MAP_PATH="$MAP_NAME"
else
    # Relative to maps directory
    MAP_PATH="$MAPS_DIR/$MAP_NAME"
fi

# Remove extension if provided
MAP_PATH="${MAP_PATH%.yaml}"
MAP_PATH="${MAP_PATH%.pgm}"

echo "============================================"
echo "  Saving Map"
echo "============================================"
echo ""
echo "Saving to: $MAP_PATH"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Ensure maps directory exists
mkdir -p "$MAPS_DIR"

# Save map using SLAM Toolbox service
echo "Saving map image and metadata..."
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$MAP_PATH'}}"

# Also serialize pose graph for localization
echo ""
echo "Saving pose graph..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$MAP_PATH'}"

echo ""
echo "============================================"
echo "  Map Saved!"
echo "============================================"
echo ""
echo "Files created:"
echo "  ${MAP_PATH}.yaml     - Map metadata"
echo "  ${MAP_PATH}.pgm      - Map image"
echo "  ${MAP_PATH}_posegraph.data"
echo "  ${MAP_PATH}_posegraph.posegraph"
echo ""
echo "To use for localization:"
echo "  ./start_localization.sh ${MAP_PATH}.yaml"
echo ""
