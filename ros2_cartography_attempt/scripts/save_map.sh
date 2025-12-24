#!/bin/bash
# Save the current map from SLAM Toolbox

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="$SCRIPT_DIR/../maps"
mkdir -p "$MAPS_DIR"

# Default map name with timestamp
MAP_NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
MAP_PATH="$MAPS_DIR/$MAP_NAME"

echo "=========================================="
echo "  Saving Map: $MAP_NAME"
echo "  Location: $MAPS_DIR/"
echo "=========================================="

# Save map using nav2_map_server
echo "Saving map..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p save_map_timeout:=10000

if [ -f "${MAP_PATH}.pgm" ]; then
    echo ""
    echo "SUCCESS! Map saved:"
    echo "  - ${MAP_PATH}.pgm (image)"
    echo "  - ${MAP_PATH}.yaml (metadata)"
    echo ""
    ls -la "${MAP_PATH}"*
else
    echo ""
    echo "ERROR: Map save may have failed."
    echo "Make sure SLAM Toolbox is running and has created a map."
fi
