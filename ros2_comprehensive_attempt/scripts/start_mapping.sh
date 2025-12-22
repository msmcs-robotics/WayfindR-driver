#!/bin/bash
#
# Start SLAM mapping with LiDAR and RViz
#
# Usage:
#   ./start_mapping.sh                    # Use default serial port
#   ./start_mapping.sh /dev/rplidar       # Specify serial port
#   ./start_mapping.sh /dev/ttyUSB0 false # No RViz
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$(dirname "$SCRIPT_DIR")/launch"

# Arguments
SERIAL_PORT="${1:-/dev/ttyUSB0}"
USE_RVIZ="${2:-true}"

echo "============================================"
echo "  Starting SLAM Mapping"
echo "============================================"
echo ""
echo "LiDAR: $SERIAL_PORT"
echo "RViz:  $USE_RVIZ"
echo ""
echo "Instructions:"
echo "  - Move the LiDAR/robot around to build map"
echo "  - Watch map appear in RViz"
echo "  - When done, run: ./save_map.sh <name>"
echo "  - Press Ctrl+C to stop"
echo ""
echo "============================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Run launch file
ros2 launch "$LAUNCH_DIR/slam.launch.py" \
    serial_port:="$SERIAL_PORT" \
    use_rviz:="$USE_RVIZ"
