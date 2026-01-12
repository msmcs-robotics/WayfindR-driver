#!/bin/bash
################################################################################
# WayfindR SLAM Mapping Script
################################################################################
#
# This script launches the WayfindR navigation system in SLAM mapping mode.
# Use this to create new maps or update existing maps of your environment.
#
# SLAM (Simultaneous Localization and Mapping) Mode:
#   - Creates a map while the robot moves through the environment
#   - Does NOT require an existing map file
#   - Use keyboard teleop or manual control to drive the robot
#   - Save the map when done using: ros2 run nav2_map_server map_saver_cli
#
# Usage:
#   ./start_mapping.sh [OPTIONS]
#
# Options:
#   --no-rviz              Don't launch RViz visualization
#   --serial-port PORT     LiDAR serial port (default: /dev/ttyUSB0)
#   --help                 Show this help message
#
# Examples:
#   ./start_mapping.sh
#   ./start_mapping.sh --no-rviz
#   ./start_mapping.sh --serial-port /dev/rplidar
#
# After Mapping:
#   Save your map with:
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
#
################################################################################

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
USE_RVIZ="true"
SERIAL_PORT="/dev/ttyUSB0"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --serial-port)
            SERIAL_PORT="$2"
            shift 2
            ;;
        --help|-h)
            grep "^#" "$0" | sed 's/^# \?//' | head -n -1
            exit 0
            ;;
        *)
            echo -e "${RED}Error: Unknown option: $1${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Get directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

# Print header
echo -e "${BLUE}╔════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║${NC}          ${GREEN}WayfindR Navigation System - SLAM Mapping Mode${NC}          ${BLUE}║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if LiDAR device exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${YELLOW}Warning: LiDAR device not found at $SERIAL_PORT${NC}"
    echo -e "${YELLOW}Available USB devices:${NC}"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "  No USB devices found"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}Aborted by user${NC}"
        exit 1
    fi
fi

# Print configuration
echo -e "${GREEN}Configuration:${NC}"
echo -e "  Mode:         ${YELLOW}SLAM Mapping${NC}"
echo -e "  RViz:         ${YELLOW}$USE_RVIZ${NC}"
echo -e "  Serial Port:  ${YELLOW}$SERIAL_PORT${NC}"
echo ""

# Print instructions
echo -e "${GREEN}Instructions:${NC}"
echo -e "  1. Wait for all nodes to start (check terminal output)"
echo -e "  2. Use keyboard teleop or joystick to drive the robot"
echo -e "  3. Drive slowly to create a good quality map"
echo -e "  4. Cover all areas you want to map"
echo -e "  5. When done, save the map with:"
echo -e "     ${YELLOW}ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map${NC}"
echo ""

# Countdown
echo -e "${BLUE}Starting in 3 seconds... (Ctrl+C to cancel)${NC}"
sleep 1
echo -e "${BLUE}Starting in 2 seconds...${NC}"
sleep 1
echo -e "${BLUE}Starting in 1 second...${NC}"
sleep 1
echo ""

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Launch command
echo -e "${GREEN}Launching SLAM Mapping...${NC}"
echo ""

ros2 launch "$PKG_DIR/launch/bringup.launch.py" \
    mode:=slam \
    use_rviz:=$USE_RVIZ \
    serial_port:=$SERIAL_PORT

# If we get here, the launch file exited
echo ""
echo -e "${YELLOW}SLAM Mapping session ended${NC}"
echo -e "${GREEN}Don't forget to save your map!${NC}"
echo -e "  ${YELLOW}ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map${NC}"
echo ""
