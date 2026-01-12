#!/bin/bash
################################################################################
# WayfindR Full Navigation Script
################################################################################
#
# This script launches the WayfindR navigation system in full navigation mode.
# Use this for autonomous navigation with localization, planning, and control.
#
# Full Navigation Mode:
#   - Requires an existing map file
#   - Uses AMCL for localization on the map
#   - Includes Nav2 planning and control for autonomous navigation
#   - Supports waypoint following
#   - Optional PI_API integration via cmd_vel bridge
#
# Usage:
#   ./start_navigation.sh --map /path/to/map.yaml [OPTIONS]
#
# Required:
#   --map PATH             Path to map YAML file
#
# Options:
#   --no-rviz              Don't launch RViz visualization
#   --serial-port PORT     LiDAR serial port (default: /dev/ttyUSB0)
#   --with-bridge          Enable cmd_vel bridge for PI_API
#   --pi-api-url URL       PI_API endpoint URL (default: http://localhost:8000)
#   --help                 Show this help message
#
# Examples:
#   ./start_navigation.sh --map ~/maps/office.yaml
#   ./start_navigation.sh --map ~/maps/office.yaml --no-rviz
#   ./start_navigation.sh --map ~/maps/office.yaml --with-bridge --pi-api-url http://192.168.1.100:8000
#
# After Starting:
#   - Set initial pose in RViz using "2D Pose Estimate"
#   - Send navigation goals using "2D Nav Goal" in RViz
#   - Or use waypoint_manager.py for mission planning
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
USE_BRIDGE="false"
PI_API_URL="http://localhost:8000"
MAP_FILE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --serial-port)
            SERIAL_PORT="$2"
            shift 2
            ;;
        --with-bridge)
            USE_BRIDGE="true"
            shift
            ;;
        --pi-api-url)
            PI_API_URL="$2"
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
echo -e "${BLUE}║${NC}        ${GREEN}WayfindR Navigation System - Full Navigation Mode${NC}        ${BLUE}║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Validate required arguments
if [ -z "$MAP_FILE" ]; then
    echo -e "${RED}Error: Map file is required${NC}"
    echo -e "Usage: $0 --map /path/to/map.yaml [OPTIONS]"
    echo -e "Use --help for more information"
    exit 1
fi

# Check if map file exists
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
    echo ""
    echo -e "${YELLOW}Available maps in $PKG_DIR/maps/:${NC}"
    ls -1 "$PKG_DIR/maps/"*.yaml 2>/dev/null || echo "  No maps found"
    exit 1
fi

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
echo -e "  Mode:         ${YELLOW}Full Navigation${NC}"
echo -e "  Map:          ${YELLOW}$MAP_FILE${NC}"
echo -e "  RViz:         ${YELLOW}$USE_RVIZ${NC}"
echo -e "  Serial Port:  ${YELLOW}$SERIAL_PORT${NC}"
echo -e "  cmd_vel Bridge: ${YELLOW}$USE_BRIDGE${NC}"
if [ "$USE_BRIDGE" = "true" ]; then
    echo -e "  PI_API URL:   ${YELLOW}$PI_API_URL${NC}"
fi
echo ""

# Print instructions
echo -e "${GREEN}Instructions:${NC}"
echo -e "  1. Wait for all nodes to start (check terminal output)"
echo -e "  2. In RViz, set initial pose using '2D Pose Estimate'"
echo -e "  3. Send navigation goals using '2D Nav Goal'"
echo -e "  4. Or use waypoint_manager.py for mission planning"
echo -e "  5. Monitor robot status in RViz and terminal"
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
echo -e "${GREEN}Launching Full Navigation...${NC}"
echo ""

ros2 launch "$PKG_DIR/launch/bringup.launch.py" \
    mode:=navigation \
    map:="$MAP_FILE" \
    use_rviz:=$USE_RVIZ \
    serial_port:=$SERIAL_PORT \
    use_cmd_vel_bridge:=$USE_BRIDGE \
    pi_api_url:=$PI_API_URL

# If we get here, the launch file exited
echo ""
echo -e "${YELLOW}Navigation session ended${NC}"
echo ""
