#!/bin/bash
################################################################################
# WayfindR Simulation Mode Script
################################################################################
#
# This script launches the WayfindR navigation system in simulation mode.
# Use this for testing with rosbag playback or Gazebo simulation.
#
# Simulation Mode Features:
#   - Sets use_sim_time=true for all nodes
#   - Disables RPLidar driver (expects /scan from rosbag or simulator)
#   - Can be used with any mode: SLAM, localization, or navigation
#   - Useful for testing algorithms before deploying to hardware
#
# Usage:
#   ./start_simulation.sh --mode MODE [OPTIONS]
#
# Required:
#   --mode MODE            Mode: slam, localization, or navigation
#
# Options:
#   --map PATH             Path to map YAML (required for localization/navigation)
#   --no-rviz              Don't launch RViz visualization
#   --help                 Show this help message
#
# Examples:
#   # SLAM with rosbag
#   ./start_simulation.sh --mode slam
#   ros2 bag play my_scan_data.bag --clock
#
#   # Navigation with rosbag
#   ./start_simulation.sh --mode navigation --map ~/maps/office.yaml
#   ros2 bag play my_scan_data.bag --clock
#
#   # Localization only
#   ./start_simulation.sh --mode localization --map ~/maps/office.yaml
#
# Before Starting:
#   Make sure you have:
#   - A rosbag with /scan data, or
#   - A Gazebo simulation publishing /scan
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
MODE=""
MAP_FILE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            MODE="$2"
            shift 2
            ;;
        --map)
            MAP_FILE="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ="false"
            shift
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
echo -e "${BLUE}║${NC}          ${GREEN}WayfindR Navigation System - Simulation Mode${NC}           ${BLUE}║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Validate mode
if [ -z "$MODE" ]; then
    echo -e "${RED}Error: Mode is required${NC}"
    echo -e "Usage: $0 --mode slam|localization|navigation [OPTIONS]"
    echo -e "Use --help for more information"
    exit 1
fi

if [[ ! "$MODE" =~ ^(slam|localization|navigation)$ ]]; then
    echo -e "${RED}Error: Invalid mode: $MODE${NC}"
    echo -e "Mode must be one of: slam, localization, navigation"
    exit 1
fi

# Validate map file for localization/navigation
if [[ "$MODE" =~ ^(localization|navigation)$ ]]; then
    if [ -z "$MAP_FILE" ]; then
        echo -e "${RED}Error: Map file is required for $MODE mode${NC}"
        echo -e "Usage: $0 --mode $MODE --map /path/to/map.yaml"
        exit 1
    fi
    
    if [ ! -f "$MAP_FILE" ]; then
        echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
        echo ""
        echo -e "${YELLOW}Available maps in $PKG_DIR/maps/:${NC}"
        ls -1 "$PKG_DIR/maps/"*.yaml 2>/dev/null || echo "  No maps found"
        exit 1
    fi
fi

# Print configuration
echo -e "${GREEN}Configuration:${NC}"
echo -e "  Mode:         ${YELLOW}$MODE (SIMULATION)${NC}"
if [ -n "$MAP_FILE" ]; then
    echo -e "  Map:          ${YELLOW}$MAP_FILE${NC}"
fi
echo -e "  RViz:         ${YELLOW}$USE_RVIZ${NC}"
echo -e "  use_sim_time: ${YELLOW}true${NC}"
echo ""

# Print instructions
echo -e "${GREEN}Instructions:${NC}"
echo -e "  1. Start this simulation launch"
echo -e "  2. In another terminal, play your rosbag with --clock:"
echo -e "     ${YELLOW}ros2 bag play my_data.bag --clock${NC}"
echo -e "  3. Or start your Gazebo simulation"
echo -e "  4. Watch the robot operate in RViz"
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
echo -e "${GREEN}Launching Simulation Mode...${NC}"
echo ""

# Build launch command
LAUNCH_CMD="ros2 launch $PKG_DIR/launch/bringup.launch.py mode:=$MODE use_sim_time:=true use_rviz:=$USE_RVIZ"

if [ -n "$MAP_FILE" ]; then
    LAUNCH_CMD="$LAUNCH_CMD map:=$MAP_FILE"
fi

# Execute
eval $LAUNCH_CMD

# If we get here, the launch file exited
echo ""
echo -e "${YELLOW}Simulation session ended${NC}"
echo ""
