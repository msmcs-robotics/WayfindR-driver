#!/bin/bash
# Replay recorded LiDAR bag for SLAM map creation or localization testing
# Usage: ./replay_for_slam.sh <bag_name> [mode] [rate]
#
# Arguments:
#   bag_name (required) - Name/path of the bag to replay
#   mode (optional)     - Replay mode: mapping, localization, or playback
#                         mapping:      Replay with SLAM for map creation (default)
#                         localization: Replay with existing map for localization test
#                         playback:     Just replay data (no SLAM)
#   rate (optional)     - Playback rate multiplier (default: 1.0)
#                         0.5 = half speed, 2.0 = double speed
#
# Examples:
#   ./replay_for_slam.sh my_session                    # Map creation at normal speed
#   ./replay_for_slam.sh my_session mapping 0.5        # Map creation at half speed
#   ./replay_for_slam.sh my_session localization       # Test localization
#   ./replay_for_slam.sh my_session playback 2.0       # Fast playback without SLAM

set -e  # Exit on error

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Parse arguments
BAG_NAME="$1"
MODE="${2:-mapping}"
RATE="${3:-1.0}"

# ROS2 setup
source /opt/ros/humble/setup.bash

# Validate arguments
if [ -z "$BAG_NAME" ]; then
    echo -e "${RED}ERROR: Bag name required${NC}"
    echo ""
    echo "Usage: $0 <bag_name> [mode] [rate]"
    echo ""
    echo "Modes:"
    echo "  mapping      - Create map with SLAM (default)"
    echo "  localization - Test localization with existing map"
    echo "  playback     - Just replay data (no SLAM)"
    echo ""
    echo "Examples:"
    echo "  $0 my_session"
    echo "  $0 my_session mapping 0.5"
    echo "  $0 my_session localization"
    exit 1
fi

# Check if bag exists
if [ ! -d "$BAG_NAME" ]; then
    echo -e "${RED}ERROR: Bag not found: $BAG_NAME${NC}"
    exit 1
fi

# Validate mode
if [[ ! "$MODE" =~ ^(mapping|localization|playback)$ ]]; then
    echo -e "${RED}ERROR: Invalid mode: $MODE${NC}"
    echo "Valid modes: mapping, localization, playback"
    exit 1
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  LiDAR Bag Replay for SLAM${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Show bag info
echo -e "${CYAN}Bag information:${NC}"
ros2 bag info "$BAG_NAME" | head -15
echo ""

# Configuration based on mode
case "$MODE" in
    mapping)
        echo -e "${GREEN}Mode: MAPPING (Create new map with SLAM)${NC}"
        SLAM_CMD="slam_toolbox"
        USE_SIM_TIME="true"
        ;;
    localization)
        echo -e "${GREEN}Mode: LOCALIZATION (Test with existing map)${NC}"
        SLAM_CMD="amcl"
        USE_SIM_TIME="true"
        ;;
    playback)
        echo -e "${GREEN}Mode: PLAYBACK (Data replay only)${NC}"
        SLAM_CMD=""
        USE_SIM_TIME="false"
        ;;
esac

echo -e "Playback rate: ${YELLOW}${RATE}x${NC}"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up...${NC}"

    # Kill all background processes in this script's process group
    jobs -p | xargs -r kill 2>/dev/null || true

    # Give processes time to terminate
    sleep 1

    echo -e "${GREEN}Cleanup complete${NC}"
}

# Set trap for cleanup
trap cleanup EXIT INT TERM

# Mode-specific instructions and execution
case "$MODE" in
    mapping)
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  SLAM Mapping Instructions${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""
        echo "This will replay your bag and create a map using SLAM Toolbox."
        echo ""
        echo "Steps:"
        echo "1. Bag will start playing in background"
        echo "2. SLAM Toolbox will launch"
        echo "3. (Optional) Open RViz to visualize"
        echo "4. Wait for replay to complete"
        echo "5. Save map when done"
        echo ""

        read -p "Press Enter to continue..."
        echo ""

        # Start bag playback in background
        echo -e "${BLUE}Starting bag playback...${NC}"
        ros2 bag play "$BAG_NAME" \
            --clock 100 \
            --rate "$RATE" \
            > /tmp/bag_play.log 2>&1 &
        BAG_PID=$!

        sleep 2

        # Check if playback started
        if ! ps -p $BAG_PID > /dev/null; then
            echo -e "${RED}ERROR: Failed to start bag playback${NC}"
            cat /tmp/bag_play.log
            exit 1
        fi

        echo -e "${GREEN}Bag playback started (PID: $BAG_PID)${NC}"
        echo ""

        # Start SLAM Toolbox
        echo -e "${BLUE}Starting SLAM Toolbox...${NC}"
        echo ""

        ros2 launch slam_toolbox online_async_launch.py \
            use_sim_time:=true &
        SLAM_PID=$!

        echo ""
        echo -e "${GREEN}SLAM Toolbox started (PID: $SLAM_PID)${NC}"
        echo ""

        # Instructions for user
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  Mapping in Progress${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""
        echo "To visualize (in another terminal):"
        echo -e "  ${YELLOW}ros2 run rviz2 rviz2${NC}"
        echo "  Then add displays: /map, /scan"
        echo ""
        echo "To save map when complete:"
        echo -e "  ${YELLOW}ros2 run nav2_map_server map_saver_cli -f my_map${NC}"
        echo ""
        echo -e "${YELLOW}Waiting for bag replay to complete...${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop early${NC}"
        echo ""

        # Wait for bag playback to finish
        wait $BAG_PID

        echo ""
        echo -e "${GREEN}Bag replay complete!${NC}"
        echo ""
        echo "SLAM Toolbox is still running to allow map saving."
        echo -e "Save your map with: ${YELLOW}ros2 run nav2_map_server map_saver_cli -f my_map${NC}"
        echo ""
        read -p "Press Enter when done to stop SLAM..."

        # Stop SLAM
        kill $SLAM_PID 2>/dev/null || true
        wait $SLAM_PID 2>/dev/null || true
        ;;

    localization)
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  Localization Testing Instructions${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""
        echo "This will test localization against an existing map."
        echo ""
        echo "Prerequisites:"
        echo "1. You must have a saved map (my_map.yaml and my_map.pgm)"
        echo "2. Map server and AMCL will be launched"
        echo "3. Bag will replay to test localization"
        echo ""

        # Ask for map file
        read -p "Enter path to map YAML file: " MAP_YAML

        if [ ! -f "$MAP_YAML" ]; then
            echo -e "${RED}ERROR: Map file not found: $MAP_YAML${NC}"
            exit 1
        fi

        echo ""
        read -p "Press Enter to continue..."
        echo ""

        # Start map server
        echo -e "${BLUE}Starting map server...${NC}"
        ros2 run nav2_map_server map_server \
            --ros-args \
            -p yaml_filename:="$MAP_YAML" \
            -p use_sim_time:=true \
            > /tmp/map_server.log 2>&1 &
        MAP_PID=$!

        sleep 2

        # Start lifecycle manager for map server
        ros2 run nav2_lifecycle_manager lifecycle_manager \
            --ros-args \
            -p node_names:="['map_server']" \
            -p autostart:=true \
            > /tmp/lifecycle.log 2>&1 &
        LIFECYCLE_PID=$!

        sleep 2

        echo -e "${GREEN}Map server started${NC}"
        echo ""

        # Start AMCL
        echo -e "${BLUE}Starting AMCL localization...${NC}"
        ros2 run nav2_amcl amcl \
            --ros-args \
            -p use_sim_time:=true \
            > /tmp/amcl.log 2>&1 &
        AMCL_PID=$!

        sleep 2

        echo -e "${GREEN}AMCL started${NC}"
        echo ""

        # Start bag playback
        echo -e "${BLUE}Starting bag playback...${NC}"
        ros2 bag play "$BAG_NAME" \
            --clock 100 \
            --rate "$RATE" &
        BAG_PID=$!

        echo ""
        echo -e "${GREEN}Bag playback started${NC}"
        echo ""

        # Instructions
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  Localization Testing in Progress${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""
        echo "To visualize (in another terminal):"
        echo -e "  ${YELLOW}ros2 run rviz2 rviz2${NC}"
        echo "  Add displays: /map, /scan, /amcl_pose, /particlecloud"
        echo ""
        echo "Monitor localization quality:"
        echo -e "  ${YELLOW}ros2 topic echo /amcl_pose${NC}"
        echo ""
        echo -e "${YELLOW}Waiting for bag replay to complete...${NC}"
        echo ""

        # Wait for bag
        wait $BAG_PID

        echo ""
        echo -e "${GREEN}Localization test complete!${NC}"
        echo ""
        read -p "Press Enter to stop all nodes..."

        # Stop all
        kill $AMCL_PID $MAP_PID $LIFECYCLE_PID 2>/dev/null || true
        ;;

    playback)
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  Data Playback Instructions${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""
        echo "This will replay the bag data without running SLAM."
        echo "Useful for:"
        echo "- Visual inspection in RViz"
        echo "- Data quality analysis"
        echo "- Testing custom nodes"
        echo ""

        read -p "Press Enter to start playback..."
        echo ""

        # Additional playback options
        echo "Playback options:"
        echo "1. Normal playback"
        echo "2. Loop continuously"
        echo "3. Start paused"
        read -p "Select option (1-3) [1]: " PLAYBACK_OPTION
        PLAYBACK_OPTION=${PLAYBACK_OPTION:-1}

        EXTRA_ARGS=""
        case "$PLAYBACK_OPTION" in
            2)
                EXTRA_ARGS="--loop"
                echo -e "${YELLOW}Loop mode enabled${NC}"
                ;;
            3)
                EXTRA_ARGS="--start-paused"
                echo -e "${YELLOW}Starting paused (press Space to unpause)${NC}"
                ;;
        esac

        echo ""
        echo -e "${BLUE}Starting playback...${NC}"
        echo ""
        echo "To visualize (in another terminal):"
        echo -e "  ${YELLOW}ros2 run rviz2 rviz2${NC}"
        echo "  Add displays: /scan, /odom, etc."
        echo ""
        echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
        echo ""

        # Play bag
        ros2 bag play "$BAG_NAME" \
            --clock 100 \
            --rate "$RATE" \
            $EXTRA_ARGS

        echo ""
        echo -e "${GREEN}Playback complete!${NC}"
        ;;
esac

echo ""
echo -e "${GREEN}Done!${NC}"
