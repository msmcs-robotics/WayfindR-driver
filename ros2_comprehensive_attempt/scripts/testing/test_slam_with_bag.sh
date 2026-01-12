#!/bin/bash
###############################################################################
# SLAM Testing with Rosbag
#
# Tests slam_toolbox with recorded or synthetic rosbag data.
# Evaluates mapping quality and saves results.
#
# Usage:
#   ./test_slam_with_bag.sh <bag_file> [config_file]
#
# Example:
#   ./test_slam_with_bag.sh test_data.db3
#   ./test_slam_with_bag.sh test_data.db3 custom_slam_config.yaml
#
# Author: WayfindR Development Team
# Date: 2026-01-11
###############################################################################

set -e  # Exit on error

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Usage
if [ $# -lt 1 ]; then
    echo "Usage: $0 <bag_file> [config_file]"
    echo ""
    echo "Arguments:"
    echo "  bag_file     - Path to rosbag file (required)"
    echo "  config_file  - Path to slam_toolbox config YAML (optional)"
    echo ""
    echo "Example:"
    echo "  $0 my_test_data.db3"
    exit 1
fi

BAG_FILE=$1
CONFIG_FILE=${2:-""}

# Validate bag file
if [ ! -e "$BAG_FILE" ]; then
    echo -e "${RED}Error: Bag file not found: $BAG_FILE${NC}"
    exit 1
fi

# Create output directory
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="slam_test_${TIMESTAMP}"
mkdir -p "$OUTPUT_DIR"

echo -e "${GREEN}=== SLAM Rosbag Testing ===${NC}"
echo "Bag file: $BAG_FILE"
echo "Output directory: $OUTPUT_DIR"
echo ""

# Inspect bag
echo -e "${YELLOW}Inspecting bag file...${NC}"
ros2 bag info "$BAG_FILE" | tee "$OUTPUT_DIR/bag_info.txt"
echo ""

# Check for required topics
REQUIRED_TOPICS=("/scan" "/odom" "/tf")
BAG_INFO=$(ros2 bag info "$BAG_FILE")

for topic in "${REQUIRED_TOPICS[@]}"; do
    if echo "$BAG_INFO" | grep -q "$topic"; then
        echo -e "${GREEN}✓${NC} Topic $topic found"
    else
        echo -e "${RED}✗${NC} Warning: Topic $topic not found in bag"
    fi
done
echo ""

# Create default SLAM config if not provided
if [ -z "$CONFIG_FILE" ]; then
    CONFIG_FILE="$OUTPUT_DIR/slam_config.yaml"
    echo -e "${YELLOW}Creating default SLAM configuration...${NC}"
    cat > "$CONFIG_FILE" << 'EOF'
slam_toolbox:
  ros__parameters:
    use_sim_time: true

    # Frames
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan

    # Mode
    mode: mapping

    # Performance
    map_update_interval: 1.0
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_publish_period: 0.02

    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # Scan Matcher
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0

    # Loop Closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
EOF
    echo -e "${GREEN}Config created: $CONFIG_FILE${NC}"
else
    if [ ! -f "$CONFIG_FILE" ]; then
        echo -e "${RED}Error: Config file not found: $CONFIG_FILE${NC}"
        exit 1
    fi
    cp "$CONFIG_FILE" "$OUTPUT_DIR/slam_config.yaml"
fi
echo ""

# Launch SLAM toolbox
echo -e "${YELLOW}Starting slam_toolbox...${NC}"
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    params_file:="$CONFIG_FILE" \
    > "$OUTPUT_DIR/slam_toolbox.log" 2>&1 &
SLAM_PID=$!

# Wait for SLAM to initialize
sleep 5

# Check if SLAM is running
if ! ps -p $SLAM_PID > /dev/null; then
    echo -e "${RED}Error: slam_toolbox failed to start. Check log: $OUTPUT_DIR/slam_toolbox.log${NC}"
    exit 1
fi
echo -e "${GREEN}slam_toolbox started (PID: $SLAM_PID)${NC}"
echo ""

# Record SLAM outputs
echo -e "${YELLOW}Starting to record SLAM outputs...${NC}"
ros2 bag record \
    -o "$OUTPUT_DIR/slam_output" \
    /map \
    /slam_toolbox/graph_visualization \
    /slam_toolbox/scan_visualization \
    > "$OUTPUT_DIR/recording.log" 2>&1 &
RECORD_PID=$!
sleep 2
echo -e "${GREEN}Recording started (PID: $RECORD_PID)${NC}"
echo ""

# Play the input bag
echo -e "${YELLOW}Playing bag file with simulation time...${NC}"
ros2 bag play "$BAG_FILE" --clock --rate 1.0

echo ""
echo -e "${GREEN}Bag playback completed${NC}"

# Allow SLAM to finalize
echo -e "${YELLOW}Allowing SLAM to finalize (5 seconds)...${NC}"
sleep 5

# Save the map
echo -e "${YELLOW}Saving final map...${NC}"
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_DIR/final_map"

if [ -f "$OUTPUT_DIR/final_map.pgm" ]; then
    echo -e "${GREEN}✓ Map saved: $OUTPUT_DIR/final_map.pgm${NC}"
else
    echo -e "${RED}✗ Warning: Map file not created${NC}"
fi
echo ""

# Cleanup
echo -e "${YELLOW}Cleaning up...${NC}"
kill $RECORD_PID 2>/dev/null || true
kill $SLAM_PID 2>/dev/null || true
sleep 2

# Force kill if still running
kill -9 $RECORD_PID 2>/dev/null || true
kill -9 $SLAM_PID 2>/dev/null || true

# Generate test report
echo -e "${YELLOW}Generating test report...${NC}"
cat > "$OUTPUT_DIR/test_report.txt" << EOF
SLAM Test Report
================
Date: $(date)
Bag File: $BAG_FILE
Config File: $CONFIG_FILE

Outputs:
- Map: $OUTPUT_DIR/final_map.pgm
- Map metadata: $OUTPUT_DIR/final_map.yaml
- SLAM log: $OUTPUT_DIR/slam_toolbox.log
- SLAM output bag: $OUTPUT_DIR/slam_output/
- Bag info: $OUTPUT_DIR/bag_info.txt

Next Steps:
1. Review the generated map: $OUTPUT_DIR/final_map.pgm
2. Check SLAM logs for errors: $OUTPUT_DIR/slam_toolbox.log
3. Visualize with: rviz2 (open the map file)
4. Tune parameters if needed in: $OUTPUT_DIR/slam_config.yaml
EOF

echo ""
echo -e "${GREEN}=== SLAM Test Complete ===${NC}"
echo "Results saved to: $OUTPUT_DIR"
echo ""
echo "Summary:"
echo "  - Map: $OUTPUT_DIR/final_map.pgm"
echo "  - Report: $OUTPUT_DIR/test_report.txt"
echo "  - Logs: $OUTPUT_DIR/slam_toolbox.log"
echo ""
echo -e "${YELLOW}Tip: View the map with an image viewer:${NC}"
echo "  eog $OUTPUT_DIR/final_map.pgm"
echo ""
