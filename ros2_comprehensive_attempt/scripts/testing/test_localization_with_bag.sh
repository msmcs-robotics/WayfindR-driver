#!/bin/bash
###############################################################################
# Localization Testing with Rosbag
#
# Tests AMCL (Adaptive Monte Carlo Localization) with a pre-existing map
# and recorded rosbag data.
#
# Usage:
#   ./test_localization_with_bag.sh <map_file> <bag_file> [initial_x] [initial_y] [initial_yaw]
#
# Example:
#   ./test_localization_with_bag.sh my_map.yaml test_data.db3
#   ./test_localization_with_bag.sh my_map.yaml test_data.db3 0.0 0.0 0.0
#
# Author: WayfindR Development Team
# Date: 2026-01-11
###############################################################################

set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Usage
if [ $# -lt 2 ]; then
    echo "Usage: $0 <map_file> <bag_file> [initial_x] [initial_y] [initial_yaw]"
    echo ""
    echo "Arguments:"
    echo "  map_file    - Path to map YAML file (required)"
    echo "  bag_file    - Path to rosbag file (required)"
    echo "  initial_x   - Initial X position (default: 0.0)"
    echo "  initial_y   - Initial Y position (default: 0.0)"
    echo "  initial_yaw - Initial yaw angle in radians (default: 0.0)"
    echo ""
    echo "Example:"
    echo "  $0 maps/office.yaml test_data.db3"
    echo "  $0 maps/office.yaml test_data.db3 1.5 2.0 1.57"
    exit 1
fi

MAP_FILE=$1
BAG_FILE=$2
INITIAL_X=${3:-0.0}
INITIAL_Y=${4:-0.0}
INITIAL_YAW=${5:-0.0}

# Validate inputs
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
    exit 1
fi

if [ ! -e "$BAG_FILE" ]; then
    echo -e "${RED}Error: Bag file not found: $BAG_FILE${NC}"
    exit 1
fi

# Create output directory
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="localization_test_${TIMESTAMP}"
mkdir -p "$OUTPUT_DIR"

echo -e "${GREEN}=== AMCL Localization Testing ===${NC}"
echo "Map file: $MAP_FILE"
echo "Bag file: $BAG_FILE"
echo "Initial pose: ($INITIAL_X, $INITIAL_Y, $INITIAL_YAW)"
echo "Output directory: $OUTPUT_DIR"
echo ""

# Create AMCL configuration
AMCL_CONFIG="$OUTPUT_DIR/amcl_config.yaml"
echo -e "${YELLOW}Creating AMCL configuration...${NC}"
cat > "$AMCL_CONFIG" << 'EOF'
amcl:
  ros__parameters:
    use_sim_time: true

    # Overall filter parameters
    min_particles: 500
    max_particles: 2000
    kld_err: 0.05
    kld_z: 0.99
    update_min_d: 0.2
    update_min_a: 0.2
    resample_interval: 1
    transform_tolerance: 0.5
    recovery_alpha_slow: 0.0
    recovery_alpha_fast: 0.0
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.1

    # Laser model parameters
    laser_max_range: 12.0
    laser_min_range: 0.15
    laser_max_beams: 60
    laser_z_hit: 0.5
    laser_z_short: 0.05
    laser_z_max: 0.05
    laser_z_rand: 0.5
    laser_sigma_hit: 0.2
    laser_lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_model_type: "likelihood_field"

    # Odometry model parameters
    odom_model_type: "diff-corrected"

    # Robot model
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    set_initial_pose: false
    always_reset_initial_pose: false

    # Frames
    global_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    scan_topic: "scan"

    # Initial pose
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: ""

lifecycle_manager:
  ros__parameters:
    use_sim_time: true
    autostart: true
    node_names: ['map_server', 'amcl']
EOF

echo -e "${GREEN}Config created: $AMCL_CONFIG${NC}"
echo ""

# Launch map server
echo -e "${YELLOW}Starting map server...${NC}"
ros2 run nav2_map_server map_server \
    --ros-args \
    -p yaml_filename:="$(realpath $MAP_FILE)" \
    -p use_sim_time:=true \
    > "$OUTPUT_DIR/map_server.log" 2>&1 &
MAP_SERVER_PID=$!
sleep 3

# Activate map server (lifecycle node)
echo -e "${YELLOW}Activating map server lifecycle...${NC}"
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
sleep 1

# Check if map server is running
if ! ros2 lifecycle get /map_server | grep -q "active"; then
    echo -e "${RED}Error: Failed to activate map server. Check: $OUTPUT_DIR/map_server.log${NC}"
    kill $MAP_SERVER_PID 2>/dev/null || true
    exit 1
fi
echo -e "${GREEN}Map server active${NC}"
echo ""

# Launch AMCL
echo -e "${YELLOW}Starting AMCL...${NC}"
ros2 run nav2_amcl amcl \
    --ros-args \
    --params-file "$AMCL_CONFIG" \
    > "$OUTPUT_DIR/amcl.log" 2>&1 &
AMCL_PID=$!
sleep 3

# Check if AMCL is running
if ! ps -p $AMCL_PID > /dev/null; then
    echo -e "${RED}Error: AMCL failed to start. Check: $OUTPUT_DIR/amcl.log${NC}"
    kill $MAP_SERVER_PID 2>/dev/null || true
    exit 1
fi
echo -e "${GREEN}AMCL started (PID: $AMCL_PID)${NC}"
echo ""

# Set initial pose
echo -e "${YELLOW}Setting initial pose...${NC}"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: $INITIAL_X, y: $INITIAL_Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $(python3 -c "import math; print(math.sin($INITIAL_YAW/2.0))"), w: $(python3 -c "import math; print(math.cos($INITIAL_YAW/2.0))")}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.06854]
  }
}" > /dev/null 2>&1

sleep 2
echo -e "${GREEN}Initial pose set${NC}"
echo ""

# Record AMCL outputs
echo -e "${YELLOW}Recording AMCL outputs...${NC}"
ros2 bag record \
    -o "$OUTPUT_DIR/amcl_output" \
    /amcl_pose \
    /particlecloud \
    /tf \
    > "$OUTPUT_DIR/recording.log" 2>&1 &
RECORD_PID=$!
sleep 2
echo -e "${GREEN}Recording started (PID: $RECORD_PID)${NC}"
echo ""

# Play the bag
echo -e "${YELLOW}Playing bag file...${NC}"
ros2 bag play "$BAG_FILE" --clock --rate 1.0

echo ""
echo -e "${GREEN}Bag playback completed${NC}"

# Allow finalization
sleep 3

# Cleanup
echo -e "${YELLOW}Cleaning up...${NC}"
kill $RECORD_PID 2>/dev/null || true
kill $AMCL_PID 2>/dev/null || true
ros2 lifecycle set /map_server deactivate 2>/dev/null || true
ros2 lifecycle set /map_server cleanup 2>/dev/null || true
kill $MAP_SERVER_PID 2>/dev/null || true
sleep 2

# Force kill
kill -9 $RECORD_PID 2>/dev/null || true
kill -9 $AMCL_PID 2>/dev/null || true
kill -9 $MAP_SERVER_PID 2>/dev/null || true

# Generate report
cat > "$OUTPUT_DIR/test_report.txt" << EOF
AMCL Localization Test Report
==============================
Date: $(date)
Map File: $MAP_FILE
Bag File: $BAG_FILE
Initial Pose: ($INITIAL_X, $INITIAL_Y, $INITIAL_YAW)

Outputs:
- AMCL log: $OUTPUT_DIR/amcl.log
- Map server log: $OUTPUT_DIR/map_server.log
- AMCL output bag: $OUTPUT_DIR/amcl_output/
- Configuration: $OUTPUT_DIR/amcl_config.yaml

Analysis:
To analyze localization quality:
1. Review AMCL logs for warnings/errors
2. Play back amcl_output bag and visualize in RViz
3. Check particle cloud convergence
4. Compare /amcl_pose with ground truth (if available)

Commands:
# Visualize recorded data
ros2 bag play $OUTPUT_DIR/amcl_output --clock
rviz2 (open RViz and add Map, PoseArray (/particlecloud), PoseWithCovariance (/amcl_pose))

# Check pose estimates
ros2 bag info $OUTPUT_DIR/amcl_output
EOF

echo ""
echo -e "${GREEN}=== Localization Test Complete ===${NC}"
echo "Results saved to: $OUTPUT_DIR"
echo ""
echo "Summary:"
echo "  - AMCL log: $OUTPUT_DIR/amcl.log"
echo "  - Output bag: $OUTPUT_DIR/amcl_output/"
echo "  - Report: $OUTPUT_DIR/test_report.txt"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Review logs for errors"
echo "  2. Visualize particle cloud convergence in RViz"
echo "  3. Analyze pose estimation accuracy"
echo ""
