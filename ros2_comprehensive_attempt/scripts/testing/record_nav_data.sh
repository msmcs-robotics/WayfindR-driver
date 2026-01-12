#!/bin/bash
###############################################################################
# Record Navigation Data to Rosbag
#
# Records all essential topics for Nav2 testing including:
# - LaserScan data
# - Odometry
# - TF transforms
# - Optional: Map, AMCL pose, cmd_vel, etc.
#
# Usage:
#   ./record_nav_data.sh [output_name] [duration]
#
# Example:
#   ./record_nav_data.sh corridor_test 60
#   ./record_nav_data.sh  # Uses default name and runs until Ctrl+C
#
# Author: WayfindR Development Team
# Date: 2026-01-11
###############################################################################

# Color output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parameters
OUTPUT_NAME=${1:-"nav_data_$(date +%Y%m%d_%H%M%S)"}
DURATION=${2:-0}  # 0 = infinite (until Ctrl+C)

# Storage format (mcap or sqlite3)
STORAGE_FORMAT="mcap"  # MCAP is recommended for better performance

# Compression
USE_COMPRESSION=true
COMPRESSION_FORMAT="zstd"

echo -e "${GREEN}=== ROS2 Navigation Data Recording ===${NC}"
echo ""
echo "Output name: $OUTPUT_NAME"
if [ $DURATION -gt 0 ]; then
    echo "Duration: ${DURATION}s"
else
    echo "Duration: Until Ctrl+C"
fi
echo "Storage format: $STORAGE_FORMAT"
echo "Compression: $USE_COMPRESSION ($COMPRESSION_FORMAT)"
echo ""

# Essential topics for Nav2
ESSENTIAL_TOPICS=(
    "/scan"           # LaserScan
    "/odom"           # Odometry
    "/tf"             # TF transforms
    "/tf_static"      # Static TF transforms
)

# Optional topics (comment out if not needed)
OPTIONAL_TOPICS=(
    "/cmd_vel"        # Velocity commands
    "/map"            # Occupancy grid map
    "/amcl_pose"      # AMCL pose estimate
    "/robot_description"  # URDF
    "/joint_states"   # Robot joint states
    # "/imu"          # IMU data (uncomment if available)
)

# Check which topics are available
echo -e "${YELLOW}Checking available topics...${NC}"
AVAILABLE_TOPICS=$(ros2 topic list)

TOPICS_TO_RECORD=()

# Check essential topics
for topic in "${ESSENTIAL_TOPICS[@]}"; do
    if echo "$AVAILABLE_TOPICS" | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} $topic"
        TOPICS_TO_RECORD+=("$topic")
    else
        echo -e "${YELLOW}⚠${NC} $topic (not available - WARNING!)"
    fi
done

# Check optional topics
for topic in "${OPTIONAL_TOPICS[@]}"; do
    if echo "$AVAILABLE_TOPICS" | grep -q "^${topic}$"; then
        echo -e "${BLUE}+${NC} $topic (optional)"
        TOPICS_TO_RECORD+=("$topic")
    fi
done

echo ""

# Verify we have minimum required topics
if [[ ! " ${TOPICS_TO_RECORD[@]} " =~ " /scan " ]]; then
    echo -e "${YELLOW}WARNING: /scan topic not available!${NC}"
    echo "Recording will continue, but Nav2 testing will not work without scan data."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Build ros2 bag record command
RECORD_CMD="ros2 bag record -o $OUTPUT_NAME"

# Add storage format
RECORD_CMD="$RECORD_CMD -s $STORAGE_FORMAT"

# Add compression if enabled
if [ "$USE_COMPRESSION" = true ]; then
    RECORD_CMD="$RECORD_CMD --compression-mode file --compression-format $COMPRESSION_FORMAT"
fi

# Add topics
RECORD_CMD="$RECORD_CMD ${TOPICS_TO_RECORD[*]}"

# Add duration if specified
if [ $DURATION -gt 0 ]; then
    RECORD_CMD="$RECORD_CMD --max-bag-duration $DURATION"
fi

echo -e "${GREEN}Starting recording...${NC}"
echo ""
echo "Command: $RECORD_CMD"
echo ""
echo -e "${YELLOW}Recording in progress. Press Ctrl+C to stop.${NC}"
echo ""

# Create metadata file
cat > "${OUTPUT_NAME}_metadata.yaml" << EOF
recording_info:
  timestamp: $(date -Iseconds)
  hostname: $(hostname)
  ros_distro: $ROS_DISTRO
  storage_format: $STORAGE_FORMAT
  compression: $USE_COMPRESSION
  compression_format: $COMPRESSION_FORMAT

topics_recorded:
$(for topic in "${TOPICS_TO_RECORD[@]}"; do echo "  - $topic"; done)

notes: |
  Add your notes about this recording here:
  - Environment description
  - Robot configuration
  - Test objectives
  - Known issues
EOF

echo "Metadata file created: ${OUTPUT_NAME}_metadata.yaml"
echo ""

# Execute recording
eval $RECORD_CMD

# Post-recording summary
echo ""
echo -e "${GREEN}=== Recording Complete ===${NC}"
echo ""

# Get bag info
if [ -d "$OUTPUT_NAME" ]; then
    echo -e "${YELLOW}Bag information:${NC}"
    ros2 bag info "$OUTPUT_NAME"

    # Calculate approximate size
    BAG_SIZE=$(du -sh "$OUTPUT_NAME" | cut -f1)
    echo ""
    echo "Bag size: $BAG_SIZE"

    # Save bag info to file
    ros2 bag info "$OUTPUT_NAME" > "${OUTPUT_NAME}_info.txt"
    echo "Bag info saved to: ${OUTPUT_NAME}_info.txt"
else
    echo "Warning: Bag directory not found: $OUTPUT_NAME"
fi

echo ""
echo -e "${GREEN}Files created:${NC}"
echo "  - Bag data: $OUTPUT_NAME/"
echo "  - Metadata: ${OUTPUT_NAME}_metadata.yaml"
echo "  - Bag info: ${OUTPUT_NAME}_info.txt"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Edit metadata: ${OUTPUT_NAME}_metadata.yaml"
echo "  2. Test playback: ros2 bag play $OUTPUT_NAME --clock"
echo "  3. Run SLAM test: ./test_slam_with_bag.sh $OUTPUT_NAME"
echo ""
