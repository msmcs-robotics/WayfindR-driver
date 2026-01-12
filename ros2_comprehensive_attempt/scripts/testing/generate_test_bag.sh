#!/bin/bash
###############################################################################
# Generate Synthetic Test Rosbag
#
# Creates a synthetic rosbag using the synthetic nav data publisher.
# Useful for testing without hardware or simulation.
#
# Usage:
#   ./generate_test_bag.sh [output_name] [duration] [motion_pattern]
#
# Example:
#   ./generate_test_bag.sh corridor_test 30 straight
#   ./generate_test_bag.sh circular_test 60 circular
#
# Motion patterns: straight, circular, square
#
# Author: WayfindR Development Team
# Date: 2026-01-11
###############################################################################

set -e

# Color output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Parameters
OUTPUT_NAME=${1:-"synthetic_test_$(date +%Y%m%d_%H%M%S)"}
DURATION=${2:-30}
MOTION_PATTERN=${3:-"circular"}

# Validate motion pattern
if [[ ! "$MOTION_PATTERN" =~ ^(straight|circular|square)$ ]]; then
    echo "Error: Invalid motion pattern: $MOTION_PATTERN"
    echo "Valid patterns: straight, circular, square"
    exit 1
fi

echo -e "${GREEN}=== Generate Synthetic Test Rosbag ===${NC}"
echo ""
echo "Output name: $OUTPUT_NAME"
echo "Duration: ${DURATION}s"
echo "Motion pattern: $MOTION_PATTERN"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if synthetic publisher exists
PUBLISHER_SCRIPT="$SCRIPT_DIR/synthetic_nav_data_publisher.py"
if [ ! -f "$PUBLISHER_SCRIPT" ]; then
    echo "Error: Synthetic publisher not found: $PUBLISHER_SCRIPT"
    exit 1
fi

# Start synthetic data publisher
echo -e "${YELLOW}Starting synthetic data publisher...${NC}"
python3 "$PUBLISHER_SCRIPT" \
    --ros-args \
    -p motion_pattern:="$MOTION_PATTERN" \
    -p linear_velocity:=0.2 \
    -p angular_velocity:=0.1 \
    > "${OUTPUT_NAME}_publisher.log" 2>&1 &
PUBLISHER_PID=$!

# Wait for publisher to start
sleep 3

# Check if publisher is running
if ! ps -p $PUBLISHER_PID > /dev/null; then
    echo "Error: Synthetic publisher failed to start"
    echo "Check log: ${OUTPUT_NAME}_publisher.log"
    exit 1
fi
echo -e "${GREEN}Publisher started (PID: $PUBLISHER_PID)${NC}"
echo ""

# Verify topics are publishing
echo -e "${YELLOW}Verifying topics...${NC}"
EXPECTED_TOPICS=("/scan" "/odom" "/tf")
for topic in "${EXPECTED_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} $topic"
    else
        echo -e "${RED}✗${NC} $topic not found!"
        kill $PUBLISHER_PID
        exit 1
    fi
done
echo ""

# Record the data
echo -e "${YELLOW}Recording for ${DURATION} seconds...${NC}"
ros2 bag record \
    -o "$OUTPUT_NAME" \
    -s sqlite3 \
    --max-bag-duration "$DURATION" \
    /scan \
    /odom \
    /tf \
    /tf_static

# Recording complete
echo ""
echo -e "${GREEN}Recording complete!${NC}"

# Stop publisher
kill $PUBLISHER_PID 2>/dev/null || true
sleep 1
kill -9 $PUBLISHER_PID 2>/dev/null || true

# Generate metadata
cat > "${OUTPUT_NAME}_metadata.yaml" << EOF
synthetic_bag_info:
  generation_date: $(date -Iseconds)
  duration_seconds: $DURATION
  motion_pattern: $MOTION_PATTERN
  linear_velocity: 0.2
  angular_velocity: 0.1

  topics:
    - /scan
    - /odom
    - /tf
    - /tf_static

  environment:
    walls: [-5.0, 5.0, -5.0, 5.0]  # [xmin, xmax, ymin, ymax]
    obstacles:
      - {x: 2.0, y: 0.0, radius: 0.3}
      - {x: -1.5, y: 1.5, radius: 0.4}
      - {x: 1.0, y: -2.0, radius: 0.5}
      - {x: -2.0, y: -1.0, radius: 0.3}
      - {x: 3.0, y: 2.0, radius: 0.4}

  notes: |
    Synthetic rosbag generated for testing Nav2 without hardware.
    Robot motion follows $MOTION_PATTERN pattern.
    Obstacles and walls are simulated in the scan data.
EOF

# Get bag info
echo ""
echo -e "${YELLOW}Bag information:${NC}"
ros2 bag info "$OUTPUT_NAME" | tee "${OUTPUT_NAME}_info.txt"

# Calculate size
BAG_SIZE=$(du -sh "$OUTPUT_NAME" | cut -f1)

echo ""
echo -e "${GREEN}=== Generation Complete ===${NC}"
echo ""
echo "Files created:"
echo "  - Bag data: $OUTPUT_NAME/"
echo "  - Metadata: ${OUTPUT_NAME}_metadata.yaml"
echo "  - Bag info: ${OUTPUT_NAME}_info.txt"
echo "  - Size: $BAG_SIZE"
echo ""
echo -e "${YELLOW}Test the bag:${NC}"
echo "  ros2 bag play $OUTPUT_NAME --clock"
echo ""
echo -e "${YELLOW}Run SLAM test:${NC}"
echo "  ./test_slam_with_bag.sh $OUTPUT_NAME"
echo ""
