#!/bin/bash
# Record LiDAR SLAM session with optimal settings
# Usage: ./record_lidar_session.sh [session_name] [topics]
#
# Arguments:
#   session_name (optional) - Name for this recording session
#                            Default: slam_session_YYYYMMDD_HHMMSS
#   topics (optional)      - Which topics to record: minimal, full, or custom
#                            minimal: /scan /tf /tf_static (LiDAR only)
#                            full:    /scan /odom /tf /tf_static (default)
#                            custom:  specify your own topic list
#
# Examples:
#   ./record_lidar_session.sh                    # Full recording with auto name
#   ./record_lidar_session.sh my_test            # Full recording named "my_test"
#   ./record_lidar_session.sh my_test minimal    # Minimal recording
#   ./record_lidar_session.sh my_test custom "/scan /odom /imu /tf /tf_static"

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
SESSION_NAME="${1:-slam_session_$(date +%Y%m%d_%H%M%S)}"
TOPIC_SET="${2:-full}"

# ROS2 setup
source /opt/ros/humble/setup.bash

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  LiDAR SLAM Recording Session${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Determine topics to record
if [ "$TOPIC_SET" = "minimal" ]; then
    TOPICS="/scan /tf /tf_static"
    echo -e "${YELLOW}Topic set: MINIMAL (LiDAR only)${NC}"
elif [ "$TOPIC_SET" = "full" ]; then
    TOPICS="/scan /odom /tf /tf_static"
    echo -e "${GREEN}Topic set: FULL (LiDAR + Odometry)${NC}"
elif [ "$TOPIC_SET" = "custom" ]; then
    TOPICS="${3}"
    if [ -z "$TOPICS" ]; then
        echo -e "${RED}ERROR: Custom topic list not provided${NC}"
        echo "Usage: $0 session_name custom \"/topic1 /topic2 ...\""
        exit 1
    fi
    echo -e "${YELLOW}Topic set: CUSTOM${NC}"
else
    echo -e "${RED}ERROR: Invalid topic set: $TOPIC_SET${NC}"
    echo "Valid options: minimal, full, custom"
    exit 1
fi

echo -e "Topics to record: ${GREEN}$TOPICS${NC}"
echo -e "Session name: ${GREEN}$SESSION_NAME${NC}"
echo ""

# Check if LiDAR is connected and publishing
echo -e "${BLUE}Checking LiDAR status...${NC}"

if ! ros2 topic list | grep -q "/scan"; then
    echo -e "${RED}ERROR: /scan topic not found!${NC}"
    echo ""
    echo "Make sure LiDAR node is running:"
    echo "  ros2 run rplidar_ros rplidar_node --ros-args \\"
    echo "      -p serial_port:=/dev/rplidar \\"
    echo "      -p serial_baudrate:=460800 \\"
    echo "      -p frame_id:=laser \\"
    echo "      -p scan_mode:=DenseBoost"
    echo ""
    exit 1
fi

# Check scan rate
echo -e "Checking /scan topic rate..."
SCAN_RATE=$(timeout 3s ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}' || echo "0")
SCAN_RATE_INT=$(printf "%.0f" "$SCAN_RATE" 2>/dev/null || echo "0")

if [ "$SCAN_RATE_INT" -lt 8 ] || [ "$SCAN_RATE_INT" -gt 12 ]; then
    echo -e "${YELLOW}WARNING: Scan rate is ${SCAN_RATE} Hz (expected ~10 Hz)${NC}"
    echo "This may indicate a problem with the LiDAR"
else
    echo -e "${GREEN}Scan rate OK: ${SCAN_RATE} Hz${NC}"
fi

# Check odometry if recording full set
if echo "$TOPICS" | grep -q "/odom"; then
    echo -e "${BLUE}Checking odometry status...${NC}"
    if ! ros2 topic list | grep -q "/odom"; then
        echo -e "${YELLOW}WARNING: /odom topic not found!${NC}"
        echo "Consider using minimal mode or start odometry node"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        echo -e "${GREEN}Odometry topic found${NC}"
    fi
fi

# Check TF tree
echo -e "${BLUE}Checking TF tree...${NC}"
if ! ros2 topic list | grep -q "/tf"; then
    echo -e "${YELLOW}WARNING: /tf topic not found!${NC}"
    echo "TF transforms are essential for SLAM"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "${GREEN}TF topics found${NC}"
fi

# Check disk space
echo -e "${BLUE}Checking disk space...${NC}"
AVAILABLE_MB=$(df -m . | awk 'NR==2 {print $4}')
if [ "$AVAILABLE_MB" -lt 100 ]; then
    echo -e "${RED}ERROR: Low disk space! Only ${AVAILABLE_MB} MB available${NC}"
    echo "Need at least 100 MB for recording"
    exit 1
fi
echo -e "${GREEN}Disk space OK: ${AVAILABLE_MB} MB available${NC}"

# Estimate recording size
echo ""
echo -e "${BLUE}Estimated recording sizes:${NC}"
echo "  1 minute:  ~1-2 MB (uncompressed), ~0.6-1 MB (compressed)"
echo "  5 minutes: ~5-10 MB (uncompressed), ~3-5 MB (compressed)"
echo "  10 minutes: ~10-20 MB (uncompressed), ~6-12 MB (compressed)"
echo "  30 minutes: ~30-60 MB (uncompressed), ~18-36 MB (compressed)"
echo ""

# Show recording tips
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  Recording Best Practices${NC}"
echo -e "${YELLOW}========================================${NC}"
echo "1. Move slowly (0.3-0.6 m/s) for best quality"
echo "2. Perform loop closures every 60-120 meters"
echo "3. Overlap loops by at least 5 meters"
echo "4. Avoid featureless areas (blank walls, open spaces)"
echo "5. Return to starting point at the end"
echo "6. Keep movement smooth and steady"
echo ""

# Confirm start
echo -e "${GREEN}Ready to start recording!${NC}"
echo -e "Press ${YELLOW}Ctrl+C${NC} to stop recording"
echo ""
read -p "Press Enter to begin recording..."
echo ""

# Start recording with optimal settings
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  RECORDING IN PROGRESS${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Session: ${YELLOW}$SESSION_NAME${NC}"
echo -e "Topics: ${YELLOW}$TOPICS${NC}"
echo ""
echo -e "${RED}Press Ctrl+C when done${NC}"
echo ""

# Record with compression and size limits
ros2 bag record \
    $TOPICS \
    --compression-mode file \
    --compression-format zstd \
    --max-bag-size 1000000000 \
    -o "$SESSION_NAME"

# Capture exit status
EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  Recording Complete!${NC}"
    echo -e "${GREEN}========================================${NC}"
else
    echo -e "${YELLOW}========================================${NC}"
    echo -e "${YELLOW}  Recording Stopped${NC}"
    echo -e "${YELLOW}========================================${NC}"
fi

# Show bag information
if [ -d "$SESSION_NAME" ]; then
    echo ""
    echo -e "${BLUE}Recording summary:${NC}"
    ros2 bag info "$SESSION_NAME" | head -20

    # Calculate actual size
    BAG_SIZE=$(du -sh "$SESSION_NAME" | awk '{print $1}')
    echo ""
    echo -e "${GREEN}Bag size: $BAG_SIZE${NC}"
    echo -e "${GREEN}Location: $(pwd)/$SESSION_NAME${NC}"

    echo ""
    echo -e "${BLUE}Next steps:${NC}"
    echo "1. Analyze quality:"
    echo -e "   ${YELLOW}python3 scripts/lidar_tools/check_lidar_quality.py $SESSION_NAME${NC}"
    echo ""
    echo "2. Replay for SLAM:"
    echo -e "   ${YELLOW}scripts/lidar_tools/replay_for_slam.sh $SESSION_NAME${NC}"
    echo ""
    echo "3. View in RViz:"
    echo -e "   ${YELLOW}ros2 bag play $SESSION_NAME --clock 100 --rate 0.5${NC}"
    echo -e "   ${YELLOW}ros2 run rviz2 rviz2${NC} (add /scan display)"
    echo ""
else
    echo -e "${RED}ERROR: Bag directory not found: $SESSION_NAME${NC}"
    exit 1
fi

echo -e "${GREEN}Done!${NC}"
