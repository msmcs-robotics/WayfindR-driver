#!/bin/bash

###############################################################################
# URDF Validation Script for WayfindR Robot
#
# This script validates the robot URDF description by:
# 1. Processing XACRO to URDF
# 2. Checking URDF syntax
# 3. Displaying robot structure
# 4. Optionally visualizing in RViz
#
# Usage:
#   ./validate_urdf.sh              # Just validate
#   ./validate_urdf.sh --visualize  # Validate and launch RViz
#
# Author: WayfindR Development Team
# Date: 2026-01-11
###############################################################################

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
URDF_DIR="$PROJECT_DIR/urdf"
XACRO_FILE="$URDF_DIR/wayfinder_robot.urdf.xacro"
TEMP_URDF="/tmp/wayfinder_robot.urdf"

echo "=========================================="
echo "WayfindR Robot URDF Validation"
echo "=========================================="
echo ""

# Check if XACRO file exists
if [ ! -f "$XACRO_FILE" ]; then
    echo -e "${RED}ERROR: XACRO file not found at $XACRO_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}Found XACRO file:${NC} $XACRO_FILE"
echo ""

# Step 1: Check if xacro is installed
echo "Step 1: Checking for xacro..."
if ! command -v xacro &> /dev/null; then
    echo -e "${RED}ERROR: xacro command not found${NC}"
    echo "Please install with: sudo apt-get install ros-humble-xacro"
    exit 1
fi
echo -e "${GREEN}✓ xacro is installed${NC}"
echo ""

# Step 2: Process XACRO to URDF
echo "Step 2: Processing XACRO to URDF..."
if xacro "$XACRO_FILE" -o "$TEMP_URDF"; then
    echo -e "${GREEN}✓ XACRO processed successfully${NC}"
else
    echo -e "${RED}ERROR: Failed to process XACRO file${NC}"
    exit 1
fi
echo ""

# Step 3: Check if check_urdf is installed
echo "Step 3: Checking for check_urdf..."
if ! command -v check_urdf &> /dev/null; then
    echo -e "${YELLOW}WARNING: check_urdf command not found${NC}"
    echo "Install with: sudo apt-get install liburdfdom-tools"
    echo "Skipping URDF validation..."
    SKIP_VALIDATION=true
else
    echo -e "${GREEN}✓ check_urdf is installed${NC}"
    SKIP_VALIDATION=false
fi
echo ""

# Step 4: Validate URDF
if [ "$SKIP_VALIDATION" = false ]; then
    echo "Step 4: Validating URDF syntax..."
    echo "----------------------------------------"
    if check_urdf "$TEMP_URDF"; then
        echo "----------------------------------------"
        echo -e "${GREEN}✓ URDF validation passed${NC}"
    else
        echo "----------------------------------------"
        echo -e "${RED}ERROR: URDF validation failed${NC}"
        exit 1
    fi
else
    echo "Step 4: Skipped (check_urdf not installed)"
fi
echo ""

# Step 5: Display robot info
echo "Step 5: Robot Information"
echo "----------------------------------------"
echo "XACRO file: $XACRO_FILE"
echo "Generated URDF: $TEMP_URDF"
echo "Robot name: wayfinder"
echo ""
echo "Expected link structure:"
echo "  base_footprint"
echo "  └─ base_link"
echo "      ├─ left_wheel"
echo "      ├─ right_wheel"
echo "      ├─ caster_wheel"
echo "      └─ laser"
echo ""
echo "Key specifications:"
echo "  - Base diameter: 200mm (0.2m)"
echo "  - Wheel diameter: 65mm (0.065m)"
echo "  - Wheel separation: 150mm (0.15m)"
echo "  - LiDAR height: 100mm above base"
echo "  - Total mass: ~2.5kg"
echo "----------------------------------------"
echo ""

# Step 6: Show file size and line count
URDF_SIZE=$(stat -f%z "$TEMP_URDF" 2>/dev/null || stat -c%s "$TEMP_URDF" 2>/dev/null)
URDF_LINES=$(wc -l < "$TEMP_URDF")
echo "Generated URDF statistics:"
echo "  - File size: $URDF_SIZE bytes"
echo "  - Line count: $URDF_LINES lines"
echo ""

# Step 7: Check for ROS2 and offer visualization
if [ "$1" = "--visualize" ] || [ "$1" = "-v" ]; then
    echo "Step 7: Launching visualization..."
    echo ""

    # Check if ROS2 is sourced
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${YELLOW}WARNING: ROS2 not sourced${NC}"
        echo "Please source ROS2 first:"
        echo "  source /opt/ros/humble/setup.bash"
        exit 1
    fi

    echo "Launching robot_state_publisher with RViz..."
    echo "Press Ctrl+C to exit"
    echo ""

    # Launch robot state publisher with RViz
    cd "$PROJECT_DIR/launch"
    ros2 launch robot_state_publisher.launch.py use_rviz:=true
else
    echo "✓ Validation complete!"
    echo ""
    echo "To visualize the robot in RViz, run:"
    echo "  $0 --visualize"
    echo ""
    echo "Or launch manually:"
    echo "  ros2 launch $PROJECT_DIR/launch/robot_state_publisher.launch.py use_rviz:=true"
fi

echo ""
echo "=========================================="
echo "Validation Complete"
echo "=========================================="
