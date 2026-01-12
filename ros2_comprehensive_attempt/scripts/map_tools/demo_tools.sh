#!/bin/bash

# Demo script for ROS2 Map Tools
# Demonstrates usage of all four map analysis tools

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps"
DEMO_MAP="$MAP_DIR/first_map.yaml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}ROS2 Map Tools - Demonstration${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# Check if demo map exists
if [ ! -f "$DEMO_MAP" ]; then
    echo -e "${RED}Error: Demo map not found at $DEMO_MAP${NC}"
    echo "Please ensure you have a map file to test with."
    exit 1
fi

echo -e "${GREEN}Using demo map: $DEMO_MAP${NC}"
echo ""

# ===== Tool 1: validate_map.py =====
echo -e "${YELLOW}===== TOOL 1: validate_map.py =====${NC}"
echo "Validates map file correctness"
echo ""
echo -e "${BLUE}Command:${NC} python3 validate_map.py $DEMO_MAP"
echo ""
read -p "Press Enter to continue..."
echo ""

python3 "$SCRIPT_DIR/validate_map.py" "$DEMO_MAP"

echo ""
echo -e "${GREEN}Validation complete!${NC}"
echo ""
read -p "Press Enter to continue to next tool..."
echo ""

# ===== Tool 2: map_info.py =====
echo -e "${YELLOW}===== TOOL 2: map_info.py =====${NC}"
echo "Displays comprehensive map information"
echo ""
echo -e "${BLUE}Command:${NC} python3 map_info.py $DEMO_MAP"
echo ""
read -p "Press Enter to continue..."
echo ""

python3 "$SCRIPT_DIR/map_info.py" "$DEMO_MAP"

echo ""
echo -e "${GREEN}Information display complete!${NC}"
echo ""
read -p "Press Enter to continue to next tool..."
echo ""

# ===== Tool 3: map_converter.py =====
echo -e "${YELLOW}===== TOOL 3: map_converter.py =====${NC}"
echo "Converts and modifies maps"
echo ""
echo "Example 1: Adjust thresholds"
echo -e "${BLUE}Command:${NC} python3 map_converter.py $DEMO_MAP -o /tmp/converted_map.yaml --thresholds 0.20 0.70"
echo ""
read -p "Press Enter to continue..."
echo ""

python3 "$SCRIPT_DIR/map_converter.py" "$DEMO_MAP" -o /tmp/converted_map.yaml --thresholds 0.20 0.70

echo ""
echo "Example 2: Crop map"
echo -e "${BLUE}Command:${NC} python3 map_converter.py $DEMO_MAP -o /tmp/cropped_map.yaml --crop 20 20 150 100"
echo ""
read -p "Press Enter to continue..."
echo ""

python3 "$SCRIPT_DIR/map_converter.py" "$DEMO_MAP" -o /tmp/cropped_map.yaml --crop 20 20 150 100

echo ""
echo -e "${GREEN}Conversion examples complete!${NC}"
echo -e "${GREEN}Created: /tmp/converted_map.yaml and /tmp/cropped_map.yaml${NC}"
echo ""
read -p "Press Enter to continue to next tool..."
echo ""

# ===== Tool 4: compare_maps.py =====
echo -e "${YELLOW}===== TOOL 4: compare_maps.py =====${NC}"
echo "Compares two maps"
echo ""
echo -e "${BLUE}Command:${NC} python3 compare_maps.py $DEMO_MAP /tmp/converted_map.yaml"
echo ""
read -p "Press Enter to continue..."
echo ""

python3 "$SCRIPT_DIR/compare_maps.py" "$DEMO_MAP" /tmp/converted_map.yaml

echo ""
echo "Generating visualizations..."
echo -e "${BLUE}Command:${NC} python3 compare_maps.py $DEMO_MAP /tmp/cropped_map.yaml --diff /tmp/diff.png --overlay /tmp/overlay.png --side-by-side /tmp/sbs.png"
echo ""
read -p "Press Enter to continue..."
echo ""

# This will show dimension mismatch for cropped map, which is expected
python3 "$SCRIPT_DIR/compare_maps.py" "$DEMO_MAP" /tmp/cropped_map.yaml --diff /tmp/diff.png --overlay /tmp/overlay.png --side-by-side /tmp/sbs.png 2>&1 || true

echo ""
echo -e "${GREEN}Comparison complete!${NC}"
echo ""

# ===== Summary =====
echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}DEMONSTRATION COMPLETE${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""
echo -e "${GREEN}All four tools demonstrated successfully!${NC}"
echo ""
echo "Tools available:"
echo "  1. validate_map.py    - Validate map correctness"
echo "  2. map_info.py        - Display map information"
echo "  3. map_converter.py   - Convert and modify maps"
echo "  4. compare_maps.py    - Compare two maps"
echo ""
echo "Generated files in /tmp/:"
echo "  - converted_map.yaml / .pgm  (threshold-adjusted map)"
echo "  - cropped_map.yaml / .pgm    (cropped map)"
echo "  - sbs.png                     (side-by-side comparison)"
echo ""
echo "For more information, see:"
echo "  $SCRIPT_DIR/README.md"
echo "  /home/devel/Desktop/WayfindR-driver/findings/map-tools-guide.md"
echo ""
echo -e "${YELLOW}Tip: Run each tool with --help for usage information${NC}"
echo ""
