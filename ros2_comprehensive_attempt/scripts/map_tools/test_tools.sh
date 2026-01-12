#!/bin/bash

# Automated test script for ROS2 Map Tools
# Tests all four tools with various scenarios

# Don't exit on error - we want to count failed tests
# set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps"
TEST_MAP="$MAP_DIR/first_map.yaml"
TEMP_DIR="/tmp/map_tools_test_$$"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

TESTS_PASSED=0
TESTS_FAILED=0

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}ROS2 Map Tools - Automated Test Suite${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# Create temp directory
mkdir -p "$TEMP_DIR"
echo -e "Created temp directory: $TEMP_DIR"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up...${NC}"
    rm -rf "$TEMP_DIR"
}
trap cleanup EXIT

# Test function
run_test() {
    local test_name="$1"
    local test_cmd="$2"

    echo -n "Testing: $test_name ... "

    if eval "$test_cmd" > "$TEMP_DIR/test_output.log" 2>&1; then
        echo -e "${GREEN}PASS${NC}"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}FAIL${NC}"
        echo "  Command: $test_cmd"
        echo "  Output:"
        cat "$TEMP_DIR/test_output.log" | sed 's/^/    /'
        ((TESTS_FAILED++))
        return 1
    fi
}

# Check test map exists
if [ ! -f "$TEST_MAP" ]; then
    echo -e "${RED}Error: Test map not found at $TEST_MAP${NC}"
    exit 1
fi

echo -e "${GREEN}Using test map: $TEST_MAP${NC}"
echo ""

# ===== Test 1: validate_map.py =====
echo -e "${YELLOW}Testing validate_map.py${NC}"
echo "-----------------------------------"

run_test "Validate existing map" \
    "python3 '$SCRIPT_DIR/validate_map.py' '$TEST_MAP'"

run_test "Validate non-existent map (should fail gracefully)" \
    "python3 '$SCRIPT_DIR/validate_map.py' '/nonexistent/map.yaml' 2>&1 | grep -q 'does not exist'; test \$? -eq 0"

echo ""

# ===== Test 2: map_info.py =====
echo -e "${YELLOW}Testing map_info.py${NC}"
echo "-----------------------------------"

run_test "Display map info" \
    "python3 '$SCRIPT_DIR/map_info.py' '$TEST_MAP'"

run_test "Display detailed map info" \
    "python3 '$SCRIPT_DIR/map_info.py' --detailed '$TEST_MAP'"

run_test "Display info for non-existent map (should fail gracefully)" \
    "python3 '$SCRIPT_DIR/map_info.py' '/nonexistent/map.yaml' 2>&1 | grep -q 'Error'; test \$? -eq 0"

echo ""

# ===== Test 3: map_converter.py =====
echo -e "${YELLOW}Testing map_converter.py${NC}"
echo "-----------------------------------"

run_test "Adjust thresholds" \
    "python3 '$SCRIPT_DIR/map_converter.py' '$TEST_MAP' -o '$TEMP_DIR/thresh_map.yaml' --thresholds 0.20 0.70"

run_test "Verify threshold adjustment" \
    "grep -q 'free_thresh: 0.2' '$TEMP_DIR/thresh_map.yaml' && grep -q 'occupied_thresh: 0.7' '$TEMP_DIR/thresh_map.yaml'"

run_test "Crop map" \
    "python3 '$SCRIPT_DIR/map_converter.py' '$TEST_MAP' -o '$TEMP_DIR/crop_map.yaml' --crop 10 10 100 80"

run_test "Verify cropped dimensions" \
    "test -f '$TEMP_DIR/crop_map.pgm'"

run_test "Resize map" \
    "python3 '$SCRIPT_DIR/map_converter.py' '$TEST_MAP' -o '$TEMP_DIR/resize_map.yaml' --resize 0.1"

run_test "Verify resize in YAML" \
    "grep -q 'resolution: 0.1' '$TEMP_DIR/resize_map.yaml'"

run_test "Invert map" \
    "python3 '$SCRIPT_DIR/map_converter.py' '$TEST_MAP' -o '$TEMP_DIR/invert_map.yaml' --invert"

run_test "Verify invert (negate flag toggled)" \
    "grep -q 'negate: 1' '$TEMP_DIR/invert_map.yaml'"

run_test "Multiple operations (resize + thresholds)" \
    "python3 '$SCRIPT_DIR/map_converter.py' '$TEST_MAP' -o '$TEMP_DIR/multi_map.yaml' --resize 0.08 --thresholds 0.30 0.60"

run_test "Invalid thresholds (should fail)" \
    "! python3 '$SCRIPT_DIR/map_converter.py' '$TEST_MAP' -o '$TEMP_DIR/bad.yaml' --thresholds 0.7 0.3 2>&1"

echo ""

# ===== Test 4: compare_maps.py =====
echo -e "${YELLOW}Testing compare_maps.py${NC}"
echo "-----------------------------------"

# Create a second map for comparison
python3 "$SCRIPT_DIR/map_converter.py" "$TEST_MAP" -o "$TEMP_DIR/compare_map.yaml" --thresholds 0.30 0.60 > /dev/null 2>&1

run_test "Compare two maps" \
    "python3 '$SCRIPT_DIR/compare_maps.py' '$TEST_MAP' '$TEMP_DIR/compare_map.yaml'"

run_test "Generate difference map" \
    "python3 '$SCRIPT_DIR/map_converter.py' '$TEST_MAP' -o '$TEMP_DIR/map2.yaml' --thresholds 0.3 0.6 > /dev/null 2>&1 && \
     python3 '$SCRIPT_DIR/compare_maps.py' '$TEST_MAP' '$TEMP_DIR/map2.yaml' --diff '$TEMP_DIR/diff.png'"

run_test "Verify difference image created" \
    "test -f '$TEMP_DIR/diff.png'"

run_test "Generate side-by-side comparison" \
    "python3 '$SCRIPT_DIR/compare_maps.py' '$TEST_MAP' '$TEMP_DIR/map2.yaml' --side-by-side '$TEMP_DIR/sbs.png'"

run_test "Verify side-by-side image created" \
    "test -f '$TEMP_DIR/sbs.png'"

echo ""

# ===== Summary =====
echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}TEST SUMMARY${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""
echo -e "Tests passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests failed: ${RED}$TESTS_FAILED${NC}"
echo -e "Total tests:  $((TESTS_PASSED + TESTS_FAILED))"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    echo ""
    exit 0
else
    echo -e "${RED}Some tests failed.${NC}"
    echo ""
    exit 1
fi
