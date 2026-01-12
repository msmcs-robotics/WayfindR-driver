#!/bin/bash
###############################################################################
# WayfindR Full Diagnostics Suite
#
# Runs a complete diagnostic check of the WayfindR navigation system.
# Generates a comprehensive report with all metrics and health checks.
#
# Usage:
#   ./run_full_diagnostics.sh [output_dir]
#
# Example:
#   ./run_full_diagnostics.sh ~/diagnostics_reports/$(date +%Y%m%d_%H%M%S)
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Output directory
OUTPUT_DIR="${1:-./diagnostics_output}"
mkdir -p "$OUTPUT_DIR"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         WayfindR Full Diagnostics Suite                       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Output directory: ${OUTPUT_DIR}${NC}"
echo -e "${BLUE}Timestamp: $(date)${NC}"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS2 not sourced. Please source your ROS2 installation.${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 environment detected (${ROS_DISTRO})${NC}"
echo ""

# Initialize report
REPORT_FILE="${OUTPUT_DIR}/diagnostics_report.txt"
echo "WayfindR Full Diagnostics Report" > "$REPORT_FILE"
echo "Generated: $(date)" >> "$REPORT_FILE"
echo "========================================" >> "$REPORT_FILE"
echo "" >> "$REPORT_FILE"

# Counter for test results
PASSED=0
FAILED=0
WARNINGS=0

###############################################################################
# Test 1: TF Tree Health Check
###############################################################################
echo -e "${BLUE}[1/7] Checking TF tree health...${NC}"

if timeout 10 python3 "${SCRIPT_DIR}/tf_tree_visualizer.py" --check > "${OUTPUT_DIR}/tf_tree_check.txt" 2>&1; then
    echo -e "${GREEN}  ✓ TF tree is healthy${NC}"
    echo "TF Tree: PASSED" >> "$REPORT_FILE"
    ((PASSED++))
else
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 124 ]; then
        echo -e "${RED}  ✗ TF tree check timeout${NC}"
        echo "TF Tree: TIMEOUT" >> "$REPORT_FILE"
    else
        echo -e "${RED}  ✗ TF tree has issues${NC}"
        echo "TF Tree: FAILED" >> "$REPORT_FILE"
    fi
    ((FAILED++))
fi

# Export TF tree
timeout 10 python3 "${SCRIPT_DIR}/tf_tree_visualizer.py" --export "${OUTPUT_DIR}/tf_tree.txt" 2>/dev/null || true

###############################################################################
# Test 2: Topic Health Check
###############################################################################
echo -e "${BLUE}[2/7] Checking topic health...${NC}"

timeout 15 python3 "${SCRIPT_DIR}/topic_checker.py" --all > "${OUTPUT_DIR}/topic_health.txt" 2>&1 &
TOPIC_PID=$!

sleep 10
kill $TOPIC_PID 2>/dev/null || true
wait $TOPIC_PID 2>/dev/null || true

if grep -q "OK" "${OUTPUT_DIR}/topic_health.txt" 2>/dev/null; then
    echo -e "${GREEN}  ✓ Topics are publishing${NC}"
    echo "Topics: PASSED" >> "$REPORT_FILE"
    ((PASSED++))
else
    echo -e "${YELLOW}  ⚠ Some topics may have issues${NC}"
    echo "Topics: WARNING" >> "$REPORT_FILE"
    ((WARNINGS++))
fi

###############################################################################
# Test 3: Localization Quality
###############################################################################
echo -e "${BLUE}[3/7] Checking localization quality...${NC}"

if timeout 15 python3 "${SCRIPT_DIR}/localization_quality.py" --monitor 10 --export "${OUTPUT_DIR}/localization_quality.txt" > /dev/null 2>&1; then
    # Check quality from output
    if grep -q "EXCELLENT\|GOOD" "${OUTPUT_DIR}/localization_quality.txt" 2>/dev/null; then
        echo -e "${GREEN}  ✓ Localization quality is good${NC}"
        echo "Localization: PASSED" >> "$REPORT_FILE"
        ((PASSED++))
    elif grep -q "FAIR" "${OUTPUT_DIR}/localization_quality.txt" 2>/dev/null; then
        echo -e "${YELLOW}  ⚠ Localization quality is fair${NC}"
        echo "Localization: WARNING" >> "$REPORT_FILE"
        ((WARNINGS++))
    else
        echo -e "${RED}  ✗ Localization quality is poor${NC}"
        echo "Localization: FAILED" >> "$REPORT_FILE"
        ((FAILED++))
    fi
else
    echo -e "${YELLOW}  ⚠ Could not check localization quality${NC}"
    echo "Localization: SKIPPED (no data)" >> "$REPORT_FILE"
fi

###############################################################################
# Test 4: Performance Profiling
###############################################################################
echo -e "${BLUE}[4/7] Profiling system performance...${NC}"

if timeout 35 python3 "${SCRIPT_DIR}/performance_profiler.py" --duration 30 --export "${OUTPUT_DIR}/performance_profile.txt" > /dev/null 2>&1; then
    # Check latency from output
    if grep -q "latency" "${OUTPUT_DIR}/performance_profile.txt" 2>/dev/null; then
        MEAN_LATENCY=$(grep "Mean:" "${OUTPUT_DIR}/performance_profile.txt" | grep -oP '\d+\.\d+' | head -1)
        if [ ! -z "$MEAN_LATENCY" ]; then
            if (( $(echo "$MEAN_LATENCY < 100" | bc -l) )); then
                echo -e "${GREEN}  ✓ Performance is good (${MEAN_LATENCY}ms)${NC}"
                echo "Performance: PASSED (${MEAN_LATENCY}ms)" >> "$REPORT_FILE"
                ((PASSED++))
            else
                echo -e "${YELLOW}  ⚠ Performance could be better (${MEAN_LATENCY}ms)${NC}"
                echo "Performance: WARNING (${MEAN_LATENCY}ms)" >> "$REPORT_FILE"
                ((WARNINGS++))
            fi
        fi
    else
        echo -e "${YELLOW}  ⚠ Could not measure performance${NC}"
        echo "Performance: WARNING (no data)" >> "$REPORT_FILE"
        ((WARNINGS++))
    fi
else
    echo -e "${YELLOW}  ⚠ Performance profiling incomplete${NC}"
    echo "Performance: WARNING (timeout)" >> "$REPORT_FILE"
    ((WARNINGS++))
fi

###############################################################################
# Test 5: Map Quality (if map exists)
###############################################################################
echo -e "${BLUE}[5/7] Checking map quality...${NC}"

MAP_FILE="${SCRIPT_DIR}/../../maps/office.yaml"
if [ -f "$MAP_FILE" ]; then
    if timeout 15 python3 "${SCRIPT_DIR}/map_quality_analyzer.py" --map "$MAP_FILE" > "${OUTPUT_DIR}/map_quality.txt" 2>&1; then
        if grep -q "Map quality is GOOD" "${OUTPUT_DIR}/map_quality.txt" 2>/dev/null; then
            echo -e "${GREEN}  ✓ Map quality is good${NC}"
            echo "Map Quality: PASSED" >> "$REPORT_FILE"
            ((PASSED++))
        else
            echo -e "${YELLOW}  ⚠ Map has some issues${NC}"
            echo "Map Quality: WARNING" >> "$REPORT_FILE"
            ((WARNINGS++))
        fi
    else
        echo -e "${YELLOW}  ⚠ Could not analyze map${NC}"
        echo "Map Quality: WARNING" >> "$REPORT_FILE"
        ((WARNINGS++))
    fi
else
    echo -e "${YELLOW}  ⚠ Map file not found (${MAP_FILE})${NC}"
    echo "Map Quality: SKIPPED (no map)" >> "$REPORT_FILE"
fi

###############################################################################
# Test 6: System Resources
###############################################################################
echo -e "${BLUE}[6/7] Checking system resources...${NC}"

CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1}')
MEM_USAGE=$(free | grep Mem | awk '{print ($3/$2) * 100.0}')

echo "System Resources:" >> "${OUTPUT_DIR}/system_resources.txt"
echo "  CPU Usage: ${CPU_USAGE}%" >> "${OUTPUT_DIR}/system_resources.txt"
echo "  Memory Usage: ${MEM_USAGE}%" >> "${OUTPUT_DIR}/system_resources.txt"

if (( $(echo "$CPU_USAGE < 70" | bc -l) )) && (( $(echo "$MEM_USAGE < 75" | bc -l) )); then
    echo -e "${GREEN}  ✓ System resources healthy (CPU: ${CPU_USAGE}%, MEM: ${MEM_USAGE}%)${NC}"
    echo "System Resources: PASSED" >> "$REPORT_FILE"
    ((PASSED++))
elif (( $(echo "$CPU_USAGE < 90" | bc -l) )) && (( $(echo "$MEM_USAGE < 90" | bc -l) )); then
    echo -e "${YELLOW}  ⚠ System resources elevated (CPU: ${CPU_USAGE}%, MEM: ${MEM_USAGE}%)${NC}"
    echo "System Resources: WARNING" >> "$REPORT_FILE"
    ((WARNINGS++))
else
    echo -e "${RED}  ✗ System resources critical (CPU: ${CPU_USAGE}%, MEM: ${MEM_USAGE}%)${NC}"
    echo "System Resources: FAILED" >> "$REPORT_FILE"
    ((FAILED++))
fi

###############################################################################
# Test 7: ROS2 Nodes
###############################################################################
echo -e "${BLUE}[7/7] Checking ROS2 nodes...${NC}"

if timeout 5 ros2 node list > "${OUTPUT_DIR}/ros2_nodes.txt" 2>&1; then
    NODE_COUNT=$(wc -l < "${OUTPUT_DIR}/ros2_nodes.txt")
    if [ $NODE_COUNT -gt 0 ]; then
        echo -e "${GREEN}  ✓ Found ${NODE_COUNT} ROS2 nodes${NC}"
        echo "ROS2 Nodes: PASSED (${NODE_COUNT} nodes)" >> "$REPORT_FILE"
        ((PASSED++))
    else
        echo -e "${RED}  ✗ No ROS2 nodes found${NC}"
        echo "ROS2 Nodes: FAILED" >> "$REPORT_FILE"
        ((FAILED++))
    fi
else
    echo -e "${RED}  ✗ Could not query ROS2 nodes${NC}"
    echo "ROS2 Nodes: FAILED" >> "$REPORT_FILE"
    ((FAILED++))
fi

###############################################################################
# Generate Summary
###############################################################################
echo ""
echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                    Diagnostics Summary                         ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

TOTAL=$((PASSED + FAILED + WARNINGS))

echo -e "  ${GREEN}Passed:   ${PASSED}/${TOTAL}${NC}"
echo -e "  ${YELLOW}Warnings: ${WARNINGS}/${TOTAL}${NC}"
echo -e "  ${RED}Failed:   ${FAILED}/${TOTAL}${NC}"
echo ""

# Append to report
echo "" >> "$REPORT_FILE"
echo "========================================" >> "$REPORT_FILE"
echo "Summary:" >> "$REPORT_FILE"
echo "  Passed:   ${PASSED}/${TOTAL}" >> "$REPORT_FILE"
echo "  Warnings: ${WARNINGS}/${TOTAL}" >> "$REPORT_FILE"
echo "  Failed:   ${FAILED}/${TOTAL}" >> "$REPORT_FILE"
echo "" >> "$REPORT_FILE"

# Overall result
if [ $FAILED -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✓ Overall Result: EXCELLENT${NC}"
    echo "Overall Result: EXCELLENT" >> "$REPORT_FILE"
    EXIT_CODE=0
elif [ $FAILED -eq 0 ]; then
    echo -e "${YELLOW}⚠ Overall Result: GOOD (with warnings)${NC}"
    echo "Overall Result: GOOD (with warnings)" >> "$REPORT_FILE"
    EXIT_CODE=0
else
    echo -e "${RED}✗ Overall Result: ISSUES DETECTED${NC}"
    echo "Overall Result: ISSUES DETECTED" >> "$REPORT_FILE"
    EXIT_CODE=1
fi

echo ""
echo -e "${BLUE}Full report saved to: ${OUTPUT_DIR}/diagnostics_report.txt${NC}"
echo -e "${BLUE}Detailed logs available in: ${OUTPUT_DIR}/${NC}"
echo ""

# List generated files
echo "Generated files:"
ls -lh "$OUTPUT_DIR" | tail -n +2 | awk '{print "  - " $9 " (" $5 ")"}'

exit $EXIT_CODE
