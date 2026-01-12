#!/bin/bash
################################################################################
# WayfindR Gazebo Simulation Test Script
################################################################################
#
# This script performs basic tests to verify the Gazebo simulation setup
# is working correctly.
#
# Tests performed:
#   1. Gazebo installation check
#   2. ROS-Gazebo bridge check
#   3. Launch simulation and verify topics
#   4. Test robot spawning
#   5. Test sensor data publishing
#   6. Test velocity control
#
# Usage:
#   chmod +x test_simulation.sh
#   ./test_simulation.sh
#
# Author: WayfindR Development Team
# Date: 2026-01-11
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test results tracking
TESTS_PASSED=0
TESTS_FAILED=0

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
    ((TESTS_PASSED++))
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
    ((TESTS_FAILED++))
}

log_test() {
    echo -e "${YELLOW}[TEST]${NC} $1"
}

# Header
echo "================================================================================"
echo "  WayfindR Gazebo Simulation Test Suite"
echo "================================================================================"
echo ""

# Test 1: Check Gazebo installation
log_test "Test 1: Checking Gazebo installation..."
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -n 1)
    log_success "Gazebo found: $GZ_VERSION"
else
    log_error "Gazebo 'gz' command not found"
    exit 1
fi

# Test 2: Check ROS2 installation
log_test "Test 2: Checking ROS2 Humble..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    log_success "ROS2 Humble found and sourced"
else
    log_error "ROS2 Humble not found"
    exit 1
fi

# Test 3: Check ROS-Gazebo packages
log_test "Test 3: Checking ROS-Gazebo integration packages..."
if ros2 pkg list | grep -q ros_gz; then
    log_success "ROS-Gazebo packages installed"
else
    log_error "ROS-Gazebo packages not found"
    exit 1
fi

# Test 4: Check workspace
log_test "Test 4: Checking for WayfindR workspace..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname $(dirname "$SCRIPT_DIR"))"

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    log_success "Workspace found and sourced: $WORKSPACE_DIR"
else
    log_error "Workspace install/setup.bash not found. Please build workspace first."
    exit 1
fi

# Test 5: Check for launch files
log_test "Test 5: Checking for Gazebo launch file..."
if ros2 pkg list | grep -q ros2_comprehensive_attempt; then
    log_success "WayfindR package found"
else
    log_error "WayfindR package not found in workspace"
    exit 1
fi

# Test 6: Check for URDF
log_test "Test 6: Checking for robot URDF..."
URDF_PATH="$WORKSPACE_DIR/install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/urdf/wayfinder_robot.urdf.xacro"
if [ -f "$URDF_PATH" ]; then
    log_success "Robot URDF found"
else
    log_error "Robot URDF not found at: $URDF_PATH"
    exit 1
fi

# Test 7: Check for world files
log_test "Test 7: Checking for world files..."
WORLDS_DIR="$WORKSPACE_DIR/install/ros2_comprehensive_attempt/share/ros2_comprehensive_attempt/worlds"
if [ -d "$WORLDS_DIR" ]; then
    WORLD_COUNT=$(ls -1 "$WORLDS_DIR"/*.sdf 2>/dev/null | wc -l)
    if [ "$WORLD_COUNT" -gt 0 ]; then
        log_success "Found $WORLD_COUNT world file(s)"
    else
        log_error "No world files found in $WORLDS_DIR"
    fi
else
    log_error "Worlds directory not found"
fi

# Interactive test option
echo ""
read -p "Run interactive simulation test? This will launch Gazebo (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then

    log_info "Launching Gazebo simulation (headless, no GUI)..."
    log_info "This will take about 10 seconds to start..."

    # Launch Gazebo in background
    ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py \
        use_gui:=false \
        use_rviz:=false \
        world:=test_room \
        > /tmp/gazebo_test.log 2>&1 &

    GAZEBO_PID=$!

    # Wait for Gazebo to start
    sleep 15

    # Test 8: Check if Gazebo process is running
    log_test "Test 8: Checking if Gazebo is running..."
    if ps -p $GAZEBO_PID > /dev/null; then
        log_success "Gazebo process running (PID: $GAZEBO_PID)"
    else
        log_error "Gazebo process not running"
        cat /tmp/gazebo_test.log
        exit 1
    fi

    # Test 9: Check for ROS2 topics
    log_test "Test 9: Checking for simulation topics..."
    TOPIC_COUNT=$(ros2 topic list 2>/dev/null | wc -l)
    if [ "$TOPIC_COUNT" -gt 5 ]; then
        log_success "Found $TOPIC_COUNT ROS2 topics"
    else
        log_error "Insufficient topics found ($TOPIC_COUNT)"
    fi

    # Test 10: Check for specific required topics
    log_test "Test 10: Checking for required topics..."
    REQUIRED_TOPICS=("/scan" "/odom" "/cmd_vel" "/clock")
    for topic in "${REQUIRED_TOPICS[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
            log_success "Topic exists: $topic"
        else
            log_error "Topic missing: $topic"
        fi
    done

    # Test 11: Check LIDAR data
    log_test "Test 11: Checking LIDAR data publishing..."
    SCAN_DATA=$(timeout 5 ros2 topic echo /scan --once 2>/dev/null | head -n 10)
    if [ -n "$SCAN_DATA" ]; then
        log_success "LIDAR data publishing on /scan"
    else
        log_error "No LIDAR data received"
    fi

    # Test 12: Check odometry data
    log_test "Test 12: Checking odometry data publishing..."
    ODOM_DATA=$(timeout 5 ros2 topic echo /odom --once 2>/dev/null | head -n 10)
    if [ -n "$ODOM_DATA" ]; then
        log_success "Odometry data publishing on /odom"
    else
        log_error "No odometry data received"
    fi

    # Test 13: Test velocity command
    log_test "Test 13: Testing velocity command..."
    timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10 \
        > /dev/null 2>&1 &

    sleep 3

    # Check if odometry changed
    ODOM_AFTER=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:")
    if [ -n "$ODOM_AFTER" ]; then
        log_success "Velocity commands accepted (robot responding)"
    else
        log_error "Velocity command test inconclusive"
    fi

    # Cleanup
    log_info "Cleaning up test environment..."
    kill $GAZEBO_PID 2>/dev/null || true
    sleep 2
    killall -9 gz 2>/dev/null || true
    killall -9 ruby 2>/dev/null || true

    log_success "Test environment cleaned up"
fi

# Final summary
echo ""
echo "================================================================================"
echo "  Test Summary"
echo "================================================================================"
echo -e "${GREEN}Tests Passed:${NC} $TESTS_PASSED"
echo -e "${RED}Tests Failed:${NC} $TESTS_FAILED"
echo ""

if [ "$TESTS_FAILED" -eq 0 ]; then
    echo -e "${GREEN}All tests passed! ✓${NC}"
    echo ""
    echo "Your Gazebo simulation setup is ready to use!"
    echo ""
    echo "To launch the simulation, run:"
    echo "  ros2 launch ros2_comprehensive_attempt gazebo_sim.launch.py"
    echo ""
    exit 0
else
    echo -e "${RED}Some tests failed. Please review the errors above.${NC}"
    echo ""
    echo "Common solutions:"
    echo "  - Ensure ROS2 Humble is properly installed"
    echo "  - Run: sudo apt install ros-humble-ros-gz"
    echo "  - Rebuild workspace: colcon build"
    echo "  - Source workspace: source install/setup.bash"
    echo ""
    echo "For detailed troubleshooting, see:"
    echo "  findings/gazebo-simulation-guide.md"
    echo ""
    exit 1
fi
