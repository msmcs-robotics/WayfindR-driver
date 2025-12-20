#!/usr/bin/env bash
# 03_verify_installation.sh
# Verify that ROS2 Humble and SLAM/Navigation packages are correctly installed

set -e

echo "=================================================================="
echo "  ROS2 HUMBLE INSTALLATION VERIFICATION"
echo "=================================================================="
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SUCCESS=0
WARNINGS=0
FAILURES=0

check_pass() {
    echo -e "  ${GREEN}✓${NC} $1"
    ((SUCCESS++))
}

check_fail() {
    echo -e "  ${RED}✗${NC} $1"
    ((FAILURES++))
}

check_warn() {
    echo -e "  ${YELLOW}⚠${NC} $1"
    ((WARNINGS++))
}

# 1. Check ROS2 installation
echo "--- Checking ROS2 Installation ---"
if [ -d "/opt/ros/humble" ]; then
    check_pass "ROS2 Humble directory exists"
else
    check_fail "ROS2 Humble not found at /opt/ros/humble"
fi

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    check_pass "ROS2 environment sourced"
else
    check_fail "Cannot source ROS2 environment"
    exit 1
fi

# Check ROS2 version
if command -v ros2 &> /dev/null; then
    VERSION=$(ros2 --version 2>&1 || echo "unknown")
    check_pass "ros2 command available: $VERSION"
else
    check_fail "ros2 command not found"
fi

# 2. Check workspace
echo ""
echo "--- Checking ROS2 Workspace ---"
WS="$HOME/ros2_ws"
if [ -d "$WS" ]; then
    check_pass "Workspace exists at $WS"
    if [ -d "$WS/src" ]; then
        check_pass "Source directory exists"
    else
        check_warn "Source directory missing"
    fi
    if [ -d "$WS/install" ]; then
        check_pass "Install directory exists"
    else
        check_warn "Workspace not built yet"
    fi
else
    check_fail "Workspace not found at $WS"
fi

# 3. Check SLAM packages
echo ""
echo "--- Checking SLAM Packages ---"
PACKAGES=(
    "slam_toolbox"
    "navigation2"
    "nav2_bringup"
    "nav2_map_server"
    "nav2_simple_commander"
    "robot_localization"
    "rviz2"
    "tf2_tools"
    "imu_tools"
)

for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        check_pass "$pkg installed"
    else
        check_fail "$pkg NOT installed"
    fi
done

# 4. Check Python packages
echo ""
echo "--- Checking Python Dependencies ---"
PYTHON_PKGS=("yaml" "numpy" "serial")
for pkg in "${PYTHON_PKGS[@]}"; do
    if python3 -c "import $pkg" 2>/dev/null; then
        check_pass "Python module '$pkg' available"
    else
        check_warn "Python module '$pkg' not found"
    fi
done

# 5. Check executables
echo ""
echo "--- Checking Key Executables ---"
EXECS=(
    "colcon"
    "rosdep"
    "rviz2"
    "ros2"
)

for exec in "${EXECS[@]}"; do
    if command -v "$exec" &> /dev/null; then
        check_pass "$exec command available"
    else
        check_fail "$exec command not found"
    fi
done

# 6. Check directories
echo ""
echo "--- Checking Map/Waypoint Directories ---"
if [ -d "$HOME/maps" ]; then
    check_pass "Maps directory exists at ~/maps"
else
    check_warn "Maps directory not found, creating..."
    mkdir -p "$HOME/maps"
fi

if [ -d "$HOME/waypoints" ]; then
    check_pass "Waypoints directory exists at ~/waypoints"
else
    check_warn "Waypoints directory not found, creating..."
    mkdir -p "$HOME/waypoints"
fi

# 7. Check user groups
echo ""
echo "--- Checking User Permissions ---"
if groups "$USER" | grep -q "\bdialout\b"; then
    check_pass "User in dialout group (serial access)"
else
    check_warn "User NOT in dialout group - serial devices may not work"
    echo "         Run: sudo usermod -a -G dialout $USER"
    echo "         Then log out and log back in"
fi

# 8. Check bashrc configuration
echo ""
echo "--- Checking Shell Configuration ---"
BASHRC="$HOME/.bashrc"
if grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    check_pass "ROS2 sourced in ~/.bashrc"
else
    check_warn "ROS2 not sourced in ~/.bashrc"
fi

if grep -q "source.*ros2_ws/install/setup.bash" "$BASHRC"; then
    check_pass "Workspace sourced in ~/.bashrc"
else
    check_warn "Workspace not sourced in ~/.bashrc"
fi

# 9. Test SLAM Toolbox launch file
echo ""
echo "--- Checking SLAM Toolbox Launch Files ---"
if ros2 launch slam_toolbox online_async_launch.py --help &>/dev/null; then
    check_pass "SLAM Toolbox launch files accessible"
else
    check_fail "Cannot access SLAM Toolbox launch files"
fi

# 10. Check rosdep
echo ""
echo "--- Checking rosdep ---"
if [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    check_pass "rosdep initialized"
else
    check_warn "rosdep not initialized"
    echo "         Run: sudo rosdep init && rosdep update"
fi

# Summary
echo ""
echo "=================================================================="
echo "  VERIFICATION SUMMARY"
echo "=================================================================="
echo -e "  ${GREEN}Passed:${NC}   $SUCCESS"
echo -e "  ${YELLOW}Warnings:${NC} $WARNINGS"
echo -e "  ${RED}Failed:${NC}   $FAILURES"
echo ""

if [ $FAILURES -eq 0 ]; then
    echo -e "${GREEN}✓ Installation appears to be successful!${NC}"
    echo ""
    echo "You can now:"
    echo "  1. Connect your LiDAR and test SLAM:"
    echo "     ros2 launch slam_toolbox online_async_launch.py"
    echo ""
    echo "  2. Save a map:"
    echo "     ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map"
    echo ""
    echo "  3. Load a map and navigate:"
    echo "     ros2 launch nav2_bringup navigation_launch.py"
    echo ""
else
    echo -e "${RED}✗ Some components are missing or not configured properly${NC}"
    echo ""
    echo "Please review the failures above and:"
    echo "  1. Ensure all installation scripts ran successfully"
    echo "  2. Source your environment: source ~/.bashrc"
    echo "  3. Re-run this verification script"
fi

echo ""
if [ $WARNINGS -gt 0 ]; then
    echo -e "${YELLOW}Note:${NC} Warnings are typically non-critical but should be addressed"
fi

echo "=================================================================="

exit $FAILURES
