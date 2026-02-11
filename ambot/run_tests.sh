#!/bin/bash
# =============================================================================
# Ambot - Comprehensive Test Runner
# =============================================================================
# Runs all tests automatically without manual SSH/curl commands.
# This script is meant to be run ON the target device (RPi or Jetson).
#
# Usage:
#   ./run_tests.sh                    # Run standard tests
#   ./run_tests.sh --all              # Run all tests including hardware
#   ./run_tests.sh --quick            # Quick verification only
#   ./run_tests.sh --hardware         # Hardware tests only (LiDAR, camera, GPIO)
#   ./run_tests.sh --integration      # Integration tests only
#   ./run_tests.sh --test=NAME        # Run a single named test
#   ./run_tests.sh --json             # Output results as JSON
#
# Individual test names (--test=NAME):
#   verify, gpio, camera, camera-basic, camera-faces, lidar, ld19,
#   motors, motors-basic, motors-individual, motors-pinout,
#   wandering-viz, imu, env, env-diag
#
# From dev machine:
#   ./deploy.sh rpi --full-test       # Deploy + run this script
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Activate venv if available
if [ -f "$SCRIPT_DIR/venv/bin/activate" ]; then
    source "$SCRIPT_DIR/venv/bin/activate"
fi

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Results
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_SKIPPED=0
RESULTS=()

# JSON output mode
JSON_OUTPUT=false
VERBOSE=false

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    if [ "$JSON_OUTPUT" = false ]; then
        echo -e "${GREEN}[INFO]${NC} $1"
    fi
}

log_warn() {
    if [ "$JSON_OUTPUT" = false ]; then
        echo -e "${YELLOW}[WARN]${NC} $1"
    fi
}

log_error() {
    if [ "$JSON_OUTPUT" = false ]; then
        echo -e "${RED}[ERROR]${NC} $1"
    fi
}

log_test() {
    if [ "$JSON_OUTPUT" = false ]; then
        echo -e "${BLUE}[TEST]${NC} $1"
    fi
}

add_result() {
    local name="$1"
    local status="$2"
    local message="$3"

    RESULTS+=("{\"name\": \"$name\", \"status\": \"$status\", \"message\": \"$message\"}")

    case "$status" in
        passed) TESTS_PASSED=$((TESTS_PASSED + 1)) ;;
        failed) TESTS_FAILED=$((TESTS_FAILED + 1)) ;;
        skipped) TESTS_SKIPPED=$((TESTS_SKIPPED + 1)) ;;
    esac
}

print_banner() {
    if [ "$JSON_OUTPUT" = false ]; then
        echo ""
        echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${CYAN}║              AMBOT COMPREHENSIVE TEST SUITE                ║${NC}"
        echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
        echo ""
    fi
}

# =============================================================================
# Device Detection
# =============================================================================

detect_platform() {
    if [ -f /proc/device-tree/model ]; then
        MODEL=$(cat /proc/device-tree/model | tr -d '\0')
        if echo "$MODEL" | grep -qi "raspberry"; then
            echo "rpi"
        elif echo "$MODEL" | grep -qi "jetson"; then
            echo "jetson"
        else
            echo "unknown"
        fi
    else
        echo "unknown"
    fi
}

has_gpio() {
    python3 -c "import RPi.GPIO" 2>/dev/null && return 0
    python3 -c "import Jetson.GPIO" 2>/dev/null && return 0
    return 1
}

has_camera() {
    ls /dev/video* &>/dev/null
}

has_lidar() {
    ls /dev/ttyUSB* &>/dev/null
}

# =============================================================================
# Environment Precheck
# =============================================================================

test_environment() {
    log_test "Running environment precheck..."

    local env_ok=true
    local env_details=""

    # Check Python version
    local py_version
    py_version=$(python3 --version 2>&1)
    if [ $? -ne 0 ]; then
        add_result "environment" "failed" "Python3 not found"
        return 1
    fi

    # Check user site-packages are enabled
    local user_site_enabled
    user_site_enabled=$(python3 -c "import site; print(site.ENABLE_USER_SITE)" 2>/dev/null)
    if [ "$user_site_enabled" = "False" ]; then
        log_warn "User site-packages DISABLED (PYTHONNOUSERSITE may be set)"
        env_details="user_site_disabled"
    fi

    # Check key packages are importable
    local missing_pkgs=""
    for pkg in cv2 numpy serial; do
        if ! python3 -c "import $pkg" 2>/dev/null; then
            missing_pkgs="$missing_pkgs $pkg"
            env_ok=false
        fi
    done

    if [ "$env_ok" = true ]; then
        # Collect package info for the result message
        local cv2_loc
        cv2_loc=$(python3 -c "import cv2; f=cv2.__file__; print('user-local' if '.local' in f else 'system')" 2>/dev/null)
        add_result "environment" "passed" "$py_version, packages: $cv2_loc"
        return 0
    else
        add_result "environment" "failed" "Missing packages:$missing_pkgs. Run: sudo ./install.sh --gui --pathfinder"
        log_error "Missing Python packages:$missing_pkgs"
        log_error "Run: sudo ./install.sh --gui --pathfinder"
        log_error "Or: python3 scripts/env_diagnostic.py  (for detailed diagnosis)"
        return 1
    fi
}

# =============================================================================
# Test Functions
# =============================================================================

test_syntax_verification() {
    log_test "Running syntax verification..."

    if python3 tests/verify_all_imports.py 2>&1; then
        add_result "syntax_verification" "passed" "All syntax checks passed"
        return 0
    else
        add_result "syntax_verification" "failed" "Syntax errors found"
        return 1
    fi
}

test_import_verification() {
    log_test "Running import verification..."

    local output
    output=$(python3 tests/verify_all_imports.py --json 2>&1)
    local exit_code=$?

    if [ $exit_code -eq 0 ]; then
        add_result "import_verification" "passed" "All imports successful"
        return 0
    else
        add_result "import_verification" "failed" "Import errors found"
        return 1
    fi
}

test_camera() {
    log_test "Testing camera..."

    if ! has_camera; then
        add_result "camera" "skipped" "No camera device found"
        log_warn "Camera not connected (no /dev/video*)"
        return 0
    fi

    if python3 tests/test_usb_camera.py 2>&1; then
        add_result "camera" "passed" "Camera test passed"
        return 0
    else
        add_result "camera" "failed" "Camera test failed"
        return 1
    fi
}

test_lidar() {
    log_test "Testing LiDAR..."

    if ! has_lidar; then
        add_result "lidar" "skipped" "No LiDAR device found"
        log_warn "LiDAR not connected (no /dev/ttyUSB*)"
        return 0
    fi

    if python3 tests/test_ld19_lidar.py 2>&1; then
        add_result "lidar" "passed" "LiDAR test passed"
        return 0
    else
        add_result "lidar" "failed" "LiDAR test failed"
        return 1
    fi
}

test_gpio() {
    log_test "Testing GPIO..."

    if ! has_gpio; then
        add_result "gpio" "skipped" "GPIO not available on this platform"
        return 0
    fi

    if python3 tests/test_gpio.py 2>&1; then
        add_result "gpio" "passed" "GPIO test passed"
        return 0
    else
        add_result "gpio" "failed" "GPIO test failed"
        return 1
    fi
}

test_integration() {
    log_test "Running integration tests..."

    if python3 tests/test_wandering_integration.py 2>&1; then
        add_result "integration" "passed" "Integration tests passed"
        return 0
    else
        add_result "integration" "failed" "Integration tests failed"
        return 1
    fi
}

test_live_monitor() {
    log_test "Testing live monitor (3 second run)..."

    if timeout 3 python3 live_monitor.py --json 2>&1 >/dev/null; then
        add_result "live_monitor" "passed" "Live monitor runs without error"
        return 0
    else
        # timeout returns 124, which is expected
        if [ $? -eq 124 ]; then
            add_result "live_monitor" "passed" "Live monitor runs without error"
            return 0
        fi
        add_result "live_monitor" "failed" "Live monitor crashed"
        return 1
    fi
}

test_wandering_demo_simulate() {
    log_test "Testing wandering demo (simulate mode, 3 seconds)..."

    if timeout 3 python3 wandering_demo.py --simulate 2>&1 >/dev/null; then
        add_result "wandering_demo" "passed" "Wandering demo runs in simulate mode"
        return 0
    else
        if [ $? -eq 124 ]; then
            add_result "wandering_demo" "passed" "Wandering demo runs in simulate mode"
            return 0
        fi
        add_result "wandering_demo" "failed" "Wandering demo crashed"
        return 1
    fi
}

test_headless_camera_basic() {
    log_test "Testing headless camera (basic feed)..."

    if ! has_camera; then
        add_result "camera_basic" "skipped" "No camera device found"
        return 0
    fi

    if python3 tests/gui_camera.py --headless --captures 1 --no-faces 2>&1; then
        add_result "camera_basic" "passed" "Basic camera capture successful"
        return 0
    else
        add_result "camera_basic" "failed" "Basic camera capture failed"
        return 1
    fi
}

test_headless_camera_faces() {
    log_test "Testing headless camera (face detection)..."

    if ! has_camera; then
        add_result "camera_faces" "skipped" "No camera device found"
        return 0
    fi

    if python3 tests/gui_camera.py --headless --captures 1 --faces 2>&1; then
        add_result "camera_faces" "passed" "Face detection capture successful"
        return 0
    else
        add_result "camera_faces" "failed" "Face detection capture failed"
        return 1
    fi
}

test_headless_lidar_scan() {
    log_test "Testing headless LiDAR scan..."

    if ! has_lidar; then
        add_result "headless_lidar" "skipped" "No LiDAR device found"
        return 0
    fi

    if python3 tests/gui_lidar.py --headless --scans 1 2>&1; then
        add_result "headless_lidar" "passed" "Headless LiDAR scan successful"
        return 0
    else
        add_result "headless_lidar" "failed" "Headless LiDAR scan failed"
        return 1
    fi
}

test_motors() {
    log_test "Testing motors (check)..."

    if ! has_gpio; then
        add_result "motors" "skipped" "GPIO not available on this platform"
        return 0
    fi

    if python3 tests/test_motors.py --check 2>&1; then
        add_result "motors" "passed" "Motor check passed"
        return 0
    else
        add_result "motors" "failed" "Motor check failed"
        return 1
    fi
}

test_motors_basic() {
    log_test "Testing motors (basic)..."

    if ! has_gpio; then
        add_result "motors_basic" "skipped" "GPIO not available on this platform"
        return 0
    fi

    if python3 tests/test_motors.py --basic 2>&1; then
        add_result "motors_basic" "passed" "Basic motor test passed"
        return 0
    else
        add_result "motors_basic" "failed" "Basic motor test failed"
        return 1
    fi
}

test_motors_individual() {
    log_test "Testing motors (individual)..."

    if ! has_gpio; then
        add_result "motors_individual" "skipped" "GPIO not available on this platform"
        return 0
    fi

    if python3 tests/test_motors.py --individual 2>&1; then
        add_result "motors_individual" "passed" "Individual motor test passed"
        return 0
    else
        add_result "motors_individual" "failed" "Individual motor test failed"
        return 1
    fi
}

test_motors_pinout() {
    log_test "Testing motors (pinout)..."

    if ! has_gpio; then
        add_result "motors_pinout" "skipped" "GPIO not available on this platform"
        return 0
    fi

    if python3 tests/test_motors.py --pinout 2>&1; then
        add_result "motors_pinout" "passed" "Motor pinout test passed"
        return 0
    else
        add_result "motors_pinout" "failed" "Motor pinout test failed"
        return 1
    fi
}

test_imu() {
    log_test "Testing IMU (MPU6050)..."

    if python3 tests/test_imu.py 2>&1; then
        add_result "imu" "passed" "IMU test passed"
        return 0
    else
        add_result "imu" "failed" "IMU test failed"
        return 1
    fi
}

test_wandering_viz() {
    log_test "Testing wandering visualization (headless, 3 scans)..."

    if ! has_lidar; then
        add_result "wandering_viz" "skipped" "No LiDAR device found"
        return 0
    fi

    if python3 tests/gui_wandering.py --headless --scans 3 2>&1; then
        add_result "wandering_viz" "passed" "Wandering visualization test passed"
        return 0
    else
        add_result "wandering_viz" "failed" "Wandering visualization test failed"
        return 1
    fi
}

test_env_diagnostic() {
    log_test "Running environment diagnostic..."

    if python3 scripts/env_diagnostic.py 2>&1; then
        add_result "env_diagnostic" "passed" "Environment diagnostic passed"
        return 0
    else
        add_result "env_diagnostic" "failed" "Environment diagnostic failed"
        return 1
    fi
}

# =============================================================================
# Individual Test Dispatch (for --test=NAME)
# =============================================================================

run_individual_test() {
    local test_name="$1"

    case "$test_name" in
        verify)
            test_syntax_verification
            ;;
        gpio)
            test_gpio
            ;;
        camera)
            test_camera
            ;;
        camera-basic)
            test_headless_camera_basic
            ;;
        camera-faces)
            test_headless_camera_faces
            ;;
        lidar|ld19)
            test_lidar
            ;;
        motors)
            test_motors
            ;;
        motors-basic)
            test_motors_basic
            ;;
        motors-individual)
            test_motors_individual
            ;;
        motors-pinout)
            test_motors_pinout
            ;;
        wandering-viz)
            test_wandering_viz
            ;;
        imu)
            test_imu
            ;;
        env|env-diag)
            test_env_diagnostic
            ;;
        *)
            log_error "Unknown test name: $test_name"
            echo ""
            echo "Valid test names:"
            echo "  verify, gpio, camera, camera-basic, camera-faces,"
            echo "  lidar (or ld19), motors, motors-basic, motors-individual,"
            echo "  motors-pinout, wandering-viz, imu, env (or env-diag)"
            echo ""
            echo "For test suites use: --all, --quick, --hardware, --integration"
            return 1
            ;;
    esac
}

# =============================================================================
# Test Suites
# =============================================================================

run_quick_tests() {
    log_info "Running quick verification tests..."
    echo ""

    test_environment
    test_syntax_verification
    test_import_verification
}

run_standard_tests() {
    log_info "Running standard test suite..."
    echo ""

    test_environment
    test_syntax_verification
    test_import_verification
    test_integration
    test_live_monitor
    test_wandering_demo_simulate
}

run_hardware_tests() {
    log_info "Running hardware tests..."
    echo ""

    test_gpio
    test_camera
    test_lidar
    test_headless_camera_basic
    test_headless_camera_faces
    test_headless_lidar_scan
}

run_all_tests() {
    log_info "Running ALL tests..."
    echo ""

    # Environment precheck
    test_environment

    # Verification
    test_syntax_verification
    test_import_verification

    # Integration
    test_integration
    test_live_monitor
    test_wandering_demo_simulate

    # Hardware
    test_gpio
    test_camera
    test_lidar
    test_headless_camera_basic
    test_headless_camera_faces
    test_headless_lidar_scan
}

# =============================================================================
# Output Results
# =============================================================================

print_summary() {
    if [ "$JSON_OUTPUT" = true ]; then
        # JSON output
        echo "{"
        echo "  \"platform\": \"$(detect_platform)\","
        echo "  \"timestamp\": \"$(date -Iseconds)\","
        echo "  \"summary\": {"
        echo "    \"passed\": $TESTS_PASSED,"
        echo "    \"failed\": $TESTS_FAILED,"
        echo "    \"skipped\": $TESTS_SKIPPED,"
        echo "    \"total\": $((TESTS_PASSED + TESTS_FAILED + TESTS_SKIPPED))"
        echo "  },"
        echo "  \"results\": ["
        local first=true
        for result in "${RESULTS[@]}"; do
            if [ "$first" = true ]; then
                first=false
            else
                echo ","
            fi
            echo -n "    $result"
        done
        echo ""
        echo "  ]"
        echo "}"
    else
        # Human-readable output
        echo ""
        echo "═══════════════════════════════════════════════════════════════"
        echo "TEST SUMMARY"
        echo "═══════════════════════════════════════════════════════════════"
        echo ""
        echo -e "Platform: ${CYAN}$(detect_platform)${NC}"
        echo ""
        echo -e "  ${GREEN}Passed:${NC}  $TESTS_PASSED"
        echo -e "  ${RED}Failed:${NC}  $TESTS_FAILED"
        echo -e "  ${YELLOW}Skipped:${NC} $TESTS_SKIPPED"
        echo -e "  Total:   $((TESTS_PASSED + TESTS_FAILED + TESTS_SKIPPED))"
        echo ""

        if [ $TESTS_FAILED -eq 0 ]; then
            echo -e "${GREEN}All tests PASSED!${NC}"
        else
            echo -e "${RED}$TESTS_FAILED test(s) FAILED${NC}"
        fi
        echo ""
    fi
}

# =============================================================================
# Help
# =============================================================================

print_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Test Suites:"
    echo "  (none)          Run standard tests (verification + integration)"
    echo "  --all, -a       Run ALL tests including hardware"
    echo "  --quick, -q     Quick verification only (syntax + imports)"
    echo "  --hardware, -w  Hardware tests only (GPIO, camera, LiDAR)"
    echo "  --integration   Integration tests only"
    echo ""
    echo "Individual Tests (--test=NAME):"
    echo "  verify          Import verification"
    echo "  gpio            GPIO test"
    echo "  camera          USB camera test"
    echo "  camera-basic    Headless camera (no face detection)"
    echo "  camera-faces    Headless camera (with face detection)"
    echo "  lidar, ld19     LiDAR test"
    echo "  motors          Motor check"
    echo "  motors-basic    Basic motor test"
    echo "  motors-individual  Individual motor test"
    echo "  motors-pinout   Motor pinout test"
    echo "  wandering-viz   Wandering visualization (headless)"
    echo "  imu             IMU (MPU6050) test"
    echo "  env, env-diag   Environment diagnostic"
    echo ""
    echo "Output Options:"
    echo "  --json, -j      Output results as JSON"
    echo "  --verbose, -v   Verbose output"
    echo ""
    echo "Other:"
    echo "  --help, -h      Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                        # Standard tests"
    echo "  $0 --all                  # All tests"
    echo "  $0 --quick --json         # Quick check with JSON output"
    echo "  $0 --hardware             # Test connected hardware"
    echo "  $0 --test=lidar           # Single LiDAR test"
    echo "  $0 --test=motors-basic    # Single motor test"
    echo ""
    echo "From dev machine:"
    echo "  ./deploy.sh rpi --full-test   # Deploy + run all tests"
    echo "  ./deploy.sh rpi --verify      # Deploy + run verification"
}

# =============================================================================
# Main
# =============================================================================

main() {
    local TEST_SUITE="standard"
    local INDIVIDUAL_TEST=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --all|-a)
                TEST_SUITE="all"
                shift
                ;;
            --quick|-q)
                TEST_SUITE="quick"
                shift
                ;;
            --hardware|-w)
                TEST_SUITE="hardware"
                shift
                ;;
            --integration)
                TEST_SUITE="integration"
                shift
                ;;
            --test=*)
                TEST_SUITE="individual"
                INDIVIDUAL_TEST="${1#*=}"
                shift
                ;;
            --json|-j)
                JSON_OUTPUT=true
                shift
                ;;
            --verbose|-v)
                VERBOSE=true
                shift
                ;;
            --help|-h)
                print_help
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
    done

    print_banner

    # Detect platform
    PLATFORM=$(detect_platform)
    log_info "Detected platform: $PLATFORM"
    echo ""

    # Run selected test suite
    case "$TEST_SUITE" in
        quick)
            run_quick_tests
            ;;
        standard)
            run_standard_tests
            ;;
        hardware)
            run_hardware_tests
            ;;
        integration)
            test_integration
            ;;
        all)
            run_all_tests
            ;;
        individual)
            run_individual_test "$INDIVIDUAL_TEST"
            ;;
    esac

    # Print summary
    print_summary

    # Exit with error code if tests failed
    if [ $TESTS_FAILED -gt 0 ]; then
        exit 1
    fi
    exit 0
}

main "$@"
