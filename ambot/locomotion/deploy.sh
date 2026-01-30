#!/bin/bash
# =============================================================================
# Locomotion - Motor Control Deployment Script
# =============================================================================
# Deployment and diagnostic script for the Ambot motor control system.
# Handles setup, diagnostics, and testing for YAHBOOM G1 motor driver.
#
# Usage:
#   ./deploy.sh <command> [options]
#
# Commands:
#   setup       Check GPIO permissions and Python dependencies
#   start       Start motor control server (if applicable)
#   stop        Stop motor processes and ensure motors are stopped (safety)
#   status      Show GPIO availability, motor driver status, permissions
#   diagnose    Run comprehensive diagnostics
#   test        Run basic motor test
#   help        Show this help
#
# All commands are idempotent - safe to run multiple times.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# =============================================================================
# Configuration
# =============================================================================
PROJECT_NAME="locomotion"
YAHBOOM_DIR="$SCRIPT_DIR/yahboomg1"
LOG_DIR="$SCRIPT_DIR/logs"
STATE_FILE="$SCRIPT_DIR/.deploy_state"

# Motor process patterns to search for
MOTOR_PROCESS_PATTERNS="motor.py|test_motors.py|motor_server|locomotion"

# Create log directory
mkdir -p "$LOG_DIR"

# =============================================================================
# Colors and Logging
# =============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()    { echo -e "${BLUE}[====]${NC} $1"; }
log_check()   { echo -e "${CYAN}[CHECK]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC} $1"; }
log_fail()    { echo -e "${RED}[FAIL]${NC} $1"; }

# Logging with timestamp
log_to_file() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$LOG_DIR/deploy.log"
}

# =============================================================================
# State Management (for idempotency)
# =============================================================================
save_state() {
    echo "$1=$(date '+%Y-%m-%d %H:%M:%S')" >> "$STATE_FILE"
}

check_state() {
    grep -q "^$1=" "$STATE_FILE" 2>/dev/null
}

clear_state() {
    rm -f "$STATE_FILE"
}

# =============================================================================
# Platform Detection
# =============================================================================
detect_platform() {
    PLATFORM="unknown"
    MACHINE=$(uname -m)

    # Check for Jetson
    if [ -f /etc/nv_tegra_release ]; then
        PLATFORM="jetson"
    # Check for Raspberry Pi
    elif [ -f /proc/device-tree/model ]; then
        MODEL=$(cat /proc/device-tree/model 2>/dev/null | tr '\0' ' ')
        if echo "$MODEL" | grep -qi "raspberry"; then
            PLATFORM="raspberry_pi"
        fi
    fi

    # Fallback: check architecture
    if [ "$PLATFORM" = "unknown" ]; then
        if [ "$MACHINE" = "aarch64" ] && [ -d /sys/class/gpio ]; then
            PLATFORM="arm64_generic"
        elif [ "$MACHINE" = "armv7l" ]; then
            PLATFORM="arm32_generic"
        fi
    fi

    echo "$PLATFORM"
}

get_platform_info() {
    PLATFORM=$(detect_platform)

    case "$PLATFORM" in
        jetson)
            if [ -f /etc/nv_tegra_release ]; then
                L4T_VERSION=$(head -1 /etc/nv_tegra_release | sed 's/.*R\([0-9]*\).*/\1/')
                echo "Jetson (L4T R${L4T_VERSION})"
            else
                echo "Jetson"
            fi
            ;;
        raspberry_pi)
            MODEL=$(cat /proc/device-tree/model 2>/dev/null | tr '\0' ' ' | sed 's/ *$//')
            echo "$MODEL"
            ;;
        arm64_generic)
            echo "ARM64 (Generic)"
            ;;
        arm32_generic)
            echo "ARM32 (Generic)"
            ;;
        *)
            echo "Unknown ($(uname -m))"
            ;;
    esac
}

# =============================================================================
# GPIO Checks
# =============================================================================
check_gpio_library() {
    local PLATFORM=$(detect_platform)

    # Check for Jetson.GPIO
    if python3 -c "import Jetson.GPIO" 2>/dev/null; then
        log_success "GPIO Library: Jetson.GPIO installed"
        return 0
    fi

    # Check for RPi.GPIO
    if python3 -c "import RPi.GPIO" 2>/dev/null; then
        log_success "GPIO Library: RPi.GPIO installed"
        return 0
    fi

    # Neither found
    log_fail "GPIO Library: Not found"
    if [ "$PLATFORM" = "jetson" ]; then
        log_info "  Install with: sudo pip3 install Jetson.GPIO"
    else
        log_info "  Install with: sudo pip3 install RPi.GPIO"
    fi
    return 1
}

check_gpio_group() {
    if groups | grep -qE '\bgpio\b'; then
        log_success "User in gpio group: Yes"
        return 0
    else
        log_fail "User in gpio group: No"
        log_info "  Fix with: sudo usermod -a -G gpio \$USER"
        log_info "  Then log out and back in"
        return 1
    fi
}

check_gpio_access() {
    # Check if /dev/gpiochip* exists
    if ls /dev/gpiochip* &>/dev/null; then
        log_success "GPIO Device: /dev/gpiochip* present"
    else
        log_warn "GPIO Device: /dev/gpiochip* not found"
    fi

    # Check /sys/class/gpio
    if [ -d /sys/class/gpio ]; then
        if [ -w /sys/class/gpio/export ] 2>/dev/null; then
            log_success "GPIO Sysfs: Writable"
        else
            log_warn "GPIO Sysfs: Not writable (may need sudo or gpio group)"
        fi
    else
        log_warn "GPIO Sysfs: /sys/class/gpio not found"
    fi

    # Test actual GPIO access via Python
    local TEST_RESULT=$(python3 -c "
import sys
try:
    try:
        import Jetson.GPIO as GPIO
    except ImportError:
        import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.cleanup()
    print('OK')
except Exception as e:
    print(f'FAIL: {e}')
" 2>&1)

    if [ "$TEST_RESULT" = "OK" ]; then
        log_success "GPIO Access Test: Passed"
        return 0
    else
        log_fail "GPIO Access Test: $TEST_RESULT"
        return 1
    fi
}

check_python_deps() {
    log_check "Python Dependencies"

    local ALL_OK=true

    # Check Python 3
    if command -v python3 &>/dev/null; then
        PY_VERSION=$(python3 --version 2>&1)
        log_success "Python: $PY_VERSION"
    else
        log_fail "Python: python3 not found"
        ALL_OK=false
    fi

    # Check GPIO library (already checked separately)

    # Check other useful packages
    for pkg in "logging" "time" "argparse"; do
        if python3 -c "import $pkg" 2>/dev/null; then
            : # Built-in, skip
        fi
    done

    if $ALL_OK; then
        return 0
    else
        return 1
    fi
}

# =============================================================================
# Motor Process Management
# =============================================================================
find_motor_processes() {
    # Find processes related to motor control
    ps aux | grep -E "$MOTOR_PROCESS_PATTERNS" | grep -v grep | grep -v "deploy.sh" || true
}

stop_motor_processes() {
    local PROCESSES=$(find_motor_processes)

    if [ -n "$PROCESSES" ]; then
        log_info "Found motor-related processes:"
        echo "$PROCESSES" | sed 's/^/    /'

        # Get PIDs
        PIDS=$(echo "$PROCESSES" | awk '{print $2}')

        if [ -n "$PIDS" ]; then
            log_info "Sending SIGTERM to processes..."
            echo "$PIDS" | xargs kill -TERM 2>/dev/null || true
            sleep 1

            # Check if still running
            REMAINING=$(find_motor_processes)
            if [ -n "$REMAINING" ]; then
                log_warn "Some processes still running, sending SIGKILL..."
                echo "$REMAINING" | awk '{print $2}' | xargs kill -9 2>/dev/null || true
            fi
        fi

        log_success "Motor processes stopped"
    else
        log_info "No motor processes running"
    fi
}

ensure_motors_stopped() {
    # Safety: Try to stop motors via Python
    log_info "Ensuring motors are stopped (safety)..."

    python3 -c "
import sys
try:
    try:
        import Jetson.GPIO as GPIO
    except ImportError:
        import RPi.GPIO as GPIO

    # Known motor pins (from config)
    MOTOR_PINS = [11, 13, 15, 16, 18, 32, 33]

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # Set all motor pins to LOW
    for pin in MOTOR_PINS:
        try:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        except:
            pass

    GPIO.cleanup()
    print('Motors stopped')
except Exception as e:
    print(f'Could not access GPIO: {e}')
    sys.exit(0)  # Don't fail, just warn
" 2>&1 | while read line; do log_info "  $line"; done

    log_success "Motor safety check complete"
}

# =============================================================================
# Commands
# =============================================================================
cmd_setup() {
    echo ""
    echo "========================================"
    echo "  Locomotion - Setup Check"
    echo "========================================"
    echo ""

    log_to_file "SETUP command initiated"

    ERRORS=0

    log_step "Platform Detection"
    PLATFORM_INFO=$(get_platform_info)
    log_info "Platform: $PLATFORM_INFO"
    echo ""

    log_step "GPIO Library"
    check_gpio_library || ((ERRORS++))
    echo ""

    log_step "User Permissions"
    check_gpio_group || ((ERRORS++))
    echo ""

    log_step "GPIO Access"
    check_gpio_access || ((ERRORS++))
    echo ""

    log_step "Python Dependencies"
    check_python_deps || ((ERRORS++))
    echo ""

    # Summary
    echo "========================================"
    if [ $ERRORS -eq 0 ]; then
        log_success "Setup check passed - all requirements met"
        save_state "setup_checked"
    else
        log_warn "Setup check found $ERRORS issue(s)"
        log_info "Fix the issues above and run ./deploy.sh setup again"
    fi
    echo "========================================"

    log_to_file "SETUP command completed with $ERRORS errors"
    return $ERRORS
}

cmd_start() {
    log_step "Starting Motor Control..."
    log_to_file "START command initiated"

    echo ""
    log_info "Motors are controlled on-demand, not as a persistent service."
    log_info "There is no background motor server to start."
    echo ""
    log_info "To test motors:"
    log_info "  ./deploy.sh test"
    echo ""
    log_info "To use motors in code:"
    log_info "  from locomotion.yahboomg1 import create_robot"
    log_info "  robot = create_robot()"
    log_info "  robot.forward(50)"
    echo ""

    # If a motor server script exists in the future, start it here
    if [ -f "$YAHBOOM_DIR/motor_server.py" ]; then
        log_info "Starting motor server..."
        python3 "$YAHBOOM_DIR/motor_server.py" &
        save_state "motor_server_started"
        log_success "Motor server started"
    fi

    log_to_file "START command completed"
}

cmd_stop() {
    log_step "Stopping Motor Control..."
    log_to_file "STOP command initiated"

    echo ""

    # Stop any motor processes
    stop_motor_processes

    # Ensure motors are physically stopped (safety)
    ensure_motors_stopped

    echo ""
    log_success "Motor control stopped"
    log_to_file "STOP command completed"
}

cmd_status() {
    echo ""
    echo "========================================"
    echo "  Locomotion - System Status"
    echo "========================================"
    echo ""

    log_step "Platform"
    PLATFORM_INFO=$(get_platform_info)
    log_info "Platform: $PLATFORM_INFO"
    log_info "Architecture: $(uname -m)"
    log_info "Kernel: $(uname -r)"
    echo ""

    log_step "GPIO Status"
    check_gpio_library 2>/dev/null || true
    check_gpio_group 2>/dev/null || true

    # Quick GPIO access check
    if python3 -c "
try:
    try:
        import Jetson.GPIO as GPIO
    except ImportError:
        import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.cleanup()
except:
    exit(1)
" 2>/dev/null; then
        log_success "GPIO Access: OK"
    else
        log_fail "GPIO Access: Failed"
    fi
    echo ""

    log_step "User Permissions"
    log_info "User: $(whoami)"
    log_info "Groups: $(groups | tr ' ' ', ')"
    echo ""

    log_step "Motor Processes"
    PROCESSES=$(find_motor_processes)
    if [ -n "$PROCESSES" ]; then
        log_info "Running motor processes:"
        echo "$PROCESSES" | awk '{printf "    PID %s: %s\n", $2, $11}'
    else
        log_info "No motor processes running"
    fi
    echo ""

    log_step "Pin Configuration"
    log_info "Motor Driver: TB6612FNG (YAHBOOM G1)"
    log_info "Pin Mode: BOARD (physical pin numbers)"
    log_info "  Standby: Pin 11"
    log_info "  Left Motor:  IN1=13, IN2=15, PWM=33"
    log_info "  Right Motor: IN1=16, IN2=18, PWM=32"
    echo ""

    echo "========================================"
}

cmd_diagnose() {
    echo ""
    echo "========================================"
    echo "  Locomotion - Comprehensive Diagnostics"
    echo "========================================"
    echo "  Time: $(date)"
    echo "========================================"
    echo ""

    log_to_file "DIAGNOSE command initiated"

    # Platform detection
    log_step "Platform Detection"
    echo "  Machine: $(uname -m)"
    echo "  OS: $(lsb_release -d 2>/dev/null | cut -f2 || cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d'"' -f2)"
    echo "  Kernel: $(uname -r)"

    PLATFORM=$(detect_platform)
    PLATFORM_INFO=$(get_platform_info)
    echo "  Detected: $PLATFORM_INFO"

    if [ "$PLATFORM" = "jetson" ]; then
        if [ -f /etc/nv_tegra_release ]; then
            echo "  L4T Release: $(cat /etc/nv_tegra_release | head -1)"
        fi
        if command -v jetson_release &>/dev/null; then
            echo "  JetPack: $(jetson_release -v 2>/dev/null | head -1 || echo 'N/A')"
        fi
    elif [ "$PLATFORM" = "raspberry_pi" ]; then
        if [ -f /proc/device-tree/model ]; then
            echo "  Model: $(cat /proc/device-tree/model | tr '\0' ' ')"
        fi
    fi
    echo ""

    # GPIO library check
    log_step "GPIO Library Check"

    # Jetson.GPIO
    JETSON_GPIO=$(python3 -c "
try:
    import Jetson.GPIO as GPIO
    print(f'Installed (version: {GPIO.VERSION if hasattr(GPIO, \"VERSION\") else \"unknown\"})')
except ImportError:
    print('Not installed')
except Exception as e:
    print(f'Error: {e}')
" 2>&1)
    echo "  Jetson.GPIO: $JETSON_GPIO"

    # RPi.GPIO
    RPI_GPIO=$(python3 -c "
try:
    import RPi.GPIO as GPIO
    print(f'Installed (version: {GPIO.VERSION})')
except ImportError:
    print('Not installed')
except Exception as e:
    print(f'Error: {e}')
" 2>&1)
    echo "  RPi.GPIO: $RPI_GPIO"
    echo ""

    # User permissions
    log_step "User Permissions"
    echo "  User: $(whoami)"
    echo "  UID: $(id -u)"
    echo "  Groups: $(groups)"

    if groups | grep -qE '\bgpio\b'; then
        echo "  GPIO Group: Yes"
    else
        echo "  GPIO Group: NO - run: sudo usermod -a -G gpio \$USER"
    fi

    if groups | grep -qE '\bi2c\b'; then
        echo "  I2C Group: Yes"
    else
        echo "  I2C Group: No (optional)"
    fi
    echo ""

    # GPIO device access
    log_step "GPIO Device Access"

    # gpiochip devices
    if ls /dev/gpiochip* &>/dev/null 2>&1; then
        echo "  /dev/gpiochip devices:"
        ls -la /dev/gpiochip* 2>/dev/null | sed 's/^/    /'
    else
        echo "  /dev/gpiochip*: Not found"
    fi

    # sysfs GPIO
    if [ -d /sys/class/gpio ]; then
        echo "  /sys/class/gpio: Present"
        if [ -w /sys/class/gpio/export ]; then
            echo "    export: Writable"
        else
            echo "    export: Not writable"
        fi
    else
        echo "  /sys/class/gpio: Not found"
    fi
    echo ""

    # Pin availability test
    log_step "Pin Availability Test"
    python3 -c "
import sys

try:
    try:
        import Jetson.GPIO as GPIO
        lib = 'Jetson.GPIO'
    except ImportError:
        import RPi.GPIO as GPIO
        lib = 'RPi.GPIO'

    print(f'  Using: {lib}')

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # Test pins used by motor driver
    TEST_PINS = {
        11: 'STBY',
        13: 'Left IN1',
        15: 'Left IN2',
        33: 'Left PWM',
        16: 'Right IN1',
        18: 'Right IN2',
        32: 'Right PWM',
    }

    print('  Testing pins:')
    for pin, name in TEST_PINS.items():
        try:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            print(f'    Pin {pin:2d} ({name:10s}): OK')
        except Exception as e:
            print(f'    Pin {pin:2d} ({name:10s}): FAIL - {e}')

    GPIO.cleanup()

except Exception as e:
    print(f'  Error: {e}')
" 2>&1
    echo ""

    # Motor processes
    log_step "Motor Processes"
    PROCESSES=$(find_motor_processes)
    if [ -n "$PROCESSES" ]; then
        echo "  Running processes:"
        echo "$PROCESSES" | awk '{printf "    PID %s: %s %s\n", $2, $11, $12}'
    else
        echo "  No motor-related processes running"
    fi
    echo ""

    # Module import test
    log_step "Module Import Test"
    IMPORT_TEST=$(python3 -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR')

try:
    from yahboomg1 import Motor, DifferentialDrive, create_robot
    print('  yahboomg1 module: OK')
    print('    Motor class: OK')
    print('    DifferentialDrive class: OK')
    print('    create_robot function: OK')
except ImportError as e:
    print(f'  yahboomg1 module: FAIL - {e}')
except Exception as e:
    print(f'  yahboomg1 module: ERROR - {e}')
" 2>&1)
    echo "$IMPORT_TEST"
    echo ""

    # Summary
    echo "========================================"
    echo "  Diagnostics Complete"
    echo "========================================"

    log_to_file "DIAGNOSE command completed"
}

cmd_test() {
    echo ""
    echo "========================================"
    echo "  Locomotion - Motor Test"
    echo "========================================"
    echo ""

    log_to_file "TEST command initiated"

    # Check if we can run the test
    if ! check_gpio_library &>/dev/null; then
        log_error "GPIO library not found. Run ./deploy.sh setup first."
        exit 1
    fi

    if ! check_gpio_group &>/dev/null; then
        log_error "User not in gpio group. Run: sudo usermod -a -G gpio \$USER"
        exit 1
    fi

    # Parse options
    SPEED="${1:-50}"
    DURATION="${2:-1.5}"

    log_info "Test Parameters:"
    log_info "  Speed: $SPEED"
    log_info "  Duration: ${DURATION}s per action"
    echo ""

    log_warn "Motors will move! Ensure robot is on a safe surface."
    echo ""
    read -p "Continue with motor test? (y/N): " confirm

    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        log_info "Test cancelled"
        exit 0
    fi

    echo ""
    log_info "Starting motor test..."
    echo ""

    # Run the test script
    if [ -f "$YAHBOOM_DIR/test_motors.py" ]; then
        cd "$YAHBOOM_DIR"
        python3 test_motors.py --basic --speed "$SPEED" --duration "$DURATION"
        TEST_RESULT=$?
        cd "$SCRIPT_DIR"

        echo ""
        if [ $TEST_RESULT -eq 0 ]; then
            log_success "Motor test completed successfully"
        else
            log_fail "Motor test failed (exit code: $TEST_RESULT)"
        fi
    else
        log_error "Test script not found: $YAHBOOM_DIR/test_motors.py"
        exit 1
    fi

    log_to_file "TEST command completed"
}

cmd_help() {
    echo ""
    echo "Locomotion - Motor Control Deployment Script"
    echo ""
    echo "Usage: ./deploy.sh <command> [options]"
    echo ""
    echo "Commands:"
    echo "  setup              Check GPIO permissions and Python dependencies"
    echo "  start              Start motor control (informational - motors are on-demand)"
    echo "  stop               Stop motor processes and ensure motors are stopped (safety)"
    echo "  status             Show GPIO availability, motor driver status, permissions"
    echo "  diagnose           Run comprehensive diagnostics"
    echo "  test [speed] [dur] Run basic motor test (default: speed=50, duration=1.5s)"
    echo "  help               Show this help"
    echo ""
    echo "Examples:"
    echo "  ./deploy.sh setup              # Check if system is ready"
    echo "  ./deploy.sh status             # Quick status overview"
    echo "  ./deploy.sh diagnose           # Full diagnostic report"
    echo "  ./deploy.sh test               # Run motor test (speed 50)"
    echo "  ./deploy.sh test 30 2.0        # Run motor test (speed 30, 2s duration)"
    echo "  ./deploy.sh stop               # Stop motors (safety)"
    echo ""
    echo "Motor Driver: TB6612FNG (YAHBOOM G1 Smart Robot Kit)"
    echo "Platforms: Jetson Orin Nano, Raspberry Pi"
    echo ""
    echo "Pin Configuration (BOARD numbering):"
    echo "  Standby:     Pin 11"
    echo "  Left Motor:  IN1=13, IN2=15, PWM=33"
    echo "  Right Motor: IN1=16, IN2=18, PWM=32"
    echo ""
    echo "All commands are idempotent - safe to run multiple times."
    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    COMMAND="${1:-help}"
    shift || true

    case "$COMMAND" in
        setup)
            cmd_setup
            ;;
        start)
            cmd_start
            ;;
        stop)
            cmd_stop
            ;;
        status)
            cmd_status
            ;;
        diagnose|diag)
            cmd_diagnose
            ;;
        test)
            cmd_test "$@"
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            log_error "Unknown command: $COMMAND"
            cmd_help
            exit 1
            ;;
    esac
}

main "$@"
