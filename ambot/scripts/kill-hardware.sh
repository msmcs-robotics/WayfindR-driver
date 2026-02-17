#!/bin/bash
#
# kill-hardware.sh — Emergency stop for all AMBOT hardware processes
#
# Kills any running Python processes that interface with motors, camera, or LiDAR.
# Also resets GPIO pins to stop motors that may still be spinning.
#
# Usage:
#   ./scripts/kill-hardware.sh          # Kill processes + stop motors
#   ./scripts/kill-hardware.sh --motors # Stop motors only (GPIO reset)
#   ./scripts/kill-hardware.sh --procs  # Kill processes only
#

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "============================================"
echo "AMBOT Emergency Hardware Stop"
echo "============================================"

# --- Kill hardware-related Python processes ---
kill_processes() {
    echo ""
    echo "[KILL] Searching for hardware-related Python processes..."

    # Patterns that indicate hardware interaction
    local patterns=(
        "gui_face_tracker"
        "gui_lidar_nav"
        "gui_lidar"
        "gui_camera"
        "gui_wandering"
        "wandering_demo"
        "live_monitor"
        "test_motors"
        "test_imu"
        "test_gpio"
        "test_usb_camera"
        "test_ld19_lidar"
    )

    local killed=0
    for pattern in "${patterns[@]}"; do
        local pids
        pids=$(pgrep -f "$pattern" 2>/dev/null)
        if [ -n "$pids" ]; then
            echo "  Killing: $pattern (PIDs: $pids)"
            kill $pids 2>/dev/null
            killed=$((killed + 1))
        fi
    done

    if [ "$killed" -eq 0 ]; then
        echo "  No hardware processes found"
    else
        echo "  Killed $killed process group(s)"
        # Wait briefly then force-kill any survivors
        sleep 1
        for pattern in "${patterns[@]}"; do
            local pids
            pids=$(pgrep -f "$pattern" 2>/dev/null)
            if [ -n "$pids" ]; then
                echo "  Force killing: $pattern"
                kill -9 $pids 2>/dev/null
            fi
        done
    fi
}

# --- Stop motors via GPIO reset ---
stop_motors() {
    echo ""
    echo "[STOP] Stopping motors via GPIO cleanup..."

    # Use Python to properly stop motors and cleanup GPIO
    python3 -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR')
try:
    from locomotion.rpi_motors.factory import create_robot, cleanup_gpio
    from locomotion.rpi_motors.drivers import DriverType

    # Try each driver type to cover whatever is wired
    for dt in [DriverType.L298N, DriverType.TB6612FNG]:
        try:
            robot = create_robot(driver_type=dt)
            robot.stop()
            robot.cleanup()
            print(f'  {dt.value}: stopped and cleaned up')
            break
        except Exception as e:
            pass

    # Final GPIO cleanup
    cleanup_gpio()
    print('  GPIO cleanup complete')
except ImportError:
    print('  WARNING: locomotion module not available')
except Exception as e:
    print(f'  WARNING: GPIO cleanup failed: {e}')

# Fallback: try raw GPIO cleanup
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    # Set all motor pins to LOW
    for pin in [11, 13, 15, 16, 18, 32, 33]:
        try:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        except Exception:
            pass
    GPIO.cleanup()
    print('  Raw GPIO pins reset')
except Exception:
    pass
" 2>&1
}

# --- Main ---
case "${1:-all}" in
    --motors)
        stop_motors
        ;;
    --procs)
        kill_processes
        ;;
    *)
        kill_processes
        stop_motors
        ;;
esac

echo ""
echo "Done."
