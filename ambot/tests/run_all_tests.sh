#!/bin/bash
#
# Run all hardware tests on Raspberry Pi
# Results are saved to tests/results/
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================"
echo "AMBOT HARDWARE TEST SUITE"
echo "========================================"
echo "Running on: $(hostname)"
echo "Date: $(date)"
echo ""

# Create results directory
mkdir -p results

# Run GPIO tests
echo ">>> Running GPIO tests..."
python3 test_gpio.py || true
echo ""

# Run camera tests
echo ">>> Running USB camera tests..."
python3 test_usb_camera.py || true
echo ""

# Run LiDAR tests
echo ">>> Running USB LiDAR tests..."
python3 test_usb_lidar.py || true
echo ""

# Summary
echo "========================================"
echo "TEST COMPLETE"
echo "========================================"
echo "Results saved to: $SCRIPT_DIR/results/"
echo ""
echo "View results:"
echo "  cat results/gpio_test_results.txt"
echo "  cat results/camera_test_results.txt"
echo "  cat results/lidar_test_results.txt"
