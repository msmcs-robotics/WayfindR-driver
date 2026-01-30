"""
Pathfinder LiDAR Configuration

Configuration settings for the RPLidar C1M1 and obstacle detection.
"""

import os

# =============================================================================
# Serial Port Settings
# =============================================================================

# Primary device path (use udev symlink if available)
SERIAL_PORT = os.environ.get("LIDAR_PORT", "/dev/rplidar")

# Fallback device path if primary not found
SERIAL_PORT_FALLBACK = "/dev/ttyUSB0"

# Baud rate for RPLidar C1M1 (460800 for C1, A2, A3; 115200 for A1)
BAUD_RATE = 460800

# Serial timeout in seconds
SERIAL_TIMEOUT = 3.0

# =============================================================================
# LiDAR Settings
# =============================================================================

# Scan mode: Standard, DenseBoost, Sensitivity, Express
# DenseBoost recommended for indoor obstacle avoidance
SCAN_MODE = "DenseBoost"

# TF frame ID for ROS compatibility
FRAME_ID = "laser"

# Target scan frequency (Hz)
SCAN_FREQUENCY = 10.0

# Maximum valid distance in millimeters (12m for C1M1 standard mode)
MAX_DISTANCE_MM = 12000

# Minimum valid distance in millimeters
MIN_DISTANCE_MM = 100

# =============================================================================
# Safety Zones (in meters)
# =============================================================================

# Immediate stop distance - obstacle too close
STOP_DISTANCE = 0.3

# Slow down distance - reduce speed
SLOW_DISTANCE = 0.8

# Warning distance - be aware
WARN_DISTANCE = 1.5

# =============================================================================
# Sector Configuration for Obstacle Detection
# =============================================================================

# Divide 360 degrees into sectors
# Each sector defined as (start_angle, end_angle) in degrees
# 0 degrees = front, clockwise positive
SECTORS = {
    "front": (-30, 30),
    "front_left": (30, 60),
    "left": (60, 120),
    "back_left": (120, 150),
    "back": (150, 210),
    "back_right": (210, 240),
    "right": (240, 300),
    "front_right": (300, 330),
}

# Simplified 4-sector mode
SECTORS_SIMPLE = {
    "front": (-45, 45),
    "left": (45, 135),
    "back": (135, 225),
    "right": (225, 315),
}

# Default to simple sectors
USE_SIMPLE_SECTORS = True

# =============================================================================
# Visualization Settings
# =============================================================================

# Maximum points to keep for visualization
MAX_PLOT_POINTS = 2000

# Point lifetime in visualization (seconds)
POINT_LIFETIME = 1.5

# Plot update rate (FPS)
PLOT_FPS = 30

# =============================================================================
# USB Device Identification
# =============================================================================

# Silicon Labs CP210x USB to UART bridge (used by RPLidar)
USB_VENDOR_ID = "10c4"
USB_PRODUCT_ID = "ea60"

# =============================================================================
# Logging
# =============================================================================

# Enable debug logging
DEBUG = os.environ.get("LIDAR_DEBUG", "").lower() in ("1", "true", "yes")

# Log file path (None for stdout only)
LOG_FILE = None
