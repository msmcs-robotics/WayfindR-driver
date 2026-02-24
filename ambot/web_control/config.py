"""Configuration for AMBOT Web Control Interface."""

import os

# Flask
SECRET_KEY = os.environ.get('SECRET_KEY', 'ambot-web-control-dev')
DEBUG = os.environ.get('FLASK_DEBUG', '0') == '1'

# Server
HOST = os.environ.get('WEB_HOST', '0.0.0.0')
PORT = int(os.environ.get('WEB_PORT', '5000'))

# Hardware
CAMERA_DEVICE = int(os.environ.get('CAMERA_DEVICE', '0'))
LIDAR_PORT = os.environ.get('LIDAR_PORT', '/dev/ttyUSB0')
LIDAR_BAUD = int(os.environ.get('LIDAR_BAUD', '230400'))
DRIVER_TYPE = os.environ.get('DRIVER_TYPE', 'L298N')
MAX_MOTOR_SPEED = int(os.environ.get('MAX_MOTOR_SPEED', '50'))

# LLM / RAG
RAG_API_URL = os.environ.get('RAG_API_URL', 'http://10.33.255.82:8000')

# Timing
MOTOR_WATCHDOG_TIMEOUT = 1.0   # seconds — stop motors if no command
TELEMETRY_HZ = 5               # system telemetry broadcast rate
LIDAR_HZ = 4                   # LiDAR scan broadcast rate
CAMERA_JPEG_QUALITY = 60       # JPEG quality for MJPEG stream (0-100)
