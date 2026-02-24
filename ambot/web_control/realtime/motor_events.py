"""SocketIO events for real-time motor control."""

import logging

from flask import request
from flask_socketio import emit

logger = logging.getLogger(__name__)


def register_motor_events(socketio, app):
    """Register motor-related SocketIO event handlers."""

    @socketio.on('connect')
    def on_connect():
        logger.info('Client connected: %s', request.sid)
        emit('status', {'connected': True})

    @socketio.on('disconnect')
    def on_disconnect():
        logger.info('Client disconnected: %s', request.sid)
        # Stop motors when client disconnects
        with app.app_context():
            app.hardware.motor_drive(0, 0)

    @socketio.on('motor_command')
    def on_motor_command(data):
        """Handle real-time motor commands.

        data: {left: -100..100, right: -100..100}
        or:   {throttle: -1..1, steering: -1..1}
        """
        with app.app_context():
            hw = app.hardware
            max_speed = app.config.get('MAX_MOTOR_SPEED', 50)

            if 'left' in data and 'right' in data:
                left = data['left']
                right = data['right']
            elif 'throttle' in data:
                # Convert throttle/steering to differential drive
                throttle = float(data.get('throttle', 0))
                steering = float(data.get('steering', 0))
                left = (throttle + steering) * max_speed
                right = (throttle - steering) * max_speed
            else:
                left = 0
                right = 0

            hw.motor_drive(left, right)
            emit('motor_state', hw.get_motor_status())

    @socketio.on('emergency_stop')
    def on_emergency_stop():
        """Immediate motor stop via SocketIO."""
        with app.app_context():
            app.hardware.emergency_stop()
            emit('motor_state', app.hardware.get_motor_status(), broadcast=True)

    @socketio.on('stop')
    def on_stop():
        """Normal stop — zero motors."""
        with app.app_context():
            app.hardware.motor_drive(0, 0)
            emit('motor_state', app.hardware.get_motor_status())
