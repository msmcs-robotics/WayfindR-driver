"""Motor control REST API."""

from flask import Blueprint, current_app, jsonify, request

motors_bp = Blueprint('motors', __name__)


@motors_bp.route('/drive', methods=['POST'])
def drive():
    """Set motor speeds. Expects JSON: {left: -100..100, right: -100..100}."""
    hw = current_app.hardware
    data = request.get_json(force=True)
    left = data.get('left', 0)
    right = data.get('right', 0)
    hw.motor_drive(left, right)
    return jsonify({'ok': True, 'left': left, 'right': right})


@motors_bp.route('/stop', methods=['POST'])
def stop():
    """Stop all motors."""
    hw = current_app.hardware
    hw.motor_drive(0, 0)
    return jsonify({'ok': True})


@motors_bp.route('/emergency', methods=['POST'])
def emergency():
    """Emergency stop — bypasses normal drive path."""
    hw = current_app.hardware
    hw.emergency_stop()
    return jsonify({'ok': True, 'action': 'emergency_stop'})


@motors_bp.route('/status')
def status():
    """Get current motor state."""
    hw = current_app.hardware
    return jsonify(hw.get_motor_status())
