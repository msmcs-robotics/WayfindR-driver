"""Diagnostics REST API — system stats and sensor health."""

from flask import Blueprint, current_app, jsonify

diagnostics_bp = Blueprint('diagnostics', __name__)


@diagnostics_bp.route('/system')
def system_info():
    """Return full system telemetry."""
    hw = current_app.hardware
    return jsonify(hw.get_telemetry())


@diagnostics_bp.route('/sensors')
def sensor_status():
    """Return sensor connection status."""
    hw = current_app.hardware
    telemetry = hw.get_telemetry()
    return jsonify(telemetry.get('sensors', {}))


@diagnostics_bp.route('/lidar_scan')
def lidar_scan():
    """Return latest LiDAR scan data."""
    hw = current_app.hardware
    scan = hw.get_scan()
    return jsonify({'scan': scan, 'count': len(scan)})


@diagnostics_bp.route('/faces')
def faces():
    """Return latest face detection results."""
    hw = current_app.hardware
    return jsonify(hw.get_face_data())
