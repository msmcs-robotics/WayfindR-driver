"""Page routes — serves the dashboard and MJPEG video feed."""

import time

from flask import Blueprint, Response, current_app, render_template

pages_bp = Blueprint('pages', __name__)


@pages_bp.route('/')
def index():
    """Dashboard page."""
    simulate = current_app.config.get('SIMULATE', False)
    return render_template('index.html', simulate=simulate)


@pages_bp.route('/video_feed')
def video_feed():
    """MJPEG stream from the camera."""
    hw = current_app.hardware

    def generate():
        while True:
            jpeg = hw.get_camera_jpeg()
            if jpeg:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n'
                       + jpeg + b'\r\n')
            else:
                time.sleep(0.1)

    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@pages_bp.route('/api/health')
def health():
    """Quick health check."""
    return {'status': 'ok'}
