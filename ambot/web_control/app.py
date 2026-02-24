"""AMBOT Web Control — Flask application factory."""

import atexit
import logging

from flask import Flask
from flask_socketio import SocketIO

logger = logging.getLogger(__name__)
socketio = SocketIO()


def create_app(simulate=False):
    """Create and configure the Flask application."""
    app = Flask(__name__)
    app.config.from_object('web_control.config')
    app.config['SIMULATE'] = simulate

    logging.basicConfig(
        level=logging.DEBUG if app.config.get('DEBUG') else logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )

    # Initialize hardware
    from web_control.hardware import HardwareManager
    hw = HardwareManager(simulate=simulate)
    hw.initialize()
    app.hardware = hw

    # Cleanup on exit
    atexit.register(hw.cleanup)

    # Register blueprints
    from web_control.routes.pages import pages_bp
    from web_control.routes.api_motors import motors_bp
    from web_control.routes.api_chat import chat_bp
    from web_control.routes.api_diagnostics import diagnostics_bp
    app.register_blueprint(pages_bp)
    app.register_blueprint(motors_bp, url_prefix='/api/motors')
    app.register_blueprint(chat_bp, url_prefix='/api/chat')
    app.register_blueprint(diagnostics_bp, url_prefix='/api/diagnostics')

    # Init SocketIO (must happen before registering background tasks)
    socketio.init_app(app, cors_allowed_origins='*', async_mode='threading')

    # Register SocketIO events + background streams
    from web_control.realtime import register_events
    register_events(socketio, app)

    mode = 'SIMULATION' if simulate else 'HARDWARE'
    logger.info('AMBOT Web Control ready (%s mode)', mode)
    return app
