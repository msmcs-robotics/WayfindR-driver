#!/usr/bin/env python3
"""AMBOT Web Control — Entry point.

Usage:
    python3 web_control/run.py                  # Real hardware
    python3 web_control/run.py --simulate       # Simulation mode
    python3 web_control/run.py --port 8080      # Custom port
"""

import argparse
import sys
from pathlib import Path

# Add ambot/ to sys.path so pathfinder/locomotion imports work
_ambot_dir = str(Path(__file__).resolve().parent.parent)
if _ambot_dir not in sys.path:
    sys.path.insert(0, _ambot_dir)

from web_control.app import create_app, socketio


def main():
    parser = argparse.ArgumentParser(description='AMBOT Web Control Interface')
    parser.add_argument('--simulate', '-s', action='store_true',
                        help='Run in simulation mode (no hardware)')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Host to bind to (default: 0.0.0.0)')
    parser.add_argument('--port', '-p', type=int, default=5000,
                        help='Port to listen on (default: 5000)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable Flask debug mode')
    args = parser.parse_args()

    app = create_app(simulate=args.simulate)
    print(f'\n  AMBOT Web Control')
    print(f'  http://{args.host}:{args.port}/')
    mode = 'SIMULATION' if args.simulate else 'HARDWARE'
    print(f'  Mode: {mode}\n')

    socketio.run(app, host=args.host, port=args.port,
                 debug=args.debug, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
