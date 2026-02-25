"""Shared fixtures for AMBOT web_control tests."""

import sys
from pathlib import Path

import pytest

# Ensure ambot/ is on sys.path so `web_control.*` imports resolve.
_ambot_dir = str(Path(__file__).resolve().parent.parent.parent)
if _ambot_dir not in sys.path:
    sys.path.insert(0, _ambot_dir)


@pytest.fixture(scope="session")
def app():
    """Create a Flask application in simulation mode for the entire test session."""
    from web_control.app import create_app

    app = create_app(simulate=True)
    app.config["TESTING"] = True
    yield app

    # Cleanup hardware manager on teardown
    if hasattr(app, "hardware"):
        app.hardware.cleanup()


@pytest.fixture()
def client(app):
    """Flask test client — one fresh client per test function."""
    with app.test_client() as client:
        yield client


@pytest.fixture()
def app_context(app):
    """Push an application context for tests that need it."""
    with app.app_context():
        yield app
