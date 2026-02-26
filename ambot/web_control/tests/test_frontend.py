"""Frontend tests for the AMBOT Web Control dashboard.

Tests the Flask routes and HTML output using Flask's test_client
and BeautifulSoup. All tests run in simulation mode (no hardware).
"""

from unittest.mock import patch

import pytest
from bs4 import BeautifulSoup


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _soup(response):
    """Parse an HTML response into a BeautifulSoup tree."""
    return BeautifulSoup(response.data, "html.parser")


# ===========================================================================
# 1. Dashboard HTML — page loads and key elements are present
# ===========================================================================

class TestDashboardPage:
    """Verify the main dashboard page renders correctly."""

    def test_index_returns_200(self, client):
        """GET / should return HTTP 200."""
        resp = client.get("/")
        assert resp.status_code == 200

    def test_index_content_type_is_html(self, client):
        """Response should be text/html."""
        resp = client.get("/")
        assert "text/html" in resp.content_type

    # -- Title ---------------------------------------------------------------

    def test_page_has_title(self, client):
        """The <title> tag should exist and contain 'AMBOT'."""
        soup = _soup(client.get("/"))
        title = soup.find("title")
        assert title is not None
        assert "AMBOT" in title.string

    def test_page_title_is_dashboard(self, client):
        """Title should be 'AMBOT Dashboard'."""
        soup = _soup(client.get("/"))
        assert soup.title.string.strip() == "AMBOT Dashboard"

    # -- Header / Navigation -------------------------------------------------

    def test_header_exists(self, client):
        """A <header> element should be present."""
        soup = _soup(client.get("/"))
        assert soup.find("header") is not None

    def test_header_has_site_title(self, client):
        """Header should contain 'AMBOT Web Control'."""
        soup = _soup(client.get("/"))
        header = soup.find("header")
        assert "AMBOT Web Control" in header.get_text()

    def test_header_has_connection_status(self, client):
        """Header status bar should have a connection-status element."""
        soup = _soup(client.get("/"))
        status = soup.find(id="connection-status")
        assert status is not None

    def test_simulation_mode_indicator_exists(self, client):
        """The sim-mode span should exist (shown via JS in simulation mode)."""
        soup = _soup(client.get("/"))
        sim = soup.find(id="sim-mode")
        assert sim is not None

    # -- All 6 Dashboard Panels ----------------------------------------------

    def test_motor_control_panel_present(self, client):
        """Motor Control panel with class 'control-panel' should exist."""
        soup = _soup(client.get("/"))
        panel = soup.find("section", class_="control-panel")
        assert panel is not None
        h2 = panel.find("h2")
        assert h2 is not None
        assert "Motor Control" in h2.get_text()

    def test_camera_panel_present(self, client):
        """Camera Feed panel should exist."""
        soup = _soup(client.get("/"))
        panel = soup.find("section", class_="camera-panel")
        assert panel is not None
        h2 = panel.find("h2")
        assert h2 is not None
        assert "Camera" in h2.get_text()

    def test_lidar_panel_present(self, client):
        """LiDAR View panel should exist."""
        soup = _soup(client.get("/"))
        panel = soup.find("section", class_="lidar-panel")
        assert panel is not None
        h2 = panel.find("h2")
        assert h2 is not None
        assert "LiDAR" in h2.get_text()

    def test_chat_panel_present(self, client):
        """LLM Chat panel should exist."""
        soup = _soup(client.get("/"))
        panel = soup.find("section", class_="chat-panel")
        assert panel is not None
        h2 = panel.find("h2")
        assert h2 is not None
        assert "Chat" in h2.get_text()

    def test_diagnostics_panel_present(self, client):
        """Diagnostics panel should exist."""
        soup = _soup(client.get("/"))
        panel = soup.find("section", class_="diagnostics-panel")
        assert panel is not None
        h2 = panel.find("h2")
        assert h2 is not None
        assert "Diagnostics" in h2.get_text()

    def test_telemetry_panel_present(self, client):
        """Telemetry panel should exist."""
        soup = _soup(client.get("/"))
        panel = soup.find("section", class_="telemetry-panel")
        assert panel is not None
        h2 = panel.find("h2")
        assert h2 is not None
        assert "Telemetry" in h2.get_text()

    def test_all_six_panels_present(self, client):
        """All 6 panels should be present in the dashboard grid."""
        soup = _soup(client.get("/"))
        panels = soup.find_all("section", class_="panel")
        panel_classes = set()
        for p in panels:
            for cls in p.get("class", []):
                if cls != "panel":
                    panel_classes.add(cls)
        expected = {
            "control-panel",
            "camera-panel",
            "lidar-panel",
            "chat-panel",
            "diagnostics-panel",
            "telemetry-panel",
        }
        assert expected.issubset(panel_classes), (
            f"Missing panels: {expected - panel_classes}"
        )

    # -- Emergency Stop Button -----------------------------------------------

    def test_emergency_stop_button_exists(self, client):
        """The emergency stop button should be in the footer."""
        soup = _soup(client.get("/"))
        btn = soup.find(id="emergency-stop")
        assert btn is not None
        assert btn.name == "button"

    def test_emergency_stop_button_text(self, client):
        """Emergency stop button should say 'EMERGENCY STOP'."""
        soup = _soup(client.get("/"))
        btn = soup.find(id="emergency-stop")
        assert "EMERGENCY STOP" in btn.get_text()

    def test_emergency_stop_has_css_class(self, client):
        """Emergency stop button should have the 'emergency-btn' class."""
        soup = _soup(client.get("/"))
        btn = soup.find(id="emergency-stop")
        assert "emergency-btn" in btn.get("class", [])

    # -- Footer --------------------------------------------------------------

    def test_footer_exists(self, client):
        """A <footer> element should be present."""
        soup = _soup(client.get("/"))
        assert soup.find("footer") is not None

    def test_footer_contains_project_name(self, client):
        """Footer should reference WayfindR Project."""
        soup = _soup(client.get("/"))
        footer = soup.find("footer")
        assert "WayfindR" in footer.get_text()

    # -- Motor Control Sub-elements ------------------------------------------

    def test_direction_pad_buttons_exist(self, client):
        """The direction pad should have at least 9 direction buttons."""
        soup = _soup(client.get("/"))
        pad = soup.find("div", class_="direction-pad")
        assert pad is not None
        buttons = pad.find_all("button", class_="dir-btn")
        assert len(buttons) >= 9

    def test_speed_slider_exists(self, client):
        """A speed slider input should be present."""
        soup = _soup(client.get("/"))
        slider = soup.find(id="speed-slider")
        assert slider is not None
        assert slider.get("type") == "range"

    def test_stop_button_in_direction_pad(self, client):
        """The STOP button should be inside the direction pad."""
        soup = _soup(client.get("/"))
        btn = soup.find(id="btn-stop")
        assert btn is not None
        assert "STOP" in btn.get_text()

    # -- Chat Sub-elements ---------------------------------------------------

    def test_chat_input_exists(self, client):
        """Chat panel should have a text input for questions."""
        soup = _soup(client.get("/"))
        chat_input = soup.find(id="chat-input")
        assert chat_input is not None
        assert chat_input.get("type") == "text"

    def test_chat_send_button_exists(self, client):
        """Chat panel should have a Send button."""
        soup = _soup(client.get("/"))
        btn = soup.find(id="chat-send")
        assert btn is not None
        assert "Send" in btn.get_text()

    # -- LiDAR Sub-elements --------------------------------------------------

    def test_lidar_canvas_exists(self, client):
        """LiDAR panel should contain a <canvas> element."""
        soup = _soup(client.get("/"))
        canvas = soup.find(id="lidar-canvas")
        assert canvas is not None
        assert canvas.name == "canvas"

    # -- Camera Sub-elements -------------------------------------------------

    def test_camera_feed_img_exists(self, client):
        """Camera panel should have an <img> tag for the video feed."""
        soup = _soup(client.get("/"))
        img = soup.find(id="camera-feed")
        assert img is not None
        assert img.get("src") == "/video_feed"

    # -- Diagnostics Sub-elements --------------------------------------------

    def test_sensor_indicators_exist(self, client):
        """Diagnostics should show LiDAR, Camera, and IMU sensor indicators."""
        soup = _soup(client.get("/"))
        for sensor_id in ("sensor-lidar", "sensor-camera", "sensor-imu"):
            dot = soup.find(id=sensor_id)
            assert dot is not None, f"Missing sensor indicator: {sensor_id}"


# ===========================================================================
# 2. Health Check Endpoint
# ===========================================================================

class TestHealthEndpoint:
    """Verify the /api/health quick-check route."""

    def test_health_returns_200(self, client):
        resp = client.get("/api/health")
        assert resp.status_code == 200

    def test_health_returns_json(self, client):
        resp = client.get("/api/health")
        data = resp.get_json()
        assert data["status"] == "ok"


# ===========================================================================
# 3. Motor API Endpoints
# ===========================================================================

class TestMotorAPI:
    """Verify motor control REST endpoints (simulation mode)."""

    def test_motor_stop_returns_200(self, client):
        """POST /api/motors/stop should return 200."""
        resp = client.post("/api/motors/stop")
        assert resp.status_code == 200

    def test_motor_stop_returns_ok(self, client):
        """POST /api/motors/stop should return {'ok': true}."""
        resp = client.post("/api/motors/stop")
        data = resp.get_json()
        assert data["ok"] is True

    def test_motor_drive_accepts_json(self, client):
        """POST /api/motors/drive should accept left/right speeds."""
        resp = client.post(
            "/api/motors/drive",
            json={"left": 50, "right": 50},
        )
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["ok"] is True
        assert data["left"] == 50
        assert data["right"] == 50

    def test_motor_drive_clamps_values(self, client):
        """Motor speeds should be clamped to -100..100 range."""
        resp = client.post(
            "/api/motors/drive",
            json={"left": 200, "right": -200},
        )
        assert resp.status_code == 200
        # The clamping is in HardwareManager.motor_drive, verify via status
        status_resp = client.get("/api/motors/status")
        data = status_resp.get_json()
        assert data["left"] <= 100
        assert data["right"] >= -100

    def test_motor_emergency_stop(self, client):
        """POST /api/motors/emergency should return success with action."""
        resp = client.post("/api/motors/emergency")
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["ok"] is True
        assert data["action"] == "emergency_stop"

    def test_motor_status_returns_json(self, client):
        """GET /api/motors/status should return motor state."""
        resp = client.get("/api/motors/status")
        assert resp.status_code == 200
        data = resp.get_json()
        assert "left" in data
        assert "right" in data
        assert "cmd_count" in data


# ===========================================================================
# 4. Chat API Endpoints
# ===========================================================================

class TestChatAPI:
    """Verify LLM chat REST endpoints."""

    def test_chat_status_returns_200(self, client):
        """GET /api/chat/status should always return 200."""
        resp = client.get("/api/chat/status")
        assert resp.status_code == 200

    def test_chat_status_has_available_field(self, client):
        """Chat status should include 'available' boolean."""
        resp = client.get("/api/chat/status")
        data = resp.get_json()
        assert "available" in data
        assert isinstance(data["available"], bool)

    def test_chat_status_has_url_field(self, client):
        """Chat status should include 'url' string."""
        resp = client.get("/api/chat/status")
        data = resp.get_json()
        assert "url" in data
        assert isinstance(data["url"], str)

    def test_chat_ask_empty_question_returns_400(self, client):
        """POST /api/chat/ask with empty question should return 400."""
        resp = client.post(
            "/api/chat/ask",
            json={"question": ""},
        )
        assert resp.status_code == 400
        data = resp.get_json()
        assert "error" in data

    def test_chat_ask_missing_question_returns_400(self, client):
        """POST /api/chat/ask with no question key should return 400."""
        resp = client.post(
            "/api/chat/ask",
            json={},
        )
        assert resp.status_code == 400

    def test_chat_ask_whitespace_only_returns_400(self, client):
        """POST /api/chat/ask with whitespace-only question should return 400."""
        resp = client.post(
            "/api/chat/ask",
            json={"question": "   "},
        )
        assert resp.status_code == 400

    def test_chat_ask_rag_unavailable_returns_503(self, client):
        """When RAG API is unreachable, /api/chat/ask should return 503."""
        import requests as real_requests

        with patch("web_control.routes.api_chat.requests.post") as mock_post:
            mock_post.side_effect = real_requests.ConnectionError(
                "Mocked: RAG API unreachable"
            )
            resp = client.post(
                "/api/chat/ask",
                json={"question": "What sensors does AMBOT have?"},
            )
            assert resp.status_code == 503
            data = resp.get_json()
            assert "error" in data
            assert "unavailable" in data["error"].lower()

    def test_chat_ask_success_with_mock_rag(self, client):
        """When RAG API responds, /api/chat/ask should return answer/sources/model."""
        mock_rag_response = {
            "answer": "AMBOT has LiDAR, camera, and IMU sensors.",
            "sources": ["docs/scope.md"],
            "model": "llama3.2:3b",
        }

        with patch("web_control.routes.api_chat.requests.post") as mock_post:
            mock_post.return_value.status_code = 200
            mock_post.return_value.raise_for_status = lambda: None
            mock_post.return_value.json.return_value = mock_rag_response

            resp = client.post(
                "/api/chat/ask",
                json={"question": "What sensors does AMBOT have?"},
            )
            assert resp.status_code == 200
            data = resp.get_json()
            assert "answer" in data
            assert "sources" in data
            assert "model" in data
            assert data["answer"] == mock_rag_response["answer"]
            assert data["model"] == "llama3.2:3b"


# ===========================================================================
# 5. Diagnostics API Endpoints
# ===========================================================================

class TestDiagnosticsAPI:
    """Verify diagnostics REST endpoints (simulation mode)."""

    def test_system_info_returns_200(self, client):
        """GET /api/diagnostics/system should return 200."""
        resp = client.get("/api/diagnostics/system")
        assert resp.status_code == 200

    def test_system_info_has_expected_fields(self, client):
        """System telemetry should include uptime, cpu_load, mem_pct, etc."""
        resp = client.get("/api/diagnostics/system")
        data = resp.get_json()
        assert "uptime" in data
        assert "cpu_load" in data
        assert "mem_pct" in data
        assert "motor" in data
        assert "lidar" in data
        assert "camera" in data
        assert "sensors" in data

    def test_system_info_uptime_is_positive(self, client):
        """Uptime should be a positive number."""
        resp = client.get("/api/diagnostics/system")
        data = resp.get_json()
        assert data["uptime"] > 0

    def test_sensor_status_returns_200(self, client):
        """GET /api/diagnostics/sensors should return 200."""
        resp = client.get("/api/diagnostics/sensors")
        assert resp.status_code == 200

    def test_sensor_status_has_all_sensors(self, client):
        """Sensor status should report lidar, camera, and imu."""
        resp = client.get("/api/diagnostics/sensors")
        data = resp.get_json()
        assert "lidar" in data
        assert "camera" in data
        assert "imu" in data

    def test_lidar_simulation_shows_not_connected(self, client):
        """In simulation mode, LiDAR should report as not connected."""
        resp = client.get("/api/diagnostics/sensors")
        data = resp.get_json()
        assert data["lidar"] is False

    def test_lidar_scan_returns_200(self, client):
        """GET /api/diagnostics/lidar_scan should return 200."""
        resp = client.get("/api/diagnostics/lidar_scan")
        assert resp.status_code == 200

    def test_lidar_scan_has_scan_data(self, client):
        """LiDAR scan response should include 'scan' list and 'count'."""
        resp = client.get("/api/diagnostics/lidar_scan")
        data = resp.get_json()
        assert "scan" in data
        assert "count" in data
        assert isinstance(data["scan"], list)
        assert isinstance(data["count"], int)

    def test_lidar_scan_empty_in_simulation(self, client):
        """In simulation mode, LiDAR scan should return empty (no mock data)."""
        resp = client.get("/api/diagnostics/lidar_scan")
        data = resp.get_json()
        assert data["count"] == 0
        assert data["scan"] == []

    def test_faces_returns_200(self, client):
        """GET /api/diagnostics/faces should return 200."""
        resp = client.get("/api/diagnostics/faces")
        assert resp.status_code == 200

    def test_faces_structure(self, client):
        """Face data should have faces list, count, and connected flag."""
        resp = client.get("/api/diagnostics/faces")
        data = resp.get_json()
        assert "faces" in data
        assert "count" in data
        assert "connected" in data

    def test_faces_no_camera_in_simulation(self, client):
        """In simulation mode, camera should not be connected."""
        resp = client.get("/api/diagnostics/faces")
        data = resp.get_json()
        assert data["connected"] is False
        assert data["count"] == 0


# ===========================================================================
# 6. Video Feed Endpoint
# ===========================================================================

class TestVideoFeed:
    """Verify the MJPEG video feed endpoint.

    Note: In simulation mode the generator sleeps forever waiting for
    frames, so we inject a mock JPEG to avoid blocking the test runner.
    """

    def test_video_feed_with_mock_frame(self, app, client):
        """GET /video_feed should stream MJPEG frames when data is available."""
        # Inject a fake JPEG so the generator yields one frame immediately
        fake_jpeg = b'\xff\xd8\xff\xe0fake-jpeg-data'
        app.hardware._latest_jpeg = fake_jpeg

        resp = client.get("/video_feed")
        assert resp.status_code == 200
        assert "multipart/x-mixed-replace" in resp.content_type

        # Clean up
        app.hardware._latest_jpeg = None


# ===========================================================================
# 7. Error Handling and Edge Cases
# ===========================================================================

class TestErrorHandling:
    """Test error responses and edge cases."""

    def test_nonexistent_route_returns_404(self, client):
        """GET /api/nonexistent should return 404."""
        resp = client.get("/api/nonexistent")
        assert resp.status_code == 404

    def test_motor_drive_defaults_to_zero(self, client):
        """POST /api/motors/drive with empty JSON should default to 0,0."""
        resp = client.post("/api/motors/drive", json={})
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["left"] == 0
        assert data["right"] == 0

    def test_motor_drive_wrong_method(self, client):
        """GET /api/motors/drive should not be allowed (POST only)."""
        resp = client.get("/api/motors/drive")
        assert resp.status_code == 405

    def test_motor_stop_wrong_method(self, client):
        """GET /api/motors/stop should not be allowed (POST only)."""
        resp = client.get("/api/motors/stop")
        assert resp.status_code == 405

    def test_chat_ask_wrong_method(self, client):
        """GET /api/chat/ask should not be allowed (POST only)."""
        resp = client.get("/api/chat/ask")
        assert resp.status_code == 405
