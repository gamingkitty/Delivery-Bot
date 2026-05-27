import json
import tempfile
import unittest
from pathlib import Path

from WebApp.app import create_app
from WebApp.config import load_config, public_config
from WebApp.geo import latlon_to_map_percent
from Jetson.config import GPS_ORIGIN
from Jetson.navigation.coordinates import latlon_to_xy, xy_to_latlon


class WebAppTests(unittest.TestCase):
    def test_latlon_to_map_percent_uses_rectangular_bounds(self):
        top_left = {"lat": 48.0, "lon": -122.0}
        bottom_right = {"lat": 47.0, "lon": -121.0}

        x, y = latlon_to_map_percent(47.5, -121.5, top_left, bottom_right)

        self.assertAlmostEqual(x, 50.0)
        self.assertAlmostEqual(y, 50.0)

    def test_xy_latlon_round_trip(self):
        origin = {"lat": 47.608013, "lon": -122.335167}
        x, y = latlon_to_xy(47.609, -122.334, origin)
        lat, lon = xy_to_latlon(x, y, origin)

        self.assertAlmostEqual(lat, 47.609, places=6)
        self.assertAlmostEqual(lon, -122.334, places=6)

    def test_public_config_does_not_expose_pins(self):
        with temporary_config() as config_path:
            config = load_config(config_path)
            public = public_config(config)

        self.assertIn("stops", public)
        self.assertNotIn("auth", public)
        self.assertNotIn("user_pin", json.dumps(public))
        self.assertEqual(public["map"]["navigation_origin"]["lat"], GPS_ORIGIN[0])
        self.assertEqual(public["map"]["navigation_origin"]["lon"], GPS_ORIGIN[1])
        self.assertEqual(len(public["stops"]), 1)

    def test_api_rejects_bad_pin_and_accepts_valid_job(self):
        with temporary_config() as config_path:
            app = create_app(config_path, controller=FakeRobotController())
            client = app.test_client()

            bad = client.post(
                "/api/jobs",
                json={"destination": "library", "pin": "bad"},
            )
            self.assertEqual(bad.status_code, 403)

            good = client.post(
                "/api/jobs",
                json={"destination": "library", "pin": "1234"},
            )
            self.assertEqual(good.status_code, 202)
            self.assertIn(good.get_json()["state"], {"planning", "driving"})

    def test_api_rejects_busy_robot(self):
        with temporary_config() as config_path:
            app = create_app(config_path, controller=FakeRobotController())
            client = app.test_client()

            first = client.post(
                "/api/jobs",
                json={"destination": "library", "pin": "1234"},
            )
            self.assertEqual(first.status_code, 202)

            second = client.post(
                "/api/jobs",
                json={"destination": "library", "pin": "1234"},
            )
            self.assertEqual(second.status_code, 409)

            client.post("/api/stop", json={"pin": "1234"})

    def test_debug_requires_admin_login(self):
        with temporary_config() as config_path:
            app = create_app(config_path, controller=FakeRobotController())
            client = app.test_client()

            denied = client.get("/api/debug/status")
            self.assertEqual(denied.status_code, 403)

            login = client.post("/api/admin/login", json={"pin": "9999"})
            self.assertEqual(login.status_code, 200)

            allowed = client.get("/api/debug/status")
            self.assertEqual(allowed.status_code, 200)
            self.assertEqual(allowed.get_json()["controller"], "FakeRobotController")
            self.assertEqual(allowed.get_json()["last_error"], "test error")

    def test_manual_control_requires_admin_login(self):
        with temporary_config() as config_path:
            app = create_app(config_path, controller=FakeRobotController())
            client = app.test_client()

            denied = client.post(
                "/api/admin/manual-control",
                json={"direction": "forward"},
            )
            self.assertEqual(denied.status_code, 403)

            login = client.post("/api/admin/login", json={"pin": "9999"})
            self.assertEqual(login.status_code, 200)

            allowed = client.post(
                "/api/admin/manual-control",
                json={"direction": "forward", "duration_sec": 0.45},
            )
            self.assertEqual(allowed.status_code, 200)
            self.assertEqual(allowed.get_json()["state"], "manual_control")
            self.assertEqual(allowed.get_json()["manual_direction"], "forward")

    def test_manual_control_rejects_unknown_direction(self):
        with temporary_config() as config_path:
            app = create_app(config_path, controller=FakeRobotController())
            client = app.test_client()

            client.post("/api/admin/login", json={"pin": "9999"})

            response = client.post(
                "/api/admin/manual-control",
                json={"direction": "sideways"},
            )
            self.assertEqual(response.status_code, 400)


class FakeRobotController:
    def __init__(self):
        self.busy = False
        self.status_data = {
            "state": "idle",
            "message": "Ready",
            "robot": {
                "lat": None,
                "lon": None,
                "x_m": None,
                "y_m": None,
                "heading_deg": None,
            },
            "destination": None,
            "planned_path": [],
            "active_waypoint_index": None,
            "progress": 0.0,
            "manual_direction": None,
            "last_error": None,
            "updated_at": 0.0,
        }

    def status(self):
        return self.status_data.copy()

    def debug_status(self):
        data = self.status()
        data["controller"] = "FakeRobotController"
        data["last_error"] = "test error"
        data["events"] = []
        return data

    def submit_job(self, stop):
        if self.busy:
            raise RuntimeError("Robot is busy")

        self.busy = True
        self.status_data["state"] = "planning"
        self.status_data["destination"] = stop
        return self.status()

    def stop_job(self):
        self.busy = False
        self.status_data["state"] = "stopped"
        return self.status()

    def manual_control(
        self,
        direction,
        duration_sec=None,
        sequence=None,
        client_id=None,
    ):
        valid_directions = {"forward", "backward", "backwards", "left", "right", "stop"}

        if direction not in valid_directions:
            raise ValueError("Unknown manual control direction")

        if direction == "backwards":
            direction = "backward"

        self.busy = False
        self.status_data["state"] = "stopped" if direction == "stop" else "manual_control"
        self.status_data["manual_direction"] = None if direction == "stop" else direction
        return self.status()

    def shutdown(self):
        pass

    def camera_stream(self, mode):
        return None


class temporary_config:
    def __init__(self):
        self.directory = None
        self.path = None

    def __enter__(self):
        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        (root / "map.svg").write_text(
            "<svg xmlns='http://www.w3.org/2000/svg'></svg>",
            encoding="utf-8",
        )
        config = {
            "server": {
                "host": "127.0.0.1",
                "port": 8000,
                "debug": False,
            },
            "auth": {
                "user_pin": "1234",
                "admin_pin": "9999",
                "session_secret": "test-secret",
            },
            "map": {
                "image": "map.svg",
                "top_left": {"lat": 48.0, "lon": -122.0},
                "bottom_right": {"lat": 47.0, "lon": -121.0},
            },
            "ui": {
                "poll_interval_ms": 100,
            },
            "debug": {
                "enabled": True,
            },
            "stops": [
                {
                    "id": "library",
                    "name": "Library",
                    "lat": 47.6,
                    "lon": -121.4,
                    "enabled": True,
                },
                {
                    "id": "closed",
                    "name": "Closed Stop",
                    "lat": 47.7,
                    "lon": -121.3,
                    "enabled": False,
                },
            ],
        }

        self.path = root / "app.json"
        self.path.write_text(json.dumps(config), encoding="utf-8")
        return self.path

    def __exit__(self, exc_type, exc_value, traceback):
        self.directory.cleanup()


if __name__ == "__main__":
    unittest.main()
