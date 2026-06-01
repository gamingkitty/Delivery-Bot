import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


JETSON_ROOT = Path(__file__).resolve().parents[1] / "Jetson"
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from hardware.camera import Camera, _SharedCamera


class CameraAutoExposureTests(unittest.TestCase):
    def test_dark_frame_increases_exposure(self):
        camera, updates = make_camera(luma=40.0, exposure_value=2)

        camera._adjust_auto_exposure(object())

        self.assertEqual(updates, [("exposure_time_absolute", 3)])
        self.assertEqual(camera._auto_exposure_value, 3)

    def test_bright_frame_decreases_exposure(self):
        camera, updates = make_camera(luma=180.0, exposure_value=6)

        camera._adjust_auto_exposure(object())

        self.assertEqual(updates, [("exposure_time_absolute", 5)])
        self.assertEqual(camera._auto_exposure_value, 5)

    def test_deadband_leaves_exposure_unchanged(self):
        camera, updates = make_camera(luma=100.0, exposure_value=4)

        camera._adjust_auto_exposure(object())

        self.assertEqual(updates, [])
        self.assertEqual(camera._auto_exposure_value, 4)

    def test_v4l2_failure_disables_auto_exposure(self):
        camera, updates = make_camera(luma=40.0, exposure_value=2, set_ok=False)

        camera._adjust_auto_exposure(object())

        self.assertEqual(updates, [("exposure_time_absolute", 3)])
        self.assertTrue(camera._auto_exposure_disabled)
        self.assertEqual(camera._auto_exposure_value, 2)

    def test_capture_passes_timeout_to_video_source(self):
        camera = Camera.__new__(Camera)
        source = FakeSource()
        camera.source = source
        camera.lock = threading.Lock()

        self.assertIsNone(camera.capture(timeout_ms=0))
        self.assertEqual(source.timeout, 0)

    def test_shared_camera_serves_same_processed_frame_to_multiple_consumers(self):
        shared = _SharedCamera(
            {"deepscene_enabled": True},
            camera_factory=FakeCaptureCamera,
        )

        try:
            frame_id, frame = shared.read_frame(timeout_ms=500)
            cached_frame_id, cached_frame = shared.read_frame(timeout_ms=0)

            self.assertEqual(cached_frame_id, frame_id)
            self.assertIs(cached_frame, frame)
        finally:
            shared.close()


def make_camera(luma, exposure_value, set_ok=True):
    camera = Camera.__new__(Camera)
    camera.camera_uri = "/dev/video0"
    camera.auto_exposure = {
        "control_name": "exposure_time_absolute",
        "min_value": 2,
        "max_value": 6,
        "target_luma": 105.0,
        "deadband": 14.0,
        "luma_per_step": 35.0,
        "interval_sec": 0.0,
        "max_step": 1,
    }
    camera._auto_exposure_value = exposure_value
    camera._auto_exposure_disabled = False
    camera._last_auto_exposure_at = 0.0
    camera._mean_luma = lambda _frame: luma
    updates = []

    def set_control(name, value):
        updates.append((name, value))
        return set_ok

    camera._set_v4l2_control = set_control
    return camera, updates


class FakeSource:
    def __init__(self):
        self.timeout = None

    def Capture(self, timeout=None):
        self.timeout = timeout
        return None


class FakeCaptureCamera:
    def __init__(self, **_kwargs):
        self.frame_id = 0

    def capture(self, timeout_ms=1000):
        if self.frame_id > 0:
            time.sleep(0.1)
            return None

        self.frame_id += 1
        return SimpleNamespace(frame_id=self.frame_id, timeout_ms=timeout_ms)

    def close(self):
        pass


if __name__ == "__main__":
    unittest.main()
