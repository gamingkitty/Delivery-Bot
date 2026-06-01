import os
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional


DEFAULT_CAMERA_URI = "/dev/video0"
DEFAULT_MODEL_DIR = (
    "/home/hiwonder/jetson-inference-data/networks/"
    "FCN-ResNet18-DeepScene-576x320"
)

TRAIL_BGR = (0, 255, 0)
OBSTACLE_BGR = (0, 0, 255)


V4L2_CONTROL_ALIASES = {
    "auto_exposure": ("auto_exposure", "exposure_auto"),
    "exposure_auto": ("exposure_auto", "auto_exposure"),
    "exposure_time_absolute": ("exposure_time_absolute", "exposure_absolute"),
    "exposure_absolute": ("exposure_absolute", "exposure_time_absolute"),
    "exposure_auto_priority": (
        "exposure_auto_priority",
        "auto_exposure_priority",
    ),
    "auto_exposure_priority": (
        "auto_exposure_priority",
        "exposure_auto_priority",
    ),
}


@dataclass
class DeepSceneFrame:
    original_bgr: Any
    mask_bgr: Any
    trail_mask: Any
    obstacle_mask: Any
    overlay_bgr: Any
    trail_fraction: float
    obstacle_fraction: float


_shared_camera_lock = threading.Lock()
_shared_camera = None


def acquire_shared_camera(**camera_kwargs):
    global _shared_camera

    with _shared_camera_lock:
        if _shared_camera is not None and not _shared_camera.supports(camera_kwargs):
            if _shared_camera.ref_count == 0:
                _shared_camera.close()
                _shared_camera = None
            else:
                raise RuntimeError(
                    "Shared camera is already running with different settings"
                )

        if _shared_camera is None:
            _shared_camera = _SharedCamera(camera_kwargs)

        _shared_camera.ref_count += 1
        return _SharedCameraHandle(_shared_camera)


def _release_shared_camera(shared_camera):
    global _shared_camera

    with _shared_camera_lock:
        if shared_camera.ref_count > 0:
            shared_camera.ref_count -= 1

        if shared_camera.ref_count == 0:
            shared_camera.close()

            if _shared_camera is shared_camera:
                _shared_camera = None


class _SharedCameraHandle:
    def __init__(self, shared_camera):
        self._shared_camera = shared_camera
        self._closed = False
        self._last_capture_frame_id = 0

    @property
    def deepscene_enabled(self):
        return self._shared_camera.deepscene_enabled

    def capture(self, timeout_ms: Optional[int] = 1000) -> Optional[DeepSceneFrame]:
        frame_id, frame = self.read_frame(
            timeout_ms=timeout_ms,
            last_frame_id=self._last_capture_frame_id,
        )

        if frame is not None:
            self._last_capture_frame_id = frame_id

        return frame

    def read_frame(
        self,
        timeout_ms: Optional[int] = 1000,
        last_frame_id: Optional[int] = None,
    ):
        if self._closed:
            raise RuntimeError("Shared camera handle is closed")

        return self._shared_camera.read_frame(
            timeout_ms=timeout_ms,
            last_frame_id=last_frame_id,
        )

    def status(self):
        return self._shared_camera.status()

    def close(self):
        if self._closed:
            return

        self._closed = True
        _release_shared_camera(self._shared_camera)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class _SharedCamera:
    def __init__(self, camera_kwargs, camera_factory=None):
        self.camera_kwargs = dict(camera_kwargs)
        self.deepscene_enabled = bool(
            self.camera_kwargs.get("deepscene_enabled", True)
        )
        self.camera_factory = Camera if camera_factory is None else camera_factory
        self.ref_count = 0
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.latest_frame = None
        self.frame_id = 0
        self.error = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def supports(self, requested_kwargs):
        requested_kwargs = dict(requested_kwargs)
        requested_deepscene = bool(requested_kwargs.get("deepscene_enabled", True))

        if requested_deepscene and not self.deepscene_enabled:
            return False

        comparable_keys = {
            "camera_uri",
            "model_dir",
            "input_width",
            "input_height",
            "input_rate",
            "input_flip",
            "v4l2_controls",
            "auto_exposure",
            "trail_threshold",
            "obstacle_threshold",
        }

        for key in comparable_keys:
            if requested_kwargs.get(key) != self.camera_kwargs.get(key):
                return False

        return True

    def read_frame(
        self,
        timeout_ms: Optional[int] = 1000,
        last_frame_id: Optional[int] = None,
    ):
        deadline = None

        if timeout_ms is not None:
            deadline = time.monotonic() + max(0, int(timeout_ms)) / 1000.0

        with self.condition:
            while True:
                has_frame = self.latest_frame is not None
                has_new_frame = last_frame_id is None or self.frame_id != last_frame_id

                if has_frame and has_new_frame:
                    return self.frame_id, self.latest_frame

                if self.closed:
                    raise RuntimeError("Shared camera is closed")

                if self.error is not None and self.latest_frame is None:
                    raise RuntimeError(self.error)

                if timeout_ms == 0:
                    return self.frame_id, None

                wait_timeout = None

                if deadline is not None:
                    wait_timeout = deadline - time.monotonic()

                    if wait_timeout <= 0.0:
                        return self.frame_id, None

                self.condition.wait(timeout=wait_timeout)

    def status(self):
        with self.condition:
            return {
                "deepscene_enabled": self.deepscene_enabled,
                "error": self.error,
                "frames": self.frame_id,
                "ref_count": self.ref_count,
            }

    def close(self):
        self.stop_event.set()

        with self.condition:
            self.closed = True
            self.condition.notify_all()

        self.thread.join(timeout=2.0)

    def _run(self):
        camera = None

        try:
            camera = self.camera_factory(**self.camera_kwargs)

            while not self.stop_event.is_set():
                frame = camera.capture(timeout_ms=1000)

                if frame is None:
                    continue

                with self.condition:
                    self.latest_frame = frame
                    self.frame_id += 1
                    self.error = None
                    self.condition.notify_all()

        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.condition.notify_all()

        finally:
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass


class Camera:
    """Camera source that runs the DeepScene segmentation model."""

    def __init__(
        self,
        camera_uri: str = DEFAULT_CAMERA_URI,
        model_dir: str = DEFAULT_MODEL_DIR,
        deepscene_enabled: bool = True,
        input_width: int = 640,
        input_height: int = 480,
        input_rate: int = 15,
        input_flip: Optional[str] = None,
        v4l2_controls: Optional[dict] = None,
        auto_exposure: Optional[dict] = None,
        trail_threshold: int = 120,
        obstacle_threshold: int = 120,
    ):
        self.camera_uri = camera_uri
        self.model_dir = model_dir
        self.deepscene_enabled = bool(deepscene_enabled)
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.input_rate = int(input_rate)
        self.input_flip = input_flip
        self.v4l2_controls = dict(v4l2_controls or {})
        self.auto_exposure = self._make_auto_exposure_config(auto_exposure)
        self.trail_threshold = int(trail_threshold)
        self.obstacle_threshold = int(obstacle_threshold)
        self._resolved_v4l2_controls = {}
        self._auto_exposure_value = None
        self._auto_exposure_disabled = False
        self._last_auto_exposure_at = 0.0

        if self.auto_exposure is not None:
            self._auto_exposure_value = self._initial_auto_exposure_value()
            self.v4l2_controls[
                self.auto_exposure["control_name"]
            ] = self._auto_exposure_value

        self.cv2 = None
        self.np = None
        self.jetson_inference = None
        self.jetson_utils = None
        self.colors_file = None
        self.net = None
        self.source = None
        self.lock = threading.Lock()

        self._load_runtime_modules()

        try:
            if self.deepscene_enabled:
                self.colors_file = self._make_runtime_colors_file()
                self.net = self._create_deepscene_model()
            self._apply_v4l2_controls()
            self.source = self._create_video_source()
            self._apply_v4l2_controls()
        except Exception:
            self.close()
            raise

    def close(self):
        with self.lock:
            if self.source is not None:
                close_source = getattr(self.source, "Close", None)
                if callable(close_source):
                    close_source()
                self.source = None

            if self.colors_file and os.path.exists(self.colors_file):
                os.remove(self.colors_file)
            self.colors_file = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def capture(self, timeout_ms: Optional[int] = 1000) -> Optional[DeepSceneFrame]:
        if self.source is None:
            raise RuntimeError("Camera is closed")

        with self.lock:
            if timeout_ms is None:
                img = self.source.Capture()
            else:
                img = self.source.Capture(timeout=max(0, int(timeout_ms)))

            if img is None:
                return None

            original = self._cuda_image_to_bgr(img)
            self._adjust_auto_exposure(original)

            if self.net is None:
                empty_mask = self.np.zeros(original.shape[:2], dtype=bool)
                return DeepSceneFrame(
                    original_bgr=original,
                    mask_bgr=original.copy(),
                    trail_mask=empty_mask,
                    obstacle_mask=empty_mask.copy(),
                    overlay_bgr=original.copy(),
                    trail_fraction=0.0,
                    obstacle_fraction=0.0,
                )

            self.net.Process(img)
            self.net.Mask(img, filter_mode="point")
            mask = self._cuda_image_to_bgr(img)
            trail_mask, obstacle_mask = self._detect_trail_and_obstacles(mask)
            overlay = self._draw_scene_overlay(original, trail_mask, obstacle_mask)

        return DeepSceneFrame(
            original_bgr=original,
            mask_bgr=mask,
            trail_mask=trail_mask,
            obstacle_mask=obstacle_mask,
            overlay_bgr=overlay,
            trail_fraction=float(trail_mask.mean()),
            obstacle_fraction=float(obstacle_mask.mean()),
        )

    def read(self) -> Optional[DeepSceneFrame]:
        return self.capture()

    def is_streaming(self) -> bool:
        if self.source is None:
            return False

        is_streaming = getattr(self.source, "IsStreaming", None)
        return bool(is_streaming()) if callable(is_streaming) else True

    def _load_runtime_modules(self):
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise ImportError("Camera requires OpenCV and NumPy to process frames") from exc

        try:
            import jetson_utils
        except ImportError:
            try:
                import jetson.utils as jetson_utils
            except ImportError as exc:
                raise ImportError("Camera requires the jetson-utils Python module") from exc

        jetson_inference = None

        if self.deepscene_enabled:
            try:
                import jetson_inference
            except ImportError:
                try:
                    import jetson.inference as jetson_inference
                except ImportError as exc:
                    raise ImportError(
                        "DeepScene requires the jetson-inference Python module"
                    ) from exc

        self.cv2 = cv2
        self.np = np
        self.jetson_inference = jetson_inference
        self.jetson_utils = jetson_utils

    def _create_deepscene_model(self):
        return self.jetson_inference.segNet(
            model=os.path.join(self.model_dir, "fcn_resnet18.onnx"),
            labels=os.path.join(self.model_dir, "classes.txt"),
            colors=self.colors_file,
            input_blob="input_0",
            output_blob="output_0",
        )

    def _create_video_source(self):
        argv = [
            f"--input-width={self.input_width}",
            f"--input-height={self.input_height}",
            f"--input-rate={self.input_rate}",
        ]

        if self.input_flip:
            argv.append(f"--input-flip={self.input_flip}")

        return self.jetson_utils.videoSource(self.camera_uri, argv=argv)

    def _apply_v4l2_controls(self):
        if not self.v4l2_controls:
            return

        if not self.camera_uri.startswith("/dev/video"):
            return

        for name, value in self.v4l2_controls.items():
            if value is None:
                continue

            self._set_v4l2_control(name, value)

    def _set_v4l2_control(self, name, value):
        candidates = self._v4l2_control_candidates(name)
        messages = []

        for control_name in candidates:
            try:
                subprocess.run(
                    [
                        "v4l2-ctl",
                        "--device",
                        self.camera_uri,
                        "--set-ctrl",
                        f"{control_name}={value}",
                    ],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    universal_newlines=True,
                )
                if control_name != name:
                    print(
                        "Camera V4L2 control applied with alternate name: "
                        f"{name}->{control_name}={value}"
                    )
                self._resolved_v4l2_controls[name] = control_name
                return True
            except FileNotFoundError:
                print("Camera V4L2 controls skipped: v4l2-ctl is not installed")
                return False
            except subprocess.CalledProcessError as exc:
                message = (exc.stderr or exc.stdout or str(exc)).strip()
                messages.append(f"{control_name}={value}: {message}")

        print(
            f"Camera V4L2 control skipped: {name}={value}: "
            + " | ".join(messages)
        )
        return False

    def _v4l2_control_candidates(self, name):
        resolved_name = self._resolved_v4l2_controls.get(name)
        aliases = V4L2_CONTROL_ALIASES.get(name, (name,))

        if resolved_name is None:
            return aliases

        return (resolved_name,) + tuple(
            alias for alias in aliases if alias != resolved_name
        )

    def _make_auto_exposure_config(self, auto_exposure):
        settings = dict(auto_exposure or {})

        if not settings.get("enabled", False):
            return None

        min_value = int(
            settings.get(
                "min_value",
                settings.get("min_exposure_time_absolute", 2),
            )
        )
        max_value = int(
            settings.get(
                "max_value",
                settings.get("max_exposure_time_absolute", 6),
            )
        )

        if max_value < min_value:
            min_value, max_value = max_value, min_value

        config = {
            "control_name": settings.get("control_name", "exposure_time_absolute"),
            "min_value": min_value,
            "max_value": max_value,
            "initial_value": settings.get("initial_value"),
            "target_luma": float(settings.get("target_luma", 105.0)),
            "deadband": max(0.0, float(settings.get("deadband", 14.0))),
            "luma_per_step": max(1.0, float(settings.get("luma_per_step", 35.0))),
            "interval_sec": max(0.0, float(settings.get("interval_sec", 0.35))),
            "max_step": max(1, int(settings.get("max_step", 1))),
            "roi_top_fraction": float(settings.get("roi_top_fraction", 0.12)),
            "roi_bottom_fraction": float(
                settings.get("roi_bottom_fraction", 0.88)
            ),
            "roi_left_fraction": float(settings.get("roi_left_fraction", 0.10)),
            "roi_right_fraction": float(settings.get("roi_right_fraction", 0.90)),
        }

        return config

    def _initial_auto_exposure_value(self):
        configured_value = self.auto_exposure["initial_value"]

        if configured_value is None:
            configured_value = self._configured_v4l2_control_value(
                self.auto_exposure["control_name"]
            )

        if configured_value is None:
            configured_value = self.auto_exposure["min_value"]

        return self._clamp_exposure_value(configured_value)

    def _configured_v4l2_control_value(self, name):
        for control_name in V4L2_CONTROL_ALIASES.get(name, (name,)):
            if control_name in self.v4l2_controls:
                return self.v4l2_controls[control_name]

        return None

    def _adjust_auto_exposure(self, frame_bgr):
        if self.auto_exposure is None or self._auto_exposure_disabled:
            return

        if not self.camera_uri.startswith("/dev/video"):
            return

        now = time.monotonic()
        if now - self._last_auto_exposure_at < self.auto_exposure["interval_sec"]:
            return

        self._last_auto_exposure_at = now
        luma = self._mean_luma(frame_bgr)

        if luma is None:
            return

        error = self.auto_exposure["target_luma"] - luma

        if abs(error) <= self.auto_exposure["deadband"]:
            return

        direction = 1 if error > 0 else -1
        excess_error = abs(error) - self.auto_exposure["deadband"]
        step = int(excess_error / self.auto_exposure["luma_per_step"]) + 1
        step = min(step, self.auto_exposure["max_step"])
        exposure_value = self._clamp_exposure_value(
            self._auto_exposure_value + direction * step
        )

        if exposure_value == self._auto_exposure_value:
            return

        if self._set_v4l2_control(
            self.auto_exposure["control_name"],
            exposure_value,
        ):
            self._auto_exposure_value = exposure_value
        else:
            self._auto_exposure_disabled = True
            print("Camera auto exposure disabled after V4L2 control failure")

    def _mean_luma(self, frame_bgr):
        if frame_bgr is None:
            return None

        height, width = frame_bgr.shape[:2]

        if height <= 0 or width <= 0:
            return None

        roi_top = _clamp_int(
            int(height * self.auto_exposure["roi_top_fraction"]),
            0,
            height - 1,
        )
        roi_bottom = _clamp_int(
            int(height * self.auto_exposure["roi_bottom_fraction"]),
            roi_top + 1,
            height,
        )
        roi_left = _clamp_int(
            int(width * self.auto_exposure["roi_left_fraction"]),
            0,
            width - 1,
        )
        roi_right = _clamp_int(
            int(width * self.auto_exposure["roi_right_fraction"]),
            roi_left + 1,
            width,
        )
        roi = frame_bgr[roi_top:roi_bottom, roi_left:roi_right]

        if roi.size == 0:
            return None

        if len(roi.shape) < 3 or roi.shape[2] < 3:
            return float(roi.mean())

        b = roi[:, :, 0].astype("float32")
        g = roi[:, :, 1].astype("float32")
        r = roi[:, :, 2].astype("float32")
        return float((0.114 * b + 0.587 * g + 0.299 * r).mean())

    def _clamp_exposure_value(self, value):
        return _clamp_int(
            int(round(float(value))),
            self.auto_exposure["min_value"],
            self.auto_exposure["max_value"],
        )

    def _make_runtime_colors_file(self):
        """
        DeepScene class colors:
          0 trail      -> green
          1 grass      -> black
          2 vegetation -> black
          3 obstacle   -> red
          4 sky        -> black
        """
        colors = """0 255 0
0 0 0
0 0 0
255 0 0
0 0 0
"""

        color_file = tempfile.NamedTemporaryFile(
            mode="w",
            delete=False,
            suffix="_deepscene_colors.txt",
        )
        color_file.write(colors)
        color_file.close()
        return color_file.name

    def _cuda_image_to_bgr(self, img):
        self.jetson_utils.cudaDeviceSynchronize()

        frame = self.jetson_utils.cudaToNumpy(img).copy()

        if frame.dtype != self.np.uint8:
            frame = self.np.clip(frame, 0, 255).astype(self.np.uint8)

        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = self.cv2.cvtColor(frame, self.cv2.COLOR_RGBA2BGR)
        elif len(frame.shape) == 3 and frame.shape[2] == 3:
            frame = self.cv2.cvtColor(frame, self.cv2.COLOR_RGB2BGR)

        return frame.copy()

    def _detect_trail_and_obstacles(self, mask_bgr):
        trail_mask = (
            (mask_bgr[:, :, 1] > self.trail_threshold)
            & (mask_bgr[:, :, 0] < 80)
            & (mask_bgr[:, :, 2] < 80)
        )

        obstacle_mask = (
            (mask_bgr[:, :, 2] > self.obstacle_threshold)
            & (mask_bgr[:, :, 0] < 80)
            & (mask_bgr[:, :, 1] < 80)
        )

        return trail_mask, obstacle_mask

    def _draw_scene_overlay(self, original_bgr, trail_mask, obstacle_mask):
        overlay = original_bgr.copy()
        overlay[trail_mask] = TRAIL_BGR
        overlay[obstacle_mask] = OBSTACLE_BGR
        return overlay


def _clamp_int(value, low, high):
    return max(low, min(high, int(value)))
