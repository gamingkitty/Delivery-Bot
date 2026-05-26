import os
import tempfile
import threading
from dataclasses import dataclass
from typing import Any, Optional


DEFAULT_CAMERA_URI = "/dev/video0"
DEFAULT_MODEL_DIR = (
    "/home/hiwonder/jetson-inference-data/networks/"
    "FCN-ResNet18-DeepScene-576x320"
)

TRAIL_BGR = (0, 255, 0)
OBSTACLE_BGR = (0, 0, 255)


@dataclass
class DeepSceneFrame:
    original_bgr: Any
    mask_bgr: Any
    trail_mask: Any
    obstacle_mask: Any
    overlay_bgr: Any
    trail_fraction: float
    obstacle_fraction: float


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
        trail_threshold: int = 120,
        obstacle_threshold: int = 120,
    ):
        self.camera_uri = camera_uri
        self.model_dir = model_dir
        self.deepscene_enabled = bool(deepscene_enabled)
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.input_rate = int(input_rate)
        self.trail_threshold = int(trail_threshold)
        self.obstacle_threshold = int(obstacle_threshold)

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
            self.source = self._create_video_source()
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

    def capture(self) -> Optional[DeepSceneFrame]:
        if self.source is None:
            raise RuntimeError("Camera is closed")

        with self.lock:
            img = self.source.Capture()

            if img is None:
                return None

            original = self._cuda_image_to_bgr(img)

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
        return self.jetson_utils.videoSource(
            self.camera_uri,
            argv=[
                f"--input-width={self.input_width}",
                f"--input-height={self.input_height}",
                f"--input-rate={self.input_rate}",
            ],
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
