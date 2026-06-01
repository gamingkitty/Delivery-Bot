#!/usr/bin/env python3

import sys
from pathlib import Path

import cv2
from flask import Flask, Response


JETSON_ROOT = Path(__file__).resolve().parents[1]
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from hardware.camera import Camera
from config import (
    CAMERA_AUTO_EXPOSURE,
    CAMERA_INPUT_FLIP,
    CAMERA_INPUT_HEIGHT,
    CAMERA_INPUT_RATE,
    CAMERA_INPUT_WIDTH,
    CAMERA_URI,
    CAMERA_V4L2_CONTROLS,
)


app = Flask(__name__)
camera = None


def generate_frames():
    while True:
        result = camera.capture()

        if result is None:
            continue

        ok, jpeg = cv2.imencode(".jpg", result.overlay_bgr)

        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg.tobytes()
            + b"\r\n"
        )


@app.route("/")
def index():
    return """
    <html>
      <head>
        <title>Jetson DeepScene Live</title>
      </head>
      <body style="background: #111; color: white; text-align: center;">
        <h2>DeepScene: Trail = Green, Obstacle = Red</h2>
        <img src="/video" style="max-width: 100%; height: auto;">
      </body>
    </html>
    """


@app.route("/video")
def video():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


def main():
    global camera
    camera = Camera(
        camera_uri=CAMERA_URI,
        input_width=CAMERA_INPUT_WIDTH,
        input_height=CAMERA_INPUT_HEIGHT,
        input_rate=CAMERA_INPUT_RATE,
        input_flip=CAMERA_INPUT_FLIP,
        v4l2_controls=CAMERA_V4L2_CONTROLS,
        auto_exposure=CAMERA_AUTO_EXPOSURE,
    )

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        camera.close()


if __name__ == "__main__":
    main()
