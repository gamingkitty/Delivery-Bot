#!/usr/bin/env python3

import sys
from pathlib import Path

import cv2
from flask import Flask, Response


JETSON_ROOT = Path(__file__).resolve().parents[1]
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from hardware.camera import Camera


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
    camera = Camera()

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        camera.close()


if __name__ == "__main__":
    main()
