import argparse
import math
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
JETSON_ROOT = REPO_ROOT / "Jetson"
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from config import (
    ARDUINO_PORT,
    CAMERA_AUTO_EXPOSURE,
    CAMERA_INPUT_FLIP,
    CAMERA_INPUT_HEIGHT,
    CAMERA_INPUT_RATE,
    CAMERA_INPUT_WIDTH,
    CAMERA_URI,
    CAMERA_V4L2_CONTROLS,
    LEFT_MOTOR,
    MAX_FORWARD_CM_PER_SEC,
    MAX_TURN_DEG_PER_SEC,
    RIGHT_MOTOR,
    TRAIL_FOLLOW_FORWARD_CM_PER_SEC,
    TRAIL_FOLLOW_MAX_TURN_DEG_PER_SEC,
    TRAIL_FOLLOW_NO_TRAIL_TIMEOUT_SEC,
    TRACK_WIDTH_CM,
    WHEEL_DIAMETER_CM,
)
from hardware.arduino_io import ArduinoIO
from hardware.camera import Camera
from navigation.drive_to_destination import _trail_center_offset
from robot import create_motor


DEFAULT_FORWARD_CM_PER_SEC = TRAIL_FOLLOW_FORWARD_CM_PER_SEC
DEFAULT_MAX_TURN_DEG_PER_SEC = TRAIL_FOLLOW_MAX_TURN_DEG_PER_SEC
DEFAULT_CONTROL_INTERVAL_SEC = 0.08
DEFAULT_NO_TRAIL_TIMEOUT_SEC = TRAIL_FOLLOW_NO_TRAIL_TIMEOUT_SEC
TRAIL_OFFSET_SMOOTHING = 0.25
TRAIL_OFFSET_DECAY = 0.75


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drive using only DeepScene trail feedback."
    )
    parser.add_argument("--arduino-port", default=ARDUINO_PORT)
    parser.add_argument("--camera-uri", default=CAMERA_URI)
    parser.add_argument(
        "--forward-cm-s",
        type=float,
        default=DEFAULT_FORWARD_CM_PER_SEC,
        help="Forward speed while a trail signal is available.",
    )
    parser.add_argument(
        "--max-turn-deg-s",
        type=float,
        default=DEFAULT_MAX_TURN_DEG_PER_SEC,
        help="Turn rate used when the trail is fully left/right in frame.",
    )
    parser.add_argument(
        "--control-interval",
        type=float,
        default=DEFAULT_CONTROL_INTERVAL_SEC,
    )
    parser.add_argument(
        "--no-trail-timeout",
        type=float,
        default=DEFAULT_NO_TRAIL_TIMEOUT_SEC,
        help="Stop if no useful trail signal is seen for this many seconds.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Read camera and print steering without commanding motors.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    smoothed_offset = None
    last_signal_at = 0.0
    last_log_at = 0.0

    arduino = None
    left_motor = None
    right_motor = None

    if not args.dry_run:
        arduino = ArduinoIO(port=args.arduino_port)
        left_motor = create_motor(arduino, LEFT_MOTOR)
        right_motor = create_motor(arduino, RIGHT_MOTOR)

    camera = Camera(
        camera_uri=args.camera_uri,
        deepscene_enabled=True,
        input_width=CAMERA_INPUT_WIDTH,
        input_height=CAMERA_INPUT_HEIGHT,
        input_rate=CAMERA_INPUT_RATE,
        input_flip=CAMERA_INPUT_FLIP,
        v4l2_controls=CAMERA_V4L2_CONTROLS,
        auto_exposure=CAMERA_AUTO_EXPOSURE,
    )

    try:
        print("Following trail with video-only steering. Press Ctrl+C to stop.")

        while True:
            frame = camera.capture()
            now = time.monotonic()
            measured_offset = (
                None if frame is None else _trail_center_offset(frame.trail_mask)
            )

            if measured_offset is None:
                smoothed_offset = _decay_feedback_offset(smoothed_offset)
            else:
                smoothed_offset = _smooth_feedback_offset(
                    smoothed_offset,
                    measured_offset,
                )
                last_signal_at = now

            has_recent_signal = (
                smoothed_offset is not None
                and now - last_signal_at <= max(0.0, args.no_trail_timeout)
            )

            if has_recent_signal:
                forward_cm_s = _clamp(
                    args.forward_cm_s,
                    -MAX_FORWARD_CM_PER_SEC,
                    MAX_FORWARD_CM_PER_SEC,
                )
                turn_deg_s = _clamp(
                    -smoothed_offset * args.max_turn_deg_s,
                    -MAX_TURN_DEG_PER_SEC,
                    MAX_TURN_DEG_PER_SEC,
                )
            else:
                forward_cm_s = 0.0
                turn_deg_s = 0.0

            if not args.dry_run:
                set_chassis_velocity(
                    left_motor,
                    right_motor,
                    forward_cm_s,
                    turn_deg_s,
                )

            if now - last_log_at >= 0.5:
                print(
                    "trail="
                    f"{'yes' if measured_offset is not None else 'no '} "
                    f"offset={_format_optional(smoothed_offset)} "
                    f"forward={forward_cm_s:.1f} cm/s "
                    f"turn={turn_deg_s:.1f} deg/s"
                )
                last_log_at = now

            time.sleep(max(0.0, args.control_interval))

    except KeyboardInterrupt:
        print()
        print("Trail follow stopped.")
    finally:
        if not args.dry_run and left_motor is not None and right_motor is not None:
            left_motor.set_velocity_pair(right_motor, 0.0, 0.0)
        camera.close()
        if arduino is not None:
            arduino.close()


def set_chassis_velocity(left_motor, right_motor, forward_cm_s, turn_deg_s):
    turn_rad_s = math.radians(float(turn_deg_s))
    half_track_cm = TRACK_WIDTH_CM / 2.0
    left_cm_s = float(forward_cm_s) - turn_rad_s * half_track_cm
    right_cm_s = float(forward_cm_s) + turn_rad_s * half_track_cm
    wheel_degrees_per_cm = 360.0 / (math.pi * WHEEL_DIAMETER_CM)
    left_deg_s = -left_cm_s * wheel_degrees_per_cm
    right_deg_s = right_cm_s * wheel_degrees_per_cm
    left_motor.set_velocity_pair(right_motor, left_deg_s, right_deg_s)


def _format_optional(value):
    if value is None:
        return "none"

    return f"{value:+.3f}"


def _smooth_feedback_offset(previous_offset, measured_offset):
    measured_offset = _clamp(measured_offset, -1.0, 1.0)

    if previous_offset is None:
        return measured_offset

    return previous_offset + (
        measured_offset - previous_offset
    ) * TRAIL_OFFSET_SMOOTHING


def _decay_feedback_offset(previous_offset):
    if previous_offset is None:
        return None

    decayed_offset = previous_offset * TRAIL_OFFSET_DECAY

    if abs(decayed_offset) < 0.01:
        return None

    return decayed_offset


def _clamp(value, low, high):
    return max(low, min(high, float(value)))


if __name__ == "__main__":
    main()
