import math
import sys
import time
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parents[1]
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
    GPS_PORT,
    MAX_FORWARD_CM_PER_SEC,
    NAV_CLEARANCE_COST_WEIGHT,
    NAV_CONTROL_INTERVAL_SEC,
    NAV_DEFAULT_DESTINATION_X_M,
    NAV_DEFAULT_DESTINATION_Y_M,
    NAV_DESTINATION_REACHED_DISTANCE_M,
    NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
    NAV_ENABLE_VIDEO_FEEDBACK,
    NAV_GPS_SNAP_ENABLED,
    NAV_GPS_SNAP_MAP_TOLERANCE_M,
    NAV_GPS_SNAP_MAX_DISTANCE_M,
    NAV_GRID_RESOLUTION_M,
    NAV_LOG_INTERVAL_SEC,
    NAV_MAP_PATH,
    NAV_MAX_MAP_DISTANCE_M,
    NAV_ROUTE_TIMEOUT_SEC,
    NAV_VIDEO_FEEDBACK_ANGLE_KP_DEG,
    NAV_VIDEO_FEEDBACK_FAR_ANGLE_CORRECTION_DEG,
    NAV_VIDEO_FEEDBACK_FAR_DISTANCE_M,
    NAV_VIDEO_FEEDBACK_MIN_TRAIL_FRACTION,
    NAV_VIDEO_FEEDBACK_MIN_TRAIL_PIXELS,
    NAV_VIDEO_FEEDBACK_NEAR_ANGLE_CORRECTION_DEG,
    NAV_VIDEO_FEEDBACK_NEAR_DISTANCE_M,
    NAV_VIDEO_FEEDBACK_ROI_BOTTOM_FRACTION,
    NAV_VIDEO_FEEDBACK_ROI_TOP_FRACTION,
    NAV_VIDEO_FEEDBACK_SMOOTHING,
    NAV_WAYPOINT_REACHED_DISTANCE_M,
    NAV_WAYPOINT_TIMEOUT_SEC,
    ZERO_IMU_ON_START,
)
from hardware.arduino_io import ArduinoIO
from hardware.camera import Camera
from navigation.path_planner import plan_path
from navigation.point_cloud import PointCloudMap
from robot import CONTROL_INTERVAL_SEC as ROBOT_CONTROL_INTERVAL_SEC
from robot import create_chassis


MAP_PATH = NAV_MAP_PATH
DESTINATION_X_M = NAV_DEFAULT_DESTINATION_X_M
DESTINATION_Y_M = NAV_DEFAULT_DESTINATION_Y_M

ARDUINO_SERIAL_PORT = ARDUINO_PORT
GPS_SERIAL_PORT = GPS_PORT

PLAN_MAX_MAP_DISTANCE_M = NAV_MAX_MAP_DISTANCE_M
PLAN_GRID_RESOLUTION_M = NAV_GRID_RESOLUTION_M
PLAN_CLEARANCE_COST_WEIGHT = NAV_CLEARANCE_COST_WEIGHT
PLAN_ENDPOINT_SNAP_MAX_DISTANCE_M = NAV_ENDPOINT_SNAP_MAX_DISTANCE_M

CONTROL_INTERVAL_SEC = NAV_CONTROL_INTERVAL_SEC or ROBOT_CONTROL_INTERVAL_SEC
WAYPOINT_REACHED_DISTANCE_M = NAV_WAYPOINT_REACHED_DISTANCE_M
DESTINATION_REACHED_DISTANCE_M = NAV_DESTINATION_REACHED_DISTANCE_M
WAYPOINT_TIMEOUT_SEC = NAV_WAYPOINT_TIMEOUT_SEC
ROUTE_TIMEOUT_SEC = NAV_ROUTE_TIMEOUT_SEC
LOG_INTERVAL_SEC = NAV_LOG_INTERVAL_SEC

ENABLE_VIDEO_FEEDBACK = NAV_ENABLE_VIDEO_FEEDBACK
VIDEO_FEEDBACK_ROI_TOP_FRACTION = NAV_VIDEO_FEEDBACK_ROI_TOP_FRACTION
VIDEO_FEEDBACK_ROI_BOTTOM_FRACTION = NAV_VIDEO_FEEDBACK_ROI_BOTTOM_FRACTION
VIDEO_FEEDBACK_MIN_TRAIL_FRACTION = NAV_VIDEO_FEEDBACK_MIN_TRAIL_FRACTION
VIDEO_FEEDBACK_MIN_TRAIL_PIXELS = NAV_VIDEO_FEEDBACK_MIN_TRAIL_PIXELS
VIDEO_FEEDBACK_SMOOTHING = NAV_VIDEO_FEEDBACK_SMOOTHING
VIDEO_FEEDBACK_ANGLE_KP_DEG = NAV_VIDEO_FEEDBACK_ANGLE_KP_DEG
VIDEO_FEEDBACK_NEAR_DISTANCE_M = NAV_VIDEO_FEEDBACK_NEAR_DISTANCE_M
VIDEO_FEEDBACK_FAR_DISTANCE_M = NAV_VIDEO_FEEDBACK_FAR_DISTANCE_M
VIDEO_FEEDBACK_NEAR_ANGLE_CORRECTION_DEG = (
    NAV_VIDEO_FEEDBACK_NEAR_ANGLE_CORRECTION_DEG
)
VIDEO_FEEDBACK_FAR_ANGLE_CORRECTION_DEG = (
    NAV_VIDEO_FEEDBACK_FAR_ANGLE_CORRECTION_DEG
)

TURN_IN_PLACE_ERROR_DEG = 45.0
NAVIGATION_VIDEO_CAPTURE_TIMEOUT_MS = 0

_navigation_camera = None
_navigation_camera_unavailable = False
_navigation_trail_offset = None


class NavigationStopped(RuntimeError):
    """Raised when an active navigation run is stopped by an external command."""


def main():
    point_map = PointCloudMap.load(MAP_PATH)

    with ArduinoIO(port=ARDUINO_SERIAL_PORT) as arduino:
        chassis, gps, _imu = create_chassis(
            arduino,
            zero_imu=ZERO_IMU_ON_START,
            gps_port=GPS_SERIAL_PORT,
            gps_snap_enabled=NAV_GPS_SNAP_ENABLED,
            gps_snap_point_map=point_map if NAV_GPS_SNAP_ENABLED else None,
            gps_snap_max_distance_m=NAV_GPS_SNAP_MAX_DISTANCE_M,
            gps_snap_map_tolerance_m=NAV_GPS_SNAP_MAP_TOLERANCE_M,
        )

        try:
            chassis.update_position()
            start_xy = current_xy(chassis)
            destination_xy = (DESTINATION_X_M, DESTINATION_Y_M)
            path = plan_route(point_map, start_xy, destination_xy)

            print(
                f"Driving {len(path)} waypoint(s) from "
                f"({start_xy[0]:.2f}, {start_xy[1]:.2f}) to "
                f"({destination_xy[0]:.2f}, {destination_xy[1]:.2f})"
            )

            follow_path(chassis, path)
            print("Destination reached.")

        except KeyboardInterrupt:
            print()
            print("Navigation interrupted.")

        finally:
            close_navigation_feedback()
            chassis.stop()
            gps.close()


def plan_route(
    point_map,
    start_xy,
    destination_xy,
    max_map_distance_m=PLAN_MAX_MAP_DISTANCE_M,
    grid_resolution_m=PLAN_GRID_RESOLUTION_M,
    clearance_cost_weight=PLAN_CLEARANCE_COST_WEIGHT,
    endpoint_snap_max_distance_m=PLAN_ENDPOINT_SNAP_MAX_DISTANCE_M,
):
    return plan_path(
        point_map,
        start_xy=start_xy,
        target_xy=destination_xy,
        max_map_distance_m=max_map_distance_m,
        grid_resolution_m=grid_resolution_m,
        clearance_cost_weight=clearance_cost_weight,
        endpoint_snap_max_distance_m=endpoint_snap_max_distance_m,
    )


def follow_path(
    chassis,
    path,
    stop_event=None,
    status_callback=None,
    enable_navigation_feedback=ENABLE_VIDEO_FEEDBACK,
    control_interval_sec=CONTROL_INTERVAL_SEC,
    waypoint_reached_distance_m=WAYPOINT_REACHED_DISTANCE_M,
    destination_reached_distance_m=DESTINATION_REACHED_DISTANCE_M,
    waypoint_timeout_sec=WAYPOINT_TIMEOUT_SEC,
    route_timeout_sec=ROUTE_TIMEOUT_SEC,
    log_interval_sec=LOG_INTERVAL_SEC,
):
    route_started_at = time.monotonic()

    for waypoint_index, waypoint in enumerate(path):
        _raise_if_stopped(chassis, stop_event)
        waypoint_xy = waypoint_to_xy(waypoint)
        is_destination = waypoint_index == len(path) - 1
        tolerance_m = (
            destination_reached_distance_m
            if is_destination
            else waypoint_reached_distance_m
        )

        drive_to_waypoint(
            chassis,
            waypoint_xy,
            waypoint_index,
            len(path),
            tolerance_m,
            route_started_at,
            stop_event=stop_event,
            status_callback=status_callback,
            enable_navigation_feedback=enable_navigation_feedback,
            control_interval_sec=control_interval_sec,
            waypoint_timeout_sec=waypoint_timeout_sec,
            route_timeout_sec=route_timeout_sec,
            log_interval_sec=log_interval_sec,
        )


def drive_to_waypoint(
    chassis,
    waypoint_xy,
    waypoint_index,
    waypoint_count,
    tolerance_m,
    route_started_at,
    stop_event=None,
    status_callback=None,
    enable_navigation_feedback=ENABLE_VIDEO_FEEDBACK,
    control_interval_sec=CONTROL_INTERVAL_SEC,
    waypoint_timeout_sec=WAYPOINT_TIMEOUT_SEC,
    route_timeout_sec=ROUTE_TIMEOUT_SEC,
    log_interval_sec=LOG_INTERVAL_SEC,
):
    waypoint_started_at = time.monotonic()
    last_log_at = 0.0
    last_time = time.monotonic()

    while True:
        _raise_if_stopped(chassis, stop_event)
        now = time.monotonic()
        chassis.update_position()
        x, y, heading = chassis.get_position()
        route_angle_deg, route_error_deg, waypoint_distance_m = _route_to_waypoint(
            x,
            y,
            heading,
            waypoint_xy,
        )

        if waypoint_distance_m <= tolerance_m:
            print(
                f"Reached waypoint {waypoint_index + 1}/{waypoint_count} "
                f"at {waypoint_distance_m:.2f} m"
            )
            return

        trail_offset = (
            _read_navigation_trail_offset()
            if _should_read_navigation_trail(
                enable_navigation_feedback,
                waypoint_distance_m,
            )
            else None
        )
        vision_correction_deg = _vision_angle_correction_deg(
            trail_offset,
            waypoint_distance_m,
        )
        wanted_angle_deg = _normalize_angle(route_angle_deg + vision_correction_deg)
        wanted_error_deg = _angle_error(wanted_angle_deg, heading)
        forward_cm_s = _forward_velocity(chassis, waypoint_distance_m, wanted_error_deg)

        _report_status(
            status_callback,
            chassis,
            waypoint_xy,
            waypoint_index,
            waypoint_count,
            waypoint_distance_m,
            drive_mode="route_video" if trail_offset is not None else "route",
            video_feedback_offset=trail_offset,
            route_error_deg=route_error_deg,
            vision_correction_deg=vision_correction_deg,
            wanted_angle_deg=wanted_angle_deg,
            forward_cm_s=forward_cm_s,
        )

        if waypoint_timeout_sec is not None:
            if now - waypoint_started_at > waypoint_timeout_sec:
                raise TimeoutError(
                    f"Timed out driving to waypoint {waypoint_index + 1}/"
                    f"{waypoint_count}"
                )

        if route_timeout_sec is not None:
            if now - route_started_at > route_timeout_sec:
                raise TimeoutError("Timed out driving the planned route")

        if now - last_log_at >= log_interval_sec:
            trail_text = (
                ""
                if trail_offset is None
                else f" trail={trail_offset:+.2f}"
            )
            print(
                f"Waypoint {waypoint_index + 1}/{waypoint_count}: "
                f"pos=({x:.2f}, {y:.2f}, {heading:.1f} deg) "
                f"target=({waypoint_xy[0]:.2f}, {waypoint_xy[1]:.2f}) "
                f"remaining={waypoint_distance_m:.2f} m "
                f"route={route_angle_deg:.1f} deg "
                f"wanted={wanted_angle_deg:.1f} deg "
                f"vision={vision_correction_deg:+.1f} deg "
                f"forward={forward_cm_s:.1f} cm/s"
                f"{trail_text}"
            )
            last_log_at = now

        chassis.set_wanted_angle(wanted_angle_deg)
        chassis.set_velocity(forward_cm_s)

        current_time = time.monotonic()
        dt = current_time - last_time
        last_time = current_time
        print(f"\nDelta time: {dt}\n")
        time.sleep(max(0.0, control_interval_sec))


def _route_to_waypoint(x, y, heading, waypoint_xy):
    dx = waypoint_xy[0] - x
    dy = waypoint_xy[1] - y
    distance_m = math.hypot(dx, dy)

    if distance_m <= 0.0:
        return heading, 0.0, 0.0

    route_angle_deg = math.degrees(math.atan2(dy, dx))
    route_error_deg = _angle_error(route_angle_deg, heading)
    return route_angle_deg, route_error_deg, distance_m


def _forward_velocity(chassis, distance_m, wanted_error_deg):
    if abs(wanted_error_deg) > TURN_IN_PLACE_ERROR_DEG:
        return 0.0

    return min(
        chassis.max_drive_speed,
        MAX_FORWARD_CM_PER_SEC,
        max(0.0, distance_m) * chassis.drive_kp,
    )


def _read_navigation_trail_offset():
    camera = _get_navigation_camera()

    if camera is None:
        return None

    try:
        frame = camera.capture(timeout_ms=NAVIGATION_VIDEO_CAPTURE_TIMEOUT_MS)
    except Exception as exc:
        _mark_navigation_camera_unavailable(
            f"Video feedback disabled after camera error: {exc}"
        )
        return None

    measured_offset = None if frame is None else _trail_center_offset(frame.trail_mask)
    return _smooth_trail_offset(measured_offset)


def _should_read_navigation_trail(enable_navigation_feedback, waypoint_distance_m):
    if not enable_navigation_feedback:
        return False

    if abs(float(VIDEO_FEEDBACK_ANGLE_KP_DEG)) <= 0.0:
        return False

    return _vision_angle_limit_deg(waypoint_distance_m) > 0.0


def close_navigation_feedback():
    global _navigation_camera
    global _navigation_trail_offset

    if _navigation_camera is not None:
        _navigation_camera.close()
        _navigation_camera = None

    _navigation_trail_offset = None


def _get_navigation_camera():
    global _navigation_camera

    if _navigation_camera_unavailable:
        return None

    if _navigation_camera is None:
        try:
            _navigation_camera = Camera(
                camera_uri=CAMERA_URI,
                input_width=CAMERA_INPUT_WIDTH,
                input_height=CAMERA_INPUT_HEIGHT,
                input_rate=CAMERA_INPUT_RATE,
                input_flip=CAMERA_INPUT_FLIP,
                v4l2_controls=CAMERA_V4L2_CONTROLS,
                auto_exposure=CAMERA_AUTO_EXPOSURE,
            )
        except Exception as exc:
            _mark_navigation_camera_unavailable(
                f"Video feedback unavailable, using route navigation only: {exc}"
            )

    return _navigation_camera


def _mark_navigation_camera_unavailable(message):
    global _navigation_camera
    global _navigation_camera_unavailable

    print(message)
    close_navigation_feedback()
    _navigation_camera = None
    _navigation_camera_unavailable = True


def _smooth_trail_offset(measured_offset):
    global _navigation_trail_offset

    if measured_offset is None:
        _navigation_trail_offset = None
        return None

    measured_offset = _clamp(measured_offset, -1.0, 1.0)

    if _navigation_trail_offset is None:
        _navigation_trail_offset = measured_offset
    else:
        alpha = _clamp(VIDEO_FEEDBACK_SMOOTHING, 0.0, 1.0)
        _navigation_trail_offset += (
            measured_offset - _navigation_trail_offset
        ) * alpha

    return _navigation_trail_offset


def _trail_center_offset(trail_mask):
    if trail_mask is None:
        return None

    height, width = trail_mask.shape[:2]

    if height <= 0 or width <= 0:
        return None

    roi_top = _clamp_int(
        int(height * VIDEO_FEEDBACK_ROI_TOP_FRACTION),
        0,
        height - 1,
    )
    roi_bottom = _clamp_int(
        int(height * VIDEO_FEEDBACK_ROI_BOTTOM_FRACTION),
        roi_top + 1,
        height,
    )
    roi = trail_mask[roi_top:roi_bottom, :]
    trail_pixels = int(roi.sum())
    min_trail_pixels = max(
        VIDEO_FEEDBACK_MIN_TRAIL_PIXELS,
        int(roi.size * VIDEO_FEEDBACK_MIN_TRAIL_FRACTION),
    )

    if trail_pixels < min_trail_pixels:
        return None

    _rows, cols = roi.nonzero()

    if len(cols) == 0:
        return None

    center_x = float(cols.mean())
    image_center_x = (width - 1) / 2.0
    half_width = max(image_center_x, 1.0)
    return _clamp((center_x - image_center_x) / half_width, -1.0, 1.0)


def _vision_angle_correction_deg(trail_offset, distance_m):
    if trail_offset is None:
        return 0.0

    angle_limit_deg = _vision_angle_limit_deg(distance_m)
    raw_correction_deg = -trail_offset * VIDEO_FEEDBACK_ANGLE_KP_DEG
    return _clamp(
        raw_correction_deg,
        -angle_limit_deg,
        angle_limit_deg,
    )


def _vision_angle_limit_deg(distance_m):
    near_distance_m = max(0.0, float(VIDEO_FEEDBACK_NEAR_DISTANCE_M))
    far_distance_m = max(
        near_distance_m,
        float(VIDEO_FEEDBACK_FAR_DISTANCE_M),
    )
    near_angle_deg = max(0.0, float(VIDEO_FEEDBACK_NEAR_ANGLE_CORRECTION_DEG))
    far_angle_deg = max(0.0, float(VIDEO_FEEDBACK_FAR_ANGLE_CORRECTION_DEG))

    if far_distance_m <= near_distance_m:
        return near_angle_deg

    near_fraction = _clamp(
        (far_distance_m - float(distance_m))
        / (far_distance_m - near_distance_m),
        0.0,
        1.0,
    )
    return far_angle_deg + (near_angle_deg - far_angle_deg) * near_fraction


def current_xy(chassis):
    x, y, _heading = chassis.get_position()
    return float(x), float(y)


def waypoint_to_xy(waypoint):
    if isinstance(waypoint, dict):
        return float(waypoint["x"]), float(waypoint["y"])

    return float(waypoint[0]), float(waypoint[1])


def distance_to(chassis, target_xy):
    x, y = current_xy(chassis)
    return math.hypot(target_xy[0] - x, target_xy[1] - y)


def _raise_if_stopped(chassis, stop_event):
    if stop_event is not None and stop_event.is_set():
        chassis.stop()
        raise NavigationStopped("Navigation stopped")


def _report_status(
    status_callback,
    chassis,
    waypoint_xy,
    waypoint_index,
    waypoint_count,
    remaining_m,
    drive_mode=None,
    video_feedback_offset=None,
    route_error_deg=None,
    vision_correction_deg=None,
    wanted_angle_deg=None,
    forward_cm_s=None,
):
    if status_callback is None:
        return

    status_callback(
        {
            "position": chassis.get_position(),
            "waypoint_xy": waypoint_xy,
            "target_xy": waypoint_xy,
            "waypoint_index": waypoint_index,
            "waypoint_count": waypoint_count,
            "remaining_m": remaining_m,
            "drive_mode": drive_mode,
            "video_feedback_offset": video_feedback_offset,
            "route_error_deg": route_error_deg,
            "vision_correction_deg": vision_correction_deg,
            "wanted_angle_deg": wanted_angle_deg,
            "forward_cm_s": forward_cm_s,
        }
    )


def _normalize_angle(angle):
    return float(angle) % 360.0


def _angle_error(target, current):
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


def _clamp(value, low, high):
    return max(low, min(high, float(value)))


def _clamp_int(value, low, high):
    return max(low, min(high, int(value)))


if __name__ == "__main__":
    main()
