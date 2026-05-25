import math
import sys
import time
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parents[1]
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from config import (
    ARDUINO_PORT,
    NAV_CLEARANCE_COST_WEIGHT,
    NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
    NAV_GRID_RESOLUTION_M,
    NAV_MAX_MAP_DISTANCE_M,
)
from hardware.arduino_io import ArduinoIO
from hardware.camera import Camera
from navigation.path_planner import plan_path
from navigation.point_cloud import PointCloudMap
from robot import CONTROL_INTERVAL_SEC as ROBOT_CONTROL_INTERVAL_SEC
from robot import create_chassis


MAP_PATH = JETSON_ROOT / "maps" / "house_map.json"
DESTINATION_X_M = 12777.302
DESTINATION_Y_M = 4029.061

ARDUINO_SERIAL_PORT = ARDUINO_PORT
ZERO_IMU_ON_START = True

PLAN_MAX_MAP_DISTANCE_M = NAV_MAX_MAP_DISTANCE_M
PLAN_GRID_RESOLUTION_M = NAV_GRID_RESOLUTION_M
PLAN_CLEARANCE_COST_WEIGHT = NAV_CLEARANCE_COST_WEIGHT
PLAN_ENDPOINT_SNAP_MAX_DISTANCE_M = NAV_ENDPOINT_SNAP_MAX_DISTANCE_M

CONTROL_INTERVAL_SEC = ROBOT_CONTROL_INTERVAL_SEC
WAYPOINT_REACHED_DISTANCE_M = 0.35
DESTINATION_REACHED_DISTANCE_M = 0.30
WAYPOINT_TIMEOUT_SEC = 45.0
ROUTE_TIMEOUT_SEC = None
LOG_INTERVAL_SEC = 1.0

ENABLE_VIDEO_FEEDBACK = True
VIDEO_FEEDBACK_LOOKAHEAD_M = 1.2
VIDEO_FEEDBACK_MAX_LATERAL_CORRECTION_M = 0.55
VIDEO_FEEDBACK_ROI_TOP_FRACTION = 0.45
VIDEO_FEEDBACK_ROI_BOTTOM_FRACTION = 0.95
VIDEO_FEEDBACK_MIN_TRAIL_FRACTION = 0.01
VIDEO_FEEDBACK_MIN_TRAIL_PIXELS = 80
VIDEO_FEEDBACK_BOTTOM_WEIGHT = 2.0

_navigation_camera = None
_navigation_camera_unavailable = False


def main():
    point_map = PointCloudMap.load(MAP_PATH)

    with ArduinoIO(port=ARDUINO_SERIAL_PORT) as arduino:
        chassis, gps, _imu = create_chassis(arduino, zero_imu=ZERO_IMU_ON_START)

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


def plan_route(point_map, start_xy, destination_xy):
    return plan_path(
        point_map,
        start_xy=start_xy,
        target_xy=destination_xy,
        max_map_distance_m=PLAN_MAX_MAP_DISTANCE_M,
        grid_resolution_m=PLAN_GRID_RESOLUTION_M,
        clearance_cost_weight=PLAN_CLEARANCE_COST_WEIGHT,
        endpoint_snap_max_distance_m=PLAN_ENDPOINT_SNAP_MAX_DISTANCE_M,
    )


def follow_path(chassis, path):
    route_started_at = time.monotonic()

    for waypoint_index, waypoint in enumerate(path):
        waypoint_xy = waypoint_to_xy(waypoint)
        is_destination = waypoint_index == len(path) - 1
        tolerance_m = (
            DESTINATION_REACHED_DISTANCE_M
            if is_destination
            else WAYPOINT_REACHED_DISTANCE_M
        )

        drive_to_waypoint(
            chassis,
            waypoint_xy,
            waypoint_index,
            len(path),
            tolerance_m,
            route_started_at,
        )


def drive_to_waypoint(
    chassis,
    waypoint_xy,
    waypoint_index,
    waypoint_count,
    tolerance_m,
    route_started_at,
):
    waypoint_started_at = time.monotonic()
    last_log_at = 0.0

    while True:
        now = time.monotonic()
        chassis.update_position()
        waypoint_distance_m = distance_to(chassis, waypoint_xy)

        if waypoint_distance_m <= tolerance_m:
            print(
                f"Reached waypoint {waypoint_index + 1}/{waypoint_count} "
                f"at {waypoint_distance_m:.2f} m"
            )
            return

        target_xy = apply_navigation_feedback(chassis, waypoint_xy, waypoint_index)

        if WAYPOINT_TIMEOUT_SEC is not None:
            if now - waypoint_started_at > WAYPOINT_TIMEOUT_SEC:
                raise TimeoutError(
                    f"Timed out driving to waypoint {waypoint_index + 1}/"
                    f"{waypoint_count}"
                )

        if ROUTE_TIMEOUT_SEC is not None:
            if now - route_started_at > ROUTE_TIMEOUT_SEC:
                raise TimeoutError("Timed out driving the planned route")

        if now - last_log_at >= LOG_INTERVAL_SEC:
            x, y, heading = chassis.get_position()
            print(
                f"Waypoint {waypoint_index + 1}/{waypoint_count}: "
                f"target=({target_xy[0]:.2f}, {target_xy[1]:.2f}) "
                f"pos=({x:.2f}, {y:.2f}, {heading:.1f} deg) "
                f"remaining={waypoint_distance_m:.2f} m"
            )
            last_log_at = now

        chassis.set_wanted_position(target_xy)
        chassis.drive_to_wanted_position()
        time.sleep(CONTROL_INTERVAL_SEC)


def apply_navigation_feedback(chassis, waypoint_xy, waypoint_index):
    if not ENABLE_VIDEO_FEEDBACK:
        return waypoint_xy

    camera = _get_navigation_camera()

    if camera is None:
        return waypoint_xy

    try:
        frame = camera.capture()
    except Exception as exc:
        _mark_navigation_camera_unavailable(
            f"Video feedback disabled after camera error: {exc}"
        )
        return waypoint_xy

    if frame is None:
        return waypoint_xy

    trail_offset = _trail_center_offset(frame.trail_mask)

    if trail_offset is None:
        return waypoint_xy

    return _feedback_target(chassis, waypoint_xy, trail_offset)


def close_navigation_feedback():
    global _navigation_camera

    if _navigation_camera is not None:
        _navigation_camera.close()
        _navigation_camera = None


def _get_navigation_camera():
    global _navigation_camera

    if _navigation_camera_unavailable:
        return None

    if _navigation_camera is None:
        try:
            _navigation_camera = Camera()
        except Exception as exc:
            _mark_navigation_camera_unavailable(
                f"Video feedback unavailable, using waypoint navigation only: {exc}"
            )

    return _navigation_camera


def _mark_navigation_camera_unavailable(message):
    global _navigation_camera
    global _navigation_camera_unavailable

    print(message)
    close_navigation_feedback()
    _navigation_camera = None
    _navigation_camera_unavailable = True


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

    rows, cols = roi.nonzero()

    if len(cols) == 0:
        return None

    roi_height = max(roi_bottom - roi_top - 1, 1)
    weights = 1.0 + (rows / roi_height) * VIDEO_FEEDBACK_BOTTOM_WEIGHT
    center_x = float((cols * weights).sum() / weights.sum())
    image_center_x = (width - 1) / 2.0
    half_width = max(image_center_x, 1.0)

    return _clamp((center_x - image_center_x) / half_width, -1.0, 1.0)


def _feedback_target(chassis, waypoint_xy, trail_offset):
    x, y, heading = chassis.get_position()
    waypoint_dx = waypoint_xy[0] - x
    waypoint_dy = waypoint_xy[1] - y
    waypoint_distance_m = math.hypot(waypoint_dx, waypoint_dy)

    if waypoint_distance_m <= 0:
        return waypoint_xy

    lookahead_m = min(VIDEO_FEEDBACK_LOOKAHEAD_M, waypoint_distance_m)
    route_unit_x = waypoint_dx / waypoint_distance_m
    route_unit_y = waypoint_dy / waypoint_distance_m

    base_x = x + route_unit_x * lookahead_m
    base_y = y + route_unit_y * lookahead_m

    heading_rad = math.radians(heading)
    right_x = math.sin(heading_rad)
    right_y = -math.cos(heading_rad)
    correction_m = _clamp(
        trail_offset * VIDEO_FEEDBACK_MAX_LATERAL_CORRECTION_M,
        -VIDEO_FEEDBACK_MAX_LATERAL_CORRECTION_M,
        VIDEO_FEEDBACK_MAX_LATERAL_CORRECTION_M,
    )

    return base_x + right_x * correction_m, base_y + right_y * correction_m


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


def _clamp(value, low, high):
    return max(low, min(high, float(value)))


def _clamp_int(value, low, high):
    return max(low, min(high, int(value)))


if __name__ == "__main__":
    main()
