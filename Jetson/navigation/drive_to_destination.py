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
    MAX_TURN_DEG_PER_SEC,
    NAV_CLEARANCE_COST_WEIGHT,
    NAV_CONTROL_INTERVAL_SEC,
    NAV_DEFAULT_DESTINATION_X_M,
    NAV_DEFAULT_DESTINATION_Y_M,
    NAV_DESTINATION_REACHED_DISTANCE_M,
    NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
    NAV_ENABLE_VIDEO_FEEDBACK,
    NAV_GRID_RESOLUTION_M,
    NAV_GPS_SNAP_ENABLED,
    NAV_GPS_SNAP_MAP_TOLERANCE_M,
    NAV_GPS_SNAP_MAX_DISTANCE_M,
    NAV_LOG_INTERVAL_SEC,
    NAV_MAP_PATH,
    NAV_MAX_MAP_DISTANCE_M,
    NAV_ROUTE_TIMEOUT_SEC,
    NAV_VIDEO_FEEDBACK_BOTTOM_WEIGHT,
    NAV_VIDEO_FEEDBACK_DEADBAND,
    NAV_VIDEO_FEEDBACK_FAR_CORRECTION_SCALE,
    NAV_VIDEO_FEEDBACK_FAR_DISTANCE_M,
    NAV_VIDEO_FEEDBACK_FAR_ROUTE_ALIGNMENT_DEG,
    NAV_VIDEO_FEEDBACK_FORWARD_CM_PER_SEC,
    NAV_VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC,
    NAV_VIDEO_FEEDBACK_MAX_OFFSET_STEP,
    NAV_VIDEO_FEEDBACK_MAX_ROW_TRAIL_FRACTION,
    NAV_VIDEO_FEEDBACK_MIN_TRAIL_FRACTION,
    NAV_VIDEO_FEEDBACK_MIN_TRAIL_PIXELS,
    NAV_VIDEO_FEEDBACK_MIN_ROW_TRAIL_PIXELS,
    NAV_VIDEO_FEEDBACK_NEAR_DISTANCE_M,
    NAV_VIDEO_FEEDBACK_NO_TRAIL_TIMEOUT_SEC,
    NAV_VIDEO_FEEDBACK_ROUTE_ALIGNMENT_DEG,
    NAV_VIDEO_FEEDBACK_ROUTE_TURN_SCALE,
    NAV_VIDEO_FEEDBACK_ROI_BOTTOM_FRACTION,
    NAV_VIDEO_FEEDBACK_ROI_TOP_FRACTION,
    NAV_VIDEO_FEEDBACK_SMOOTHING,
    NAV_VIDEO_FEEDBACK_TOP_WEIGHT,
    NAV_VIDEO_FEEDBACK_MAX_TURN_STEP_DEG_PER_SEC,
    NAV_VIDEO_FEEDBACK_TURN_SMOOTHING,
    NAV_VIDEO_FEEDBACK_TURN_IN_PLACE_DEG,
    NAV_WAYPOINT_REACHED_DISTANCE_M,
    NAV_WAYPOINT_TIMEOUT_SEC,
    ZERO_IMU_ON_START,
)
from hardware.arduino_io import ArduinoIO
from hardware.camera import acquire_shared_camera
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
VIDEO_FEEDBACK_MAX_ROW_TRAIL_FRACTION = NAV_VIDEO_FEEDBACK_MAX_ROW_TRAIL_FRACTION
VIDEO_FEEDBACK_MIN_ROW_TRAIL_PIXELS = NAV_VIDEO_FEEDBACK_MIN_ROW_TRAIL_PIXELS
VIDEO_FEEDBACK_TOP_WEIGHT = NAV_VIDEO_FEEDBACK_TOP_WEIGHT
VIDEO_FEEDBACK_DEADBAND = NAV_VIDEO_FEEDBACK_DEADBAND
VIDEO_FEEDBACK_SMOOTHING = NAV_VIDEO_FEEDBACK_SMOOTHING
VIDEO_FEEDBACK_MAX_OFFSET_STEP = NAV_VIDEO_FEEDBACK_MAX_OFFSET_STEP
VIDEO_FEEDBACK_BOTTOM_WEIGHT = NAV_VIDEO_FEEDBACK_BOTTOM_WEIGHT
VIDEO_FEEDBACK_FORWARD_CM_PER_SEC = NAV_VIDEO_FEEDBACK_FORWARD_CM_PER_SEC
VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC = NAV_VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC
VIDEO_FEEDBACK_NO_TRAIL_TIMEOUT_SEC = NAV_VIDEO_FEEDBACK_NO_TRAIL_TIMEOUT_SEC
VIDEO_FEEDBACK_NEAR_DISTANCE_M = NAV_VIDEO_FEEDBACK_NEAR_DISTANCE_M
VIDEO_FEEDBACK_FAR_DISTANCE_M = NAV_VIDEO_FEEDBACK_FAR_DISTANCE_M
VIDEO_FEEDBACK_FAR_CORRECTION_SCALE = NAV_VIDEO_FEEDBACK_FAR_CORRECTION_SCALE
VIDEO_FEEDBACK_FAR_ROUTE_ALIGNMENT_DEG = NAV_VIDEO_FEEDBACK_FAR_ROUTE_ALIGNMENT_DEG
VIDEO_FEEDBACK_ROUTE_ALIGNMENT_DEG = NAV_VIDEO_FEEDBACK_ROUTE_ALIGNMENT_DEG
VIDEO_FEEDBACK_ROUTE_TURN_SCALE = NAV_VIDEO_FEEDBACK_ROUTE_TURN_SCALE
VIDEO_FEEDBACK_TURN_SMOOTHING = NAV_VIDEO_FEEDBACK_TURN_SMOOTHING
VIDEO_FEEDBACK_MAX_TURN_STEP_DEG_PER_SEC = NAV_VIDEO_FEEDBACK_MAX_TURN_STEP_DEG_PER_SEC
VIDEO_FEEDBACK_TURN_IN_PLACE_DEG = NAV_VIDEO_FEEDBACK_TURN_IN_PLACE_DEG
NAVIGATION_VIDEO_CAPTURE_TIMEOUT_MS = 0
WAYPOINT_PASS_LATERAL_TOLERANCE_M = 10.0
VIDEO_FEEDBACK_ROUTE_SANITY_DEG = 150.0
VIDEO_FEEDBACK_ROUTE_LOOKAHEAD_M = 14.0
VIDEO_FEEDBACK_ROUTE_BEND_SCAN_M = 30.0
VIDEO_FEEDBACK_ROUTE_BEND_START_M = 14.0
VIDEO_FEEDBACK_ROUTE_BEND_FULL_M = 5.0
VIDEO_FEEDBACK_ROUTE_BEND_MIN_DEG = 3.0
VIDEO_FEEDBACK_ROUTE_BEND_MAJOR_DEG = 35.0
VIDEO_FEEDBACK_ROUTE_BEND_BASE_FRACTION = 0.5
VIDEO_FEEDBACK_ROUTE_REORIENT_DEG = 70.0
VIDEO_FEEDBACK_ROUTE_EXIT_REORIENT_DEG = 60.0
VIDEO_FEEDBACK_ROUTE_INITIAL_ALIGNMENT_DEG = 30.0
VIDEO_FEEDBACK_ROUTE_HINT_GAIN = 0.60
VIDEO_FEEDBACK_ROUTE_REORIENT_KP = 0.90
VIDEO_FEEDBACK_ROUTE_REORIENT_MAX_TURN_DEG_PER_SEC = 28.0
VIDEO_FEEDBACK_MIN_ROI_TOP_FRACTION = 0.30
VIDEO_FEEDBACK_CENTER_ROW_PEAK_FRACTION = 0.55
VIDEO_FEEDBACK_CENTER_ROW_MIN_WEIGHT = 0.45
VIDEO_FEEDBACK_TRAIL_TURN_GAIN = 1.25
VIDEO_FEEDBACK_ROUTE_PIXEL_WEIGHT_MAX = 0.65
VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG = 20.0
VIDEO_FEEDBACK_TURN_OPTION_MAX_BIAS = 0.55
VIDEO_FEEDBACK_CENTERING_PROTECT_OFFSET = 0.35
VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 8.0
VIDEO_FEEDBACK_TURN_OPTION_ROI_TOP_FRACTION = 0.15
VIDEO_FEEDBACK_TURN_OPTION_ROI_BOTTOM_FRACTION = 1.00
VIDEO_FEEDBACK_TURN_OPTION_MID_ROW_FRACTION = 0.50
VIDEO_FEEDBACK_TURN_OPTION_MIN_ROW_WEIGHT = 0.25
VIDEO_FEEDBACK_TURN_OPTION_SIDE_FRACTION = 0.45
VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_PIXELS = 120
VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_FRACTION = 0.02
VIDEO_FEEDBACK_TURN_OPTION_MIN_OFFSET = 0.25

_navigation_camera = None
_navigation_camera_unavailable = False
_navigation_feedback_offset = None
_navigation_feedback_last_signal_at = 0.0
_navigation_turn_command = None


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
    route_origin_xy = current_xy(chassis)
    path_xy = [waypoint_to_xy(waypoint) for waypoint in path]
    route_path_xy = [route_origin_xy] + path_xy
    guidance_uses_planned_path = len(path_xy) >= 2
    guidance_path_xy = path_xy if guidance_uses_planned_path else route_path_xy
    guidance_index_offset = 1 if guidance_uses_planned_path else 0

    for waypoint_index, waypoint in enumerate(path):
        _raise_if_stopped(chassis, stop_event)
        waypoint_xy = path_xy[waypoint_index]
        previous_waypoint_xy = route_path_xy[waypoint_index]
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
            previous_waypoint_xy=previous_waypoint_xy,
            allow_passed_completion=not is_destination,
            route_path_xy=guidance_path_xy,
            route_segment_index=max(0, waypoint_index - guidance_index_offset),
            initial_route_alignment=waypoint_index <= guidance_index_offset,
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
    previous_waypoint_xy=None,
    allow_passed_completion=True,
    route_path_xy=None,
    route_segment_index=0,
    initial_route_alignment=False,
):
    waypoint_started_at = time.monotonic()
    last_log_at = 0.0
    needs_initial_route_alignment = bool(initial_route_alignment)
    _reset_navigation_turn_command()

    while True:
        start_time = time.time()
        _raise_if_stopped(chassis, stop_event)
        now = time.monotonic()
        chassis.update_position()
        waypoint_complete, completion_reason, waypoint_distance_m = (
            _waypoint_completion(
                chassis,
                waypoint_xy,
                tolerance_m,
                previous_waypoint_xy=previous_waypoint_xy,
                allow_passed_completion=allow_passed_completion,
            )
        )

        if waypoint_complete:
            completion_text = (
                "Reached" if completion_reason == "reached" else "Passed"
            )
            print(
                f"{completion_text} waypoint {waypoint_index + 1}/{waypoint_count} "
                f"at {waypoint_distance_m:.2f} m"
            )
            return

        target_xy = waypoint_xy
        route_command = _waypoint_velocity(
            chassis,
            target_xy,
            turn_in_place_deg=(
                VIDEO_FEEDBACK_TURN_IN_PLACE_DEG
                if enable_navigation_feedback
                else 10.0
            ),
        )
        route_guidance = _route_guidance(
            chassis,
            route_path_xy,
            route_segment_index,
        )
        route_error_for_trail = _route_guidance_trail_error(chassis, route_guidance)
        route_alignment_error = _route_guidance_alignment_error(
            chassis,
            route_guidance,
        )
        force_route_alignment = needs_initial_route_alignment

        if (
            needs_initial_route_alignment
            and not _route_initial_alignment_needed(route_alignment_error)
        ):
            needs_initial_route_alignment = False
            force_route_alignment = False

        feedback_offset = None
        feedback_available = False
        if enable_navigation_feedback:
            feedback_offset, feedback_available = _read_navigation_feedback_offset(
                route_error_for_trail
            )
        drive_command, drive_mode = _navigation_drive_command(
            chassis,
            route_command,
            feedback_offset,
            feedback_available,
            route_error_for_trail=route_error_for_trail,
            route_alignment_error=route_alignment_error,
            force_route_alignment=force_route_alignment,
        )
        drive_command = _smooth_navigation_drive_command(
            drive_command,
            drive_mode,
            route_error_for_trail=route_error_for_trail,
        )
        navigation_bend = _route_guidance_status(
            chassis,
            route_guidance,
            drive_command,
            drive_mode,
        )

        _report_status(
            status_callback,
            chassis,
            waypoint_xy,
            target_xy,
            waypoint_index,
            waypoint_count,
            waypoint_distance_m,
            feedback_offset=feedback_offset,
            route_error=(
                route_alignment_error
                if drive_mode == "route_reorient"
                else route_error_for_trail
            ),
            drive_mode=drive_mode,
            navigation_bend=navigation_bend,
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
            x, y, heading = chassis.get_position()
            feedback_text = (
                ""
                if feedback_offset is None
                else f" video_offset={feedback_offset:+.2f}"
            )
            print(
                f"Waypoint {waypoint_index + 1}/{waypoint_count}: "
                f"mode={drive_mode} "
                f"target=({target_xy[0]:.2f}, {target_xy[1]:.2f}) "
                f"pos=({x:.2f}, {y:.2f}, {heading:.1f} deg) "
                f"remaining={waypoint_distance_m:.2f} m"
                f" align_error={route_alignment_error:+.1f} deg"
                f" route_error={route_error_for_trail:+.1f} deg"
                f"{feedback_text}"
            )
            last_log_at = now

        forward_cm_s, turn_deg_s = drive_command
        chassis.set_velocity(forward_cm_s, turn_deg_s)

        if drive_mode == "route_video" and forward_cm_s > 0.0:
            record_heading = getattr(chassis, "record_video_drive_heading", None)

            if callable(record_heading):
                record_heading()

        time.sleep(max(0.0, control_interval_sec))

        dt = time.time() - start_time
        print(f"\nDt: {dt}\n")


def _read_navigation_feedback_offset(route_error_deg=None):
    if not ENABLE_VIDEO_FEEDBACK:
        return None, False

    camera = _get_navigation_camera()

    if camera is None:
        return None, False

    try:
        frame = camera.capture(timeout_ms=NAVIGATION_VIDEO_CAPTURE_TIMEOUT_MS)
    except Exception as exc:
        _mark_navigation_camera_unavailable(
            f"Video feedback disabled after camera error: {exc}"
        )
        return None, False

    measured_offset = (
        None
        if frame is None
        else _trail_center_offset(
            frame.trail_mask,
            route_error_deg=route_error_deg,
        )
    )
    return _update_navigation_feedback_offset(measured_offset), True


def _update_navigation_feedback_offset(measured_offset, now=None):
    global _navigation_feedback_offset
    global _navigation_feedback_last_signal_at

    now = time.monotonic() if now is None else float(now)

    if measured_offset is None:
        _navigation_feedback_offset = _decay_feedback_offset(
            _navigation_feedback_offset
        )
    else:
        _navigation_feedback_offset = _smooth_feedback_offset(
            _navigation_feedback_offset,
            measured_offset,
        )
        _navigation_feedback_last_signal_at = now

    has_recent_signal = (
        _navigation_feedback_offset is not None
        and now - _navigation_feedback_last_signal_at
        <= max(0.0, VIDEO_FEEDBACK_NO_TRAIL_TIMEOUT_SEC)
    )

    return _navigation_feedback_offset if has_recent_signal else None


def _waypoint_velocity(chassis, target_xy, turn_in_place_deg=10.0):
    x, y, heading = chassis.get_position()
    dx = target_xy[0] - x
    dy = target_xy[1] - y
    distance = math.hypot(dx, dy)

    if distance < 0.2:
        return 0.0, 0.0, 0.0, distance

    route_angle = math.degrees(math.atan2(dy, dx))
    route_error = _angle_error(route_angle, heading)

    if abs(route_error) > max(0.0, float(turn_in_place_deg)):
        forward_cm_s = 0.0
    else:
        forward_cm_s = min(
            chassis.max_drive_speed,
            MAX_FORWARD_CM_PER_SEC,
            distance * chassis.drive_kp,
        )

    if abs(route_error) < 1.0:
        turn_deg_s = 0.0
    else:
        turn_deg_s = route_error * chassis.angle_kp

    return (
        forward_cm_s,
        _clamp_turn(chassis, turn_deg_s),
        route_error,
        distance,
    )


def _route_lookahead_error(
    chassis,
    route_path_xy,
    route_segment_index=0,
    lookahead_m=VIDEO_FEEDBACK_ROUTE_LOOKAHEAD_M,
):
    guidance = _route_guidance(
        chassis,
        route_path_xy,
        route_segment_index,
        lookahead_m,
    )

    if guidance is None:
        return 0.0

    return _route_guidance_lookahead_error(guidance)


def _route_alignment_error(
    chassis,
    route_path_xy,
    route_segment_index=0,
    lookahead_m=VIDEO_FEEDBACK_ROUTE_LOOKAHEAD_M,
):
    if route_path_xy is None or len(route_path_xy) < 2:
        return 0.0

    guidance = _route_guidance(
        chassis,
        route_path_xy,
        route_segment_index,
        lookahead_m,
    )

    if guidance is None:
        return 0.0

    return _route_guidance_alignment_error(chassis, guidance)


def _route_guidance(
    chassis,
    route_path_xy,
    route_segment_index=0,
    lookahead_m=VIDEO_FEEDBACK_ROUTE_LOOKAHEAD_M,
):
    if route_path_xy is None or len(route_path_xy) < 2:
        return None

    x, y, _heading = chassis.get_position()
    last_segment_index = len(route_path_xy) - 2
    segment_index = _clamp_int(route_segment_index, 0, last_segment_index)
    current_xy_value = (float(x), float(y))
    fraction = _project_onto_route_segment(
        current_xy_value,
        route_path_xy,
        segment_index,
    )
    current_path_heading = _route_segment_heading(route_path_xy, segment_index)
    base_fraction = _clamp(VIDEO_FEEDBACK_ROUTE_BEND_BASE_FRACTION, 0.0, 1.0)
    guidance = {
        "detected": False,
        "active": False,
        "segment_index": segment_index,
        "segment_fraction": fraction,
        "lookahead_m": max(0.0, float(lookahead_m)),
        "scan_m": max(
            max(0.0, float(lookahead_m)),
            max(0.0, float(VIDEO_FEEDBACK_ROUTE_BEND_SCAN_M)),
        ),
        "base_fraction": base_fraction,
        "current_path_heading_deg": _normalize_angle(current_path_heading),
        "base_heading_deg": _normalize_angle(current_path_heading),
        "active_heading_deg": _normalize_angle(current_path_heading),
        "approach_heading_deg": None,
        "exit_heading_deg": None,
        "bend_angle_deg": 0.0,
        "distance_m": None,
        "strength": 0.0,
        "candidate_error_deg": 0.0,
        "base_error_deg": 0.0,
    }

    if fraction is None:
        return guidance

    bend = _next_route_bend(
        route_path_xy,
        segment_index,
        fraction,
        max(
            max(0.0, float(lookahead_m)),
            max(0.0, float(VIDEO_FEEDBACK_ROUTE_BEND_START_M)),
        ),
        max(0.0, float(VIDEO_FEEDBACK_ROUTE_BEND_MIN_DEG)),
        max(0.0, float(VIDEO_FEEDBACK_ROUTE_BEND_MAJOR_DEG)),
    )

    if bend is None:
        return guidance

    approach_heading = bend["approach_heading_deg"]
    bend_error = bend["bend_angle_deg"]
    base_heading = approach_heading + bend_error * base_fraction
    candidate_error = _angle_error(base_heading, current_path_heading)
    strength = _route_bend_strength(bend["distance_m"])
    active = strength > 0.0
    active_error = candidate_error * strength
    guidance.update(
        {
            "detected": True,
            "active": active,
            "bend_segment_index": bend["segment_index"],
            "approach_heading_deg": _normalize_angle(approach_heading),
            "exit_heading_deg": _normalize_angle(bend["exit_heading_deg"]),
            "bend_angle_deg": bend_error,
            "distance_m": bend["distance_m"],
            "base_heading_deg": _normalize_angle(base_heading),
            "active_heading_deg": _normalize_angle(
                current_path_heading + active_error
            ),
            "strength": strength,
            "candidate_error_deg": candidate_error,
            "base_error_deg": active_error,
        }
    )
    return guidance


def _route_bend_strength(distance_m):
    if distance_m is None:
        return 0.0

    start_m = max(0.0, float(VIDEO_FEEDBACK_ROUTE_BEND_START_M))
    full_m = max(0.0, float(VIDEO_FEEDBACK_ROUTE_BEND_FULL_M))

    if start_m <= full_m:
        return 1.0 if float(distance_m) <= full_m else 0.0

    return _clamp((start_m - float(distance_m)) / (start_m - full_m), 0.0, 1.0)


def _route_guidance_lookahead_error(guidance):
    if guidance is None:
        return 0.0

    return float(guidance.get("base_error_deg") or 0.0)


def _route_guidance_trail_error(chassis, guidance):
    if guidance is None or not guidance.get("active"):
        return 0.0

    bend_angle_deg = float(guidance.get("bend_angle_deg") or 0.0)
    strength = _clamp(guidance.get("strength") or 0.0, 0.0, 1.0)
    planned_error = bend_angle_deg * strength

    if abs(planned_error) < 1.0:
        return 0.0

    _x, _y, heading = chassis.get_position()
    current_path_heading = guidance.get("current_path_heading_deg", heading)
    target_heading = current_path_heading + planned_error
    remaining_error = _angle_error(target_heading, heading)

    if not _same_turn_direction(planned_error, remaining_error):
        return 0.0

    gain = _clamp(VIDEO_FEEDBACK_ROUTE_HINT_GAIN, 0.0, 1.0)
    return remaining_error * gain


def _route_guidance_alignment_error(chassis, guidance):
    if guidance is None:
        return 0.0

    _x, _y, heading = chassis.get_position()
    active_error = _angle_error(guidance.get("active_heading_deg", heading), heading)

    if not guidance.get("active"):
        return active_error

    exit_heading = guidance.get("exit_heading_deg")

    if exit_heading is None:
        return active_error

    exit_error = _angle_error(exit_heading, heading)
    use_exit_error = (
        abs(active_error) >= max(0.0, float(VIDEO_FEEDBACK_ROUTE_EXIT_REORIENT_DEG))
        and abs(exit_error) > abs(active_error)
        and _same_turn_direction(active_error, exit_error)
    )

    return exit_error if use_exit_error else active_error


def _route_guidance_status(chassis, guidance, drive_command=None, drive_mode=None):
    if guidance is None:
        return None

    alignment_error_deg = _route_guidance_alignment_error(chassis, guidance)
    trail_error_deg = _route_guidance_trail_error(chassis, guidance)
    forward_cm_s = None
    turn_deg_s = None

    if drive_command is not None:
        forward_cm_s, turn_deg_s = drive_command

    bend_angle_deg = float(guidance.get("bend_angle_deg") or 0.0)
    base_error_deg = float(guidance.get("base_error_deg") or 0.0)
    return {
        "detected": bool(guidance.get("detected")),
        "active": bool(guidance.get("active")),
        "direction": _signed_turn_direction(bend_angle_deg),
        "bend_angle_deg": bend_angle_deg,
        "distance_m": guidance.get("distance_m"),
        "lookahead_m": guidance.get("lookahead_m"),
        "scan_m": guidance.get("scan_m"),
        "approach_heading_deg": guidance.get("approach_heading_deg"),
        "exit_heading_deg": guidance.get("exit_heading_deg"),
        "base_heading_deg": guidance.get("base_heading_deg"),
        "base_fraction": guidance.get("base_fraction"),
        "strength": guidance.get("strength"),
        "lookahead_error_deg": base_error_deg,
        "trail_hint_error_deg": trail_error_deg,
        "candidate_error_deg": guidance.get("candidate_error_deg"),
        "alignment_error_deg": alignment_error_deg,
        "trail_bias_direction": _signed_turn_direction(trail_error_deg),
        "command_forward_cm_s": forward_cm_s,
        "command_turn_deg_s": turn_deg_s,
        "command_turn_direction": _signed_turn_direction(turn_deg_s),
        "drive_mode": drive_mode,
    }


def _project_onto_route_segment(current_xy_value, route_path_xy, segment_index):
    segment_index = _clamp_int(segment_index, 0, len(route_path_xy) - 2)
    start_xy = route_path_xy[segment_index]
    end_xy = route_path_xy[segment_index + 1]
    segment_x = end_xy[0] - start_xy[0]
    segment_y = end_xy[1] - start_xy[1]
    segment_length_sq = segment_x * segment_x + segment_y * segment_y

    if segment_length_sq <= 0.0:
        return None

    from_start_x = current_xy_value[0] - start_xy[0]
    from_start_y = current_xy_value[1] - start_xy[1]
    return _clamp(
        (from_start_x * segment_x + from_start_y * segment_y)
        / segment_length_sq,
        0.0,
        1.0,
    )


def _next_route_bend(
    route_path_xy,
    segment_index,
    fraction,
    activation_m,
    bend_min_deg,
    major_bend_deg,
):
    last_segment_before_bend = len(route_path_xy) - 3

    if last_segment_before_bend < 0:
        return None

    segment_index = max(0, int(segment_index))

    if segment_index > last_segment_before_bend:
        return None

    fraction = _clamp(fraction, 0.0, 1.0)
    activation_m = max(0.0, float(activation_m))
    scan_m = max(activation_m, max(0.0, float(VIDEO_FEEDBACK_ROUTE_BEND_SCAN_M)))
    bend_min_deg = max(0.0, float(bend_min_deg))
    major_bend_deg = max(bend_min_deg, float(major_bend_deg))
    distance_to_joint_m = _route_segment_length(route_path_xy, segment_index) * (
        1.0 - fraction
    )
    approach_heading = _route_segment_heading(route_path_xy, segment_index)
    group = None
    candidates = []

    for bend_segment_index in range(segment_index, last_segment_before_bend + 1):
        if bend_segment_index > segment_index:
            distance_to_joint_m += _route_segment_length(
                route_path_xy,
                bend_segment_index,
            )

        if distance_to_joint_m > scan_m:
            break

        exit_heading = _route_segment_heading(route_path_xy, bend_segment_index + 1)
        bend_error = _angle_error(exit_heading, approach_heading)

        if abs(bend_error) < bend_min_deg:
            if group is not None:
                _record_major_bend_candidate(group, candidates, major_bend_deg)
                group = None

            continue

        if group is None or bend_error * group["bend_angle_deg"] < 0.0:
            if group is not None:
                _record_major_bend_candidate(group, candidates, major_bend_deg)

            group = {
                "segment_index": bend_segment_index,
                "approach_heading_deg": approach_heading,
                "exit_heading_deg": exit_heading,
                "bend_angle_deg": bend_error,
                "distance_m": distance_to_joint_m,
            }
            continue

        if abs(bend_error) > abs(group["bend_angle_deg"]):
            group["exit_heading_deg"] = exit_heading
            group["bend_angle_deg"] = bend_error

    if group is not None:
        _record_major_bend_candidate(group, candidates, major_bend_deg)

    if not candidates:
        return None

    candidate = _select_major_route_bend(candidates)
    candidate["active"] = candidate["distance_m"] <= activation_m
    return candidate


def _record_major_bend_candidate(group, candidates, major_bend_deg):
    if abs(group["bend_angle_deg"]) < major_bend_deg:
        return

    candidates.append(group.copy())


def _select_major_route_bend(candidates):
    return min(
        candidates,
        key=lambda candidate: (
            -abs(candidate["bend_angle_deg"]),
            candidate["distance_m"],
        ),
    )


def _project_onto_route(current_xy_value, route_path_xy, start_segment_index=0):
    best_projection = None
    best_distance_sq = None
    last_segment_index = len(route_path_xy) - 2
    start_segment_index = _clamp_int(start_segment_index, 0, last_segment_index)

    for segment_index in range(start_segment_index, len(route_path_xy) - 1):
        start_xy = route_path_xy[segment_index]
        end_xy = route_path_xy[segment_index + 1]
        segment_x = end_xy[0] - start_xy[0]
        segment_y = end_xy[1] - start_xy[1]
        segment_length_sq = segment_x * segment_x + segment_y * segment_y

        if segment_length_sq <= 0.0:
            continue

        from_start_x = current_xy_value[0] - start_xy[0]
        from_start_y = current_xy_value[1] - start_xy[1]
        fraction = _clamp(
            (from_start_x * segment_x + from_start_y * segment_y)
            / segment_length_sq,
            0.0,
            1.0,
        )
        projected_x = start_xy[0] + segment_x * fraction
        projected_y = start_xy[1] + segment_y * fraction
        distance_sq = (
            (current_xy_value[0] - projected_x) ** 2
            + (current_xy_value[1] - projected_y) ** 2
        )

        if best_distance_sq is None or distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_projection = (
                segment_index,
                fraction,
                (projected_x, projected_y),
            )

    return best_projection


def _route_segment_length(route_path_xy, segment_index):
    segment_index = _clamp_int(segment_index, 0, len(route_path_xy) - 2)
    start_xy = route_path_xy[segment_index]
    end_xy = route_path_xy[segment_index + 1]
    return math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])


def _route_point_after_projection(
    route_path_xy,
    segment_index,
    fraction,
    distance_m,
):
    remaining_m = max(0.0, float(distance_m))

    for current_segment_index in range(segment_index, len(route_path_xy) - 1):
        start_xy = route_path_xy[current_segment_index]
        end_xy = route_path_xy[current_segment_index + 1]
        segment_length = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])

        if segment_length <= 0.0:
            continue

        start_fraction = fraction if current_segment_index == segment_index else 0.0
        available_m = segment_length * (1.0 - start_fraction)

        if remaining_m <= available_m:
            target_fraction = start_fraction + remaining_m / segment_length
            return (
                (
                    start_xy[0] + (end_xy[0] - start_xy[0]) * target_fraction,
                    start_xy[1] + (end_xy[1] - start_xy[1]) * target_fraction,
                ),
                current_segment_index,
            )

        remaining_m -= available_m

    return route_path_xy[-1], max(0, len(route_path_xy) - 2)


def _route_segment_heading(route_path_xy, segment_index):
    segment_index = _clamp_int(segment_index, 0, len(route_path_xy) - 2)
    start_xy = route_path_xy[segment_index]
    end_xy = route_path_xy[segment_index + 1]
    return math.degrees(math.atan2(end_xy[1] - start_xy[1], end_xy[0] - start_xy[0]))


def _navigation_drive_command(
    chassis,
    route_command,
    trail_offset,
    feedback_available,
    route_error_for_trail=None,
    route_alignment_error=None,
    force_route_alignment=False,
):
    route_forward_cm_s, route_turn_deg_s, route_error_deg, _route_distance_m = (
        route_command
    )
    trail_route_error_deg = (
        route_error_deg
        if route_error_for_trail is None
        else float(route_error_for_trail)
    )
    vision_route_error_deg = (
        None if route_error_for_trail is None else float(route_error_for_trail)
    )
    alignment_error_deg = (
        None if route_alignment_error is None else float(route_alignment_error)
    )

    if (
        feedback_available
        and alignment_error_deg is not None
        and _route_requires_reorientation(
            alignment_error_deg,
            force_route_alignment=force_route_alignment,
        )
    ):
        return _route_reorientation_drive_command(
            chassis,
            alignment_error_deg,
        ), "route_reorient"

    if trail_offset is None:
        if feedback_available:
            return (0.0, 0.0), "no_trail"

        return (route_forward_cm_s, route_turn_deg_s), "waypoint"

    if not _route_sane_for_trail(trail_route_error_deg):
        return (route_forward_cm_s, route_turn_deg_s), "route_select"

    return _trail_follow_drive_command(
        chassis,
        trail_offset,
        route_error_deg=vision_route_error_deg,
    ), "route_video"


def _route_requires_reorientation(
    route_alignment_error_deg,
    force_route_alignment=False,
):
    reorient_deg = max(0.0, float(VIDEO_FEEDBACK_ROUTE_REORIENT_DEG))
    alignment_error = abs(float(route_alignment_error_deg))

    if alignment_error > reorient_deg:
        return True

    return bool(force_route_alignment) and _route_initial_alignment_needed(
        route_alignment_error_deg
    )


def _route_initial_alignment_needed(route_alignment_error_deg):
    initial_alignment_deg = max(
        0.0,
        float(VIDEO_FEEDBACK_ROUTE_INITIAL_ALIGNMENT_DEG),
    )
    return abs(float(route_alignment_error_deg)) > initial_alignment_deg


def _route_reorientation_drive_command(chassis, route_alignment_error_deg):
    turn_deg_s = float(route_alignment_error_deg) * max(
        0.0,
        float(VIDEO_FEEDBACK_ROUTE_REORIENT_KP),
    )
    max_turn_deg_s = min(
        MAX_TURN_DEG_PER_SEC,
        abs(float(getattr(chassis, "max_turn_deg_per_sec", MAX_TURN_DEG_PER_SEC))),
        max(0.0, float(VIDEO_FEEDBACK_ROUTE_REORIENT_MAX_TURN_DEG_PER_SEC)),
    )
    return 0.0, _clamp(turn_deg_s, -max_turn_deg_s, max_turn_deg_s)


def _smooth_navigation_drive_command(
    drive_command,
    drive_mode,
    route_error_for_trail=None,
):
    global _navigation_turn_command

    forward_cm_s, raw_turn_deg_s = drive_command

    if drive_mode != "route_video":
        _navigation_turn_command = raw_turn_deg_s
        return drive_command

    if _navigation_turn_command is None:
        _navigation_turn_command = 0.0

    alpha = _clamp(VIDEO_FEEDBACK_TURN_SMOOTHING, 0.0, 1.0)
    smoothed_turn_deg_s = (
        _navigation_turn_command * (1.0 - alpha) + raw_turn_deg_s * alpha
    )
    max_step = max(0.0, float(VIDEO_FEEDBACK_MAX_TURN_STEP_DEG_PER_SEC))

    if max_step > 0.0:
        smoothed_turn_deg_s = _navigation_turn_command + _clamp(
            smoothed_turn_deg_s - _navigation_turn_command,
            -max_step,
            max_step,
        )

    smoothed_turn_deg_s = _route_limited_vision_turn(
        smoothed_turn_deg_s,
        route_error_for_trail,
    )
    _navigation_turn_command = smoothed_turn_deg_s
    return forward_cm_s, smoothed_turn_deg_s


def _video_route_turn_scale():
    return _clamp(VIDEO_FEEDBACK_ROUTE_TURN_SCALE, 0.0, 1.0)


def _reset_navigation_turn_command():
    global _navigation_turn_command

    _navigation_turn_command = None


def _route_sane_for_trail(route_error_deg):
    return abs(float(route_error_deg)) <= max(
        0.0,
        float(VIDEO_FEEDBACK_ROUTE_SANITY_DEG),
    )


def _trail_follow_drive_command(
    chassis,
    trail_offset,
    route_error_deg=None,
):
    max_forward_cm_s = min(
        abs(float(VIDEO_FEEDBACK_FORWARD_CM_PER_SEC)),
        MAX_FORWARD_CM_PER_SEC,
        abs(float(getattr(chassis, "max_drive_speed", MAX_FORWARD_CM_PER_SEC))),
    )
    forward_cm_s = _clamp(
        VIDEO_FEEDBACK_FORWARD_CM_PER_SEC,
        -max_forward_cm_s,
        max_forward_cm_s,
    )
    max_turn_deg_s = min(
        abs(float(VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC)),
        MAX_TURN_DEG_PER_SEC,
    )
    turn_deg_s = (
        -_clamp(trail_offset, -1.0, 1.0)
        * max_turn_deg_s
        * max(0.0, float(VIDEO_FEEDBACK_TRAIL_TURN_GAIN))
    )
    turn_deg_s = _clamp(turn_deg_s, -max_turn_deg_s, max_turn_deg_s)
    turn_deg_s = _route_limited_vision_turn(
        turn_deg_s,
        route_error_deg,
    )
    return forward_cm_s, _clamp_turn(chassis, turn_deg_s)


def _route_limited_vision_turn(
    turn_deg_s,
    route_error_deg=None,
):
    tolerance_deg = max(
        0.0,
        float(VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG),
    )

    if route_error_deg is None:
        return _clamp(turn_deg_s, -tolerance_deg, tolerance_deg)

    route_error_deg = float(route_error_deg)

    if abs(route_error_deg) < 1.0:
        return _clamp(turn_deg_s, -tolerance_deg, tolerance_deg)

    if route_error_deg > 0.0:
        return _clamp(turn_deg_s, -tolerance_deg, route_error_deg + tolerance_deg)

    return _clamp(turn_deg_s, route_error_deg - tolerance_deg, tolerance_deg)


def _route_aligned_for_video(route_error_deg, distance_m):
    alignment_deg = _video_route_alignment_deg(distance_m)
    return abs(route_error_deg) <= alignment_deg


def _video_route_alignment_deg(distance_m):
    near_alignment_deg = max(0.0, float(VIDEO_FEEDBACK_ROUTE_ALIGNMENT_DEG))
    far_alignment_deg = max(0.0, float(VIDEO_FEEDBACK_FAR_ROUTE_ALIGNMENT_DEG))
    distance_blend = _near_waypoint_blend(distance_m)
    return far_alignment_deg + (
        near_alignment_deg - far_alignment_deg
    ) * distance_blend


def _video_feedback_turn_correction(trail_offset, distance_m):
    max_turn_deg_s = min(
        abs(float(VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC)),
        MAX_TURN_DEG_PER_SEC,
    )
    correction_scale = _video_correction_scale(distance_m)
    return _clamp(
        -trail_offset * max_turn_deg_s * correction_scale,
        -MAX_TURN_DEG_PER_SEC,
        MAX_TURN_DEG_PER_SEC,
    )


def _video_correction_scale(distance_m):
    far_scale = _clamp(VIDEO_FEEDBACK_FAR_CORRECTION_SCALE, 0.0, 1.0)
    distance_blend = _near_waypoint_blend(distance_m)
    return far_scale + (1.0 - far_scale) * distance_blend


def _near_waypoint_blend(distance_m):
    near_m = max(0.0, float(VIDEO_FEEDBACK_NEAR_DISTANCE_M))
    far_m = max(near_m, float(VIDEO_FEEDBACK_FAR_DISTANCE_M))

    if far_m <= near_m:
        return 1.0

    return _clamp((far_m - float(distance_m)) / (far_m - near_m), 0.0, 1.0)


def _route_video_forward(route_forward_cm_s):
    max_forward_cm_s = min(
        abs(float(VIDEO_FEEDBACK_FORWARD_CM_PER_SEC)),
        MAX_FORWARD_CM_PER_SEC,
    )
    return _clamp(route_forward_cm_s, -max_forward_cm_s, max_forward_cm_s)


def _clamp_turn(chassis, turn_deg_s):
    max_turn_deg_s = getattr(chassis, "max_turn_deg_per_sec", None)

    if max_turn_deg_s is None:
        max_turn_deg_s = MAX_TURN_DEG_PER_SEC

    return _clamp(turn_deg_s, -max_turn_deg_s, max_turn_deg_s)


def _clamp_turn_value(turn_deg_s):
    return _clamp(turn_deg_s, -MAX_TURN_DEG_PER_SEC, MAX_TURN_DEG_PER_SEC)


def close_navigation_feedback():
    global _navigation_camera
    global _navigation_feedback_offset
    global _navigation_feedback_last_signal_at
    global _navigation_turn_command

    if _navigation_camera is not None:
        _navigation_camera.close()
        _navigation_camera = None
    _navigation_feedback_offset = None
    _navigation_feedback_last_signal_at = 0.0
    _navigation_turn_command = None


def _get_navigation_camera():
    global _navigation_camera

    if _navigation_camera_unavailable:
        return None

    if _navigation_camera is None:
        try:
            _navigation_camera = acquire_shared_camera(
                camera_uri=CAMERA_URI,
                deepscene_enabled=True,
                input_width=CAMERA_INPUT_WIDTH,
                input_height=CAMERA_INPUT_HEIGHT,
                input_rate=CAMERA_INPUT_RATE,
                input_flip=CAMERA_INPUT_FLIP,
                v4l2_controls=CAMERA_V4L2_CONTROLS,
                auto_exposure=CAMERA_AUTO_EXPOSURE,
            )
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


def _trail_center_offset(trail_mask, route_error_deg=None):
    if trail_mask is None:
        return None

    height, width = trail_mask.shape[:2]

    if height <= 0 or width <= 0:
        return None

    roi_top_fraction = max(
        float(VIDEO_FEEDBACK_ROI_TOP_FRACTION),
        float(VIDEO_FEEDBACK_MIN_ROI_TOP_FRACTION),
    )
    roi_top = _clamp_int(
        int(height * roi_top_fraction),
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

    row_centers = []
    row_weights = []
    min_row_pixels = max(1, int(VIDEO_FEEDBACK_MIN_ROW_TRAIL_PIXELS))
    max_row_fraction = _clamp(VIDEO_FEEDBACK_MAX_ROW_TRAIL_FRACTION, 0.0, 1.0)

    for row_index, row in enumerate(roi):
        row_pixels = int(row.sum())

        if row_pixels < min_row_pixels:
            continue

        row_fraction = row_pixels / width

        if row_fraction > max_row_fraction:
            continue

        cols = row.nonzero()[0]

        if len(cols) == 0:
            continue

        row_centers.append(_trail_row_center(cols, width, route_error_deg))
        row_norm = row_index / max(roi_bottom - roi_top - 1, 1)
        top_weight = 1.0 + (1.0 - row_norm) * VIDEO_FEEDBACK_TOP_WEIGHT
        bottom_weight = 1.0 + row_norm * VIDEO_FEEDBACK_BOTTOM_WEIGHT
        edge_weight = max(1.0 - row_fraction, 0.05)
        center_weight = _trail_center_row_weight(row_norm)
        row_weights.append(
            row_pixels * edge_weight * top_weight * bottom_weight * center_weight
        )

    if not row_centers:
        return None

    total_weight = sum(row_weights)

    if total_weight <= 0:
        return None

    center_x = sum(
        center * weight for center, weight in zip(row_centers, row_weights)
    ) / total_weight
    image_center_x = (width - 1) / 2.0
    half_width = max(image_center_x, 1.0)

    offset = _clamp((center_x - image_center_x) / half_width, -1.0, 1.0)

    if abs(offset) < VIDEO_FEEDBACK_DEADBAND:
        offset = 0.0

    turn_option_offset = _select_route_turn_option_offset(
        roi,
        width,
        route_error_deg,
    )

    if turn_option_offset is not None:
        return _apply_turn_option_bias(
            offset,
            turn_option_offset,
            route_scale=_route_bias_scale(route_error_deg),
        )

    return offset


def _trail_row_center(cols, width, route_error_deg=None):
    route_scale = _route_bias_scale(route_error_deg)

    if route_scale <= 0.0:
        return float(cols.mean())

    direction = -1.0 if float(route_error_deg) > 0.0 else 1.0
    max_weight = max(0.0, float(VIDEO_FEEDBACK_ROUTE_PIXEL_WEIGHT_MAX))
    weighted_sum = 0.0
    total_weight = 0.0

    for col in cols:
        offset = _image_x_to_offset(float(col), width)
        side_alignment = max(direction * offset, 0.0)
        weight = 1.0 + max_weight * route_scale * side_alignment
        weighted_sum += float(col) * weight
        total_weight += weight

    if total_weight <= 0.0:
        return float(cols.mean())

    return weighted_sum / total_weight


def _trail_center_row_weight(row_norm):
    peak = _clamp(VIDEO_FEEDBACK_CENTER_ROW_PEAK_FRACTION, 0.0, 1.0)
    min_weight = _clamp(VIDEO_FEEDBACK_CENTER_ROW_MIN_WEIGHT, 0.05, 1.0)
    span = max(peak, 1.0 - peak, 0.001)
    distance = _clamp(abs(float(row_norm) - peak) / span, 0.0, 1.0)
    return min_weight + (1.0 - min_weight) * (1.0 - distance)


def _route_bias_scale(route_error_deg):
    if route_error_deg is None:
        return 0.0

    route_error = abs(float(route_error_deg))
    min_error = _route_bias_min_error_deg()

    if route_error < min_error:
        return 0.0

    return _clamp(route_error / max(min_error, 1.0), 0.0, 1.0)


def _route_bias_min_error_deg():
    return max(
        0.0,
        float(VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG),
    )


def _apply_turn_option_bias(center_offset, turn_option_offset, route_scale=1.0):
    center_offset = _clamp(center_offset, -1.0, 1.0)
    turn_option_offset = _clamp(turn_option_offset, -1.0, 1.0)

    if turn_option_offset == 0.0:
        return center_offset

    blend = _clamp(VIDEO_FEEDBACK_TURN_OPTION_MAX_BIAS, 0.0, 1.0) * _clamp(
        route_scale,
        0.0,
        1.0,
    )
    return _clamp(
        center_offset + (turn_option_offset - center_offset) * blend,
        -1.0,
        1.0,
    )


def _select_route_turn_option_offset(roi, width, route_error_deg=None):
    if route_error_deg is None:
        return None

    route_error_deg = float(route_error_deg)
    min_route_error_deg = _route_bias_min_error_deg()

    if abs(route_error_deg) < min_route_error_deg:
        return None

    options = _trail_turn_options(roi, width)

    if options["left"] is None or options["right"] is None:
        return None

    if route_error_deg > 0.0 and options["left"] is not None:
        return options["left"]

    if route_error_deg < 0.0 and options["right"] is not None:
        return options["right"]

    return None


def _trail_turn_options(roi, width):
    roi_height = roi.shape[0]

    if roi_height <= 0 or width <= 0:
        return {"left": None, "right": None}

    option_roi_top = _clamp_int(
        int(roi_height * VIDEO_FEEDBACK_TURN_OPTION_ROI_TOP_FRACTION),
        0,
        roi_height - 1,
    )
    option_roi_bottom = _clamp_int(
        int(roi_height * VIDEO_FEEDBACK_TURN_OPTION_ROI_BOTTOM_FRACTION),
        option_roi_top + 1,
        roi_height,
    )
    side_fraction = _clamp(VIDEO_FEEDBACK_TURN_OPTION_SIDE_FRACTION, 0.05, 0.50)
    left_limit = _clamp_int(int(width * side_fraction), 1, width)
    right_start = _clamp_int(int(width * (1.0 - side_fraction)), 0, width - 1)
    option_roi = roi[option_roi_top:option_roi_bottom, :]
    return {
        "left": _trail_option_offset(option_roi[:, :left_limit], 0, width, "left"),
        "right": _trail_option_offset(
            option_roi[:, right_start:],
            right_start,
            width,
            "right",
        ),
    }


def _trail_option_offset(option_roi, col_offset, image_width, side):
    if option_roi.size == 0:
        return None

    min_pixels = max(
        int(VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_PIXELS),
        int(option_roi.size * VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_FRACTION),
    )

    row_centers = []
    row_weights = []
    min_row_pixels = max(1, int(VIDEO_FEEDBACK_MIN_ROW_TRAIL_PIXELS))
    max_row_fraction = _clamp(VIDEO_FEEDBACK_MAX_ROW_TRAIL_FRACTION, 0.0, 1.0)
    height, width = option_roi.shape[:2]

    for row_index, row in enumerate(option_roi):
        row_pixels = int(row.sum())

        if row_pixels < min_row_pixels:
            continue

        row_fraction = row_pixels / width

        if row_fraction > max_row_fraction:
            continue

        cols = row.nonzero()[0]

        if len(cols) == 0:
            continue

        row_centers.append(col_offset + float(cols.mean()))
        row_norm = row_index / max(height - 1, 1)
        row_weights.append(row_pixels * _turn_option_row_weight(row_norm))

    if not row_centers:
        return None

    total_weight = sum(row_weights)

    if total_weight <= 0:
        return None

    if total_weight < min_pixels:
        return None

    center_x = sum(
        center * weight for center, weight in zip(row_centers, row_weights)
    ) / total_weight
    offset = _image_x_to_offset(center_x, image_width)
    min_offset = _clamp(VIDEO_FEEDBACK_TURN_OPTION_MIN_OFFSET, 0.0, 1.0)

    if side == "left":
        return min(offset, -min_offset)

    return max(offset, min_offset)


def _turn_option_row_weight(row_norm):
    peak = _clamp(VIDEO_FEEDBACK_TURN_OPTION_MID_ROW_FRACTION, 0.0, 1.0)
    min_weight = _clamp(VIDEO_FEEDBACK_TURN_OPTION_MIN_ROW_WEIGHT, 0.0, 1.0)
    span = max(peak, 1.0 - peak, 0.001)
    center_weight = 1.0 - abs(float(row_norm) - peak) / span
    return max(min_weight, center_weight * center_weight)


def _image_x_to_offset(center_x, width):
    image_center_x = (width - 1) / 2.0
    half_width = max(image_center_x, 1.0)
    return _clamp((float(center_x) - image_center_x) / half_width, -1.0, 1.0)


def _smooth_feedback_offset(previous_offset, measured_offset):
    measured_offset = _clamp(measured_offset, -1.0, 1.0)

    if previous_offset is None:
        return measured_offset

    max_step = max(0.0, float(VIDEO_FEEDBACK_MAX_OFFSET_STEP))
    delta = _clamp(
        measured_offset - previous_offset,
        -max_step,
        max_step,
    )
    stepped_offset = previous_offset + delta
    smoothing = _clamp(VIDEO_FEEDBACK_SMOOTHING, 0.0, 1.0)
    return previous_offset * (1.0 - smoothing) + stepped_offset * smoothing


def _decay_feedback_offset(previous_offset):
    if previous_offset is None:
        return None

    smoothing = _clamp(VIDEO_FEEDBACK_SMOOTHING, 0.0, 1.0)
    decayed_offset = previous_offset * (1.0 - smoothing)

    if abs(decayed_offset) < VIDEO_FEEDBACK_DEADBAND:
        return None

    return decayed_offset


def _waypoint_completion(
    chassis,
    waypoint_xy,
    tolerance_m,
    previous_waypoint_xy=None,
    allow_passed_completion=True,
):
    x, y, heading = chassis.get_position()
    current_xy_value = (float(x), float(y))
    distance_m = math.hypot(
        waypoint_xy[0] - current_xy_value[0],
        waypoint_xy[1] - current_xy_value[1],
    )

    if distance_m <= tolerance_m:
        return True, "reached", distance_m

    if allow_passed_completion and _waypoint_passed(
        current_xy_value,
        heading,
        waypoint_xy,
        previous_waypoint_xy,
        tolerance_m,
    ):
        return True, "passed", distance_m

    return False, None, distance_m


def _waypoint_passed(
    current_xy_value,
    heading_deg,
    waypoint_xy,
    previous_waypoint_xy=None,
    tolerance_m=0.0,
):
    if previous_waypoint_xy is not None and _waypoint_passed_on_segment(
        current_xy_value,
        previous_waypoint_xy,
        waypoint_xy,
    ):
        return True

    return _waypoint_is_behind_heading(
        current_xy_value,
        heading_deg,
        waypoint_xy,
        tolerance_m,
    )


def _waypoint_passed_on_segment(
    current_xy_value,
    previous_waypoint_xy,
    waypoint_xy,
):
    segment_x = waypoint_xy[0] - previous_waypoint_xy[0]
    segment_y = waypoint_xy[1] - previous_waypoint_xy[1]
    segment_length_sq = segment_x * segment_x + segment_y * segment_y

    if segment_length_sq <= 0.0:
        return False

    from_previous_x = current_xy_value[0] - previous_waypoint_xy[0]
    from_previous_y = current_xy_value[1] - previous_waypoint_xy[1]
    along_fraction = (
        from_previous_x * segment_x + from_previous_y * segment_y
    ) / segment_length_sq

    if along_fraction < 1.0:
        return False

    segment_length = math.sqrt(segment_length_sq)
    lateral_m = abs(
        segment_x * from_previous_y - segment_y * from_previous_x
    ) / segment_length
    return lateral_m <= WAYPOINT_PASS_LATERAL_TOLERANCE_M


def _waypoint_is_behind_heading(
    current_xy_value,
    heading_deg,
    waypoint_xy,
    tolerance_m=0.0,
):
    dx = waypoint_xy[0] - current_xy_value[0]
    dy = waypoint_xy[1] - current_xy_value[1]
    heading_rad = math.radians(heading_deg)
    forward_x = math.cos(heading_rad)
    forward_y = math.sin(heading_rad)
    forward_m = dx * forward_x + dy * forward_y
    lateral_m = abs(forward_x * dy - forward_y * dx)
    return (
        forward_m < -max(0.0, float(tolerance_m))
        and lateral_m <= WAYPOINT_PASS_LATERAL_TOLERANCE_M
    )


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
    target_xy,
    waypoint_index,
    waypoint_count,
    remaining_m,
    feedback_offset=None,
    route_error=None,
    drive_mode=None,
    navigation_bend=None,
):
    if status_callback is None:
        return

    status_callback(
        {
            "position": chassis.get_position(),
            "waypoint_xy": waypoint_xy,
            "target_xy": target_xy,
            "waypoint_index": waypoint_index,
            "waypoint_count": waypoint_count,
            "remaining_m": remaining_m,
            "video_feedback_offset": feedback_offset,
            "route_error_deg": route_error,
            "drive_mode": drive_mode,
            "navigation_bend": navigation_bend,
        }
    )


def _clamp(value, low, high):
    return max(low, min(high, float(value)))


def _clamp_int(value, low, high):
    return max(low, min(high, int(value)))


def _normalize_angle(angle):
    return float(angle) % 360.0


def _angle_error(target, current):
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


def _signed_turn_direction(value):
    if value is None:
        return "none"

    value = float(value)

    if abs(value) < 1.0:
        return "none"

    return "left" if value > 0.0 else "right"


def _same_turn_direction(first, second):
    return (
        abs(float(first)) >= 1.0
        and abs(float(second)) >= 1.0
        and float(first) * float(second) > 0.0
    )


if __name__ == "__main__":
    main()
