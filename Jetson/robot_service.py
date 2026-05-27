import copy
import sys
import threading
import time
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parent
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from config import (
    ARDUINO_PORT,
    CAMERA_DEBUG_STREAM_ENABLED,
    CAMERA_DEEPSCENE_ENABLED,
    CAMERA_INPUT_FLIP,
    CAMERA_INPUT_HEIGHT,
    CAMERA_INPUT_RATE,
    CAMERA_INPUT_WIDTH,
    CAMERA_URI,
    CAMERA_V4L2_CONTROLS,
    GPS_ORIGIN,
    GPS_PORT,
    MAX_FORWARD_CM_PER_SEC,
    MAX_TURN_DEG_PER_SEC,
    NAV_CLEARANCE_COST_WEIGHT,
    NAV_CONTROL_INTERVAL_SEC,
    NAV_DESTINATION_REACHED_DISTANCE_M,
    NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
    NAV_ENABLE_VIDEO_FEEDBACK,
    NAV_GRID_RESOLUTION_M,
    NAV_LOG_INTERVAL_SEC,
    NAV_MAP_PATH,
    NAV_MAX_MAP_DISTANCE_M,
    NAV_ROUTE_TIMEOUT_SEC,
    NAV_WAYPOINT_REACHED_DISTANCE_M,
    NAV_WAYPOINT_TIMEOUT_SEC,
    ZERO_IMU_ON_START,
)
from hardware.arduino_io import ArduinoIO
from hardware.camera import Camera
from navigation.coordinates import latlon_to_xy, path_xy_to_latlon, xy_to_latlon
from navigation.drive_to_destination import (
    NavigationStopped,
    close_navigation_feedback,
    follow_path,
    plan_route,
)
from navigation.point_cloud import PointCloudMap
from robot import create_chassis


STATE_IDLE = "idle"
STATE_PLANNING = "planning"
STATE_DRIVING = "driving"
STATE_ARRIVED = "arrived"
STATE_STOPPED = "stopped"
STATE_ERROR = "error"
STATE_MANUAL_CONTROL = "manual_control"

MANUAL_CONTROL_TIMEOUT_SEC = 0.45
MANUAL_CONTROL_MIN_TIMEOUT_SEC = 0.1
MANUAL_CONTROL_MAX_TIMEOUT_SEC = 1.0
MANUAL_CONTROL_HARDWARE_WAIT_SEC = 1.5
MANUAL_CONTROL_STOP = "stop"
MANUAL_CONTROL_VELOCITIES = {
    "forward": (MAX_FORWARD_CM_PER_SEC, 0.0),
    "backward": (-MAX_FORWARD_CM_PER_SEC, 0.0),
    "left": (0.0, MAX_TURN_DEG_PER_SEC),
    "right": (0.0, -MAX_TURN_DEG_PER_SEC),
}
MANUAL_CONTROL_LABELS = {
    "forward": "forward",
    "backward": "backward",
    "left": "turn left",
    "right": "turn right",
}


class RobotServiceError(RuntimeError):
    """Raised when the robot cannot accept a command."""


class DeliveryRobotService:
    """Owns robot hardware, web-submitted delivery jobs, and debug streams."""

    def __init__(self):
        self.lock = threading.RLock()
        self.hardware_lock = threading.RLock()
        self.events = []
        self.stop_event = threading.Event()
        self.job_thread = None

        self.arduino = None
        self.chassis = None
        self.gps = None
        self.imu = None
        self._debug_camera_streamer = None
        self._debug_camera_lock = threading.Lock()
        self.manual_stop_timer = None
        self.manual_control_expires_at = None
        self.manual_direction = None
        self.manual_control_sequences = {}

        self.status_data = {
            "state": STATE_IDLE,
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
            "remaining_m": None,
            "manual_direction": None,
            "last_error": None,
            "updated_at": time.time(),
        }

    def status(self):
        self.refresh_position()

        with self.lock:
            return copy.deepcopy(self.status_data)

    def debug_status(self):
        data = self.status()

        with self.lock:
            data["controller"] = self.__class__.__name__
            data["events"] = list(self.events[-80:])
            data["hardware_initialized"] = self.chassis is not None

            if self._debug_camera_streamer is not None:
                data["debug_camera"] = self._debug_camera_streamer.status()

        return data

    def submit_job(self, stop):
        with self.lock:
            if self.status_data["state"] in {STATE_PLANNING, STATE_DRIVING}:
                raise RobotServiceError("Robot is busy")

            self._cancel_manual_stop()
            self._stop_chassis()
            self.manual_direction = None
            self.stop_event.clear()
            self._set_status(
                state=STATE_PLANNING,
                message="Planning route",
                destination=_public_stop(stop),
                planned_path=[],
                active_waypoint_index=None,
                progress=0.0,
                remaining_m=None,
                manual_direction=None,
                last_error=None,
            )

            self.job_thread = threading.Thread(
                target=self._run_job,
                args=(copy.deepcopy(stop),),
                daemon=True,
            )
            self.job_thread.start()

        return self.status()

    def stop_job(self, reason="Stopped by user"):
        self.stop_event.set()
        self._cancel_manual_stop()
        self._stop_chassis()
        self._set_status(
            state=STATE_STOPPED,
            message=reason,
            manual_direction=None,
        )
        self._log_event(reason)

    def shutdown(self):
        self.stop_job("Server shutting down")
        self._close_hardware()

    def camera_stream(self, mode):
        if mode == "camera" and not CAMERA_DEBUG_STREAM_ENABLED:
            return None

        if mode == "deepscene" and not CAMERA_DEEPSCENE_ENABLED:
            return None

        return lambda: self._camera_frames(mode)

    def manual_control(
        self,
        direction,
        duration_sec=None,
        sequence=None,
        client_id=None,
    ):
        direction = _normalize_manual_direction(direction)
        duration_sec = _manual_control_duration(duration_sec)
        sequence = _manual_control_sequence(sequence)

        with self.lock:
            if sequence is not None:
                client_id = _manual_control_client_id(client_id)
                last_sequence = self.manual_control_sequences.get(client_id, 0)

                if sequence <= last_sequence:
                    return copy.deepcopy(self.status_data)

                self.manual_control_sequences[client_id] = sequence

            active_navigation = self.status_data["state"] in {
                STATE_PLANNING,
                STATE_DRIVING,
            }

        if active_navigation:
            self.stop_event.set()

        acquired = self.hardware_lock.acquire(
            timeout=MANUAL_CONTROL_HARDWARE_WAIT_SEC
        )

        if not acquired:
            raise RobotServiceError("Robot hardware is busy")

        try:
            self._ensure_hardware()

            if direction == MANUAL_CONTROL_STOP:
                self._cancel_manual_stop()
                self._stop_chassis()
                self.manual_direction = None
                self._set_status(
                    state=STATE_STOPPED,
                    message="Manual control stopped",
                    manual_direction=None,
                )
                self._log_event("Manual control stopped")
                return self.status()

            forward, turn = MANUAL_CONTROL_VELOCITIES[direction]
            self.chassis.set_velocity(forward, turn)
            self._schedule_manual_stop(duration_sec)

            label = MANUAL_CONTROL_LABELS[direction]
            log_changed = self.manual_direction != direction
            self.manual_direction = direction
            self._set_status(
                state=STATE_MANUAL_CONTROL,
                message=f"Manual control: {label}",
                destination=None,
                planned_path=[],
                active_waypoint_index=None,
                progress=0.0,
                remaining_m=None,
                manual_direction=direction,
                last_error=None,
            )

            if log_changed:
                self._log_event(f"Manual control: {label}")

            return self.status()

        finally:
            self.hardware_lock.release()

    def refresh_position(self):
        if self.status_data["state"] in {
            STATE_PLANNING,
            STATE_DRIVING,
            STATE_MANUAL_CONTROL,
        }:
            return

        if not self.hardware_lock.acquire(blocking=False):
            return

        try:
            self._ensure_hardware()
            self.chassis.update_position()
            self._set_robot_pose(*self.chassis.get_position())
        except Exception as exc:
            self._set_error(f"Position update failed: {exc}", change_state=False)
        finally:
            self.hardware_lock.release()

    def _run_job(self, stop):
        try:
            with self.hardware_lock:
                self._ensure_hardware()
                point_map = PointCloudMap.load(NAV_MAP_PATH)
                self.chassis.update_position()
                self._set_robot_pose(*self.chassis.get_position())
                start_xy = self._current_xy()
                destination_xy = self._stop_to_xy(stop)

                path = plan_route(
                    point_map,
                    start_xy,
                    destination_xy,
                    max_map_distance_m=NAV_MAX_MAP_DISTANCE_M,
                    grid_resolution_m=NAV_GRID_RESOLUTION_M,
                    clearance_cost_weight=NAV_CLEARANCE_COST_WEIGHT,
                    endpoint_snap_max_distance_m=NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
                )
                planned_path = path_xy_to_latlon(path, GPS_ORIGIN)

                self._set_status(
                    state=STATE_DRIVING,
                    message=f"Driving to {stop['name']}",
                    planned_path=planned_path,
                )
                self._log_event(f"Started delivery to {stop['name']}")

                follow_path(
                    self.chassis,
                    path,
                    stop_event=self.stop_event,
                    status_callback=self._handle_navigation_status,
                    enable_navigation_feedback=NAV_ENABLE_VIDEO_FEEDBACK,
                    control_interval_sec=NAV_CONTROL_INTERVAL_SEC,
                    waypoint_reached_distance_m=NAV_WAYPOINT_REACHED_DISTANCE_M,
                    destination_reached_distance_m=NAV_DESTINATION_REACHED_DISTANCE_M,
                    waypoint_timeout_sec=NAV_WAYPOINT_TIMEOUT_SEC,
                    route_timeout_sec=NAV_ROUTE_TIMEOUT_SEC,
                    log_interval_sec=NAV_LOG_INTERVAL_SEC,
                )

                if self.stop_event.is_set():
                    return

                self._stop_chassis()
                self._set_status(
                    state=STATE_ARRIVED,
                    message=f"Arrived at {stop['name']}",
                    progress=1.0,
                    remaining_m=0.0,
                )
                self._log_event(f"Arrived at {stop['name']}")

        except NavigationStopped:
            self._set_status(state=STATE_STOPPED, message="Stopped")
            self._log_event("Navigation stopped")

        except Exception as exc:
            self._stop_chassis()
            self._set_error(f"Navigation error: {exc}")

    def _handle_navigation_status(self, update):
        position = update.get("position")

        if position is not None:
            self._set_robot_pose(*position)

        waypoint_count = max(int(update.get("waypoint_count") or 1), 1)
        waypoint_index = int(update.get("waypoint_index") or 0)
        progress = min(waypoint_index / waypoint_count, 0.999)
        self._set_status(
            active_waypoint_index=waypoint_index,
            progress=round(progress, 3),
            remaining_m=update.get("remaining_m"),
        )

    def _ensure_hardware(self):
        if self.chassis is not None:
            return

        self.arduino = ArduinoIO(port=ARDUINO_PORT)
        self.chassis, self.gps, self.imu = create_chassis(
            self.arduino,
            zero_imu=ZERO_IMU_ON_START,
            gps_port=GPS_PORT,
            gps_origin=GPS_ORIGIN,
        )
        self._log_event("Robot hardware initialized")

    def _close_hardware(self):
        try:
            close_navigation_feedback()
        except Exception:
            pass

        if self._debug_camera_streamer is not None:
            self._debug_camera_streamer.close()
            self._debug_camera_streamer = None

        if self.gps is not None:
            try:
                self.gps.close()
            except Exception:
                pass

        if self.arduino is not None:
            try:
                self.arduino.close()
            except Exception:
                pass

        self.arduino = None
        self.chassis = None
        self.gps = None
        self.imu = None

    def _stop_chassis(self):
        if self.chassis is None:
            return

        try:
            self.chassis.stop()
        except Exception as exc:
            self._log_event(f"Motor stop failed: {exc}")

    def _set_robot_pose(self, x_m, y_m, heading_deg):
        lat, lon = xy_to_latlon(x_m, y_m, GPS_ORIGIN)

        with self.lock:
            robot = self.status_data["robot"]
            robot["x_m"] = float(x_m)
            robot["y_m"] = float(y_m)
            robot["lat"] = lat
            robot["lon"] = lon
            robot["heading_deg"] = float(heading_deg)
            self.status_data["updated_at"] = time.time()

    def _set_status(self, **changes):
        with self.lock:
            self.status_data.update(changes)
            self.status_data["updated_at"] = time.time()

    def _set_error(self, message, change_state=True):
        self._stop_chassis()
        self._log_event(message)
        changes = {
            "message": message,
            "last_error": message,
        }

        if change_state:
            changes["state"] = STATE_ERROR

        self._set_status(**changes)

    def _log_event(self, message):
        with self.lock:
            self.events.append(
                {
                    "time": time.time(),
                    "message": str(message),
                }
            )
            self.events = self.events[-200:]

    def _schedule_manual_stop(self, duration_sec):
        expires_at = time.monotonic() + duration_sec

        with self.lock:
            if self.manual_stop_timer is not None:
                self.manual_stop_timer.cancel()

            self.manual_control_expires_at = expires_at
            self.manual_stop_timer = threading.Timer(
                duration_sec,
                self._manual_control_timeout,
                args=(expires_at,),
            )
            self.manual_stop_timer.daemon = True
            self.manual_stop_timer.start()

    def _cancel_manual_stop(self):
        with self.lock:
            timer = self.manual_stop_timer
            self.manual_stop_timer = None
            self.manual_control_expires_at = None

        if timer is not None:
            timer.cancel()

    def _manual_control_timeout(self, expected_expires_at):
        with self.lock:
            if self.manual_control_expires_at != expected_expires_at:
                return

            if self.status_data["state"] != STATE_MANUAL_CONTROL:
                return

            self.manual_stop_timer = None
            self.manual_control_expires_at = None
            self.manual_direction = None

        self._stop_chassis()
        self._set_status(
            state=STATE_STOPPED,
            message="Manual control timed out",
            manual_direction=None,
        )
        self._log_event("Manual control timed out")

    def _current_xy(self):
        x, y, _heading = self.chassis.get_position()
        return float(x), float(y)

    def _stop_to_xy(self, stop):
        if "x_m" in stop and "y_m" in stop:
            return float(stop["x_m"]), float(stop["y_m"])

        return latlon_to_xy(stop["lat"], stop["lon"], GPS_ORIGIN)

    def _camera_frames(self, mode):
        streamer = self._get_debug_camera_streamer()
        return streamer.frames(mode)

    def _get_debug_camera_streamer(self):
        with self._debug_camera_lock:
            if self._debug_camera_streamer is not None:
                if self._debug_camera_streamer.deepscene_enabled != CAMERA_DEEPSCENE_ENABLED:
                    self._debug_camera_streamer.close()
                    self._debug_camera_streamer = None

            if self._debug_camera_streamer is None:
                self._debug_camera_streamer = DebugCameraStreamer(
                    deepscene_enabled=CAMERA_DEEPSCENE_ENABLED,
                    log_event=self._log_event,
                )

            return self._debug_camera_streamer


class DebugCameraStreamer:
    def __init__(self, deepscene_enabled=False, log_event=None):
        self.deepscene_enabled = bool(deepscene_enabled)
        self.log_event = log_event
        self.stop_event = threading.Event()
        self.condition = threading.Condition()
        self.latest = {
            "camera": None,
            "deepscene": None,
        }
        self.frame_ids = {
            "camera": 0,
            "deepscene": 0,
        }
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def status(self):
        return {
            "deepscene_enabled": self.deepscene_enabled,
            "error": self.error,
            "camera_frames": self.frame_ids["camera"],
            "deepscene_frames": self.frame_ids["deepscene"],
        }

    def close(self):
        self.stop_event.set()

        with self.condition:
            self.condition.notify_all()

    def frames(self, mode):
        last_frame_id = 0

        while not self.stop_event.is_set():
            with self.condition:
                self.condition.wait_for(
                    lambda: (
                        self.stop_event.is_set()
                        or self.frame_ids.get(mode, 0) != last_frame_id
                    ),
                    timeout=2.0,
                )

                if self.stop_event.is_set():
                    return

                data = self.latest.get(mode)
                last_frame_id = self.frame_ids.get(mode, last_frame_id)

            if data is None:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + data
                + b"\r\n"
            )

    def _run(self):
        camera = None

        try:
            import cv2

            camera = Camera(
                camera_uri=CAMERA_URI,
                deepscene_enabled=self.deepscene_enabled,
                input_width=CAMERA_INPUT_WIDTH,
                input_height=CAMERA_INPUT_HEIGHT,
                input_rate=CAMERA_INPUT_RATE,
                input_flip=CAMERA_INPUT_FLIP,
                v4l2_controls=CAMERA_V4L2_CONTROLS,
            )
            self._log(
                "Started debug camera stream"
                + (" with DeepScene" if self.deepscene_enabled else "")
            )

            while not self.stop_event.is_set():
                frame = camera.capture()

                if frame is None:
                    time.sleep(0.05)
                    continue

                self._publish("camera", cv2, frame.original_bgr)

                if self.deepscene_enabled:
                    self._publish("deepscene", cv2, frame.overlay_bgr)

        except Exception as exc:
            self.error = str(exc)
            self._log(f"Debug camera stream failed: {exc}")

        finally:
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass

    def _publish(self, mode, cv2, image):
        ok, jpeg = cv2.imencode(".jpg", image)

        if not ok:
            return

        with self.condition:
            self.latest[mode] = jpeg.tobytes()
            self.frame_ids[mode] += 1
            self.condition.notify_all()

    def _log(self, message):
        if self.log_event is not None:
            self.log_event(message)


def _public_stop(stop):
    return {
        "id": stop["id"],
        "name": stop["name"],
        "lat": float(stop["lat"]),
        "lon": float(stop["lon"]),
        "description": stop.get("description", ""),
        "enabled": bool(stop.get("enabled", True)),
    }


def _normalize_manual_direction(direction):
    direction = str(direction or "").strip().lower()

    if direction == "backwards":
        direction = "backward"

    if direction in MANUAL_CONTROL_VELOCITIES or direction == MANUAL_CONTROL_STOP:
        return direction

    raise ValueError("Unknown manual control direction")


def _manual_control_duration(duration_sec):
    if duration_sec is None:
        duration_sec = MANUAL_CONTROL_TIMEOUT_SEC

    try:
        duration_sec = float(duration_sec)
    except (TypeError, ValueError) as exc:
        raise ValueError("Manual control duration must be numeric") from exc

    return max(
        MANUAL_CONTROL_MIN_TIMEOUT_SEC,
        min(MANUAL_CONTROL_MAX_TIMEOUT_SEC, duration_sec),
    )


def _manual_control_sequence(sequence):
    if sequence is None:
        return None

    try:
        return int(sequence)
    except (TypeError, ValueError) as exc:
        raise ValueError("Manual control sequence must be numeric") from exc


def _manual_control_client_id(client_id):
    client_id = str(client_id or "default").strip()

    if not client_id:
        return "default"

    return client_id[:80]
