import math
import time

from drive.motor import Motor
from hardware.gps import GPS
from hardware.imu import IMU
from navigation.point_cloud import PointMapSnapper


ANGLE_KP = 3.0


class Chassis:
    """
    Differential-drive chassis using two velocity-controlled motors.

    Dimensions are in centimeters. Forward velocity is cm/s and turn velocity is
    chassis yaw rate in deg/s. Positive turn drives the right wheel faster than
    the left wheel. The left motor command is negative for forward wheel motion;
    the right motor command is positive for forward wheel motion.
    """

    def __init__(
        self,
        wheel_diameter_cm: float,
        track_width_cm: float,
        left_motor: Motor,
        right_motor: Motor,
        imu: IMU,
        gps: GPS,
        angle_kp: float = ANGLE_KP,
        drive_kp: float = 50,
        max_turn_deg_per_sec: float = None,
        gps_snap_enabled: bool = True,
        gps_snap_point_map=None,
        gps_snap_max_distance_m: float = None,
        gps_snap_map_tolerance_m: float = 0.0,
    ):
        self.wheel_diameter_cm = float(wheel_diameter_cm)
        self.track_width_cm = float(track_width_cm)
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.imu = imu
        self.gps = gps
        self.angle_kp = float(angle_kp)
        self.drive_kp = float(drive_kp)
        self.max_drive_speed = 50.0
        self.max_turn_deg_per_sec = (
            None if max_turn_deg_per_sec is None else abs(float(max_turn_deg_per_sec))
        )
        self.gps_snapper = (
            PointMapSnapper(gps_snap_point_map)
            if gps_snap_enabled and gps_snap_point_map is not None
            else None
        )
        self.gps_snap_max_distance_m = (
            None
            if gps_snap_max_distance_m is None
            else max(0.0, float(gps_snap_max_distance_m))
        )
        self.gps_snap_map_tolerance_m = max(0.0, float(gps_snap_map_tolerance_m))
        self.gps_snap_heading_weight_m = 6.0
        self.gps_snap_heading_max_error_deg = 60.0
        self.video_drive_heading_window_sec = 4.0
        self.video_drive_heading_samples = []

        initial_pos_x = 0
        initial_pos_y = 0
        num_gps_pos = 0
        start = time.time()
        while num_gps_pos < 10:
            if time.time() - start > 5.0:
                print(f"Warning: GPS only has {num_gps_pos} fixes on initialization")
                break
            if self.gps.update():
                gps_pos = self.gps.get_position_meters()
                if gps_pos is None:
                    continue

                initial_pos_x += gps_pos[0]
                initial_pos_y += gps_pos[1]
                num_gps_pos += 1

        if num_gps_pos:
            initial_pos_x /= num_gps_pos
            initial_pos_y /= num_gps_pos
            initial_pos_x, initial_pos_y = self._snap_xy_to_point_map(
                initial_pos_x,
                initial_pos_y,
            )

        print(f"Old initial pos: {initial_pos_x}, {initial_pos_y}")
        # self.offset_x = 19892.513 - initial_pos_x
        # self.offset_y = 9533.847 - initial_pos_y
        self.offset_x = 0
        self.offset_y = 0
        # initial_pos_x = 19892.513
        # initial_pos_y = 9533.847

        self.left_motor_position = left_motor.read_position_degrees()
        self.right_motor_position = right_motor.read_position_degrees()

        # X: Meters, Y: Meters, Heading: Degrees
        # Assumes robot is initially facing east.
        self.position = (initial_pos_x, initial_pos_y, 0)
        self.raw_gps_position = None
        self.wanted_position = None

        # Can replace these later with a full kalman filter
        self.odom_uncertainty_m = 1.0

        self.gps_uncertainty_m = 8.0
        self.odom_error_per_meter = 0.05
        self.min_gps_weight = 0.02
        self.max_gps_weight = 0.35
        self.gps_recovery_distance_m = 2.0
        self.gps_recovery_margin_m = 1.0
        self.gps_recovery_weight = 0.65

        if self.wheel_diameter_cm <= 0:
            raise ValueError("wheel_diameter_cm must be positive")

        if self.track_width_cm <= 0:
            raise ValueError("track_width_cm must be positive")

        self._wheel_degrees_per_cm = 360.0 / (math.pi * self.wheel_diameter_cm)
        self.wanted_angle = 0

    def update_position(self):
        gps_updated = self.gps.update()
        gps_pos = self.gps.get_position_meters()
        self.raw_gps_position = (
            None
            if gps_pos is None
            else (float(gps_pos[0]), float(gps_pos[1]))
        )

        if gps_pos is not None:
            gps_pos = (gps_pos[0] + self.offset_x, gps_pos[1] + self.offset_y)

        new_left_pos = self.left_motor.read_position_degrees()
        new_right_pos = self.right_motor.read_position_degrees()

        dl = -(new_left_pos - self.left_motor_position) / self._wheel_degrees_per_cm
        dr = (new_right_pos - self.right_motor_position) / self._wheel_degrees_per_cm

        self.left_motor_position = new_left_pos
        self.right_motor_position = new_right_pos

        old_heading = self.position[2]
        new_heading = self.imu.get_angle()

        d_heading = self._angle_error(new_heading, old_heading)
        drive_angle = math.radians(old_heading + d_heading / 2.0)

        distance_cm = (dl + dr) / 2.0
        distance_m = distance_cm / 100.0

        encoder_dx = distance_m * math.cos(drive_angle)
        encoder_dy = distance_m * math.sin(drive_angle)

        odom_x = self.position[0] + encoder_dx
        odom_y = self.position[1] + encoder_dy

        # Odometry gets less trustworthy as you drive farther.
        self.odom_uncertainty_m += abs(distance_m) * self.odom_error_per_meter

        gps_recovery = None

        if gps_pos is not None and gps_updated:
            gps_x, gps_y = gps_pos
            gps_recovery = self._gps_recovery_correction(
                gps_x,
                gps_y,
                new_heading,
            )

            # Ignore GPS if it jumps insanely far from odometry.
            gps_error = math.hypot(gps_x - odom_x, gps_y - odom_y)

            if True: #gps_error < 40.0:
                gps_weight = self.odom_uncertainty_m / (
                        self.odom_uncertainty_m + self.gps_uncertainty_m
                )

                gps_weight = max(self.min_gps_weight, min(self.max_gps_weight, gps_weight))
                gps_weight = max(
                    gps_weight,
                    0.0 if gps_recovery is None else gps_recovery["weight"],
                )

                x = odom_x * (1.0 - gps_weight) + gps_x * gps_weight
                y = odom_y * (1.0 - gps_weight) + gps_y * gps_weight

                # After GPS correction, reduce uncertainty.
                self.odom_uncertainty_m *= (1.0 - gps_weight)
            else:
                x = odom_x
                y = odom_y
        else:
            x = odom_x
            y = odom_y

        snapped_x, snapped_y = self._snap_xy_to_point_map(
            x,
            y,
            heading_deg=new_heading,
        )

        if gps_recovery is not None and self._snap_still_stuck(
            snapped_x,
            snapped_y,
            gps_recovery,
        ):
            x, y = gps_recovery["snap_xy"]
        else:
            x, y = snapped_x, snapped_y

        self.position = (x, y, new_heading)

    def get_position(self):
        return self.position

    def get_raw_gps_position(self):
        return self.raw_gps_position

    def set_wanted_position(self, position):
        self.wanted_position = position

    def drive_to_wanted_position(self):
        if self.wanted_position is None:
            raise RuntimeError("wanted_position must be set before driving to it")

        dx = self.wanted_position[0] - self.position[0]
        dy = self.wanted_position[1] - self.position[1]
        drive_angle = math.degrees(math.atan2(dy, dx))
        distance = math.hypot(dx, dy)

        if distance < 0.2:
            self.stop()
            return

        self.set_wanted_angle(drive_angle)
        angle_error = self._angle_error(self.wanted_angle, self.position[2])

        if abs(angle_error) > 10:
            forward = 0
        else:
            forward = min(self.max_drive_speed, distance * self.drive_kp)

        self.set_velocity(forward)

    def set_wanted_angle(self, angle: float):
        self.wanted_angle = self._normalize_angle(angle)

    def set_velocity(self, forward: float, turn: float = None):
        forward_cm_per_sec = float(forward)

        if turn is None:
            angle_error = self._angle_error(self.wanted_angle, self.position[2])
            if abs(angle_error) < 1:
                turn = 0
            else:
                turn = angle_error * self.angle_kp

        if self.max_turn_deg_per_sec is not None:
            turn = self._clamp(
                turn,
                -self.max_turn_deg_per_sec,
                self.max_turn_deg_per_sec,
            )

        turn_rad_per_sec = math.radians(float(turn))

        half_track_cm = self.track_width_cm / 2.0
        left_cm_per_sec = forward_cm_per_sec - turn_rad_per_sec * half_track_cm
        right_cm_per_sec = forward_cm_per_sec + turn_rad_per_sec * half_track_cm

        left_deg_per_sec = -self._wheel_linear_to_motor_deg(left_cm_per_sec)
        right_deg_per_sec = self._wheel_linear_to_motor_deg(right_cm_per_sec)

        self.left_motor.set_velocity_pair(
            self.right_motor,
            left_deg_per_sec,
            right_deg_per_sec,
        )

    def stop(self):
        self.left_motor.set_velocity_pair(self.right_motor, 0.0, 0.0)

    def record_video_drive_heading(self, heading_deg=None, now=None):
        now = time.monotonic() if now is None else float(now)

        if heading_deg is None:
            heading_deg = self.position[2]

        samples = getattr(self, "video_drive_heading_samples", None)

        if samples is None:
            samples = []
            self.video_drive_heading_samples = samples

        samples.append((now, self._normalize_angle(heading_deg)))
        self._trim_video_drive_heading_samples(now)

    def _video_drive_heading(self, now=None):
        now = time.monotonic() if now is None else float(now)
        samples = self._trim_video_drive_heading_samples(now)

        if not samples:
            return None

        sin_sum = sum(math.sin(math.radians(heading)) for _time, heading in samples)
        cos_sum = sum(math.cos(math.radians(heading)) for _time, heading in samples)

        if sin_sum == 0.0 and cos_sum == 0.0:
            return None

        return self._normalize_angle(math.degrees(math.atan2(sin_sum, cos_sum)))

    def _trim_video_drive_heading_samples(self, now=None):
        now = time.monotonic() if now is None else float(now)
        samples = getattr(self, "video_drive_heading_samples", None)

        if samples is None:
            samples = []
            self.video_drive_heading_samples = samples

        window_sec = max(
            0.0,
            float(getattr(self, "video_drive_heading_window_sec", 4.0)),
        )
        cutoff = now - window_sec
        samples[:] = [
            (sample_time, heading)
            for sample_time, heading in samples
            if sample_time >= cutoff
        ]
        return samples

    def _wheel_linear_to_motor_deg(self, wheel_cm_per_sec: float) -> float:
        return wheel_cm_per_sec * self._wheel_degrees_per_cm

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return float(angle) % 360.0

    @staticmethod
    def _angle_error(target: float, current: float) -> float:
        return (float(target) - float(current) + 180.0) % 360.0 - 180.0

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))

    def _gps_recovery_correction(self, gps_x: float, gps_y: float, heading_deg: float):
        if self.gps_snapper is None:
            return None

        result = self.gps_snapper.nearest(
            gps_x,
            gps_y,
            self.gps_snap_max_distance_m,
            heading_deg=heading_deg,
            heading_weight_m=getattr(self, "gps_snap_heading_weight_m", 6.0),
            heading_max_error_deg=getattr(
                self,
                "gps_snap_heading_max_error_deg",
                60.0,
            ),
        )

        if result is None:
            return None

        candidate_x, candidate_y, gps_candidate_distance_m = result
        current_x, current_y, _current_heading = self.position
        current_to_candidate_m = math.hypot(
            candidate_x - current_x,
            candidate_y - current_y,
        )
        current_to_gps_m = math.hypot(gps_x - current_x, gps_y - current_y)
        recovery_distance_m = max(
            float(getattr(self, "gps_recovery_distance_m", 2.0)),
            self.gps_snap_map_tolerance_m * 2.0,
        )
        recovery_margin_m = max(
            0.0,
            float(getattr(self, "gps_recovery_margin_m", 1.0)),
        )

        if current_to_candidate_m < recovery_distance_m:
            return None

        if current_to_gps_m <= gps_candidate_distance_m + recovery_margin_m:
            return None

        weight = self._clamp(
            getattr(self, "gps_recovery_weight", 0.65),
            self.min_gps_weight,
            1.0,
        )
        snap_xy = self._snap_xy_to_point_map(
            gps_x,
            gps_y,
            heading_deg=heading_deg,
        )
        return {
            "candidate_xy": (candidate_x, candidate_y),
            "snap_xy": snap_xy,
            "weight": weight,
        }

    def _snap_still_stuck(self, snapped_x, snapped_y, gps_recovery):
        candidate_x, candidate_y = gps_recovery["candidate_xy"]
        snap_to_candidate_m = math.hypot(
            candidate_x - float(snapped_x),
            candidate_y - float(snapped_y),
        )
        tolerance_m = max(
            self.gps_snap_map_tolerance_m,
            float(getattr(self, "gps_recovery_margin_m", 1.0)),
        )
        return snap_to_candidate_m > tolerance_m

    def _snap_xy_to_point_map(self, x: float, y: float, heading_deg=None):
        if self.gps_snapper is None:
            return float(x), float(y)

        snap_heading = heading_deg

        if snap_heading is None:
            snap_heading = self._video_drive_heading()

        return self.gps_snapper.constrain(
            x,
            y,
            self.gps_snap_max_distance_m,
            self.gps_snap_map_tolerance_m,
            heading_deg=snap_heading,
            heading_weight_m=getattr(self, "gps_snap_heading_weight_m", 6.0),
            heading_max_error_deg=getattr(
                self,
                "gps_snap_heading_max_error_deg",
                60.0,
            ),
        )
