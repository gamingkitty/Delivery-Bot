import math
import time

from drive.motor import Motor
from hardware.gps import GPS
from hardware.imu import IMU


ANGLE_KP = -3.0


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
        max_turn_deg_per_sec: float = None,
    ):
        self.wheel_diameter_cm = float(wheel_diameter_cm)
        self.track_width_cm = float(track_width_cm)
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.imu = imu
        self.gps = gps
        self.angle_kp = float(angle_kp)
        self.max_turn_deg_per_sec = (
            None if max_turn_deg_per_sec is None else abs(float(max_turn_deg_per_sec))
        )

        self.gps.update()
        initial_pos = self.gps.get_position_meters()
        start = time.time()
        while initial_pos is None:
            if time.time() - start > 2.0:
                print("Warning: GPS doesn't have fix on initialization.")
                initial_pos = (0, 0)
                break
            self.gps.update()
            initial_pos = self.gps.get_position_meters()

        self.left_motor_position = left_motor.read_position_degrees()
        self.right_motor_position = right_motor.read_position_degrees()

        # X: Meters, Y: Meters, Heading: Degrees
        # Assumes robot is initially facing east.
        self.position = (initial_pos[0], initial_pos[1], 0)

        # Can replace these later with a full kalman filter
        self.odom_uncertainty_m = 1.0

        self.gps_uncertainty_m = 3.0
        self.odom_error_per_meter = 0.05
        self.min_gps_weight = 0.02
        self.max_gps_weight = 0.35

        if self.wheel_diameter_cm <= 0:
            raise ValueError("wheel_diameter_cm must be positive")

        if self.track_width_cm <= 0:
            raise ValueError("track_width_cm must be positive")

        self._wheel_degrees_per_cm = 360.0 / (math.pi * self.wheel_diameter_cm)
        self.wanted_angle = 0

    def update_position(self):
        gps_updated = self.gps.update()
        gps_pos = self.gps.get_position_meters()

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

        if gps_pos is not None and gps_updated:
            gps_x, gps_y = gps_pos

            # Ignore GPS if it jumps insanely far from odometry.
            gps_error = math.hypot(gps_x - odom_x, gps_y - odom_y)

            if gps_error < 15.0:
                gps_weight = self.odom_uncertainty_m / (
                        self.odom_uncertainty_m + self.gps_uncertainty_m
                )

                gps_weight = max(self.min_gps_weight, min(self.max_gps_weight, gps_weight))

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

        self.position = (x, y, new_heading)

    def get_position(self):
        return self.position

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

        self.left_motor.set_velocity(
            -self._wheel_linear_to_motor_deg(left_cm_per_sec)
        )
        self.right_motor.set_velocity(
            self._wheel_linear_to_motor_deg(right_cm_per_sec)
        )

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

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
