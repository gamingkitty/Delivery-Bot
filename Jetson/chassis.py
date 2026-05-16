import math

from motor import Motor


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
    ):
        self.wheel_diameter_cm = float(wheel_diameter_cm)
        self.track_width_cm = float(track_width_cm)
        self.left_motor = left_motor
        self.right_motor = right_motor

        if self.wheel_diameter_cm <= 0:
            raise ValueError("wheel_diameter_cm must be positive")

        if self.track_width_cm <= 0:
            raise ValueError("track_width_cm must be positive")

        self._wheel_degrees_per_cm = 360.0 / (math.pi * self.wheel_diameter_cm)

    def set_velocity(self, forward: float, turn: float):
        forward_cm_per_sec = float(forward)
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
