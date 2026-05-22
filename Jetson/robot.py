from drive.chassis import Chassis
from drive.motor import Motor
from hardware.arduino_io import ArduinoIO
from hardware.gps import GPS
from hardware.imu import IMU
from config import (
    LEFT_MOTOR,
    MAX_FORWARD_CM_PER_SEC,
    MAX_TURN_DEG_PER_SEC,
    MOTOR_TUNING,
    RIGHT_MOTOR,
    TRACK_WIDTH_CM,
    WHEEL_DIAMETER_CM,
)
from teleop.controller import Controller


CONTROL_INTERVAL_SEC = 0.05


def create_motor(arduino: ArduinoIO, motor_config: dict) -> Motor:
    return Motor(
        arduino,
        motor_config["pwm_pin"],
        motor_config["dir_pin"],
        motor_config["encoder_index"],
        inverted=motor_config.get("inverted", False),
        encoder_reversed=motor_config.get("encoder_reversed", True),
        **MOTOR_TUNING,
    )


def create_chassis(arduino: ArduinoIO, zero_imu: bool = True):
    imu = IMU(arduino)

    if zero_imu:
        imu.zero()

    gps = GPS()
    chassis = Chassis(
        WHEEL_DIAMETER_CM,
        TRACK_WIDTH_CM,
        create_motor(arduino, LEFT_MOTOR),
        create_motor(arduino, RIGHT_MOTOR),
        imu,
        gps,
        max_turn_deg_per_sec=MAX_TURN_DEG_PER_SEC,
    )

    return chassis, gps, imu


def controller_velocity(
    controller: Controller,
    max_forward_cm_per_sec: float = MAX_FORWARD_CM_PER_SEC,
    max_turn_deg_per_sec: float = MAX_TURN_DEG_PER_SEC,
):
    forward = -controller.get_axis("left_y") * max_forward_cm_per_sec
    turn = -controller.get_axis("right_x") * max_turn_deg_per_sec
    return forward, turn
