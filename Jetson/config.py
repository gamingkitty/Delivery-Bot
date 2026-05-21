ARDUINO_PORT = "/dev/ttyUSB1"

WHEEL_DIAMETER_CM = 10.16
TRACK_WIDTH_CM = 34.826
MAX_FORWARD_CM_PER_SEC = 30.0
MAX_TURN_DEG_PER_SEC = 45.0

LEFT_MOTOR = {
    "pwm_pin": 9,
    "dir_pin": 8,
    "encoder_index": 2,
    "encoder_reversed": True,
}

RIGHT_MOTOR = {
    "pwm_pin": 10,
    "dir_pin": 7,
    "encoder_index": 1,
    "encoder_reversed": True,
}

MOTOR_TUNING = {
    "kp": 0.2,
    "ki": 0.08,
    "kd": 0.0,
    "static_ff_pwm": 15.0,
    "velocity_ff_pwm_per_deg_per_sec": 0.108,
}

NAV_MAX_MAP_DISTANCE_M = 1.0
NAV_GRID_RESOLUTION_M = 0.2
NAV_CLEARANCE_COST_WEIGHT = 1.5
NAV_ENDPOINT_SNAP_MAX_DISTANCE_M = NAV_MAX_MAP_DISTANCE_M
