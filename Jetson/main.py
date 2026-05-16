import time

from arduino_io import ArduinoIO
from chassis import Chassis
from config import (
    ARDUINO_PORT,
    LEFT_MOTOR,
    MAX_FORWARD_CM_PER_SEC,
    MAX_TURN_DEG_PER_SEC,
    MOTOR_TUNING,
    RIGHT_MOTOR,
    TRACK_WIDTH_CM,
    WHEEL_DIAMETER_CM,
)
from controller import Controller
from encoder import Encoder
from motor import Motor


CONTROL_INTERVAL_SEC = 0.05


def create_motor(arduino: ArduinoIO, motor_config: dict) -> Motor:
    return Motor(
        arduino,
        motor_config["pwm_pin"],
        motor_config["dir_pin"],
        Encoder(arduino, motor_config["encoder_index"]),
        inverted=motor_config.get("inverted", False),
        encoder_reversed=motor_config.get("encoder_reversed", True),
        **MOTOR_TUNING,
    )


def controller_velocity(controller: Controller):
    forward = -controller.get_axis("left_y") * MAX_FORWARD_CM_PER_SEC
    turn = -controller.get_axis("right_x") * MAX_TURN_DEG_PER_SEC
    return forward, turn


def main():
    controller = Controller()

    with ArduinoIO(port=ARDUINO_PORT) as arduino:
        chassis = Chassis(
            WHEEL_DIAMETER_CM,
            TRACK_WIDTH_CM,
            create_motor(arduino, LEFT_MOTOR),
            create_motor(arduino, RIGHT_MOTOR),
        )

        try:
            while True:
                if controller.update():
                    forward, turn = controller_velocity(controller)
                    print(chassis.left_motor.get_velocity())
                    chassis.set_velocity(forward, turn)
                else:
                    chassis.stop()

                time.sleep(CONTROL_INTERVAL_SEC)

        except KeyboardInterrupt:
            print()

        finally:
            chassis.stop()


if __name__ == "__main__":
    main()
