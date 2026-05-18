import threading
import time

from arduino_io import ArduinoIO
from config import ARDUINO_PORT, MOTOR_TUNING, RIGHT_MOTOR
from encoder import Encoder
from motor import Motor


PRINT_INTERVAL_SEC = 0.1
DEFAULT_TARGET_VELOCITY = 300.0


class TuningState:
    def __init__(self, target_velocity: float):
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._target_velocity = float(target_velocity)

    @property
    def running(self) -> bool:
        return not self._stop_event.is_set()

    def stop(self):
        self._stop_event.set()

    def set_target(self, target_velocity: float):
        with self._lock:
            self._target_velocity = float(target_velocity)

    def get_target(self) -> float:
        with self._lock:
            return self._target_velocity


def create_tuning_motor(arduino: ArduinoIO) -> Motor:
    return Motor(
        arduino,
        RIGHT_MOTOR["pwm_pin"],
        RIGHT_MOTOR["dir_pin"],
        Encoder(arduino, RIGHT_MOTOR["encoder_index"]),
        encoder_reversed=RIGHT_MOTOR.get("encoder_reversed", True),
        **MOTOR_TUNING,
    )


def velocity_input_loop(motor: Motor, state: TuningState):
    while state.running:
        raw_value = input("Target velocity deg/s, or q to quit: ").strip()

        if raw_value.lower() in {"q", "quit", "exit"}:
            state.stop()
            return

        try:
            velocity = float(raw_value)
        except ValueError:
            print("Enter a number in deg/s, or q to quit.")
            continue

        motor.set_velocity(velocity)
        state.set_target(velocity)


def main():
    time.sleep(2)

    state = TuningState(DEFAULT_TARGET_VELOCITY)

    with ArduinoIO(port=ARDUINO_PORT) as arduino:
        right_motor = create_tuning_motor(arduino)
        right_motor.set_velocity(DEFAULT_TARGET_VELOCITY)
        print(f"Initial target velocity set to {DEFAULT_TARGET_VELOCITY:.2f} deg/s.")

        threading.Thread(
            target=velocity_input_loop,
            args=(right_motor, state),
            daemon=True,
        ).start()

        try:
            while state.running:
                target = state.get_target()
                current = right_motor.get_velocity()
                error = target - current

                print(
                    f"target={target:8.2f} deg/s "
                    f"current={current:8.2f} deg/s "
                    f"error={error:8.2f} deg/s"
                )
                time.sleep(PRINT_INTERVAL_SEC)

        except KeyboardInterrupt:
            state.stop()

        finally:
            right_motor.stop()


if __name__ == "__main__":
    main()
