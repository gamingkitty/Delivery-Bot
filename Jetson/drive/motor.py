from drive.encoder import Encoder
from hardware.arduino_io import ArduinoIO


class Motor:
    """Encoder-backed motor with Arduino-side velocity PID."""

    def __init__(
        self,
        arduino: ArduinoIO,
        pwm_pin: int,
        dir_pin: int,
        encoder: Encoder,
        motor_index: int = None,
        inverted: bool = False,
        encoder_reversed: bool = True,
        counts_per_rev: float = 268.8,
        kp: float = 0.2,
        ki: float = 0.08,
        kd: float = 0.0,
        static_ff_pwm: float = 15.0,
        velocity_ff_pwm_per_deg_per_sec: float = 0.108,
    ):
        if not isinstance(encoder, Encoder):
            raise TypeError("encoder must be an Encoder instance")

        self.arduino = arduino
        self.pwm_pin = int(pwm_pin)
        self.dir_pin = int(dir_pin)
        self.encoder = encoder
        self.encoder_reversed = bool(encoder_reversed)
        self.inverted = bool(inverted)
        self.counts_per_rev = float(counts_per_rev)
        self.motor_index = int(
            encoder.encoder_index if motor_index is None else motor_index
        )

        if self.counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")

        if self.motor_index not in {1, 2}:
            raise ValueError("motor_index must be 1 or 2")

        self.arduino.motor_config(
            motor_index=self.motor_index,
            pwm_pin=self.pwm_pin,
            dir_pin=self.dir_pin,
            encoder_index=self.encoder.encoder_index,
            motor_inverted=self.inverted,
            encoder_reversed=self.encoder_reversed,
            counts_per_rev=self.counts_per_rev,
            kp=kp,
            ki=ki,
            kd=kd,
            static_ff_pwm=static_ff_pwm,
            velocity_ff_pwm_per_deg_per_sec=velocity_ff_pwm_per_deg_per_sec,
        )

    def get_position(self) -> int:
        counts = self.encoder.read()

        if self.encoder_reversed:
            counts = -counts

        return counts

    def read_position_degrees(self) -> float:
        return self.get_position() * 360.0 / self.counts_per_rev

    def reset_position(self):
        self.encoder.reset()

    def set_velocity(self, deg_per_sec: float):
        self.arduino.motor_velocity(self.motor_index, deg_per_sec)

    def set_power(self, power: float):
        self.arduino.motor_power(self.motor_index, power)

    def get_velocity(self) -> float:
        return self.arduino.motor_velocity_read(self.motor_index)

    def stop(self):
        self.set_velocity(0.0)
