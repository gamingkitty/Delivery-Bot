from hardware.arduino_io import ArduinoIO


class Motor:
    """Encoder-backed motor with Arduino-side velocity PID."""

    def __init__(
        self,
        arduino: ArduinoIO,
        pwm_pin: int,
        dir_pin: int,
        encoder_index: int,
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
        self.arduino = arduino
        self.pwm_pin = int(pwm_pin)
        self.dir_pin = int(dir_pin)
        self.encoder_index = int(encoder_index)
        self.encoder_reversed = bool(encoder_reversed)
        self.inverted = bool(inverted)
        self.counts_per_rev = float(counts_per_rev)

        if self.counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")

        if self.encoder_index not in {1, 2}:
            raise ValueError("encoder_index must be 1 or 2")

        self.motor_index = int(
            self.encoder_index if motor_index is None else motor_index
        )

        if self.motor_index not in {1, 2}:
            raise ValueError("motor_index must be 1 or 2")

        self.arduino.motor_config(
            motor_index=self.motor_index,
            pwm_pin=self.pwm_pin,
            dir_pin=self.dir_pin,
            encoder_index=self.encoder_index,
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
        counts = self.arduino.encoder_read(self.encoder_index)

        if self.encoder_reversed:
            counts = -counts

        return counts

    def read_position_degrees(self) -> float:
        return self.get_position() * 360.0 / self.counts_per_rev

    def set_velocity(self, deg_per_sec: float):
        self.arduino.motor_velocity(self.motor_index, deg_per_sec)

    def set_velocity_pair(
        self,
        other_motor,
        deg_per_sec: float,
        other_deg_per_sec: float,
    ):
        if not isinstance(other_motor, Motor):
            raise TypeError("other_motor must be a Motor instance")

        if self.arduino is not other_motor.arduino:
            raise ValueError("paired motors must share the same ArduinoIO instance")

        self.arduino.motor_velocity_pair(
            self.motor_index,
            deg_per_sec,
            other_motor.motor_index,
            other_deg_per_sec,
        )

    def get_velocity(self) -> float:
        return self.arduino.motor_velocity_read(self.motor_index)

    def stop(self):
        self.set_velocity(0.0)
