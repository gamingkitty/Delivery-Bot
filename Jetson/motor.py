from arduino_io import ArduinoIO


class Motor:
    """
    Motor controlled through a Cytron MDD20A-style PWM + DIR input.

    speed:
    - 1.0 = full forward
    - 0.5 = half forward
    - 0.0 = stopped
    - -0.5 = half reverse
    - -1.0 = full reverse
    """

    def __init__(
        self,
        arduino: ArduinoIO,
        pwm_pin: int,
        dir_pin: int,
        inverted: bool = False,
        min_pwm: int = 0,
    ):
        self.arduino = arduino
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.inverted = inverted
        self.min_pwm = min_pwm

        self.arduino.pin_mode(self.pwm_pin, "OUTPUT")
        self.arduino.pin_mode(self.dir_pin, "OUTPUT")

        self.stop()

    def set_speed(self, speed: float):
        """
        Set motor speed from -1.0 to 1.0.
        """
        speed = max(-1.0, min(1.0, float(speed)))

        if self.inverted:
            speed = -speed

        if speed == 0:
            self.arduino.pwm_write(self.pwm_pin, 0)
            return

        direction = 1 if speed > 0 else 0
        pwm = int(abs(speed) * 255)

        if pwm > 0:
            pwm = max(self.min_pwm, pwm)

        self.arduino.digital_write(self.dir_pin, direction)
        self.arduino.pwm_write(self.pwm_pin, pwm)

    def stop(self):
        self.arduino.pwm_write(self.pwm_pin, 0)

    def forward(self, speed=1.0):
        self.set_speed(abs(speed))

    def reverse(self, speed=1.0):
        self.set_speed(-abs(speed))