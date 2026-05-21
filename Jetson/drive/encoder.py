from hardware.arduino_io import ArduinoIO


class Encoder:
    """
    Read one of the Arduino-managed quadrature encoder counters.
    """

    def __init__(self, arduino: ArduinoIO, encoder_index: int):
        encoder_index = int(encoder_index)

        if encoder_index not in {1, 2}:
            raise ValueError("encoder_index must be 1 or 2")

        self.arduino = arduino
        self.encoder_index = encoder_index

    def read(self) -> int:
        return self.arduino.encoder_read(self.encoder_index)

    def reset(self):
        self.arduino.encoder_reset(self.encoder_index)
