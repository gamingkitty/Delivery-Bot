import time
import threading
import serial


class ArduinoIO:
    """
    Simple serial interface to the Arduino Nano GPIO/PWM server.
    """

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1.0):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.lock = threading.Lock()

        # Give Arduino time to reset after serial connection opens.
        time.sleep(2.0)

        # Clear any startup text like "READY".
        self.serial.reset_input_buffer()

    def close(self):
        self.serial.close()

    def _send_command(self, command, expect_value=False):
        with self.lock:
            self.serial.write((command + "\n").encode("utf-8"))
            self.serial.flush()

            response = self.serial.readline().decode("utf-8", errors="replace").strip()

        if not response:
            raise TimeoutError(f"No response from Arduino for command: {command}")

        if response.startswith("ERR"):
            raise RuntimeError(f"Arduino error for command '{command}': {response}")

        if expect_value:
            if not response.startswith("VALUE "):
                raise RuntimeError(
                    f"Expected VALUE response for command '{command}', got: {response}"
                )

            return int(response.split()[1])

        if response != "OK":
            raise RuntimeError(f"Expected OK for command '{command}', got: {response}")

        return None

    def pin_mode(self, pin, mode):
        """
        mode can be:
        - "INPUT"
        - "OUTPUT"
        - "INPUT_PULLUP"
        """
        mode = mode.upper()

        if mode not in {"INPUT", "OUTPUT", "INPUT_PULLUP"}:
            raise ValueError("mode must be INPUT, OUTPUT, or INPUT_PULLUP")

        self._send_command(f"MODE {pin} {mode}")

    def digital_write(self, pin, value):
        value = 1 if value else 0
        self._send_command(f"WRITE {pin} {value}")

    def pwm_write(self, pin, value):
        """
        value should be from 0 to 255.
        """
        value = int(max(0, min(255, value)))
        self._send_command(f"PWM {pin} {value}")

    def digital_read(self, pin):
        return self._send_command(f"READ {pin}", expect_value=True)

    def analog_read(self, pin):
        return self._send_command(f"AREAD {pin}", expect_value=True)

    def encoder_read(self, encoder_index):
        encoder_index = int(encoder_index)

        if encoder_index not in {1, 2}:
            raise ValueError("encoder_index must be 1 or 2")

        return self._send_command(f"ENCREAD {encoder_index}", expect_value=True)

    def encoder_reset(self, encoder_index):
        encoder_index = int(encoder_index)

        if encoder_index not in {1, 2}:
            raise ValueError("encoder_index must be 1 or 2")

        self._send_command(f"ENCRESET {encoder_index}")

    def servo_write(self, pin, angle):
        """
        Move a servo to an angle from 0 to 180 degrees.
        """
        angle = int(max(0, min(180, angle)))
        self._send_command(f"SERVO {pin} {angle}")
