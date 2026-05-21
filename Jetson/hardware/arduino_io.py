import threading
import time

import serial


VALID_INDEXES = {1, 2}
STARTUP_LINE = "READY"
RESYNC_RESPONSES = {
    "",
    STARTUP_LINE,
    "ERR unknown_command",
    "ERR buffer_overflow",
}
RESYNC_READ_TIMEOUT_SEC = 0.1
PING_INTERVAL_SEC = 0.25


def _require_valid_index(name: str, value: int) -> int:
    value = int(value)

    if value not in VALID_INDEXES:
        raise ValueError(f"{name} must be 1 or 2")

    return value


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class ArduinoIO:
    """Serial client for the Arduino motor/encoder controller."""

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        command_timeout=0.1,
        startup_timeout=5.0,
        recovery_timeout=2.0,
    ):
        self.serial = serial.Serial(
            port,
            baudrate,
            timeout=command_timeout,
            write_timeout=0.1,
        )
        self.lock = threading.Lock()
        self.recovery_timeout = float(recovery_timeout)

        # Opening serial normally resets the Arduino Nano. A ping loop also
        # handles cases where the board was already running or just power-cycled.
        time.sleep(0.1)
        with self.lock:
            self._resync_unlocked(float(startup_timeout))

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _send_command(self, command: str, value_type=None, timeout=None):
        with self.lock:
            response = self._send_command_once_unlocked(command, timeout)

            if response in RESYNC_RESPONSES:
                self._resync_unlocked(self.recovery_timeout)
                response = self._send_command_once_unlocked(command, timeout)

        if not response:
            raise TimeoutError(f"No response from Arduino for command: {command}")

        if response.startswith("ERR"):
            raise RuntimeError(f"Arduino error for command '{command}': {response}")

        if value_type is None:
            if response != "OK":
                raise RuntimeError(f"Expected OK for command '{command}', got: {response}")
            return None

        if not response.startswith("VALUE "):
            raise RuntimeError(
                f"Expected VALUE response for command '{command}', got: {response}"
            )

        return value_type(response.split(maxsplit=1)[1])

    def _send_command_once_unlocked(self, command: str, timeout=None) -> str:
        original_timeout = self.serial.timeout

        if timeout is not None:
            self.serial.timeout = timeout

        self.serial.reset_input_buffer()
        self.serial.write(f"{command}\n".encode("utf-8"))
        self.serial.flush()

        try:
            return self._readline_unlocked()
        finally:
            if timeout is not None:
                self.serial.timeout = original_timeout

    def _resync_unlocked(self, timeout: float):
        deadline = time.monotonic() + max(0.0, timeout)
        original_timeout = self.serial.timeout
        last_ping = 0.0

        self.serial.timeout = RESYNC_READ_TIMEOUT_SEC
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        try:
            while time.monotonic() < deadline:
                now = time.monotonic()

                if now - last_ping >= PING_INTERVAL_SEC:
                    self.serial.write(b"\nPING\n")
                    self.serial.flush()
                    last_ping = now

                response = self._readline_unlocked()

                if response == "OK":
                    self.serial.reset_input_buffer()
                    return

            raise TimeoutError("Arduino did not respond to PING during startup/resync")

        finally:
            self.serial.timeout = original_timeout

    def _readline_unlocked(self) -> str:
        return self.serial.readline().decode("utf-8", errors="replace").strip()

    def ping(self):
        self._send_command("PING")

    def encoder_read(self, encoder_index: int) -> int:
        encoder_index = _require_valid_index("encoder_index", encoder_index)
        return self._send_command(f"ENCREAD {encoder_index}", int)

    def encoder_reset(self, encoder_index: int):
        encoder_index = _require_valid_index("encoder_index", encoder_index)
        self._send_command(f"ENCRESET {encoder_index}")

    def motor_config(
        self,
        motor_index: int,
        pwm_pin: int,
        dir_pin: int,
        encoder_index: int,
        motor_inverted: bool = False,
        encoder_reversed: bool = False,
        counts_per_rev: float = 268.8,
        kp: float = 0.2,
        ki: float = 0.08,
        kd: float = 0.0,
        static_ff_pwm: float = 15.0,
        velocity_ff_pwm_per_deg_per_sec: float = 0.108,
    ):
        motor_index = _require_valid_index("motor_index", motor_index)
        encoder_index = _require_valid_index("encoder_index", encoder_index)
        counts_per_rev = float(counts_per_rev)

        if counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")

        values = (
            motor_index,
            int(pwm_pin),
            int(dir_pin),
            encoder_index,
            int(bool(motor_inverted)),
            int(bool(encoder_reversed)),
            f"{counts_per_rev:g}",
            f"{float(kp):g}",
            f"{float(ki):g}",
            f"{float(kd):g}",
            f"{_clamp(float(static_ff_pwm), 0.0, 255.0):g}",
            f"{max(0.0, float(velocity_ff_pwm_per_deg_per_sec)):g}",
        )
        self._send_command("MOTCFG " + " ".join(map(str, values)))

    def motor_velocity(self, motor_index: int, deg_per_sec: float):
        motor_index = _require_valid_index("motor_index", motor_index)
        self._send_command(f"MOTVEL {motor_index} {float(deg_per_sec):g}")

    def motor_power(self, motor_index: int, power: float):
        motor_index = _require_valid_index("motor_index", motor_index)
        power = _clamp(float(power), -1.0, 1.0)
        self._send_command(f"MOTPWR {motor_index} {power:g}")

    def motor_velocity_read(self, motor_index: int) -> float:
        motor_index = _require_valid_index("motor_index", motor_index)
        return self._send_command(f"MOTVELREAD {motor_index}", float)

    def imu_angle_read(self) -> float:
        return self._send_command("IMUANGLE", float, timeout=0.5)
