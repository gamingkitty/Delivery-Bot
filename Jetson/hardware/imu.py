import math
import time

from hardware.arduino_io import ArduinoIO


STARTUP_ZERO_EPSILON_DEG = 0.01
DEFAULT_ZERO_TIMEOUT_SEC = 2.0
DEFAULT_ZERO_SAMPLE_INTERVAL_SEC = 0.05
DEFAULT_ZERO_SETTLE_SAMPLES = 3


class IMU:
    """
    Read the bot heading from the BNO055 connected to the Arduino Nano I2C bus.

    The Arduino reports the BNO055 Euler heading in degrees. A zero offset can
    be applied when the current heading should be treated as 0 degrees.
    """

    def __init__(self, arduino: ArduinoIO, zero_on_start: bool = False):
        self.arduino = arduino
        self.zero_offset = 0.0

        if zero_on_start:
            self.zero()

    def read_angle(self) -> float:
        """Return the current raw BNO055 heading in degrees, normalized to 0-360."""
        return self._normalize_angle(self.arduino.imu_angle_read())

    def get_angle(self) -> float:
        """Return the current heading in degrees after applying the zero offset."""
        return self._normalize_angle(self.read_angle() - self.zero_offset)

    def zero(
        self,
        timeout_sec: float = DEFAULT_ZERO_TIMEOUT_SEC,
        sample_interval_sec: float = DEFAULT_ZERO_SAMPLE_INTERVAL_SEC,
        settle_samples: int = DEFAULT_ZERO_SETTLE_SAMPLES,
    ):
        """Use the current heading as the 0 degree reference."""
        self.zero_offset = self._read_settled_angle(
            timeout_sec,
            sample_interval_sec,
            settle_samples,
        )

    def _read_settled_angle(
        self,
        timeout_sec: float,
        sample_interval_sec: float,
        settle_samples: int,
    ) -> float:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        sample_interval_sec = max(0.0, float(sample_interval_sec))
        settle_samples = max(1, int(settle_samples))
        nonzero_samples = []
        last_angle = self.read_angle()

        while time.monotonic() < deadline:
            last_angle = self.read_angle()

            if abs(last_angle) > STARTUP_ZERO_EPSILON_DEG:
                nonzero_samples.append(last_angle)

                if len(nonzero_samples) >= settle_samples:
                    return self._circular_mean(nonzero_samples[-settle_samples:])
            else:
                nonzero_samples.clear()

            if sample_interval_sec > 0.0:
                time.sleep(sample_interval_sec)

        return last_angle

    @staticmethod
    def _circular_mean(angles):
        total_x = 0.0
        total_y = 0.0

        for angle in angles:
            radians = math.radians(angle)
            total_x += math.cos(radians)
            total_y += math.sin(radians)

        return math.degrees(math.atan2(total_y, total_x)) % 360.0

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return float(angle) % 360.0
