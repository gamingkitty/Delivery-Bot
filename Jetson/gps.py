import time

import pynmea2
import serial


KNOT_TO_MPS = 0.514444


class GPS:
    def __init__(self, port="/dev/ttyUSB0", baud_rate=9600):
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        self._buffer = ""

        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed_knots = None
        self.speed_mps = None
        self.course = None
        self.num_satellites = None
        self.fix_quality = 0
        self.has_fix = False
        self.last_update_time = None

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def update(self) -> bool:
        """Read available NMEA data and return True when GPS state changes."""
        waiting = self.serial.in_waiting

        if waiting <= 0:
            return False

        self._buffer += self.serial.read(waiting).decode("ascii", errors="ignore")
        updated = False

        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            updated = self._handle_line(line.strip()) or updated

        return updated

    def _handle_line(self, line: str) -> bool:
        if not line.startswith("$"):
            return False

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return False

        if msg.sentence_type == "RMC":
            return self._handle_rmc(msg)

        if msg.sentence_type == "GGA":
            return self._handle_gga(msg)

        return False

    def _handle_rmc(self, msg) -> bool:
        if msg.status != "A":
            self.has_fix = False
            return False

        self.latitude = -msg.latitude if msg.lat_dir == "S" else msg.latitude
        self.longitude = -msg.longitude if msg.lon_dir == "W" else msg.longitude
        self.speed_knots = float(msg.spd_over_grnd or 0.0)
        self.speed_mps = self.speed_knots * KNOT_TO_MPS
        self.course = float(msg.true_course) if msg.true_course else None
        self.has_fix = True
        self.last_update_time = time.time()
        return True

    def _handle_gga(self, msg) -> bool:
        self.fix_quality = int(msg.gps_qual or 0)
        self.num_satellites = int(msg.num_sats or 0)
        self.altitude = float(msg.altitude) if msg.altitude else None
        self.has_fix = self.fix_quality > 0

        if self.has_fix:
            self.last_update_time = time.time()

        return True

    def get_position(self):
        if self.latitude is None or self.longitude is None:
            return None

        return self.latitude, self.longitude

    def get_data(self):
        return {
            "has_fix": self.has_fix,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "speed_knots": self.speed_knots,
            "speed_mps": self.speed_mps,
            "course": self.course,
            "num_satellites": self.num_satellites,
            "fix_quality": self.fix_quality,
            "last_update_time": self.last_update_time,
        }
