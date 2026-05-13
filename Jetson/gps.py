import serial
import pynmea2
import time


class GPS:
    def __init__(self, port="/dev/ttyUSB0", baud_rate=9600, timeout=1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout

        self.serial = serial.Serial(
            self.port,
            self.baud_rate,
            timeout=self.timeout
        )

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

    def knots_to_mps(self, knots):
        return knots * 0.514444

    def read_line(self):
        try:
            line = self.serial.readline().decode("ascii", errors="ignore").strip()
            return line
        except Exception:
            return None

    def update(self):
        """
        Reads one NMEA sentence from the GPS and updates stored data.
        Call this repeatedly in a loop.
        Returns True if useful GPS data was updated.
        """
        line = self.read_line()

        if not line or not line.startswith("$"):
            return False

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return False

        if msg.sentence_type == "RMC":
            return self._handle_rmc(msg)

        elif msg.sentence_type == "GGA":
            return self._handle_gga(msg)

        return False

    def _handle_rmc(self, msg):
        # RMC contains position, speed, course, date/time
        if msg.status != "A":
            self.has_fix = False
            return False

        lat = msg.latitude
        lon = msg.longitude

        if msg.lat_dir == "S":
            lat = -lat
        if msg.lon_dir == "W":
            lon = -lon

        self.latitude = lat
        self.longitude = lon

        self.speed_knots = float(msg.spd_over_grnd or 0)
        self.speed_mps = self.knots_to_mps(self.speed_knots)

        self.course = float(msg.true_course) if msg.true_course else None

        self.has_fix = True
        self.last_update_time = time.time()

        return True

    def _handle_gga(self, msg):
        # GGA contains fix quality, satellites, altitude
        self.fix_quality = int(msg.gps_qual or 0)
        self.num_satellites = int(msg.num_sats or 0)

        self.altitude = float(msg.altitude) if msg.altitude else None

        if self.fix_quality > 0:
            self.has_fix = True
            self.last_update_time = time.time()
        else:
            self.has_fix = False

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