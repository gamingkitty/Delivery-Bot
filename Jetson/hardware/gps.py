import math
import time
import serial

KNOT_TO_MPS = 0.514444
EARTH_RADIUS_M = 6378137.0


class GPS:
    def __init__(
        self,
        port="/dev/ttyUSB0",
        baud_rate=9600,
        origin=(47.608013, -122.335167),
    ):
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        self._buffer = ""

        self.origin_latitude = origin[0]
        self.origin_longitude = origin[1]

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
        self.last_raw_line = None

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def update(self) -> bool:
        waiting = self.serial.in_waiting

        if waiting <= 0:
            return False

        data = self.serial.read(waiting).decode("ascii", errors="ignore")
        self._buffer += data.replace("\r", "\n")
        updated = False

        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()

            if line:
                updated = self._handle_line(line) or updated

        return updated

    def _nmea_to_decimal(self, value, direction):
        """
        Convert NMEA coordinate format to signed decimal degrees.

        Example:
            4738.6403,N -> 47.644005
            12209.9010,W -> -122.1650166667
        """
        if value is None or direction is None:
            return None

        if value == "" or direction == "":
            return None

        value = str(value).strip()
        direction = str(direction).strip().upper()

        if direction not in ("N", "S", "E", "W"):
            return None

        try:
            value = float(value)
        except (TypeError, ValueError):
            return None

        if not math.isfinite(value):
            return None

        degrees = int(value // 100)
        minutes = value - degrees * 100

        if minutes >= 60.0:
            return None

        decimal = degrees + minutes / 60.0

        if direction in ("S", "W"):
            decimal = -decimal

        return decimal

    def _handle_line(self, line: str) -> bool:
        self.last_raw_line = line

        if not line.startswith("$"):
            return False

        if "$" in line[1:]:
            return False

        # Remove checksum part after *
        line_no_checksum = line.split("*", 1)[0]
        parts = line_no_checksum.split(",")

        sentence = parts[0]

        if sentence in ("$GPRMC", "$GNRMC"):
            return self._handle_rmc(parts)

        if sentence in ("$GPGGA", "$GNGGA"):
            return self._handle_gga(parts)

        return False

    def _handle_rmc(self, parts) -> bool:
        # Example:
        # $GPRMC,032008.000,A,4738.6403,N,12209.9010,W,0.42,89.25,210526,,,A*4E
        if len(parts) < 10:
            return False

        status = parts[2]

        if status != "A":
            self.has_fix = False
            return True

        lat = self._nmea_to_decimal(parts[3], parts[4])
        lon = self._nmea_to_decimal(parts[5], parts[6])

        if lat is not None and lon is not None:
            self.latitude = lat
            self.longitude = lon

        try:
            self.speed_knots = float(parts[7]) if parts[7] else 0.0
            self.speed_mps = self.speed_knots * KNOT_TO_MPS
        except ValueError:
            self.speed_knots = None
            self.speed_mps = None

        try:
            self.course = float(parts[8]) if parts[8] else None
        except ValueError:
            self.course = None

        self.has_fix = True
        self.last_update_time = time.time()
        return True

    def _handle_gga(self, parts) -> bool:
        # Example:
        # $GPGGA,032009.000,4738.6403,N,12209.9009,W,1,04,2.13,153.3,M,-17.2,M,,*5E
        if len(parts) < 10:
            return False

        try:
            self.fix_quality = int(parts[6]) if parts[6] else 0
        except ValueError:
            self.fix_quality = 0

        try:
            self.num_satellites = int(parts[7]) if parts[7] else 0
        except ValueError:
            self.num_satellites = None

        try:
            self.altitude = float(parts[9]) if parts[9] else None
        except ValueError:
            self.altitude = None

        self.has_fix = self.fix_quality > 0

        if self.has_fix:
            lat = self._nmea_to_decimal(parts[2], parts[3])
            lon = self._nmea_to_decimal(parts[4], parts[5])

            if lat is not None and lon is not None:
                self.latitude = lat
                self.longitude = lon

            self.last_update_time = time.time()

        return True

    def get_position(self):
        if self.latitude is None or self.longitude is None:
            return None

        return self.latitude, self.longitude

    def get_position_meters(self):
        if self.latitude is None or self.longitude is None:
            return None

        origin_lat_rad = math.radians(self.origin_latitude)

        d_lat = math.radians(self.latitude - self.origin_latitude)
        d_lon = math.radians(self.longitude - self.origin_longitude)

        x = EARTH_RADIUS_M * d_lon * math.cos(origin_lat_rad)
        y = EARTH_RADIUS_M * d_lat

        return x, y

    def get_data(self):
        position_meters = self.get_position_meters()

        return {
            "has_fix": self.has_fix,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "position_meters": position_meters,
            "x_meters": position_meters[0] if position_meters is not None else None,
            "y_meters": position_meters[1] if position_meters is not None else None,
            "speed_knots": self.speed_knots,
            "speed_mps": self.speed_mps,
            "course": self.course,
            "num_satellites": self.num_satellites,
            "fix_quality": self.fix_quality,
            "last_update_time": self.last_update_time,
            "last_raw_line": self.last_raw_line,
        }
