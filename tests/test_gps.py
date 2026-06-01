import sys
import unittest
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parents[1] / "Jetson"
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from hardware.gps import GPS


class GPSTests(unittest.TestCase):
    def setUp(self):
        self.gps = GPS.__new__(GPS)
        self.gps.origin_latitude = 47.608013
        self.gps.origin_longitude = -122.335167
        self.gps.latitude = None
        self.gps.longitude = None
        self.gps.altitude = None
        self.gps.speed_knots = None
        self.gps.speed_mps = None
        self.gps.course = None
        self.gps.num_satellites = None
        self.gps.fix_quality = 0
        self.gps.has_fix = False
        self.gps.last_update_time = None
        self.gps.last_raw_line = None

    def test_malformed_gga_coordinate_is_ignored(self):
        updated = self.gps._handle_line(
            "$GPGGA,032009.000,4738.6403,N,122$GPGGA,W,1,04,2.13,153.3,M,-17.2,M,,*5E"
        )

        self.assertFalse(updated)
        self.assertFalse(self.gps.has_fix)
        self.assertIsNone(self.gps.latitude)
        self.assertIsNone(self.gps.longitude)

    def test_non_numeric_gga_coordinate_is_ignored(self):
        updated = self.gps._handle_line(
            "$GPGGA,032009.000,4738.6403,N,not-a-number,W,1,04,2.13,153.3,M,-17.2,M,,*5E"
        )

        self.assertTrue(updated)
        self.assertTrue(self.gps.has_fix)
        self.assertIsNone(self.gps.latitude)
        self.assertIsNone(self.gps.longitude)

    def test_concatenated_sentence_is_ignored(self):
        updated = self.gps._handle_line(
            "$GPGGA,032009.000,4738.6403,N,12209.9009,W,1,04,2.13,153.3,M,-17.2,M,,*5E$GPGGA"
        )

        self.assertFalse(updated)
        self.assertFalse(self.gps.has_fix)

    def test_valid_gga_updates_position(self):
        updated = self.gps._handle_line(
            "$GPGGA,032009.000,4738.6403,N,12209.9009,W,1,04,2.13,153.3,M,-17.2,M,,*5E"
        )

        self.assertTrue(updated)
        self.assertTrue(self.gps.has_fix)
        self.assertAlmostEqual(self.gps.latitude, 47.644005)
        self.assertAlmostEqual(self.gps.longitude, -122.165015)


if __name__ == "__main__":
    unittest.main()
