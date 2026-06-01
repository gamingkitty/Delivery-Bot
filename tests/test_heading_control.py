import sys
import types
import unittest
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parents[1] / "Jetson"
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

serial_stub = types.ModuleType("serial")
serial_stub.Serial = object
sys.modules.setdefault("serial", serial_stub)

from drive.chassis import Chassis
from hardware.imu import IMU


class FakeArduino:
    def __init__(self, angle):
        self.angle = angle

    def imu_angle_read(self):
        return self.angle


class FakeMotor:
    def __init__(self):
        self.pair_command = None

    def set_velocity_pair(self, other_motor, deg_per_sec, other_deg_per_sec):
        self.pair_command = (deg_per_sec, other_deg_per_sec)


class HeadingControlTests(unittest.TestCase):
    def test_imu_heading_is_counterclockwise_positive_after_zeroing(self):
        arduino = FakeArduino(100.0)
        imu = IMU(arduino)
        imu.zero_offset = 100.0

        arduino.angle = 90.0
        self.assertAlmostEqual(imu.get_angle(), 10.0)

        arduino.angle = 110.0
        self.assertAlmostEqual(imu.get_angle(), 350.0)

    def test_positive_heading_error_commands_left_turn(self):
        chassis = fake_chassis()
        chassis.wanted_angle = 90.0

        chassis.set_velocity(0.0)

        left_deg_per_sec, right_deg_per_sec = chassis.left_motor.pair_command
        self.assertGreater(right_deg_per_sec, 0.0)
        self.assertGreater(left_deg_per_sec, 0.0)

    def test_negative_heading_error_commands_right_turn(self):
        chassis = fake_chassis()
        chassis.wanted_angle = 315.0

        chassis.set_velocity(0.0)

        left_deg_per_sec, right_deg_per_sec = chassis.left_motor.pair_command
        self.assertLess(right_deg_per_sec, 0.0)
        self.assertLess(left_deg_per_sec, 0.0)


def fake_chassis():
    chassis = Chassis.__new__(Chassis)
    chassis.track_width_cm = 34.826
    chassis.max_turn_deg_per_sec = None
    chassis.angle_kp = 3.0
    chassis.position = (0.0, 0.0, 0.0)
    chassis._wheel_degrees_per_cm = 360.0 / (3.141592653589793 * 10.16)
    chassis.left_motor = FakeMotor()
    chassis.right_motor = FakeMotor()
    return chassis


if __name__ == "__main__":
    unittest.main()
