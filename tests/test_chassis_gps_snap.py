import sys
import time
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
from navigation.point_cloud import PointMapSnapper


class FakeGPS:
    def __init__(self, position):
        self.position = position

    def update(self):
        return True

    def get_position_meters(self):
        return self.position


class FakeIMU:
    def __init__(self, angle=0.0):
        self.angle = angle

    def get_angle(self):
        return self.angle


class FakeMotor:
    def __init__(self):
        self.position_degrees = 0.0

    def read_position_degrees(self):
        return self.position_degrees


def point_map(points):
    map_points = []

    for point in points:
        x = point[0]
        y = point[1]
        map_point = {"x": x, "y": y}

        if len(point) > 2:
            map_point["heading_deg"] = point[2]

        map_points.append(map_point)

    return {"points": map_points}


def chassis_with_snap(
    points,
    gps_position,
    enabled=True,
    max_distance_m=10.0,
    tolerance_m=1.0,
):
    chassis = Chassis.__new__(Chassis)
    chassis.gps = FakeGPS(gps_position)
    chassis.imu = FakeIMU()
    chassis.left_motor = FakeMotor()
    chassis.right_motor = FakeMotor()
    chassis.left_motor_position = 0.0
    chassis.right_motor_position = 0.0
    chassis._wheel_degrees_per_cm = 360.0
    chassis.position = (0.0, 0.0, 0.0)
    chassis.raw_gps_position = None
    chassis.offset_x = 0.0
    chassis.offset_y = 0.0
    chassis.odom_uncertainty_m = 1.0
    chassis.gps_uncertainty_m = 0.0
    chassis.odom_error_per_meter = 0.06
    chassis.min_gps_weight = 1.0
    chassis.max_gps_weight = 1.0
    chassis.gps_snapper = PointMapSnapper(point_map(points)) if enabled else None
    chassis.gps_snap_max_distance_m = max_distance_m
    chassis.gps_snap_map_tolerance_m = tolerance_m
    chassis.gps_snap_heading_weight_m = 6.0
    chassis.gps_snap_heading_max_error_deg = 60.0
    chassis.video_drive_heading_window_sec = 4.0
    chassis.video_drive_heading_samples = []
    chassis.gps_recovery_distance_m = 2.0
    chassis.gps_recovery_margin_m = 1.0
    chassis.gps_recovery_weight = 0.65
    return chassis


class ChassisGpsSnapTests(unittest.TestCase):
    def test_update_position_constrains_fused_xy_to_map_tolerance(self):
        chassis = chassis_with_snap(
            [(0.0, 0.0)],
            gps_position=(3.0, 0.0),
            tolerance_m=1.0,
        )

        chassis.update_position()

        self.assertEqual(chassis.get_position(), (1.0, 0.0, 0.0))

    def test_update_position_keeps_raw_gps_before_snapping(self):
        chassis = chassis_with_snap(
            [(0.0, 0.0)],
            gps_position=(3.0, 0.0),
            tolerance_m=1.0,
        )

        chassis.update_position()

        self.assertEqual(chassis.get_raw_gps_position(), (3.0, 0.0))

    def test_update_position_keeps_position_inside_map_tolerance(self):
        chassis = chassis_with_snap(
            [(0.0, 0.0)],
            gps_position=(0.5, 0.0),
            tolerance_m=1.0,
        )

        chassis.update_position()

        self.assertEqual(chassis.get_position(), (0.5, 0.0, 0.0))

    def test_update_position_does_not_snap_when_disabled(self):
        chassis = chassis_with_snap(
            [(0.0, 0.0)],
            gps_position=(3.0, 0.0),
            enabled=False,
            tolerance_m=1.0,
        )

        chassis.update_position()

        self.assertEqual(chassis.get_position(), (3.0, 0.0, 0.0))

    def test_update_position_does_not_snap_when_fix_is_outside_snap_radius(self):
        chassis = chassis_with_snap(
            [(10.0, 0.0)],
            gps_position=(0.8, 0.0),
            max_distance_m=0.5,
        )

        chassis.update_position()

        x, y, heading = chassis.get_position()
        self.assertEqual((x, y), (0.8, 0.0))
        self.assertEqual(heading, 0.0)

    def test_snapper_prefers_candidate_with_matching_heading(self):
        snapper = PointMapSnapper(
            {
                "points": [
                    {"x": 0.5, "y": 0.0, "heading_deg": 90.0},
                    {"x": 1.0, "y": 0.0, "heading_deg": 0.0},
                ]
            }
        )

        x, y, _distance = snapper.nearest(
            0.0,
            0.0,
            max_distance_m=5.0,
            heading_deg=0.0,
            heading_weight_m=6.0,
            heading_max_error_deg=60.0,
        )

        self.assertEqual((x, y), (1.0, 0.0))

    def test_snapper_rejects_candidate_more_than_60_degrees_off_heading(self):
        snapper = PointMapSnapper(
            {
                "points": [
                    {"x": 0.5, "y": 0.0, "heading_deg": 80.0},
                ]
            }
        )

        result = snapper.nearest(
            0.0,
            0.0,
            max_distance_m=5.0,
            heading_deg=0.0,
            heading_weight_m=6.0,
            heading_max_error_deg=60.0,
        )

        self.assertIsNone(result)

    def test_video_drive_heading_is_used_for_map_snap(self):
        chassis = chassis_with_snap(
            [(0.5, 0.0, 90.0), (1.0, 0.0, 0.0)],
            gps_position=(0.0, 0.0),
            tolerance_m=0.0,
        )
        chassis.record_video_drive_heading(0.0, now=time.monotonic())

        x, y = chassis._snap_xy_to_point_map(0.0, 0.0)

        self.assertEqual((x, y), (1.0, 0.0))

    def test_video_drive_heading_blocks_snap_to_mismatched_map_heading(self):
        chassis = chassis_with_snap(
            [(0.5, 0.0, 80.0)],
            gps_position=(0.0, 0.0),
            tolerance_m=0.0,
        )
        chassis.record_video_drive_heading(0.0, now=time.monotonic())

        x, y = chassis._snap_xy_to_point_map(0.0, 0.0)

        self.assertEqual((x, y), (0.0, 0.0))

    def test_gps_recovery_escapes_old_snap_point_when_encoders_do_not_move(self):
        chassis = chassis_with_snap(
            [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)],
            gps_position=(10.0, 0.0),
            tolerance_m=0.5,
        )
        chassis.gps_uncertainty_m = 8.0
        chassis.min_gps_weight = 0.02
        chassis.max_gps_weight = 0.02

        chassis.update_position()

        x, y, heading = chassis.get_position()
        self.assertGreaterEqual(x, 9.5)
        self.assertEqual(y, 0.0)
        self.assertEqual(heading, 0.0)

    def test_gps_recovery_escapes_old_snap_point_when_encoders_move(self):
        chassis = chassis_with_snap(
            [(0.0, 0.0, 0.0), (20.0, 0.0, 0.0)],
            gps_position=(20.0, 0.0),
            tolerance_m=0.5,
        )
        chassis.gps_uncertainty_m = 8.0
        chassis.min_gps_weight = 0.02
        chassis.max_gps_weight = 0.02
        chassis.gps_recovery_weight = 0.10
        chassis.left_motor.position_degrees = -72000.0
        chassis.right_motor.position_degrees = 72000.0

        chassis.update_position()

        x, y, heading = chassis.get_position()
        self.assertGreaterEqual(x, 19.5)
        self.assertEqual(y, 0.0)
        self.assertEqual(heading, 0.0)

    def test_position_update_uses_live_heading_for_map_snap_after_turn(self):
        chassis = chassis_with_snap(
            [(0.0, 0.0, 0.0), (0.0, 10.0, 90.0)],
            gps_position=(0.0, 10.0),
            tolerance_m=0.5,
        )
        chassis.imu.angle = 90.0
        chassis.gps_uncertainty_m = 8.0
        chassis.min_gps_weight = 0.02
        chassis.max_gps_weight = 0.02
        chassis.record_video_drive_heading(0.0, now=time.monotonic())

        chassis.update_position()

        x, y, heading = chassis.get_position()
        self.assertEqual(x, 0.0)
        self.assertGreaterEqual(y, 9.5)
        self.assertEqual(heading, 90.0)


if __name__ == "__main__":
    unittest.main()
