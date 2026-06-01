import sys
import types
import unittest
import math
from pathlib import Path

import numpy as np


JETSON_ROOT = Path(__file__).resolve().parents[1] / "Jetson"
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

serial_stub = types.ModuleType("serial")
serial_stub.Serial = object
sys.modules.setdefault("serial", serial_stub)

from navigation import drive_to_destination as nav


class NavigationVideoFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.saved = {
            "VIDEO_FEEDBACK_FORWARD_CM_PER_SEC": nav.VIDEO_FEEDBACK_FORWARD_CM_PER_SEC,
            "VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC": (
                nav.VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC
            ),
            "VIDEO_FEEDBACK_ROUTE_SANITY_DEG": nav.VIDEO_FEEDBACK_ROUTE_SANITY_DEG,
            "VIDEO_FEEDBACK_ROUTE_LOOKAHEAD_M": nav.VIDEO_FEEDBACK_ROUTE_LOOKAHEAD_M,
            "VIDEO_FEEDBACK_ROUTE_BEND_SCAN_M": (
                nav.VIDEO_FEEDBACK_ROUTE_BEND_SCAN_M
            ),
            "VIDEO_FEEDBACK_ROUTE_BEND_MIN_DEG": nav.VIDEO_FEEDBACK_ROUTE_BEND_MIN_DEG,
            "VIDEO_FEEDBACK_ROUTE_BEND_START_M": (
                nav.VIDEO_FEEDBACK_ROUTE_BEND_START_M
            ),
            "VIDEO_FEEDBACK_ROUTE_BEND_FULL_M": (
                nav.VIDEO_FEEDBACK_ROUTE_BEND_FULL_M
            ),
            "VIDEO_FEEDBACK_ROUTE_BEND_MAJOR_DEG": (
                nav.VIDEO_FEEDBACK_ROUTE_BEND_MAJOR_DEG
            ),
            "VIDEO_FEEDBACK_ROUTE_BEND_BASE_FRACTION": (
                nav.VIDEO_FEEDBACK_ROUTE_BEND_BASE_FRACTION
            ),
            "VIDEO_FEEDBACK_ROUTE_REORIENT_DEG": nav.VIDEO_FEEDBACK_ROUTE_REORIENT_DEG,
            "VIDEO_FEEDBACK_ROUTE_EXIT_REORIENT_DEG": (
                nav.VIDEO_FEEDBACK_ROUTE_EXIT_REORIENT_DEG
            ),
            "VIDEO_FEEDBACK_ROUTE_INITIAL_ALIGNMENT_DEG": (
                nav.VIDEO_FEEDBACK_ROUTE_INITIAL_ALIGNMENT_DEG
            ),
            "VIDEO_FEEDBACK_ROUTE_HINT_GAIN": nav.VIDEO_FEEDBACK_ROUTE_HINT_GAIN,
            "VIDEO_FEEDBACK_ROUTE_REORIENT_KP": (
                nav.VIDEO_FEEDBACK_ROUTE_REORIENT_KP
            ),
            "VIDEO_FEEDBACK_ROUTE_REORIENT_MAX_TURN_DEG_PER_SEC": (
                nav.VIDEO_FEEDBACK_ROUTE_REORIENT_MAX_TURN_DEG_PER_SEC
            ),
            "VIDEO_FEEDBACK_MIN_ROI_TOP_FRACTION": (
                nav.VIDEO_FEEDBACK_MIN_ROI_TOP_FRACTION
            ),
            "VIDEO_FEEDBACK_CENTER_ROW_PEAK_FRACTION": (
                nav.VIDEO_FEEDBACK_CENTER_ROW_PEAK_FRACTION
            ),
            "VIDEO_FEEDBACK_CENTER_ROW_MIN_WEIGHT": (
                nav.VIDEO_FEEDBACK_CENTER_ROW_MIN_WEIGHT
            ),
            "VIDEO_FEEDBACK_ROUTE_PIXEL_WEIGHT_MAX": (
                nav.VIDEO_FEEDBACK_ROUTE_PIXEL_WEIGHT_MAX
            ),
            "VIDEO_FEEDBACK_TRAIL_TURN_GAIN": nav.VIDEO_FEEDBACK_TRAIL_TURN_GAIN,
            "VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG": (
                nav.VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG
            ),
            "VIDEO_FEEDBACK_TURN_OPTION_MAX_BIAS": (
                nav.VIDEO_FEEDBACK_TURN_OPTION_MAX_BIAS
            ),
            "VIDEO_FEEDBACK_CENTERING_PROTECT_OFFSET": (
                nav.VIDEO_FEEDBACK_CENTERING_PROTECT_OFFSET
            ),
            "VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG": (
                nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG
            ),
            "VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_PIXELS": (
                nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_PIXELS
            ),
            "VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_FRACTION": (
                nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_FRACTION
            ),
            "VIDEO_FEEDBACK_TURN_OPTION_MID_ROW_FRACTION": (
                nav.VIDEO_FEEDBACK_TURN_OPTION_MID_ROW_FRACTION
            ),
            "VIDEO_FEEDBACK_TURN_OPTION_MIN_ROW_WEIGHT": (
                nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROW_WEIGHT
            ),
            "WAYPOINT_PASS_LATERAL_TOLERANCE_M": (
                nav.WAYPOINT_PASS_LATERAL_TOLERANCE_M
            ),
            "VIDEO_FEEDBACK_SMOOTHING": nav.VIDEO_FEEDBACK_SMOOTHING,
            "VIDEO_FEEDBACK_MAX_OFFSET_STEP": nav.VIDEO_FEEDBACK_MAX_OFFSET_STEP,
            "_navigation_feedback_offset": nav._navigation_feedback_offset,
            "_navigation_feedback_last_signal_at": (
                nav._navigation_feedback_last_signal_at
            ),
            "_navigation_turn_command": nav._navigation_turn_command,
        }

    def tearDown(self):
        for name, value in self.saved.items():
            setattr(nav, name, value)

    def test_trail_signal_drives_like_follow_trail_even_when_route_is_90_degrees(self):
        nav.VIDEO_FEEDBACK_FORWARD_CM_PER_SEC = 30.0
        nav.VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC = 25.0
        chassis = FakeChassis()
        route_command = (0.0, 45.0, 90.0, 10.0)

        drive_command, drive_mode = nav._navigation_drive_command(
            chassis,
            route_command,
            -0.5,
            True,
        )

        self.assertEqual(drive_mode, "route_video")
        self.assertEqual(drive_command, (30.0, 15.625))

    def test_route_turn_does_not_bleed_into_trail_following(self):
        nav.VIDEO_FEEDBACK_FORWARD_CM_PER_SEC = 30.0
        nav.VIDEO_FEEDBACK_MAX_TURN_DEG_PER_SEC = 25.0
        chassis = FakeChassis()
        left_route_turn = (0.0, 45.0, 90.0, 10.0)
        right_route_turn = (0.0, -45.0, 90.0, 10.0)

        left_command, _mode = nav._navigation_drive_command(
            chassis,
            left_route_turn,
            0.25,
            True,
        )
        right_command, _mode = nav._navigation_drive_command(
            chassis,
            right_route_turn,
            0.25,
            True,
        )

        self.assertEqual(left_command, right_command)
        self.assertEqual(left_command, (30.0, -7.8125))

    def test_opposite_route_direction_uses_route_to_select_the_trail(self):
        nav.VIDEO_FEEDBACK_ROUTE_SANITY_DEG = 150.0
        chassis = FakeChassis()
        route_command = (0.0, 45.0, 170.0, 20.0)

        drive_command, drive_mode = nav._navigation_drive_command(
            chassis,
            route_command,
            0.0,
            True,
        )

        self.assertEqual(drive_mode, "route_select")
        self.assertEqual(drive_command, (0.0, 45.0))

    def test_no_trail_with_camera_available_stops(self):
        chassis = FakeChassis()
        route_command = (30.0, 20.0, 20.0, 20.0)

        drive_command, drive_mode = nav._navigation_drive_command(
            chassis,
            route_command,
            None,
            True,
        )

        self.assertEqual(drive_mode, "no_trail")
        self.assertEqual(drive_command, (0.0, 0.0))

    def test_camera_available_reorients_before_trusting_vision(self):
        nav.VIDEO_FEEDBACK_ROUTE_INITIAL_ALIGNMENT_DEG = 45.0
        chassis = FakeChassis()
        route_command = (30.0, 0.0, 0.0, 20.0)

        drive_command, drive_mode = nav._navigation_drive_command(
            chassis,
            route_command,
            0.0,
            True,
            route_alignment_error=60.0,
            force_route_alignment=True,
        )

        self.assertEqual(drive_mode, "route_reorient")
        self.assertEqual(drive_command, (0.0, 28.0))

    def test_wrong_direction_reorients_in_place_before_continuing(self):
        nav.VIDEO_FEEDBACK_ROUTE_INITIAL_ALIGNMENT_DEG = 45.0
        chassis = FakeChassis()
        route_command = (30.0, 0.0, 0.0, 20.0)

        drive_command, drive_mode = nav._navigation_drive_command(
            chassis,
            route_command,
            -0.5,
            True,
            route_alignment_error=-120.0,
        )

        self.assertEqual(drive_mode, "route_reorient")
        self.assertEqual(drive_command, (0.0, -28.0))

    def test_camera_unavailable_falls_back_to_waypoint_navigation(self):
        chassis = FakeChassis()
        route_command = (30.0, 20.0, 20.0, 20.0)

        drive_command, drive_mode = nav._navigation_drive_command(
            chassis,
            route_command,
            None,
            False,
        )

        self.assertEqual(drive_mode, "waypoint")
        self.assertEqual(drive_command, (30.0, 20.0))

    def test_passed_waypoint_completes_inside_off_trail_corridor(self):
        nav.WAYPOINT_PASS_LATERAL_TOLERANCE_M = 10.0
        chassis = FakeChassis(position=(11.0, 9.5, 0.0))

        complete, reason, distance = nav._waypoint_completion(
            chassis,
            (10.0, 0.0),
            0.35,
            previous_waypoint_xy=(0.0, 0.0),
            allow_passed_completion=True,
        )

        self.assertTrue(complete)
        self.assertEqual(reason, "passed")
        self.assertGreater(distance, 0.35)

    def test_destination_does_not_complete_by_passed_position(self):
        nav.WAYPOINT_PASS_LATERAL_TOLERANCE_M = 10.0
        chassis = FakeChassis(position=(11.0, 9.5, 0.0))

        complete, reason, _distance = nav._waypoint_completion(
            chassis,
            (10.0, 0.0),
            0.35,
            previous_waypoint_xy=(0.0, 0.0),
            allow_passed_completion=False,
        )

        self.assertFalse(complete)
        self.assertIsNone(reason)

    def test_heading_can_complete_a_waypoint_that_is_behind_the_robot(self):
        nav.WAYPOINT_PASS_LATERAL_TOLERANCE_M = 10.0
        chassis = FakeChassis(position=(2.0, 0.0, 0.0))

        complete, reason, distance = nav._waypoint_completion(
            chassis,
            (0.0, 0.0),
            0.35,
            allow_passed_completion=True,
        )

        self.assertTrue(complete)
        self.assertEqual(reason, "passed")
        self.assertEqual(distance, 2.0)

    def test_feedback_offset_smoothing_uses_new_sample_weight(self):
        nav.VIDEO_FEEDBACK_SMOOTHING = 0.25
        nav.VIDEO_FEEDBACK_MAX_OFFSET_STEP = 2.0

        self.assertEqual(nav._smooth_feedback_offset(0.0, 1.0), 0.25)

    def test_top_thirty_percent_of_image_is_ignored_for_trail_detection(self):
        mask = np.zeros((100, 100), dtype=bool)
        mask[0:30, 10:90] = True

        offset = nav._trail_center_offset(mask)

        self.assertIsNone(offset)

    def test_middle_rows_count_more_than_bottom_rows_for_steering(self):
        nav.VIDEO_FEEDBACK_CENTER_ROW_PEAK_FRACTION = 0.55
        nav.VIDEO_FEEDBACK_CENTER_ROW_MIN_WEIGHT = 0.45

        middle_weight = nav._trail_center_row_weight(0.55)
        bottom_weight = nav._trail_center_row_weight(1.0)

        self.assertGreater(middle_weight, bottom_weight)
        self.assertGreater(bottom_weight, 0.40)

    def test_route_heading_selects_visible_left_turn_option(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 10.0
        mask = fork_trail_mask()

        offset = nav._trail_center_offset(mask, route_error_deg=60.0)

        self.assertLess(offset, -0.08)

    def test_major_bend_hint_significantly_biases_visible_left_option(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 15.0
        nav.VIDEO_FEEDBACK_TURN_OPTION_MAX_BIAS = 0.55
        mask = fork_trail_mask()

        offset = nav._trail_center_offset(mask, route_error_deg=25.0)

        self.assertLess(offset, -0.10)

    def test_route_heading_selects_visible_right_turn_option(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 10.0
        mask = fork_trail_mask()

        offset = nav._trail_center_offset(mask, route_error_deg=-60.0)

        self.assertGreater(offset, 0.08)

    def test_visible_right_turn_option_commands_negative_turn(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 10.0
        mask = fork_trail_mask()

        offset = nav._trail_center_offset(mask, route_error_deg=-60.0)
        _forward_cm_s, turn_deg_s = nav._trail_follow_drive_command(
            FakeChassis(),
            offset,
        )

        self.assertGreater(offset, 0.08)
        self.assertLess(turn_deg_s, 0.0)

    def test_straight_path_caps_hard_vision_steering(self):
        nav.VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG = 20.0
        chassis = FakeChassis()

        _forward_cm_s, turn_deg_s = nav._trail_follow_drive_command(
            chassis,
            -1.0,
            route_error_deg=0.0,
        )

        self.assertEqual(turn_deg_s, 20.0)

    def test_bend_hint_allows_more_vision_steering_in_path_direction(self):
        nav.VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG = 20.0
        chassis = FakeChassis()

        _forward_cm_s, turn_deg_s = nav._trail_follow_drive_command(
            chassis,
            -1.0,
            route_error_deg=30.0,
        )

        self.assertEqual(turn_deg_s, 35.0)

    def test_right_bend_hint_caps_left_vision_steering_at_trail_tolerance(self):
        nav.VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG = 20.0
        chassis = FakeChassis()

        _forward_cm_s, turn_deg_s = nav._trail_follow_drive_command(
            chassis,
            -1.0,
            route_error_deg=-25.0,
        )

        self.assertEqual(turn_deg_s, 20.0)

    def test_smoothed_turn_is_clamped_to_trail_to_bend_range(self):
        nav.VIDEO_FEEDBACK_VISION_TURN_AWAY_TOLERANCE_DEG = 20.0
        nav.VIDEO_FEEDBACK_TURN_SMOOTHING = 0.0
        nav._navigation_turn_command = 20.0

        _forward_cm_s, turn_deg_s = nav._smooth_navigation_drive_command(
            (30.0, -25.0),
            "route_video",
            route_error_for_trail=-25.0,
        )

        self.assertEqual(turn_deg_s, 20.0)

    def test_bottom_trail_pixels_do_not_create_turn_options(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 10.0
        mask = fork_trail_mask()
        mask[68:78, :] = True

        offset = nav._trail_center_offset(mask, route_error_deg=60.0)

        self.assertLess(offset, -0.08)

    def test_unavailable_desired_turn_falls_back_to_center_following(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 10.0
        mask = right_turn_only_mask()

        offset = nav._trail_center_offset(mask, route_error_deg=60.0)

        self.assertGreater(offset, 0.2)

    def test_turn_option_weights_middle_rows_more_than_bottom_rows(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MID_ROW_FRACTION = 0.55
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROW_WEIGHT = 0.15

        middle_weight = nav._turn_option_row_weight(0.55)
        bottom_weight = nav._turn_option_row_weight(1.0)

        self.assertGreater(middle_weight, bottom_weight)

    def test_bottom_only_pixels_do_not_enable_turn_option(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_PIXELS = 120
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_TRAIL_FRACTION = 0.02
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROW_WEIGHT = 0.03
        roi = np.zeros((60, 100), dtype=bool)
        roi[50:60, 0:45] = True

        options = nav._trail_turn_options(roi, 100)

        self.assertIsNone(options["left"])

    def test_turn_bias_can_override_centering_when_route_is_confident(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MAX_BIAS = 0.55

        offset = nav._apply_turn_option_bias(0.50, -0.60, route_scale=1.0)

        self.assertLess(offset, -0.09)

    def test_small_route_hint_does_not_bias_fork_selection(self):
        nav.VIDEO_FEEDBACK_TURN_OPTION_MIN_ROUTE_ERROR_DEG = 20.0
        mask = fork_trail_mask()

        offset = nav._trail_center_offset(mask, route_error_deg=6.0)

        self.assertAlmostEqual(offset, 0.0, places=3)

    def test_route_lookahead_does_not_request_turn_on_straight_path(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (20.0, 0.0)]

        route_error = nav._route_lookahead_error(chassis, path, 0)

        self.assertEqual(route_error, 0.0)

    def test_route_alignment_uses_current_path_heading_on_straight_path(self):
        chassis = FakeChassis(position=(0.0, 0.0, 90.0))
        path = [(0.0, 0.0), (20.0, 0.0)]

        route_error = nav._route_alignment_error(chassis, path, 0)

        self.assertEqual(route_error, -90.0)

    def test_route_lookahead_requests_turn_when_path_bends(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (8.0, 0.0), (8.0, 10.0)]

        route_error = nav._route_lookahead_error(chassis, path, 0)

        self.assertAlmostEqual(route_error, 30.0, places=3)

    def test_route_lookahead_uses_negative_error_for_right_bend(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (8.0, 0.0), (8.0, -10.0)]

        route_error = nav._route_lookahead_error(chassis, path, 0)

        self.assertAlmostEqual(route_error, -30.0, places=3)

    def test_route_bend_correction_scales_linearly_with_distance(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (9.5, 0.0), (9.5, 10.0)]

        route_error = nav._route_lookahead_error(chassis, path, 0)

        self.assertAlmostEqual(route_error, 22.5, places=3)

    def test_route_lookahead_groups_gradual_major_right_bend(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = path_from_headings([0.0, -15.0, -30.0, -45.0, -60.0, -75.0, -90.0])

        route_error = nav._route_lookahead_error(chassis, path, 0)

        self.assertEqual(route_error, -45.0)

    def test_route_lookahead_uses_net_right_bend_after_left_wiggle(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = path_from_headings(
            [0.0, 40.0, 20.0, 0.0, -15.0, -30.0, -45.0, -60.0, -75.0, -90.0]
        )

        guidance = nav._route_guidance(chassis, path, 0)
        status = nav._route_guidance_status(chassis, guidance)

        self.assertTrue(guidance["active"])
        self.assertEqual(status["direction"], "right")
        self.assertEqual(guidance["bend_angle_deg"], -90.0)
        self.assertAlmostEqual(
            nav._route_guidance_lookahead_error(guidance),
            -30.0,
            places=3,
        )

    def test_route_lookahead_ignores_small_gradual_bend(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = path_from_headings([0.0, 5.0, 10.0, 15.0, 20.0])

        route_error = nav._route_lookahead_error(chassis, path, 0)

        self.assertEqual(route_error, 0.0)

    def test_route_lookahead_ignores_bend_beyond_lookahead(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0)]

        route_error = nav._route_lookahead_error(chassis, path, 0)

        self.assertEqual(route_error, 0.0)

    def test_route_guidance_detects_bend_before_activating_it(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (20.0, 0.0), (20.0, -10.0)]

        guidance = nav._route_guidance(chassis, path, 0)

        self.assertTrue(guidance["detected"])
        self.assertFalse(guidance["active"])
        self.assertEqual(guidance["base_heading_deg"], 315.0)
        self.assertEqual(nav._route_guidance_lookahead_error(guidance), 0.0)

    def test_route_alignment_uses_average_heading_near_bend(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (5.0, 0.0), (5.0, 10.0)]

        route_error = nav._route_alignment_error(chassis, path, 0)

        self.assertEqual(route_error, 45.0)

    def test_route_trail_hint_scales_remaining_bend_heading_error(self):
        nav.VIDEO_FEEDBACK_ROUTE_HINT_GAIN = 0.60
        chassis = FakeChassis(position=(0.0, 0.0, 25.0))
        path = [(0.0, 0.0), (5.0, 0.0), (5.0, 10.0)]
        guidance = nav._route_guidance(chassis, path, 0)

        route_error = nav._route_guidance_trail_error(chassis, guidance)

        self.assertEqual(route_error, 39.0)

    def test_route_trail_hint_stops_after_heading_reaches_bend_exit(self):
        nav.VIDEO_FEEDBACK_ROUTE_HINT_GAIN = 0.60
        chassis = FakeChassis(position=(0.0, 0.0, 90.0))
        path = [(0.0, 0.0), (5.0, 0.0), (5.0, 10.0)]
        guidance = nav._route_guidance(chassis, path, 0)

        route_error = nav._route_guidance_trail_error(chassis, guidance)

        self.assertEqual(route_error, 0.0)

    def test_route_trail_hint_does_not_reverse_after_overshooting_bend_target(self):
        nav.VIDEO_FEEDBACK_ROUTE_HINT_GAIN = 0.60
        chassis = FakeChassis(position=(0.0, 0.0, 105.0))
        path = [(0.0, 0.0), (5.0, 0.0), (5.0, 10.0)]
        guidance = nav._route_guidance(chassis, path, 0)

        route_error = nav._route_guidance_trail_error(chassis, guidance)

        self.assertEqual(route_error, 0.0)

    def test_route_alignment_does_not_snap_to_next_leg_from_noisy_position(self):
        chassis = FakeChassis(position=(8.0, 4.0, 0.0))
        path = [(0.0, 0.0), (8.0, 0.0), (8.0, 10.0)]

        route_error = nav._route_alignment_error(chassis, path, 0)

        self.assertEqual(route_error, 45.0)

    def test_route_alignment_uses_exit_heading_when_clearly_on_wrong_trail(self):
        chassis = FakeChassis(position=(0.0, 0.0, 90.0))
        path = path_from_headings([0.0, -50.0], segment_length_m=5.0)

        route_error = nav._route_alignment_error(chassis, path, 0)

        self.assertAlmostEqual(route_error, -140.0, places=3)

    def test_route_guidance_stops_reporting_bend_after_advancing_to_next_leg(self):
        chassis = FakeChassis(position=(8.0, 4.0, 90.0))
        path = [(0.0, 0.0), (8.0, 0.0), (8.0, 10.0)]

        route_lookahead_error = nav._route_lookahead_error(chassis, path, 1)
        route_alignment_error = nav._route_alignment_error(chassis, path, 1)

        self.assertEqual(route_lookahead_error, 0.0)
        self.assertEqual(route_alignment_error, 0.0)

    def test_route_guidance_status_reports_bend_and_command(self):
        chassis = FakeChassis(position=(0.0, 0.0, 0.0))
        path = [(0.0, 0.0), (5.0, 0.0), (5.0, 10.0)]
        guidance = nav._route_guidance(chassis, path, 0)

        status = nav._route_guidance_status(
            chassis,
            guidance,
            drive_command=(30.0, 12.0),
            drive_mode="route_video",
        )

        self.assertTrue(status["detected"])
        self.assertTrue(status["active"])
        self.assertEqual(status["direction"], "left")
        self.assertEqual(status["bend_angle_deg"], 90.0)
        self.assertEqual(status["base_heading_deg"], 45.0)
        self.assertEqual(status["strength"], 1.0)
        self.assertEqual(status["lookahead_error_deg"], 45.0)
        self.assertEqual(status["trail_hint_error_deg"], 54.0)
        self.assertEqual(status["alignment_error_deg"], 45.0)
        self.assertEqual(status["trail_bias_direction"], "left")
        self.assertEqual(status["command_turn_direction"], "left")
        self.assertEqual(status["command_turn_deg_s"], 12.0)
        self.assertEqual(status["drive_mode"], "route_video")


class FakeChassis:
    drive_kp = 50.0
    angle_kp = 3.0
    max_drive_speed = 50.0
    max_turn_deg_per_sec = 45.0

    def __init__(self, position=(0.0, 0.0, 0.0)):
        self.position = position

    def get_position(self):
        return self.position


def fork_trail_mask():
    mask = np.zeros((100, 100), dtype=bool)
    mask[25:66, 20:31] = True
    mask[25:66, 69:80] = True
    return mask


def right_turn_only_mask():
    mask = np.zeros((100, 100), dtype=bool)
    mask[25:66, 69:80] = True
    return mask


def path_from_headings(headings, segment_length_m=2.0):
    path = [(0.0, 0.0)]
    x = 0.0
    y = 0.0

    for heading in headings:
        heading_rad = math.radians(heading)
        x += math.cos(heading_rad) * segment_length_m
        y += math.sin(heading_rad) * segment_length_m
        path.append((round(x, 6), round(y, 6)))

    return path


if __name__ == "__main__":
    unittest.main()
