import math
import sys
import unittest
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parents[1] / "Jetson"
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from navigation.path_planner import (
    EndpointNotDriveableError,
    NoPathError,
    build_navigation_grid,
    plan_path,
)


def point_map(points):
    return {
        "points": [
            {
                "x": x,
                "y": y,
            }
            for x, y in points
        ]
    }


def filled_points(min_x, max_x, min_y, max_y, step):
    points = []
    x_count = int(round((max_x - min_x) / step)) + 1
    y_count = int(round((max_y - min_y) / step)) + 1

    for y_index in range(y_count):
        y = min_y + y_index * step

        for x_index in range(x_count):
            x = min_x + x_index * step
            points.append((round(x, 6), round(y, 6)))

    return points


class PathPlannerTests(unittest.TestCase):
    def test_grid_marks_driveable_cells_from_pointcloud_distance(self):
        grid = build_navigation_grid(
            point_map([(0.0, 0.0)]),
            max_map_distance_m=0.5,
            grid_resolution_m=0.25,
        )

        self.assertTrue(grid.is_driveable(grid.world_to_cell(0.0, 0.0)))
        self.assertTrue(grid.is_driveable(grid.world_to_cell(0.5, 0.0)))
        self.assertFalse(grid.is_driveable(grid.world_to_cell(0.75, 0.0)))

    def test_clearance_is_higher_inside_wide_driveable_area(self):
        points = filled_points(-1.0, 1.0, -1.0, 1.0, 0.5)
        grid = build_navigation_grid(
            point_map(points),
            max_map_distance_m=0.55,
            grid_resolution_m=0.25,
        )

        center_clearance = grid.clearance_at(grid.world_to_cell(0.0, 0.0))
        edge_clearance = grid.clearance_at(grid.world_to_cell(1.25, 0.0))

        self.assertGreater(center_clearance, edge_clearance)

    def test_endpoint_snaps_when_near_driveable_space(self):
        path = plan_path(
            point_map([(0.0, 0.0)]),
            (0.8, 0.0),
            (0.0, 0.0),
            max_map_distance_m=1.0,
            grid_resolution_m=0.2,
        )

        self.assertGreaterEqual(len(path), 1)
        self.assertLessEqual(math.hypot(path[0]["x"] - 0.8, path[0]["y"]), 1.0)

    def test_endpoint_fails_when_far_from_driveable_space(self):
        with self.assertRaises(EndpointNotDriveableError):
            plan_path(
                point_map([(0.0, 0.0)]),
                (2.5, 0.0),
                (0.0, 0.0),
                max_map_distance_m=1.0,
                grid_resolution_m=0.2,
            )

    def test_theta_star_uses_line_of_sight_in_open_space(self):
        points = filled_points(0.0, 2.0, 0.0, 2.0, 0.5)
        path = plan_path(
            point_map(points),
            (0.0, 0.0),
            (2.0, 2.0),
            max_map_distance_m=0.6,
            grid_resolution_m=0.2,
        )

        self.assertLessEqual(len(path), 3)
        self.assertAlmostEqual(path[0]["x"], 0.0, places=1)
        self.assertAlmostEqual(path[-1]["x"], 2.0, places=1)
        self.assertAlmostEqual(path[-1]["y"], 2.0, places=1)

    def test_no_path_when_driveable_regions_are_disconnected(self):
        with self.assertRaises(NoPathError):
            plan_path(
                point_map([(0.0, 0.0), (3.0, 0.0)]),
                (0.0, 0.0),
                (3.0, 0.0),
                max_map_distance_m=0.3,
                grid_resolution_m=0.2,
            )


if __name__ == "__main__":
    unittest.main()
