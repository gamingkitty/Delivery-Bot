import json
import math
import time
from pathlib import Path


MAP_VERSION = 1


class PointCloudMap:
    """JSON-backed driveable pointcloud map using meter coordinates."""

    def __init__(self, path, data=None):
        self.path = Path(path)
        self.data = data if data is not None else self._new_data()
        self.points = self.data["points"]

    @classmethod
    def new(cls, path):
        return cls(path)

    @classmethod
    def load(cls, path, create: bool = False):
        path = Path(path)

        if not path.exists():
            if create:
                return cls.new(path)
            raise FileNotFoundError(path)

        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)

        if isinstance(data, list):
            data = cls._wrap_legacy_points(data)

        if not isinstance(data, dict) or not isinstance(data.get("points"), list):
            raise ValueError(
                f"Map file must contain a JSON object with a points list: {path}"
            )

        data.setdefault("version", MAP_VERSION)
        data.setdefault("units", "meters")
        data.setdefault("created_at", time.time())
        return cls(path, data)

    @staticmethod
    def _new_data():
        now = time.time()
        return {
            "version": MAP_VERSION,
            "units": "meters",
            "created_at": now,
            "updated_at": now,
            "points": [],
        }

    @staticmethod
    def _wrap_legacy_points(points):
        now = time.time()
        return {
            "version": MAP_VERSION,
            "units": "meters",
            "created_at": now,
            "updated_at": now,
            "points": points,
        }

    def add_point(self, x, y, heading_deg=None, timestamp=None):
        point = {
            "x": round(float(x), 3),
            "y": round(float(y), 3),
        }

        if heading_deg is not None:
            point["heading_deg"] = round(float(heading_deg), 2)

        if timestamp is not None:
            point["timestamp"] = float(timestamp)

        self.points.append(point)
        return point

    def should_add_point(self, x, y, min_distance_m: float) -> bool:
        min_distance_m = float(min_distance_m)

        if min_distance_m <= 0.0:
            return True

        if not self.points:
            return True

        x = float(x)
        y = float(y)

        for point in self.points:
            try:
                point_x = float(point["x"])
                point_y = float(point["y"])
            except (KeyError, TypeError, ValueError):
                continue

            if math.hypot(x - point_x, y - point_y) < min_distance_m:
                return False

        return True

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.data["version"] = MAP_VERSION
        self.data["units"] = "meters"
        self.data["updated_at"] = time.time()
        self.data["point_count"] = len(self.points)

        with self.path.open("w", encoding="utf-8") as file:
            json.dump(self.data, file, indent=2)
            file.write("\n")

    def __len__(self):
        return len(self.points)


class PointMapSnapper:
    """Find nearest recorded point-map coordinates for localization constraints."""

    def __init__(self, point_map):
        self.map_points = _extract_map_points(point_map)
        self.points = [(point[0], point[1]) for point in self.map_points]

        if not self.points:
            raise ValueError("point_map must contain at least one valid x/y point")

        self.point_headings = _map_point_headings(self.map_points)

    def nearest(
        self,
        x,
        y,
        max_distance_m=None,
        heading_deg=None,
        heading_weight_m=0.0,
        heading_max_error_deg=None,
    ):
        x = float(x)
        y = float(y)
        max_distance_m = (
            None if max_distance_m is None else max(0.0, float(max_distance_m))
        )
        best_point = None
        best_distance_sq = None
        best_score = None
        heading_weight_m = max(0.0, float(heading_weight_m))
        heading_max_error_deg = (
            None
            if heading_max_error_deg is None
            else max(0.0, float(heading_max_error_deg))
        )
        max_distance_sq = None if max_distance_m is None else max_distance_m ** 2

        for index, (point_x, point_y) in enumerate(self.points):
            distance_sq = (x - point_x) ** 2 + (y - point_y) ** 2

            if max_distance_sq is not None and distance_sq > max_distance_sq:
                continue

            score = distance_sq

            if heading_deg is not None and heading_weight_m > 0.0:
                map_heading = self.point_headings[index]

                if map_heading is not None:
                    heading_error = _undirected_angle_error(heading_deg, map_heading)

                    if (
                        heading_max_error_deg is not None
                        and heading_error > heading_max_error_deg
                    ):
                        continue

                    score += (heading_error / 90.0 * heading_weight_m) ** 2

            if best_score is None or score < best_score:
                best_score = score
                best_distance_sq = distance_sq
                best_point = (point_x, point_y)

        if best_point is None:
            return None

        distance_m = math.sqrt(best_distance_sq)

        if max_distance_m is not None and distance_m > max_distance_m:
            return None

        return best_point[0], best_point[1], distance_m

    def constrain(
        self,
        x,
        y,
        max_distance_m=None,
        tolerance_m=0.0,
        heading_deg=None,
        heading_weight_m=0.0,
        heading_max_error_deg=None,
    ):
        result = self.nearest(
            x,
            y,
            max_distance_m,
            heading_deg=heading_deg,
            heading_weight_m=heading_weight_m,
            heading_max_error_deg=heading_max_error_deg,
        )

        if result is None:
            return float(x), float(y)

        nearest_x, nearest_y, distance_m = result
        tolerance_m = max(0.0, float(tolerance_m))

        if distance_m <= tolerance_m or distance_m <= 1e-9:
            return float(x), float(y)

        scale = tolerance_m / distance_m
        constrained_x = nearest_x + (float(x) - nearest_x) * scale
        constrained_y = nearest_y + (float(y) - nearest_y) * scale
        return constrained_x, constrained_y


def _extract_map_points(point_map):
    if hasattr(point_map, "points"):
        raw_points = point_map.points
    elif isinstance(point_map, dict):
        raw_points = point_map.get("points", [])
    else:
        raw_points = point_map

    points = []

    if raw_points is None:
        return points

    try:
        iterator = iter(raw_points)
    except TypeError:
        return points

    for point in iterator:
        try:
            heading_deg = None

            if isinstance(point, dict):
                x = point["x"]
                y = point["y"]

                if point.get("heading_deg") is not None:
                    heading_deg = float(point["heading_deg"])
            else:
                x = point[0]
                y = point[1]

                if len(point) > 2 and point[2] is not None:
                    heading_deg = float(point[2])

            points.append((float(x), float(y), heading_deg))
        except (KeyError, TypeError, ValueError, IndexError):
            continue

    return points


def _map_point_headings(map_points):
    headings = []

    for index, point in enumerate(map_points):
        heading_deg = point[2]

        if heading_deg is None:
            heading_deg = _estimated_point_heading(map_points, index)

        headings.append(heading_deg)

    return headings


def _estimated_point_heading(map_points, index):
    previous_point = map_points[index - 1] if index > 0 else None
    next_point = map_points[index + 1] if index < len(map_points) - 1 else None

    if previous_point is not None and next_point is not None:
        dx = next_point[0] - previous_point[0]
        dy = next_point[1] - previous_point[1]
    elif next_point is not None:
        dx = next_point[0] - map_points[index][0]
        dy = next_point[1] - map_points[index][1]
    elif previous_point is not None:
        dx = map_points[index][0] - previous_point[0]
        dy = map_points[index][1] - previous_point[1]
    else:
        return None

    if dx == 0.0 and dy == 0.0:
        return None

    return math.degrees(math.atan2(dy, dx))


def _undirected_angle_error(first_deg, second_deg):
    error = abs(_angle_error(first_deg, second_deg))
    return min(error, 180.0 - error)


def _angle_error(target, current):
    return (float(target) - float(current) + 180.0) % 360.0 - 180.0


def _extract_xy_points(point_map):
    return [(x, y) for x, y, _heading in _extract_map_points(point_map)]
