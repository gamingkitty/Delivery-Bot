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
