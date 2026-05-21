import heapq
import math

try:
    from config import (
        NAV_CLEARANCE_COST_WEIGHT,
        NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
        NAV_GRID_RESOLUTION_M,
        NAV_MAX_MAP_DISTANCE_M,
    )
except ModuleNotFoundError:
    from Jetson.config import (
        NAV_CLEARANCE_COST_WEIGHT,
        NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
        NAV_GRID_RESOLUTION_M,
        NAV_MAX_MAP_DISTANCE_M,
    )


MAX_GRID_CELLS = 2000000


class NavigationError(Exception):
    """Base exception for navigation planning failures."""


class InvalidMapError(NavigationError):
    """Raised when a map does not contain usable x/y points."""


class EndpointNotDriveableError(NavigationError):
    """Raised when the start or target cannot be snapped to driveable space."""


class NoPathError(NavigationError):
    """Raised when no driveable path connects the start and target."""


class SpatialIndex:
    def __init__(self, points, bucket_size):
        self.bucket_size = max(float(bucket_size), 1e-9)
        self.buckets = {}

        for x, y, payload in points:
            key = self._key(x, y)
            self.buckets.setdefault(key, []).append((float(x), float(y), payload))

    def _key(self, x, y):
        return (
            int(math.floor(float(x) / self.bucket_size)),
            int(math.floor(float(y) / self.bucket_size)),
        )

    def nearest_within(self, x, y, radius):
        radius = float(radius)
        if radius < 0.0:
            return None

        x = float(x)
        y = float(y)
        center_x, center_y = self._key(x, y)
        bucket_radius = int(math.ceil(radius / self.bucket_size)) + 1
        best_distance_sq = radius * radius
        best_payload = None
        found = False

        for bucket_y in range(center_y - bucket_radius, center_y + bucket_radius + 1):
            for bucket_x in range(
                center_x - bucket_radius,
                center_x + bucket_radius + 1,
            ):
                bucket = self.buckets.get((bucket_x, bucket_y), ())

                for point_x, point_y, payload in bucket:
                    distance_sq = (x - point_x) ** 2 + (y - point_y) ** 2
                    if distance_sq <= best_distance_sq + 1e-12:
                        best_distance_sq = distance_sq
                        best_payload = payload
                        found = True

        if not found:
            return None

        return math.sqrt(best_distance_sq), best_payload


class NavigationGrid:
    def __init__(self, origin_x, origin_y, resolution, driveable, clearance):
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.driveable = driveable
        self.clearance = clearance
        self.rows = len(driveable)
        self.cols = len(driveable[0]) if self.rows else 0
        self._driveable_points = self._build_driveable_points()
        self._driveable_index = SpatialIndex(
            self._driveable_points,
            self.resolution,
        )

    def _build_driveable_points(self):
        points = []

        for row in range(self.rows):
            for col in range(self.cols):
                if self.driveable[row][col]:
                    x, y = self.cell_to_world((row, col))
                    points.append((x, y, (row, col)))

        return points

    def in_bounds(self, cell):
        row, col = cell
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_driveable(self, cell):
        if not self.in_bounds(cell):
            return False

        row, col = cell
        return self.driveable[row][col]

    def clearance_at(self, cell):
        row, col = cell
        return self.clearance[row][col]

    def cell_to_world(self, cell):
        row, col = cell
        return (
            self.origin_x + col * self.resolution,
            self.origin_y + row * self.resolution,
        )

    def world_to_cell(self, x, y):
        col = int(math.floor((float(x) - self.origin_x) / self.resolution + 0.5))
        row = int(math.floor((float(y) - self.origin_y) / self.resolution + 0.5))
        return row, col

    def nearest_driveable_cell(self, x, y, max_distance_m):
        result = self._driveable_index.nearest_within(x, y, max_distance_m)
        if result is None:
            return None

        return result[1]

    def neighbors(self, cell):
        row, col = cell

        for d_row in (-1, 0, 1):
            for d_col in (-1, 0, 1):
                if d_row == 0 and d_col == 0:
                    continue

                neighbor = row + d_row, col + d_col

                if d_row and d_col:
                    if not self.is_driveable((row + d_row, col)):
                        continue

                    if not self.is_driveable((row, col + d_col)):
                        continue

                if self.is_driveable(neighbor):
                    yield neighbor


def plan_path(
    point_map,
    start_xy,
    target_xy,
    max_map_distance_m=NAV_MAX_MAP_DISTANCE_M,
    grid_resolution_m=NAV_GRID_RESOLUTION_M,
    clearance_cost_weight=NAV_CLEARANCE_COST_WEIGHT,
    endpoint_snap_max_distance_m=NAV_ENDPOINT_SNAP_MAX_DISTANCE_M,
):
    """Plan a driveable path through the pointcloud map using Theta*."""

    max_map_distance_m = _require_positive(max_map_distance_m, "max_map_distance_m")
    grid_resolution_m = _require_positive(grid_resolution_m, "grid_resolution_m")
    endpoint_snap_max_distance_m = min(
        _require_nonnegative(
            endpoint_snap_max_distance_m,
            "endpoint_snap_max_distance_m",
        ),
        max_map_distance_m,
    )
    grid = build_navigation_grid(
        point_map,
        max_map_distance_m=max_map_distance_m,
        grid_resolution_m=grid_resolution_m,
        start_xy=start_xy,
        target_xy=target_xy,
    )
    start = _resolve_endpoint(
        grid,
        start_xy,
        endpoint_snap_max_distance_m,
        "start",
    )
    target = _resolve_endpoint(
        grid,
        target_xy,
        endpoint_snap_max_distance_m,
        "target",
    )

    cells = _theta_star(
        grid,
        start,
        target,
        max_map_distance_m=float(max_map_distance_m),
        clearance_cost_weight=float(clearance_cost_weight),
    )
    return [_cell_to_waypoint(grid, cell) for cell in cells]


def build_navigation_grid(
    point_map,
    max_map_distance_m=NAV_MAX_MAP_DISTANCE_M,
    grid_resolution_m=NAV_GRID_RESOLUTION_M,
    start_xy=None,
    target_xy=None,
):
    max_map_distance_m = _require_positive(max_map_distance_m, "max_map_distance_m")
    grid_resolution_m = _require_positive(grid_resolution_m, "grid_resolution_m")
    map_points = _extract_map_points(point_map)
    origin_x, origin_y, rows, cols = _grid_bounds(
        map_points,
        grid_resolution_m,
        max_map_distance_m,
        start_xy,
        target_xy,
    )

    if rows * cols > MAX_GRID_CELLS:
        raise NavigationError(
            f"Navigation grid is too large: {rows} rows x {cols} cols"
        )

    map_index = SpatialIndex(
        ((x, y, None) for x, y in map_points),
        max(max_map_distance_m, grid_resolution_m),
    )
    driveable = []

    for row in range(rows):
        driveable_row = []
        y = origin_y + row * grid_resolution_m

        for col in range(cols):
            x = origin_x + col * grid_resolution_m
            driveable_row.append(
                map_index.nearest_within(x, y, max_map_distance_m) is not None
            )

        driveable.append(driveable_row)

    clearance = _compute_clearance(driveable, grid_resolution_m)
    return NavigationGrid(origin_x, origin_y, grid_resolution_m, driveable, clearance)


def _extract_map_points(point_map):
    if hasattr(point_map, "points"):
        raw_points = point_map.points
    elif isinstance(point_map, dict):
        raw_points = point_map.get("points", [])
    else:
        raw_points = point_map

    if raw_points is None:
        raise InvalidMapError("Map must contain at least one valid x/y point")

    points = []

    try:
        iterator = iter(raw_points)
    except TypeError:
        raise InvalidMapError("Map must contain at least one valid x/y point")

    for point in iterator:
        try:
            if isinstance(point, dict):
                x = point["x"]
                y = point["y"]
            else:
                x = point[0]
                y = point[1]

            points.append((float(x), float(y)))
        except (KeyError, TypeError, ValueError, IndexError):
            continue

    if not points:
        raise InvalidMapError("Map must contain at least one valid x/y point")

    return points


def _grid_bounds(points, resolution, max_map_distance_m, start_xy, target_xy):
    bounds_points = list(points)

    for endpoint in (start_xy, target_xy):
        if endpoint is not None:
            bounds_points.append(_xy(endpoint))

    padding = max_map_distance_m + resolution
    min_x = min(x for x, _y in bounds_points) - padding
    max_x = max(x for x, _y in bounds_points) + padding
    min_y = min(y for _x, y in bounds_points) - padding
    max_y = max(y for _x, y in bounds_points) + padding

    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution
    end_x = math.ceil(max_x / resolution) * resolution
    end_y = math.ceil(max_y / resolution) * resolution
    cols = int(round((end_x - origin_x) / resolution)) + 1
    rows = int(round((end_y - origin_y) / resolution)) + 1
    return origin_x, origin_y, rows, cols


def _compute_clearance(driveable, resolution):
    rows = len(driveable)
    cols = len(driveable[0]) if rows else 0

    if rows == 0 or cols == 0:
        return []

    feature_grid = [
        [math.inf if driveable[row][col] else 0.0 for col in range(cols)]
        for row in range(rows)
    ]
    row_distances = [_distance_transform_1d(row) for row in feature_grid]
    clearance = [[math.inf for _col in range(cols)] for _row in range(rows)]

    for col in range(cols):
        column_distances = _distance_transform_1d(
            [row_distances[row][col] for row in range(rows)]
        )

        for row in range(rows):
            clearance[row][col] = math.sqrt(column_distances[row]) * resolution

    return clearance


def _distance_transform_1d(values):
    count = len(values)
    finite_indexes = [
        index for index, value in enumerate(values) if not math.isinf(value)
    ]

    if not finite_indexes:
        return [math.inf for _index in range(count)]

    envelope = [0 for _index in range(len(finite_indexes))]
    boundaries = [0.0 for _index in range(len(finite_indexes) + 1)]
    envelope_index = 0
    envelope[0] = finite_indexes[0]
    boundaries[0] = -math.inf
    boundaries[1] = math.inf

    for q in finite_indexes[1:]:
        while True:
            previous = envelope[envelope_index]
            separator = (
                (values[q] + q * q) - (values[previous] + previous * previous)
            ) / (2.0 * q - 2.0 * previous)

            if separator > boundaries[envelope_index]:
                break

            envelope_index -= 1

        envelope_index += 1
        envelope[envelope_index] = q
        boundaries[envelope_index] = separator
        boundaries[envelope_index + 1] = math.inf

    distances = [0.0 for _index in range(count)]
    envelope_index = 0

    for q in range(count):
        while boundaries[envelope_index + 1] < q:
            envelope_index += 1

        nearest = envelope[envelope_index]
        distances[q] = (q - nearest) ** 2 + values[nearest]

    return distances


def _resolve_endpoint(grid, xy, max_snap_distance_m, label):
    x, y = _xy(xy)
    cell = grid.world_to_cell(x, y)

    if grid.is_driveable(cell):
        return cell

    snapped = grid.nearest_driveable_cell(x, y, max_snap_distance_m)
    if snapped is not None:
        return snapped

    raise EndpointNotDriveableError(
        f"{label} is not within {max_snap_distance_m:g} m of driveable map space"
    )


def _theta_star(
    grid,
    start,
    target,
    max_map_distance_m,
    clearance_cost_weight,
):
    if start == target:
        return [start]

    open_heap = []
    counter = 0
    g_score = {start: 0.0}
    parent = {start: start}
    closed = set()
    heapq.heappush(open_heap, (_heuristic(grid, start, target), counter, start))

    while open_heap:
        _priority, _counter, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        if current == target:
            return _reconstruct_path(parent, target)

        closed.add(current)

        for neighbor in grid.neighbors(current):
            if neighbor in closed:
                continue

            current_parent = parent.get(current, current)

            if current_parent != current and _line_of_sight(
                grid,
                current_parent,
                neighbor,
            ):
                source = current_parent
            else:
                source = current

            tentative_g = g_score[source] + _segment_cost(
                grid,
                source,
                neighbor,
                max_map_distance_m,
                clearance_cost_weight,
            )

            if tentative_g < g_score.get(neighbor, math.inf):
                parent[neighbor] = source
                g_score[neighbor] = tentative_g
                counter += 1
                priority = tentative_g + _heuristic(grid, neighbor, target)
                heapq.heappush(open_heap, (priority, counter, neighbor))

    raise NoPathError("No driveable path found between start and target")


def _line_of_sight(grid, start, target):
    for cell in _sample_segment_cells(grid, start, target):
        if not grid.is_driveable(cell):
            return False

    return True


def _segment_cost(
    grid,
    start,
    target,
    max_map_distance_m,
    clearance_cost_weight,
):
    start_x, start_y = grid.cell_to_world(start)
    target_x, target_y = grid.cell_to_world(target)
    distance = math.hypot(target_x - start_x, target_y - start_y)

    if distance <= 0.0:
        return 0.0

    penalties = []

    for cell in _sample_segment_cells(grid, start, target):
        if not grid.is_driveable(cell):
            return math.inf

        penalties.append(
            _clearance_penalty(grid.clearance_at(cell), max_map_distance_m)
        )

    average_penalty = sum(penalties) / len(penalties) if penalties else 0.0
    return distance * (1.0 + clearance_cost_weight * average_penalty)


def _sample_segment_cells(grid, start, target):
    start_x, start_y = grid.cell_to_world(start)
    target_x, target_y = grid.cell_to_world(target)
    distance = math.hypot(target_x - start_x, target_y - start_y)
    sample_spacing = max(grid.resolution / 2.0, 1e-9)
    steps = max(1, int(math.ceil(distance / sample_spacing)))
    cells = []
    last_cell = None

    for index in range(steps + 1):
        t = index / steps
        x = start_x + (target_x - start_x) * t
        y = start_y + (target_y - start_y) * t
        cell = grid.world_to_cell(x, y)

        if cell != last_cell:
            cells.append(cell)
            last_cell = cell

    return cells


def _clearance_penalty(clearance_m, max_map_distance_m):
    if max_map_distance_m <= 0.0 or math.isinf(clearance_m):
        return 0.0

    low_clearance_ratio = max(
        0.0,
        (max_map_distance_m - float(clearance_m)) / max_map_distance_m,
    )
    return low_clearance_ratio * low_clearance_ratio


def _heuristic(grid, cell, target):
    x, y = grid.cell_to_world(cell)
    target_x, target_y = grid.cell_to_world(target)
    return math.hypot(target_x - x, target_y - y)


def _reconstruct_path(parent, target):
    path = [target]
    current = target

    while parent[current] != current:
        current = parent[current]
        path.append(current)

    path.reverse()
    return path


def _cell_to_waypoint(grid, cell):
    x, y = grid.cell_to_world(cell)
    return {
        "x": round(x, 3),
        "y": round(y, 3),
    }


def _xy(value):
    if isinstance(value, dict):
        return float(value["x"]), float(value["y"])

    return float(value[0]), float(value[1])


def _require_positive(value, name):
    value = float(value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")

    return value


def _require_nonnegative(value, name):
    value = float(value)
    if value < 0.0:
        raise ValueError(f"{name} must be nonnegative")

    return value
