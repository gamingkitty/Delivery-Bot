import math


EARTH_RADIUS_M = 6378137.0


def latlon_to_xy(lat, lon, origin):
    origin_lat, origin_lon = _origin_latlon(origin)
    origin_lat_rad = math.radians(origin_lat)
    d_lat = math.radians(float(lat) - origin_lat)
    d_lon = math.radians(float(lon) - origin_lon)

    x = EARTH_RADIUS_M * d_lon * math.cos(origin_lat_rad)
    y = EARTH_RADIUS_M * d_lat
    return x, y


def xy_to_latlon(x, y, origin):
    origin_lat, origin_lon = _origin_latlon(origin)
    origin_lat_rad = math.radians(origin_lat)

    lat = origin_lat + math.degrees(float(y) / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(
        float(x) / (EARTH_RADIUS_M * math.cos(origin_lat_rad))
    )
    return lat, lon


def path_xy_to_latlon(path, origin):
    points = []

    for waypoint in path:
        x = waypoint["x"] if isinstance(waypoint, dict) else waypoint[0]
        y = waypoint["y"] if isinstance(waypoint, dict) else waypoint[1]
        lat, lon = xy_to_latlon(x, y, origin)
        points.append(
            {
                "x_m": float(x),
                "y_m": float(y),
                "lat": lat,
                "lon": lon,
            }
        )

    return points


def _origin_latlon(origin):
    if isinstance(origin, dict):
        return float(origin["lat"]), float(origin["lon"])

    return float(origin[0]), float(origin[1])

