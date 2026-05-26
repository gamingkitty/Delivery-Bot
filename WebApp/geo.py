def latlon_to_map_percent(lat, lon, top_left, bottom_right):
    lat = float(lat)
    lon = float(lon)

    left_lon = float(top_left["lon"])
    right_lon = float(bottom_right["lon"])
    top_lat = float(top_left["lat"])
    bottom_lat = float(bottom_right["lat"])

    x = (lon - left_lon) / (right_lon - left_lon)
    y = (top_lat - lat) / (top_lat - bottom_lat)

    return _clamp(x, 0.0, 1.0) * 100.0, _clamp(y, 0.0, 1.0) * 100.0
def _clamp(value, low, high):
    return max(low, min(high, float(value)))
