import copy
import json
from pathlib import Path


WEBAPP_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = WEBAPP_ROOT / "config" / "app.json"


class ConfigError(ValueError):
    """Raised when the web app config is missing required values."""


def load_config(path=None):
    config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    config_path = config_path.resolve()

    with config_path.open("r", encoding="utf-8") as file:
        config = json.load(file)

    if not isinstance(config, dict):
        raise ConfigError("Config must be a JSON object")

    config["_config_path"] = str(config_path)
    config["_config_dir"] = str(config_path.parent)

    _require_mapping(config, "server")
    _require_mapping(config, "auth")
    _require_mapping(config, "map")
    _require_mapping(config, "ui")
    _require_mapping(config, "debug")

    map_config = config["map"]
    map_config["image_path"] = str(_resolve_path(map_config["image"], config_path.parent))

    _require_point(map_config, "top_left")
    _require_point(map_config, "bottom_right")

    if map_config["top_left"]["lat"] == map_config["bottom_right"]["lat"]:
        raise ConfigError("Map top_left.lat and bottom_right.lat cannot match")

    if map_config["top_left"]["lon"] == map_config["bottom_right"]["lon"]:
        raise ConfigError("Map top_left.lon and bottom_right.lon cannot match")

    stops = config.get("stops")
    if not isinstance(stops, list):
        raise ConfigError("Config must contain a stops list")

    seen_stop_ids = set()
    for stop in stops:
        _normalize_stop(stop)
        if stop["id"] in seen_stop_ids:
            raise ConfigError(f"Duplicate stop id: {stop['id']}")
        seen_stop_ids.add(stop["id"])

    return config


def public_config(config):
    public = {
        "map": {
            "image_url": "/api/map-image",
            "top_left": _public_point(config["map"]["top_left"]),
            "bottom_right": _public_point(config["map"]["bottom_right"]),
        },
        "stops": [
            public_stop(stop)
            for stop in config["stops"]
            if stop.get("enabled", True)
        ],
        "ui": copy.deepcopy(config["ui"]),
        "debug": {
            "enabled": bool(config.get("debug", {}).get("enabled", False)),
        },
    }
    return public


def public_stop(stop):
    return {
        "id": stop["id"],
        "name": stop["name"],
        "lat": float(stop["lat"]),
        "lon": float(stop["lon"]),
        "description": stop.get("description", ""),
        "enabled": bool(stop.get("enabled", True)),
    }


def find_enabled_stop(config, stop_id):
    for stop in config["stops"]:
        if stop["id"] == stop_id and stop.get("enabled", True):
            return stop
    return None


def validate_pin(config, pin, admin=False):
    auth = config.get("auth", {})
    allowed = {str(auth.get("admin_pin", ""))}

    if not admin:
        allowed.add(str(auth.get("user_pin", "")))

    return str(pin or "") in allowed


def _resolve_path(value, base_dir):
    if not value:
        raise ConfigError("Path value cannot be empty")

    path = Path(value)
    if not path.is_absolute():
        path = base_dir / path

    return path.resolve()


def _require_mapping(config, key):
    if not isinstance(config.get(key), dict):
        raise ConfigError(f"Config must contain object: {key}")


def _require_point(config, key):
    point = config.get(key)
    if not isinstance(point, dict):
        raise ConfigError(f"Config must contain point: {key}")

    try:
        point["lat"] = float(point["lat"])
        point["lon"] = float(point["lon"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(f"{key} must contain numeric lat and lon") from exc


def _normalize_stop(stop):
    if not isinstance(stop, dict):
        raise ConfigError("Each stop must be a JSON object")

    for key in ("id", "name", "lat", "lon"):
        if key not in stop:
            raise ConfigError(f"Stop is missing required key: {key}")

    stop["id"] = str(stop["id"]).strip()
    stop["name"] = str(stop["name"]).strip()

    if not stop["id"]:
        raise ConfigError("Stop id cannot be empty")

    if not stop["name"]:
        raise ConfigError(f"Stop name cannot be empty: {stop['id']}")

    try:
        stop["lat"] = float(stop["lat"])
        stop["lon"] = float(stop["lon"])
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"Stop lat/lon must be numeric: {stop['id']}") from exc

    if "x_m" in stop or "y_m" in stop:
        try:
            stop["x_m"] = float(stop["x_m"])
            stop["y_m"] = float(stop["y_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ConfigError(
                f"Stop x_m/y_m must both be numeric when set: {stop['id']}"
            ) from exc

    stop["enabled"] = bool(stop.get("enabled", True))


def _public_point(point):
    return {
        "lat": float(point["lat"]),
        "lon": float(point["lon"]),
    }
