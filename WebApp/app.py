import atexit
import logging
from pathlib import Path

from flask import (
    Flask,
    Response,
    jsonify,
    request,
    send_file,
    send_from_directory,
    session,
)

from .config import (
    DEFAULT_CONFIG_PATH,
    find_enabled_stop,
    load_config,
    public_config,
    validate_pin,
)


WEBAPP_ROOT = Path(__file__).resolve().parent


def create_app(config_path=None, controller=None, config=None):
    config = config or load_config(config_path or DEFAULT_CONFIG_PATH)
    controller = controller or _create_default_controller()

    app = Flask(
        __name__,
        static_folder=str(WEBAPP_ROOT / "static"),
        static_url_path="/static",
    )
    app.secret_key = str(config.get("auth", {}).get("session_secret", "dev-secret"))
    app.config["DELIVERY_CONFIG"] = config
    app.config["ROBOT_CONTROLLER"] = controller

    @app.route("/")
    def index():
        return _no_store(send_from_directory(app.static_folder, "index.html"))

    @app.route("/api/config/public")
    def api_public_config():
        return _no_store(jsonify(public_config(config)))

    @app.route("/api/map-image")
    def api_map_image():
        return send_file(config["map"]["image_path"])

    @app.route("/api/status")
    def api_status():
        return jsonify(app.config["ROBOT_CONTROLLER"].status())

    @app.route("/api/jobs", methods=["POST"])
    def api_jobs():
        data = request.get_json(silent=True) or {}

        if not validate_pin(config, data.get("pin")):
            return jsonify({"error": "Invalid PIN"}), 403

        stop = find_enabled_stop(config, data.get("destination"))
        if stop is None:
            return jsonify({"error": "Unknown or disabled destination"}), 404

        try:
            status = app.config["ROBOT_CONTROLLER"].submit_job(stop)
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

        return jsonify(status), 202

    @app.route("/api/stop", methods=["POST"])
    def api_stop():
        data = request.get_json(silent=True) or {}

        if not validate_pin(config, data.get("pin")):
            return jsonify({"error": "Invalid PIN"}), 403

        app.config["ROBOT_CONTROLLER"].stop_job()
        return jsonify(app.config["ROBOT_CONTROLLER"].status())

    @app.route("/api/admin/login", methods=["POST"])
    def api_admin_login():
        data = request.get_json(silent=True) or {}

        if not validate_pin(config, data.get("pin"), admin=True):
            session.pop("admin", None)
            return jsonify({"error": "Invalid admin PIN"}), 403

        session["admin"] = True
        return jsonify({"ok": True})

    @app.route("/api/debug/status")
    def api_debug_status():
        if not _is_admin():
            return jsonify({"error": "Admin access required"}), 403

        return jsonify(app.config["ROBOT_CONTROLLER"].debug_status())

    @app.route("/api/admin/manual-control", methods=["POST"])
    def api_admin_manual_control():
        if not _is_admin():
            return jsonify({"error": "Admin access required"}), 403

        data = request.get_json(silent=True) or {}

        try:
            status = app.config["ROBOT_CONTROLLER"].manual_control(
                data.get("direction"),
                duration_sec=data.get("duration_sec"),
                sequence=data.get("sequence"),
                client_id=data.get("client_id"),
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 409

        return jsonify(status)

    @app.route("/api/debug/camera")
    def api_debug_camera():
        return _debug_stream_or_placeholder("camera")

    @app.route("/api/debug/deepscene")
    def api_debug_deepscene():
        return _debug_stream_or_placeholder("deepscene")

    def _is_admin():
        return bool(session.get("admin"))

    def _debug_stream_or_placeholder(mode):
        if not _is_admin():
            return jsonify({"error": "Admin access required"}), 403

        stream_factory = app.config["ROBOT_CONTROLLER"].camera_stream(mode)

        if stream_factory is None:
            placeholder = (
                WEBAPP_ROOT
                / "assets"
                / (
                    "deepscene_unavailable.svg"
                    if mode == "deepscene"
                    else "camera_unavailable.svg"
                )
            )
            return send_file(placeholder, mimetype="image/svg+xml")

        return Response(
            stream_factory(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    def _no_store(response):
        response.headers["Cache-Control"] = "no-store"
        return response

    @app.teardown_appcontext
    def _shutdown_on_teardown(_exception=None):
        return None

    return app


def _create_default_controller():
    from Jetson.robot_service import DeliveryRobotService

    return DeliveryRobotService()


def main():
    run_app(create_app())


def run_app(app):
    config = app.config["DELIVERY_CONFIG"]
    controller = app.config["ROBOT_CONTROLLER"]
    atexit.register(controller.shutdown)
    _quiet_request_logs()

    try:
        app.run(
            host=config.get("server", {}).get("host", "0.0.0.0"),
            port=int(config.get("server", {}).get("port", 8000)),
            debug=bool(config.get("server", {}).get("debug", False)),
            threaded=True,
        )
    finally:
        controller.shutdown()


def _quiet_request_logs():
    logging.getLogger("werkzeug").setLevel(logging.WARNING)


if __name__ == "__main__":
    main()
