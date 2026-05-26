import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from WebApp.app import create_app, run_app
from WebApp.config import DEFAULT_CONFIG_PATH, load_config
from robot_service import DeliveryRobotService


def main():
    config = load_config(DEFAULT_CONFIG_PATH)
    app = create_app(config=config, controller=DeliveryRobotService())
    run_app(app)


if __name__ == "__main__":
    main()
