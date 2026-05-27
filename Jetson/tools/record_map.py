import argparse
import sys
import time
from pathlib import Path

JETSON_ROOT = Path(__file__).resolve().parents[1]
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from config import ARDUINO_PORT
from hardware.arduino_io import ArduinoIO
from navigation.point_cloud import PointCloudMap
from robot import CONTROL_INTERVAL_SEC, controller_velocity, create_chassis
from teleop.controller import Controller


DEFAULT_MAP_PATH = JETSON_ROOT / "maps" / "overlake_campus_map.json"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drive the robot manually and record driveable x/y points to JSON."
    )
    parser.add_argument(
        "--map",
        default=str(DEFAULT_MAP_PATH),
        help="JSON map path to create or append to.",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--new",
        action="store_true",
        help="Start a fresh map and overwrite the selected JSON file.",
    )
    mode.add_argument(
        "--append",
        action="store_true",
        help="Append to the selected JSON file. This is the default.",
    )
    parser.add_argument(
        "--sample-distance",
        type=float,
        default=0.25,
        help="Minimum distance in meters between recorded points.",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=0.20,
        help="Minimum seconds between position samples.",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=10,
        help="Save after this many new points.",
    )
    return parser.parse_args()


def load_map(args):
    if args.new:
        return PointCloudMap.new(args.map)

    return PointCloudMap.load(args.map, create=True)


def print_controls(point_map, args):
    mode = "new" if args.new else "append"
    print(f"Recording map: {point_map.path} ({mode}, {len(point_map)} existing points)")
    print(
        "Controls: drive normally, CROSS toggles heading hold, "
        "CIRCLE pauses recording, START saves and exits."
    )
    print(
        f"Sampling: {args.sample_distance:.2f} m minimum spacing, "
        f"{args.sample_interval:.2f} s minimum interval."
    )


def main():
    args = parse_args()
    args.sample_distance = max(0.0, args.sample_distance)
    args.sample_interval = max(0.0, args.sample_interval)
    args.save_every = max(1, args.save_every)

    point_map = load_map(args)
    print_controls(point_map, args)

    controller = Controller()
    correct_heading = False
    recording_enabled = True
    unsaved_points = 0
    last_sample_time = 0.0

    with ArduinoIO(port=ARDUINO_PORT) as arduino:
        chassis, gps, _imu = create_chassis(arduino)

        try:
            while True:
                if controller.update():
                    chassis.update_position()

                    if controller.get_button_down("START"):
                        break

                    if controller.get_button_down("CIRCLE"):
                        recording_enabled = not recording_enabled
                        state = "resumed" if recording_enabled else "paused"
                        print(f"Recording {state}.")

                    if controller.get_button_down("CROSS"):
                        correct_heading = not correct_heading
                        if correct_heading:
                            chassis.set_wanted_angle(chassis.get_position()[2])

                    forward, turn = controller_velocity(controller)
                    forward *= 1.5
                    if correct_heading:
                        turn = None
                    chassis.set_velocity(forward, turn)

                    now = time.monotonic()
                    if recording_enabled and now - last_sample_time >= args.sample_interval:
                        x, y, heading = chassis.get_position()
                        last_sample_time = now

                        if point_map.should_add_point(x, y, args.sample_distance):
                            point_map.add_point(
                                x,
                                y,
                                heading_deg=heading,
                                timestamp=time.time(),
                            )
                            unsaved_points += 1
                            print(f"Point {len(point_map)}: x={x:.2f}, y={y:.2f}")

                            if unsaved_points >= args.save_every:
                                point_map.save()
                                unsaved_points = 0
                else:
                    chassis.stop()

                time.sleep(CONTROL_INTERVAL_SEC)

        except KeyboardInterrupt:
            print()

        finally:
            chassis.stop()
            gps.close()
            point_map.save()
            print(f"Saved {len(point_map)} points to {point_map.path}")


if __name__ == "__main__":
    main()
