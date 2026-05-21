# Delivery-Bot
Code for an autonomous delivery bot created for my senior project. The delivery bot is controlled by a jetson nano and an arduino nano.

The Arduino sketch now exposes only the motor and encoder commands used by the
Jetson drive code. Robot dimensions, motor pins, serial ports, and PID/feed-forward
tuning values live in `Jetson/config.py`.

## Layout

- `Arduino/` contains the Arduino Nano motor, encoder, and IMU bridge sketch.
- `Jetson/main.py` is the Jetson-side runtime entry point.
- `Jetson/config.py` keeps robot dimensions, pins, ports, and tuning constants.
- `Jetson/drive/` contains the differential-drive chassis, motors, and encoders.
- `Jetson/hardware/` contains serial, GPS, IMU, and sonar hardware interfaces.
- `Jetson/mapping/` contains pointcloud map storage helpers.
- `Jetson/maps/` is the default location for saved map JSON files.
- `Jetson/navigation/` contains grid generation and path planning over maps.
- `Jetson/teleop/` contains manual controller input.
- `Jetson/tools/` contains maintenance utilities such as PID tuning.

Future camera feedback code should fit beside these as a focused package, for
example `Jetson/vision/`, once that system has more than one small module.

Run the main Jetson program with:

```bash
python Jetson/main.py
```

Run the motor tuning utility with:

```bash
python Jetson/tools/pid_tuning.py
```

Create or extend a driveable pointcloud map with:

```bash
python Jetson/tools/record_map.py --map Jetson/maps/driveable_points.json
```

Use `--new` to overwrite the selected map, or omit it to append to an existing
map. The recorder saves JSON points in meters as `x` and `y`, with heading and
timestamp included for later navigation work.

Plan a path in code with:

```python
from mapping.point_cloud import PointCloudMap
from navigation.path_planner import plan_path

point_map = PointCloudMap.load("Jetson/maps/driveable_points.json")
path = plan_path(point_map, start_xy=(0.0, 0.0), target_xy=(2.0, 1.0))
```

Navigation defaults live in `Jetson/config.py`. The planner defaults to a 1 m
maximum distance from recorded map points, a 0.2 m grid resolution, and a
balanced clearance cost that favors centered paths without ignoring distance.

Run hardware-free planner tests with:

```bash
python -m unittest discover -s tests
```
