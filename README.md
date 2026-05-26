# Delivery-Bot
Code for an autonomous delivery bot created for my senior project. The delivery bot is controlled by a jetson nano and an arduino nano.

The Arduino sketch exposes the motor velocity, encoder read, and IMU commands
used by the Jetson drive code. Robot dimensions, motor pins, serial ports, and
PID/feed-forward tuning values live in `Jetson/config.py`.

## Layout

- `Arduino/` contains the Arduino Nano motor, encoder, and IMU bridge sketch.
- `Jetson/main.py` is the Jetson-side web dispatch runtime entry point.
- `Jetson/config.py` keeps robot dimensions, pins, ports, and tuning constants.
- `Jetson/drive/` contains the differential-drive chassis and motor/encoder handling.
- `Jetson/hardware/` contains serial, GPS, IMU, and sonar hardware interfaces.
- `Jetson/navigation/` contains pointcloud map storage, grid generation, and path planning.
- `Jetson/maps/` is the default location for saved map JSON files.
- `Jetson/teleop/` contains manual controller input.
- `Jetson/tools/` contains maintenance utilities such as PID tuning.
- `WebApp/` contains the Jetson-hosted campus dispatch web app.

Future camera feedback code should fit beside these as a focused package, for
example `Jetson/vision/`, once that system has more than one small module.

Run the Jetson web dispatch runtime with:

```bash
python Jetson/main.py
```

This starts the web app with real robot hardware enabled. Run manual controller
teleop with:

```bash
python Jetson/teleop/teleop.py
```

Run the same web app module directly with:

```bash
python -m WebApp.app
```

Edit `WebApp/config/app.json` to set the campus map image, GPS bounds, named
stops, and PINs. Edit `Jetson/config.py` for robot ports, navigation defaults,
point map path, camera, and DeepScene settings.

Run the motor tuning utility with:

```bash
python Jetson/tools/pid_tuning.py
```

Open the local map visualizer in a browser with:

```text
Jetson/tools/map_visualizer.html
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
from navigation.point_cloud import PointCloudMap
from navigation.path_planner import plan_path

point_map = PointCloudMap.load("Jetson/maps/driveable_points.json")
path = plan_path(point_map, start_xy=(0.0, 0.0), target_xy=(2.0, 1.0))
```

Navigation defaults live in `Jetson/config.py`. The planner defaults to a 1 m
maximum distance from recorded map points, a 0.2 m grid resolution, and a
balanced clearance cost that favors centered paths without ignoring distance.
The visualizer has matching editable defaults for local path experiments.

Run hardware-free planner tests with:

```bash
python -m unittest discover -s tests
```
