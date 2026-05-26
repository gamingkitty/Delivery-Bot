# Delivery Bot WebApp

Jetson-hosted campus dispatch app for sending the delivery bot between configured stops.

## Run

Run on the Jetson with robot hardware enabled:

```bash
python Jetson/main.py
```

Run the same web app module directly:

```bash
python -m WebApp.app
```

Then open:

```text
http://<jetson-ip>:8000
```

The web app always uses the real Jetson robot service. It does not include a simulated robot controller.

## Edit The Campus Setup

Edit `WebApp/config/app.json`:

- `map.image`: campus map image path, relative to `WebApp/config/`.
- `map.top_left`: latitude/longitude of the image's top-left corner.
- `map.bottom_right`: latitude/longitude of the image's bottom-right corner.
- `stops`: dropdown destinations.
- `auth.user_pin`: dispatch and stop PIN.
- `auth.admin_pin`: debug panel PIN.

Edit `Jetson/config.py` for robot-specific settings:

- serial ports
- GPS origin
- point map path
- navigation timing and tolerances
- camera and DeepScene settings

For v1, the map image should be north-up. The app maps latitude/longitude linearly inside the configured top-left and bottom-right bounds.
