import sys
import time
from pathlib import Path


JETSON_ROOT = Path(__file__).resolve().parents[1]
if str(JETSON_ROOT) not in sys.path:
    sys.path.insert(0, str(JETSON_ROOT))

from config import ARDUINO_PORT
from hardware.arduino_io import ArduinoIO
from robot import CONTROL_INTERVAL_SEC, controller_velocity, create_chassis
from teleop.controller import Controller


# sudo pkill wpa_supplicant
# sudo ip link set wlan0 up
# sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant-wlan0.conf -D nl80211
# sudo dhclient wlan0
# ping -c 4 8.8.8.8


def main():
    controller = Controller()

    with ArduinoIO(port=ARDUINO_PORT) as arduino:
        chassis, gps, _imu = create_chassis(arduino)

        correct_heading = False

        try:
            while True:
                if controller.update():
                    chassis.update_position()
                    print(chassis.get_position())
                    if controller.get_button_down("CROSS"):
                        correct_heading = not correct_heading
                        if correct_heading:
                            chassis.set_wanted_angle(chassis.get_position()[2])
                    forward, turn = controller_velocity(controller)
                    if correct_heading:
                        turn = None
                    chassis.set_velocity(forward, turn)
                else:
                    chassis.stop()

                time.sleep(CONTROL_INTERVAL_SEC)

        except KeyboardInterrupt:
            print()

        finally:
            chassis.stop()
            gps.close()


if __name__ == "__main__":
    main()

