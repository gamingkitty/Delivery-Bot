import time
from hardware.arduino_io import ArduinoIO
from config import ARDUINO_PORT
from robot import CONTROL_INTERVAL_SEC, controller_velocity, create_chassis
from teleop.controller import Controller


def main():
    controller = Controller()

    with ArduinoIO(port=ARDUINO_PORT) as arduino:
        chassis, gps, _imu = create_chassis(arduino)
        data = gps.get_data()
        print(data)

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
