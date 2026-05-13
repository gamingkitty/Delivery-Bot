from arduino_io import ArduinoIO
from motor import Motor
from encoder import Encoder
from sonar import Sonar
from gps import GPS
import time


# arduino = ArduinoIO(port="/dev/ttyUSB0", baudrate=115200)

# motor1 = Motor(arduino, 9, 8)
# motor2 = Motor(arduino, 10, 7)
#
# encoder1 = Encoder(arduino, 1)
# encoder2 = Encoder(arduino, 2)

sonar = Sonar()

sonar.set_rgb_mode(0)
sonar.set_color(0, 0x000000)
sonar.set_color(1, 0x000000)

gps = GPS()

print("Starting program in 5 seconds...")

time.sleep(5)

try:
    while True:
        # print(f"Motor1 Encoder: {encoder1.read() * (360 / 268.8)} degrees")
        # print(f"Motor2 Encoder: {encoder2.read() * (360 / 268.8)} degrees")
        dist = sonar.get_distance()
        print(dist)

        # if dist < 100:
        #     motor1.set_speed(0.1)
        #     motor2.set_speed(0.1)
        # else:
        #     motor1.stop()
        #     motor2.stop()

        gps.update()
        data = gps.get_data()
        print(data)
        print()

        # motor1.set_speed(0.5)
        # motor2.set_speed(0.5)
        # time.sleep(0.1)

except KeyboardInterrupt:
    print("hi")
    # motor1.stop()
    # motor2.stop()
    # arduino.close()

