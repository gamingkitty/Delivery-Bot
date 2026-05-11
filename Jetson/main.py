from arduino_io import ArduinoIO
from motor import Motor
import time


arduino = ArduinoIO(port="/dev/ttyUSB0", baudrate=115200)

while True:
    print("Rotating to 0...")
    arduino.servo_write(9, 0)

    time.sleep(2)

    print("Rotating to 180...")
    arduino.servo_write(9, 180)

    time.sleep(2)

