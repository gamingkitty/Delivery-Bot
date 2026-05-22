import os
import threading

# Allows pygame joystick input without opening a display window
os.environ["SDL_VIDEODRIVER"] = "dummy"

import pygame as pg


class Controller:
    BUTTON_NAMES = (
        "CROSS", "CIRCLE", "None_1", "SQUARE",
        "TRIANGLE", "None_2", "L1", "R1",
        "L2", "R2", "SELECT", "START", "MODE",
    )

    VIRTUAL_BUTTON_NAMES = (
        "L_HAT_LEFT", "L_HAT_RIGHT", "L_HAT_DOWN", "L_HAT_UP",
        "L_AXIS_LEFT", "L_AXIS_RIGHT", "L_AXIS_UP", "L_AXIS_DOWN",
        "R_AXIS_LEFT", "R_AXIS_RIGHT", "R_AXIS_UP", "R_AXIS_DOWN",
    )

    ALL_BUTTON_NAMES = BUTTON_NAMES + VIRTUAL_BUTTON_NAMES

    def __init__(self, device_index=0, deadzone=0.15, virtual_threshold=0.95):
        self.device_index = device_index
        self.deadzone = deadzone
        self.virtual_threshold = virtual_threshold

        self.js = None
        self.connected = False

        self.buttons = {name: False for name in self.ALL_BUTTON_NAMES}
        self.last_buttons = self.buttons.copy()

        self.axes = {
            "left_x": 0.0,
            "left_y": 0.0,
            "right_x": 0.0,
            "right_y": 0.0,
        }

        self.hat = (0, 0)

        self.lock = threading.Lock()

        pg.display.init()
        pg.joystick.init()

        self.connect()

    def connect(self):
        if not os.path.exists("/dev/input/js0"):
            self.connected = False
            self.js = None
            return False

        try:
            pg.joystick.quit()
            pg.joystick.init()

            self.js = pg.joystick.Joystick(self.device_index)
            self.js.init()

            self.connected = True
            return True

        except Exception as e:
            print("Failed to connect controller:", e)
            self.connected = False
            self.js = None
            return False

    def update(self):
        """
        Call this once per loop before reading inputs.
        """
        with self.lock:
            if self.js is None or not self.connected:
                self.connect()
                return False

            pg.event.pump()

            self.last_buttons = self.buttons.copy()

            # Real buttons
            real_buttons = []
            for i in range(min(13, self.js.get_numbuttons())):
                real_buttons.append(bool(self.js.get_button(i)))

            while len(real_buttons) < 13:
                real_buttons.append(False)

            for name, value in zip(self.BUTTON_NAMES, real_buttons):
                self.buttons[name] = value

            # D-pad / hat
            if self.js.get_numhats() > 0:
                self.hat = self.js.get_hat(0)
            else:
                self.hat = (0, 0)

            hat_x, hat_y = self.hat

            self.buttons["L_HAT_LEFT"] = hat_x < 0
            self.buttons["L_HAT_RIGHT"] = hat_x > 0
            self.buttons["L_HAT_DOWN"] = hat_y < 0
            self.buttons["L_HAT_UP"] = hat_y > 0

            # Analog axes
            raw_axes = []
            for i in range(min(4, self.js.get_numaxes())):
                raw_axes.append(self._apply_deadzone(self.js.get_axis(i)))

            while len(raw_axes) < 4:
                raw_axes.append(0.0)

            self.axes["left_x"] = raw_axes[0]
            self.axes["left_y"] = raw_axes[1]
            self.axes["right_x"] = raw_axes[2]
            self.axes["right_y"] = raw_axes[3]

            # Virtual analog-direction buttons, matching Hiwonder's style
            self.buttons["L_AXIS_LEFT"] = self.axes["left_x"] < -self.virtual_threshold
            self.buttons["L_AXIS_RIGHT"] = self.axes["left_x"] > self.virtual_threshold
            self.buttons["L_AXIS_UP"] = self.axes["left_y"] < -self.virtual_threshold
            self.buttons["L_AXIS_DOWN"] = self.axes["left_y"] > self.virtual_threshold

            self.buttons["R_AXIS_LEFT"] = self.axes["right_x"] < -self.virtual_threshold
            self.buttons["R_AXIS_RIGHT"] = self.axes["right_x"] > self.virtual_threshold
            self.buttons["R_AXIS_UP"] = self.axes["right_y"] < -self.virtual_threshold
            self.buttons["R_AXIS_DOWN"] = self.axes["right_y"] > self.virtual_threshold

            return True

    def _apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def get_button(self, name):
        """
        Returns True while a button is held.
        Example: controller.get_button("CROSS")
        """
        return self.buttons.get(name, False)

    def get_button_down(self, name):
        """
        Returns True only on the frame the button is first pressed.
        """
        return self.buttons.get(name, False) and not self.last_buttons.get(name, False)

    def get_button_up(self, name):
        """
        Returns True only on the frame the button is released.
        """
        return not self.buttons.get(name, False) and self.last_buttons.get(name, False)

    def get_axis(self, name):
        """
        Axis names:
        left_x, left_y, right_x, right_y
        """
        return self.axes.get(name, 0.0)

    def get_hat(self):
        """
        Returns d-pad as (x, y), like:
        left  = (-1, 0)
        right = (1, 0)
        up    = (0, 1)
        down  = (0, -1)
        """
        return self.hat
