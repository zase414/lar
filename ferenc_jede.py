# Ferenc je robot

from __future__ import print_function
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time
import numpy as np

import cv2

class Button(IntEnum):
    BUTTON0 = 0
    BUTTON1 = 1
    BUTTON2 = 2

class State(IntEnum):
    PRESSED = 1
    RELEASED = 0

class Bumper(IntEnum):
    LEFT = 0
    CENTER = 1
    RIGHT = 2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True)
        self.stop = False

    def _button_cb(self, msg):
        """Button event"""

        if (msg.state == State.PRESSED) and (msg.event == Button.BUTTON0):
            self.stop = False

        button_state = msg.state
        button = msg.button
        print('{} button {}'.format(button, button_state))
        print('Stopped? ', self.stop)

    def _bumper_cb(self, msg):
        """Bumber callback."""
        
        if msg.state == State.PRESSED:
            self.turtle.cmd_velocity(0, 0)
            self.stop = True

        bumper_state = msg.state
        bumper = msg.bumper

        print('{} bumper {}'.format(bumper, bumper_state))
        print('Stopped? ', self.stop)


    def detect_balls(self):
        turtle = self.turtle
        HUE_SIZE = 179
        HUE_REF = 125 #random green from color picker
        HUE_MAX = 0.9
        SAT_MIN = 0.1
        VALUE_MIN = 0.1

        while True:    
            im = turtle.get_rgb_image()
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
                    
            h = hsv[:, :, 0]
            s = hsv[:, :, 1]
            v = hsv[:, :, 2]

            mask = (
                (np.minimum(np.abs(h - HUE_REF), 180 - np.abs(h - HUE_REF)) < HUE_MAX) &
                (s > SAT_MIN) &
                (v > VALUE_MIN))

            filtered = im.copy()
            filtered[~mask] = 0

            cv2.imshow("HSV_FILTER", im_color)
            cv2.waitKey(1)


    def main(self):
        turtle = self.turtle
        
        turtle.register_bumper_event_cb(self._bumper_cb)
        turtle.register_button_event_cb(self._button_cb)

        self.detect_balls()

        t = get_time()

        rate = Rate(10)
        while (not turtle.is_shutting_down()) and (get_time() - t < 5):
            turtle.cmd_velocity(0.2)
            while self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()

            rate.sleep()

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
