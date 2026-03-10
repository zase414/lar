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

        if (msg.state == State.PRESSED) and (msg.button == Button.BUTTON0):
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

    def main(self):
        turtle = self.turtle
        
        turtle.register_bumper_event_cb(self._bumper_cb)
        turtle.register_button_event_cb(self._button_cb)

        #self.detect_balls()

        t = get_time()

        rate = Rate(10)
        while (not turtle.is_shutting_down()) and (get_time() - t < 3):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(1)
            else:
                turtle.cmd_velocity(0.1)
            rate.sleep()

        while (not turtle.is_shutting_down()) and (get_time() - t < 15):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(1)
            else:
                turtle.cmd_velocity(0.05, 0.15)
            rate.sleep()

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
