# Ferenc je robot

from __future__ import print_function
from enum import Enum
from robolab_turtlebot import Turtlebot, Rate, get_time

class Button(Enum):
    BUTTON0 = 0
    BUTTON1 = 1
    BUTTON2 = 2

class State(Enum):
    PRESSED = 1
    RELEASED = 0

class Bumper(Enum):
    LEFT = 0
    CENTER = 1
    RIGHT = 2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot()
        self.stop = False

    def _button_cb(self, msg):
        """Button event"""

        if (msg.state == State.PRESSED) and (msg.event == Button.BUTTON0):
            self.stop = False

        button_state = msg.state
        button = msg.button
        print('{} button {}'.format(button, button_state))

    def _bumper_cb(self, msg):
        """Bumber callback."""
        
        if msg.state == State.PRESSED:
            self.turtle.cmd_velocity(0, 0)
            self.stop = True

        bumper_state = msg.state
        bumper = msg.bumper

        print('{} bumper {}'.format(bumper, bumper_state))

    def main(self):
        turtle = self.turtle
        turtle.register_bumper_event_cb(self._bumper_cb)
        turtle.register_button_event_cb(self._button_cb)

        t = get_time()

        rate = Rate(10)
        while (not turtle.is_shutting_down()) and (get_time() - t < 2):
            turtle.cmd_velocity(0.1)
            while self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()

            rate.sleep()

        while (not turtle.is_shutting_down()) and (get_time() - t < (2*3.14)/0.6):
            turtle.cmd_velocity(0.2, 0.4)
            while self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()

            rate.sleep()

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
