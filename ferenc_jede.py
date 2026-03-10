# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume

from robolab_turtlebot import Turtlebot, Rate, get_time

import numpy as np
import cv2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, depth=True)
        self.stop = False

    def main(self):
        turtle = self.turtle

        turtle.register_bumper_event_cb(lambda msge : callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge : callback_button0_resume(self, msge))

        t = get_time()

        rate = Rate(10)
        while (not turtle.is_shutting_down()) and (get_time() - t < 1000):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(2)
            else:
                turtle.cmd_velocity(0.001, 0.1)
            rate.sleep()


if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
