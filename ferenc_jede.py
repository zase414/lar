# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from visuals import edging
from robolab_turtlebot import Turtlebot, Rate, get_time

import numpy as np
import cv2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False

    def main(self):
        turtle = self.turtle

        turtle.register_bumper_event_cb(lambda msge : callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge : callback_button0_resume(self, msge))
        turtle.wait_for_point_cloud()
        t = get_time()

        rate = Rate(10)
        # dokud ferenc nenajde výjezd z garáže, tak se spiní
        edgin = edging(turtle=turtle)
        while (not turtle.is_shutting_down()) and (not edgin):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                print("rotating")
                turtle.cmd_velocity(0.002, 0.4)
            print(edgin)
            edgin = edging(turtle=turtle)
            rate.sleep()

        stop_edgin_time = get_time()

        while (not turtle.is_shutting_down()) and (get_time() - stop_edgin_time < 5):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0.1)
            rate.sleep()


if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
