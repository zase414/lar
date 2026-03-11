# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront
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
        rate = Rate(10)

        # checking bumber status
        while self.stop:
            turtle.cmd_velocity(0, 0)
            rate.sleep()

        # until robot finds garage exit spin
        space = space_infront(turtle=turtle)
        while (not turtle.is_shutting_down()) and (not space):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0.002, 0.4)
            space = space_infront(turtle=turtle)
            rate.sleep()

        space_detect_time = get_time()

        # reset parameters
        turtle.cmd_velocity(0, 0)

        # garage exit
        while (not turtle.is_shutting_down()) and (get_time() - space_detect_time < 3):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
                if get_time() - space_detect_time > 5:
                    continue
            else:
                # go forward
                turtle.cmd_velocity(0.3, 0.002)
                rate.sleep()


if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
