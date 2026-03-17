# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront
from robolab_turtlebot import Turtlebot, Rate, get_time
from visuals import detect_balls

import numpy as np
import cv2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False

    def main(self):
        turtle = self.turtle

        turtle.wait_for_point_cloud()

        turtle.register_bumper_event_cb(lambda msge : callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge : callback_button0_resume(self, msge))
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
        while (not turtle.is_shutting_down()) and (get_time() - space_detect_time < 0.8):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
                if get_time() - space_detect_time > 5:
                    continue
            else:
                # go forward
                turtle.cmd_velocity(0.4, 0)
                rate.sleep()

        #find ball turns on to it
        (center_y, center_x), radius = detect_balls(turtle)
        DEAD_CENTER_X = 640/2
        TOLERANCE_PIXEL_BAND = 15
        dist = DEAD_CENTER_X- center_x
        while(abs(dist) <TOLERANCE_PIXEL_BAND):
            ang_speed = 0.5 if dist < 0 else -0.5
            turtle.cmd_velocity(0, ang_speed)
            (center_y, center_x), radius = detect_balls(turtle)

            rate.sleep()





if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
