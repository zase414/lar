# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront
from robolab_turtlebot import Turtlebot, Rate, get_time
from visuals import detect_balls
from math import pi, cos, sqrt

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False

    def main(self):
        """Robot named Ferenc exits garage, then drives around the ball and parks back."""
        turtle = self.turtle

        turtle.wait_for_point_cloud()

        # initialize bumber and buttons
        turtle.register_bumper_event_cb(lambda msge : callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge : callback_button0_resume(self, msge))
        rate = Rate(10)

        # checking bumber status
        while self.stop:
            turtle.cmd_velocity(0, 0)
            rate.sleep()

        # until robot finds garage exit spin
        self.find_exit(rate)
        space_detect_time = get_time()

        self.exit_garage(rate, space_detect_time)

        # find and ball turn on to it
        self.find_ball(rate)

        # points = self.calculate_points(1, [0, 0, 0]) # checking output


    def find_exit(self, rate):
        """Until robot finds garage exit spin"""
        turtle = self.turtle
        space = space_infront(turtle=turtle)
        while (not turtle.is_shutting_down()) and (not space):
            print("Finding exit")
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0.002, 0.4)
            space = space_infront(turtle=turtle)
            rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()

    def exit_garage(self, rate, space_detect_time):
        turtle = self.turtle
        while (not turtle.is_shutting_down()) and (get_time() - space_detect_time < 1.2):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                # go forward
                turtle.cmd_velocity(0.4, 0)
                rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()

    def find_ball(self, rate):
        """Until ferenc finds ball he's spinning"""
        turtle = self.turtle
        (center_x, center_y), radius = detect_balls(turtle)

        DEAD_CENTER_X = 640 / 2
        TOLERANCE_PIXEL_BAND = 10
        dist = DEAD_CENTER_X - center_x

        while (not turtle.is_shutting_down()) and (abs(dist) > TOLERANCE_PIXEL_BAND):
            ang_speed = -0.3 if dist < 0 else 0.3

            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)

            turtle.cmd_velocity(0.002, ang_speed)
            (center_x, center_y), radius = detect_balls(turtle)
            dist = DEAD_CENTER_X - center_x
            print("distance: ", dist, "center x y ", center_x, center_y, "should while end",
                  abs(dist) > TOLERANCE_PIXEL_BAND)
            rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()


    def drive_around_ball(self, rate, distance):
        turtle = self.turtle

        turtle.cmd_velocity(0, 0)
        turtle.reset_odometry()
        current_coords = turtle.get_odometry()
        # points = self.calculate_points(distance, current_coords)

    def calculate_points(self, dist, coords) -> list:
        """Calculates coordinates of sextagon to drive around the ball"""
        points = []
        ball_radius = 0.004
        ball_center = [dist + ball_radius, 0, 0] # 4cm radius of ball

        # make all the points of a sextagon
        for i in range(5):
            angle = i*(pi/3)

            # pythagoras theorem
            y = cos(pi/6)*(dist + ball_radius)
            x = sqrt(y**2 - (dist + ball_radius)**2)
            points[i] = [x, y, angle]

        # point of return
        # starting point, but ferenc is looking the other way
        points[5] = [coords[0], coords[2], coords[3] + pi]
        print(points)

        return points




if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
