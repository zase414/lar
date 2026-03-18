# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront, get_depth
from robolab_turtlebot import Turtlebot, Rate, get_time
from visuals import detect_balls
from math import pi, cos, sqrt, sin

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False

    def main(self):
        """Ferenc exits garage, then drives around the ball and parks back."""
        turtle = self.turtle

        turtle.wait_for_point_cloud()

        # initialize bumber and buttons
        turtle.register_bumper_event_cb(lambda msge : callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge : callback_button0_resume(self, msge))
        rate = Rate(10)

        # until robot finds garage exit spin
        self.find_exit(rate)
        space_detect_time = get_time()

        self.exit_garage(rate, space_detect_time)

        # find and ball turn on to it
        self.rotate_toward_ball(rate)
        # drives until ball is 40cm infront of camera
        self.drive_toward_ball(rate, 0.40)



    def find_exit(self, rate) -> None:
        """Until robot finds garage exit spin"""
        turtle = self.turtle
        space = space_infront(turtle=turtle)
        while (not turtle.is_shutting_down()) and (not space):
            print("Finding exit")
            if self.stop:
                print("Stopped")
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                print("Spin")
                turtle.cmd_velocity(0.002, 0.4)
            space = space_infront(turtle=turtle)
            rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()

    def exit_garage(self, rate, space_detect_time) -> None:
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

    def rotate_toward_ball(self, rate) -> None:
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

    def drive_toward_ball(self, rate, final_dist) -> None:
        """until distance to ball is final_dist"""
        turtle = self.turtle
        DISTANCE_TOLERANCE = 0.05 #5cm
        TOLERANCE_PIXEL_BAND = 15
        DEAD_CENTER_X = 640 / 2

        (center_x, center_y), radius = detect_balls(turtle)
        dist = get_depth(turtle, center_x, center_y, radius)
        diff = dist - final_dist

        while (not turtle.is_shutting_down()) and (abs(diff) > DISTANCE_TOLERANCE):
            lin_speed = -0.5 if diff < 0 else 0.5

            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)

            #if ball not totally infront, rotate
            if (abs(DEAD_CENTER_X - center_x) >  TOLERANCE_PIXEL_BAND):
                self.rotate_toward_ball(rate)


            turtle.cmd_velocity(lin_speed, 0)
            
            (center_x, center_y), radius = detect_balls(turtle)
            dist = get_depth(turtle, center_x, center_y, radius)
            diff = final_dist - dist
            print("distance from ball is :", dist, "diff from designated distance ", diff)
            rate.sleep()

        # reset params
        print("distance achieved, final distance is :", dist, "diff from designated distance ", diff)
        turtle.cmd_velocity(0, 0)
        rate.sleep()






    def drive_around_ball(self, rate, distance) -> None:
        turtle = self.turtle

        turtle.cmd_velocity(0, 0)
        turtle.reset_odometry()
        current_coords = turtle.get_odometry()
        points = self.calculate_points(distance, current_coords)

    def calculate_points(self, dist, coords) -> list:
        """Calculates coordinates of sextagon to drive around the ball"""
        points = []
        ball_radius = 0.004
        ball_center = [dist + ball_radius, 0, 0]  # 4cm radius of ball

        # make all the points of a sextagon
        x = 0
        y = 0

        for i in range(5):
            angle = i * (pi / 3)
            if i == 0:
                # pythagoras theorem
                y = cos(pi / 6) * (dist + ball_radius)
                x = sqrt(((dist + ball_radius) ** 2) - (y ** 2))

            if i == 1:
                x += dist + 0.004

            if i == 2:
                y = coords[1]
                x += sin(pi / 6) * (dist + ball_radius)

            if i == 3:
                y = -(cos(pi / 6) * (dist + ball_radius))
                x -= sin(pi / 6) * (dist + ball_radius)

            if i == 4:
                x -= dist + 0.004

            points.append([x, y, angle])

        # point of return
        # starting point, but ferenc is looking the other way
        points.append([coords[0], coords[1], coords[2] + pi])
        print(points)

        return points

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
