# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront, get_depth
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from visuals import detect_balls
from math import pi, cos, sqrt, sin, atan2

import cv2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False

    def main(self):
        """Ferenc exits garage, then drives around the ball and parks back."""
        turtle = self.turtle

        print("Main started")

        turtle.wait_for_point_cloud()

        # initialize bumber and buttons
        turtle.register_bumper_event_cb(lambda msge : callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge : callback_button0_resume(self, msge))
        rate = Rate(10)

        # until robot finds garage exit spin
        # self.find_exit(rate)
        space_detect_time = get_time()
#
        # self.exit_garage(rate, space_detect_time)
#
        ## find and ball turn on to it
        self.rotate_toward_ball(rate)
        ## drives until ball is 40cm infront of camera
        self.drive_toward_ball(rate, 0.7)
        self.drive_around_ball(rate)


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
                turtle.cmd_velocity(0, 0.5)
            space = space_infront(turtle=turtle)
            rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()

    def exit_garage(self, rate, space_detect_time) -> None:
        turtle = self.turtle
        turtle.reset_odometry()
        sleep(0.1)
        while (not turtle.is_shutting_down()) and (get_time() - space_detect_time < 1.7):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                # go forward with 5° offset to negate early exit
                self.go_forward(0.28, turtle.get_odometry()[2], pi/36)
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
            im = turtle.get_rgb_image()
            cv2.circle(im, (center_x, center_y), int(radius), (0, 255, 0), 2)
            cv2.circle(im, (center_x, center_y), 2, (0, 0, 255), 3)
            cv2.imshow("IMAGE", im)
            cv2.waitKey(1)
            ang_speed = -0.5 if dist < 0 else 0.5

            if abs(dist) > TOLERANCE_PIXEL_BAND*3:
                ang_speed = -0.2 if dist < 0 else 0.2

            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0, ang_speed)
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
        if dist is None:
            dist = 0
        diff = dist - final_dist

        while (not turtle.is_shutting_down()) and (abs(diff) > DISTANCE_TOLERANCE):
            lin_speed = 0.15

            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                #if ball not totally infront, rotate
                if abs(DEAD_CENTER_X - center_x) >  TOLERANCE_PIXEL_BAND:
                    self.rotate_toward_ball(rate)
                else:
                    turtle.cmd_velocity(lin_speed, 0)
                
                (center_x, center_y), radius = detect_balls(turtle)
                dist = get_depth(turtle, center_x, center_y, radius)
                if dist is None:
                    break
                diff = dist - final_dist
                # print("distance from ball is :", dist, "diff from designated distance ", diff, "X_pixel distance: ", DEAD_CENTER_X - center_x)
                rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)
        (center_x, center_y), radius = detect_balls(turtle)
        dist = get_depth(turtle, center_x, center_y, radius)
        if dist is None:
            print("NO DISTANCE!!!")
            dist = 0
        diff = dist - final_dist
        print("distance achieved, final distance is :", dist, "diff from designated distance ", diff)
        rate.sleep()


    def drive_around_ball(self, rate) -> None:
        """When close enough to the ball drive around it from point to point of calculated hexagon"""
        turtle = self.turtle

        (center_x, center_y), radius = detect_balls(turtle)
        dist = get_depth(turtle, center_x, center_y, radius)
        if dist is None:
            print("nevidim ho možo")
            return

        turtle.cmd_velocity(0, 0)
        turtle.reset_odometry()
        sleep(0.1)
        current_coords = turtle.get_odometry()

        # hexagon trajectory
        points = self.calculate_points(dist, current_coords)

        print("\nPoints are:", points, "\n\n")

        # go from point to point for each point of the hexagon
        p_num = 0
        point_of_return = False  # final point the hexagon (starting point but rotated by 180°)
        for point in points:
            p_num += 1
            if p_num == len(points):
                point_of_return = True
            self.go_ptp(point, rate, point_of_return)

    def calculate_points(self, dist, coords) -> list:
        """Calculates coordinates of hexagon to drive around the ball"""
        points = []
        ball_radius = 0.04 # 4cm radius of ball

        # make all the points of a hexagon
        x = 0
        y = 0

        for i in range(5):
            angle = i * (pi / 3)
            if i == 0:
                # pythagoras theorem
                y = -(cos(pi / 6) * (dist + ball_radius))
                x = sqrt(((dist + ball_radius) ** 2) - (y ** 2))

            if i == 1:
                x += dist + ball_radius

            if i == 2:
                y = coords[1]
                x += sin(pi / 6) * (dist + ball_radius)

            if i == 3:
                y = cos(pi / 6) * (dist + ball_radius)
                x -= sin(pi / 6) * (dist + ball_radius)

            if i == 4:
                x -= dist + ball_radius
                angle = -2 * (pi / 3)

            points.append([x, y, angle])

        # point of return
        # starting point, but ferenc is looking the other way
        points.append([coords[0], coords[1], coords[2] + pi])

        return points

    def go_ptp(self, point, rate, point_of_return):
        """Function that navigates from one point of a hexagon to the next"""
        turtle = self.turtle
        cur_coords = turtle.get_odometry()

        # thresholds fo accurate enough stopping in given points
        dist_thresh = 0.07
        angle_thresh = 0.024
        angle_is_close_thresh = 0.05

        # current location and distance from goal point
        x = point[0] - cur_coords[0]
        y = point[1] - cur_coords[1]
        d = sqrt(x**2 + y**2)

        # calculate angle to the next point
        angle = atan2(y, x)

        # while ferenc is not rotated at the calculated angle -> rotate
        angle_diff = self.normalize_angle(angle - cur_coords[2])
        while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)

            elif abs(angle_diff) < angle_is_close_thresh:
                turtle.cmd_velocity(0, -0.21)
            else:
                turtle.cmd_velocity(0, -0.52)

            cur_coords = turtle.get_odometry()
            angle_diff = self.normalize_angle(angle - cur_coords[2])

            rate.sleep()

        # while ferenc is not located at x,y coords, drive forward:
        while (not turtle.is_shutting_down()) and (abs(d) > dist_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                self.go_forward(0.34, cur_coords[2], angle)

            cur_coords = turtle.get_odometry()
            x = point[0] - cur_coords[0]
            y = point[1] - cur_coords[1]
            d = sqrt(x**2 + y**2) # distance from point

            rate.sleep()

        # while ferenc is not rotated at the calculated angle -> rotate
        angle_diff = self.normalize_angle((point[2] + 0.1) - cur_coords[2])  # little over-rotation so it can spin only in one direction
        while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)

            elif point_of_return:
                turtle.cmd_velocity(0, -0.51)
            else:
                turtle.cmd_velocity(0, 0.51)

            cur_coords = turtle.get_odometry()
            angle_diff = self.normalize_angle((point[2]+0.03) - cur_coords[2])   # little over-rotation so it can spin only in one direction

            rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)

    def normalize_angle(self, angle):
        """Normalizes an angle to be strictly within -pi and pi"""
        return (angle + pi) % (2 * pi) - pi

    def go_forward(self, lin_velocity, current_angle, needed_angle):
        """Simple P regulated driving in a straight line"""
        turtle = self.turtle
        angle_diff = self.normalize_angle(needed_angle - current_angle)

        # based on how off course is our robot rotated >>> steer it to go straight
        Kp = 0.5
        angular_velocity = Kp * angle_diff

        turtle.cmd_velocity(lin_velocity, angular_velocity)

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
