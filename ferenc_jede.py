# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront, get_depth
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from visuals import detect_balls
from math import pi, cos, sqrt, sin, atan2
from typing import Optional

import cv2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False
        self.saved_odometry = list()
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
        self.find_exit(rate)
        space_detect_time = get_time()
#
        self.exit_garage(rate, space_detect_time)
#
        ## find and ball turn on to it
        self.rotate_toward_ball(rate)
        ## drives until ball is 1m infront of camera
        self.drive_toward_ball(rate, 0.8)
        ##saved odometry contains 1. exiting garage movement 2. rotation toward balls 3. distance driven towards ball, also should contain the final closure in drive_around_ball
        print(self.saved_odometry)
        self.drive_around_ball(rate)
        #self.test_odometry()

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
        rate.sleep()
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
        #save this drive to robot
        self.saved_odometry.append(turtle.get_odometry())

    def rotate_toward_ball(self, rate) -> None:
        """Until ferenc finds ball he's spinning"""
        turtle = self.turtle
        turtle.reset_odometry()
        (center_x, center_y), radius = detect_balls(turtle)

        DEAD_CENTER_X = 640 / 2
        TOLERANCE_PIXEL_BAND = 10
        dist = DEAD_CENTER_X - center_x

        while (not turtle.is_shutting_down()) and (abs(dist) > TOLERANCE_PIXEL_BAND):
            ang_speed = max(min(abs(dist * 0.01), 0.5), 0.1)
            ang_speed = -1 * ang_speed if dist < 0 else ang_speed

            print("balls position on camera x ", center_x, "calculated ang speed ", ang_speed)

            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0, ang_speed)
                (center_x, center_y), radius = detect_balls(turtle)
                dist = DEAD_CENTER_X - center_x
                rate.sleep()


        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()
        #save this drive to robot
        self.saved_odometry.append(turtle.get_odometry())

    def drive_toward_ball(self, rate, final_dist) -> None:
        """until distance to ball is final_dist"""
        turtle = self.turtle
        DISTANCE_TOLERANCE = 0.03 # 3cm
        CONSECUTIVE_READS_NEEDED = 2
        consecutive_readings = 0

        turtle.reset_odometry()
        rate.sleep()

        diff = 0
        dist = 0

        while not turtle.is_shutting_down():
            (center_x, center_y), radius = detect_balls(turtle)
            if center_x == 0: #cant find ball
                print("Ignoring frame")
                turtle.cmd_velocity(0.007, 0)  # small movement so it is possible to detect again
                rate.sleep()
                continue

            dist = get_depth(turtle, center_x, center_y, radius)

            diff = dist - final_dist

            if diff <= DISTANCE_TOLERANCE:
                consecutive_readings += 1
                if consecutive_readings >= CONSECUTIVE_READS_NEEDED:
                    break
            else:
                consecutive_readings = 0  # Reset if we get a reading further away

            lin_speed = max(0.04, min(0.16, diff))

            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                self.go_forward(lin_speed, turtle.get_odometry()[2], 0)
                rate.sleep()



        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()
        (center_x, center_y), radius = detect_balls(turtle)
        dist = get_depth(turtle, center_x, center_y, radius)
        if dist is None:
            print("NO DISTANCE!!!")
            dist = 0
        diff = dist - final_dist
        print("distance achieved is :", dist, "diff is ", diff)
        #save this drive to robot
        self.saved_odometry.append(turtle.get_odometry())


    def drive_around_ball(self, rate) -> None:
        """When close enough to the ball drive around it from point to point of calculated hexagon"""
        turtle = self.turtle
        wanted_distance = 0.30  # 30 cm before ball stop
        rate.sleep()
        rate.sleep()

        dist = self.average_depth()
        if dist is None:
            print("Object not seen")
            return

        print("This is average dist: ", dist)

        final_dist = self.drive_closer(wanted_distance, dist, rate)

        turtle.reset_odometry()
        rate.sleep()
        current_coords = turtle.get_odometry()
        print("\nThis is final distance: ", final_dist)
        # hexagon trajectory
        points = self.calculate_points(final_dist, current_coords)

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
        dist_thresh = 0.02
        angle_thresh = 0.01

        # current location and distance from goal point
        x = point[0] - cur_coords[0]
        y = point[1] - cur_coords[1]
        d = sqrt(x ** 2 + y ** 2)

        # calculate angle to the next point
        angle = atan2(y, x)

        # while ferenc is not rotated at the calculated angle -> rotate
        angle_diff = self.normalize_angle(angle - cur_coords[2])
        while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                self.rotate_to_angle(angle_diff)

            cur_coords = turtle.get_odometry()
            angle_diff = self.normalize_angle(angle - cur_coords[2])

            rate.sleep()

        # while ferenc is not located at x,y coords, drive forward:
        while (not turtle.is_shutting_down()) and (d > dist_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                self.go_forward(0.23, cur_coords[2], angle)

            cur_coords = turtle.get_odometry()
            x = point[0] - cur_coords[0]
            y = point[1] - cur_coords[1]
            d = sqrt(x ** 2 + y ** 2)  # distance from point

            rate.sleep()

        if point_of_return:
            angle_diff = self.normalize_angle(point[2] - cur_coords[2])
            while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
                if self.stop:
                    turtle.cmd_velocity(0, 0)
                    turtle.play_sound(4)
                else:
                    self.rotate_to_angle(angle_diff)
                cur_coords = turtle.get_odometry()
                angle_diff = self.normalize_angle(point[2] - cur_coords[2])

                rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)


    def drive_closer(self, wanted_distance, starting_distance, rate) -> float:
        """Goes closer to the ball even if it is too close to see"""
        turtle = self.turtle
        turtle.reset_odometry()
        sleep(0.1)
        ball_radius = 0.04 # 4cm

        cur_coords = turtle.get_odometry()
        final_distance = starting_distance - (cur_coords[0] + ball_radius)
        while not turtle.is_shutting_down() and final_distance > wanted_distance:
            self.go_forward(0.08, cur_coords[2], 0)

            cur_coords = turtle.get_odometry()
            final_distance = starting_distance - (cur_coords[0] + ball_radius)
            rate.sleep()

        turtle.cmd_velocity(0, 0)
        rate.sleep()
        return final_distance

    def average_depth(self) -> Optional[float]:
        """Calculates average depth of the object so it can more accurately drive close to object"""
        turtle = self.turtle
        distance_sum = 0
        avg_den = 0
        for i in range(3):
            (center_x, center_y), radius = detect_balls(turtle)
            dist = get_depth(turtle, center_x, center_y, radius)
            if dist is None:
                continue
            else:
                distance_sum += dist
                avg_den += 1

        if distance_sum == 0:
            return None

        actual_dist = distance_sum / avg_den
        return actual_dist

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

    def rotate_to_angle(self, angle_diff):
        """Simple P regulated rotating to wanted angle"""
        turtle = self.turtle
        max_speed = 0.6
        Kp = 0.3
        ang_vel = Kp * angle_diff
        ang_vel = max(min(ang_vel, max_speed), -max_speed)   # limit max speed
        turtle.cmd_velocity(0, ang_vel)

    def test_odometry(self):
        rate = Rate(10)
        self.turtle.reset_odometry()

        #go 1x 1y
        self.saved_odometry.append([0.5,0.5,0])
        #go toward by y
        self.saved_odometry.append([0,0.5,0])
        #go toward by x
        self.saved_odometry.append([0.5,0,0])

        #rotate +45
        self.saved_odometry.append([0,0,pi/4])
        #rotate -45
        self.saved_odometry.append([0,0,-pi/4])

        while len(self.saved_odometry) != 0:
            point = self.saved_odometry.pop()
#            point *= -1
            print("going for point", point)
            self.go_ptp(point, rate, False)



if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
