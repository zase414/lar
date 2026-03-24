# Ferenc je robot

from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront, get_depth
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from visuals import detect_balls
from math import pi, cos, sqrt, sin, atan2

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

        # self.test_odometry(rate)
        # until robot finds garage exit spin
        #self.find_exit(rate)
        #space_detect_time = get_time()
#
        #self.exit_garage(rate, space_detect_time)
#
        ## find and ball turn on to it
        # self.rotate_toward_ball(rate)
        ## drives until ball is 40cm infront of camera
        #self.drive_toward_ball(rate, 1)
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
                print("Spin")
                turtle.cmd_velocity(0.0022, 0.25)
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
                rate.sleep()
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
                rate.sleep()
                turtle.play_sound(4)
            else:
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
            lin_speed = 0.2

            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                #if ball not totally infront, rotate
                if (abs(DEAD_CENTER_X - center_x) >  TOLERANCE_PIXEL_BAND):
                    self.rotate_toward_ball(rate)
                else:
                    turtle.cmd_velocity(lin_speed, 0)
                
                (center_x, center_y), radius = detect_balls(turtle)
                dist = get_depth(turtle, center_x, center_y, radius)
                if dist is None:
                    break
                diff = final_dist - dist
                print("distance from ball is :", dist, "diff from designated distance ", diff, "X_pixel distance: ", DEAD_CENTER_X - center_x)
                rate.sleep()

        # reset params
        print("distance achieved, final distance is :", dist, "diff from designated distance ", diff)
        turtle.cmd_velocity(0, 0)
        rate.sleep()






    def drive_around_ball(self, rate) -> None:
        turtle = self.turtle

        (center_x, center_y), radius = detect_balls(turtle)
        dist = get_depth(turtle, center_x, center_y, radius)
        if dist is None:
            return

        turtle.cmd_velocity(0, 0)
        turtle.reset_odometry()
        sleep(0.5)
        current_coords = turtle.get_odometry()

        # hexagon trajectory
        points = self.calculate_points(dist, current_coords)

        print("\nPoints are:", points, "\n\n")

        # go from point to point for each point of the hexagon
        for point in points:
            self.go_ptp(point, rate)

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
                x += dist + 0.04

            if i == 2:
                y = coords[1]
                x += sin(pi / 6) * (dist + ball_radius)

            if i == 3:
                y = cos(pi / 6) * (dist + ball_radius)
                x -= sin(pi / 6) * (dist + ball_radius)

            if i == 4:
                x -= dist + 0.04
                angle = -2 * (pi / 3)

            points.append([x, y, angle])

        # point of return
        # starting point, but ferenc is looking the other way
        points.append([coords[0], coords[1], coords[2] + pi])

        return points

    def go_ptp(self, point, rate) -> None:
        turtle = self.turtle
        cur_coords = turtle.get_odometry()
        dist_thresh = 0.08
        angle_thresh = 0.02
        x = point[0] - cur_coords[0]
        y = point[1] - cur_coords[1]
        d = sqrt(x**2 + y**2)

        # calculate angle to the next point
        angle = atan2(y, x)

        angle_diff = self.normalize_angle(angle - cur_coords[2])
        # while ferenc is not rotated at the calculated angle -> rotate
        while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0, -0.2)

            cur_coords = turtle.get_odometry()
            angle_diff = self.normalize_angle(angle - cur_coords[2])

            rate.sleep()

        # while ferenc is not located at x,y coords, drive forward:
        while (not turtle.is_shutting_down()) and (abs(d) > dist_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0.2, 0)

            cur_coords = turtle.get_odometry()
            x = point[0] - cur_coords[0]
            y = point[1] - cur_coords[1]
            d = sqrt(x**2 + y**2) # distance from point

            print(d)
            rate.sleep()

        # while ferenc is not rotated at the calculated angle -> rotate
        angle_diff = self.normalize_angle((point[2] + 0.03) - cur_coords[2])  # little over-rotation so it can spin only in one direction
        while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0, 0.3)

            cur_coords = turtle.get_odometry()
            angle_diff = self.normalize_angle((point[2]+0.03) - cur_coords[2])   # little over-rotation so it can spin only in one direction

            rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)

    def normalize_angle(self, angle):
        """Normalizes an angle to be strictly within -pi and pi"""
        return (angle + pi) % (2 * pi) - pi

    def test_odometry(self, rate):
        turtle = self.turtle
        turtle.cmd_velocity(0, 0)
        turtle.reset_odometry()
        sleep(0.5)
        cur_coords = turtle.get_odometry()
        angle = pi
        angle_diff = angle - cur_coords[2]

        # while ferenc is not rotated at the calculated angle -> rotate
        while (not turtle.is_shutting_down()) and (not abs(angle_diff) < 0.02):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                turtle.cmd_velocity(0, 0.4)

            cur_coords = turtle.get_odometry()
            angle_diff = angle - cur_coords[2]
            print("Current angle is ", cur_coords[2], " rad")
            rate.sleep()
        turtle.cmd_velocity(0, 0)

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
