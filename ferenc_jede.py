# Ferenc je robot

from __future__ import print_function

from callbacks import callback_bumper_stop, callback_button0_resume
from image_proccesing import space_infront, get_depth
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from visuals import detect_balls, detect_rectangles
from math import pi, cos, sqrt, sin, atan2
from typing import Optional

import cv2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False
        self.garage_ball_dist = [0, 0] # [angle, dist]
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

        self.exit_garage(rate, space_detect_time)

        ## find and ball turn on to it
        self.rotate_toward_ball(rate)
        ## drives until ball is 58 cm infront of camera
        self.drive_toward_ball(rate, 0.53)
        # self.rotate_toward_ball(rate)

        self.drive_around_ball(rate)
        self.return_to_garage_from_odometry(rate)
        # self.go_home(rate)

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
        while (not turtle.is_shutting_down()) and (get_time() - space_detect_time < 2):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                # go forward with 5° offset to negate early exit
                self.go_forward(turtle.get_odometry()[2], pi/36, dist_diff = None, prefered_lin_vel = 0.25)
                rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()

    def rotate_toward_ball(self, rate) -> None:
        """rotates robot toward ball"""
        turtle = self.turtle
        turtle.reset_odometry()
        rate.sleep()

        DEAD_CENTER_X = 358
        TOLERANCE_PIXEL_BAND = 3
        PIXELS_TO_DEG = 38 / 320  # pixels to degrees conversion

        while not turtle.is_shutting_down():
            (center_x, _), _ = detect_balls(turtle)

            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
                continue

            # sees nothing, rotate
            if center_x == 0:
                turtle.cmd_velocity(0, 0.6)
                print("i don't see ball im rotating")
                rate.sleep()
                continue

            # sees something - measure
            turtle.cmd_velocity(0, 0)
            rate.sleep()
            rate.sleep()


            print("i saw ball, im measuring ...")
            measurements = []
            for _ in range(5):
                (cx, _), _ = detect_balls(turtle)
                if cx != 0: #append only non zero values
                    print("measured: ",cx)
                    measurements.append(cx)
                rate.sleep()

            if not measurements:
                print("no measurement")
                continue

            avg_center = sum(measurements) / len(measurements)
            dist = DEAD_CENTER_X - avg_center
            print("average center:", avg_center, " distance from center", dist)

            # not in tolerance, calc angle and rotate
            if abs(dist) > TOLERANCE_PIXEL_BAND:
                angle = dist * PIXELS_TO_DEG * (pi/180)
                print("calculated angle: ", angle)
                self.rotate_to_angle(angle, rate)
            else:
                turtle.cmd_velocity(0, 0)
                break


        # reset params
        turtle.cmd_velocity(0, 0)
        rate.sleep()
        #save this drive to robot
        ball_angle = turtle.get_odometry()[2]
        print("angle i drove : ", self.normalize_angle(ball_angle))
        self.garage_ball_dist[0] = -1*self.normalize_angle(ball_angle)

    def drive_toward_ball(self, rate, final_dist) -> None:
        """until distance to ball is final_dist"""
        turtle = self.turtle
        DISTANCE_TOLERANCE = 0.03 # 3cm
        CONSECUTIVE_READS_NEEDED = 2
        consecutive_readings = 0
        consecutive_ignores = 0

        turtle.reset_odometry()
        rate.sleep()

        while not turtle.is_shutting_down():
            (center_x, center_y), radius = detect_balls(turtle)
            if consecutive_ignores == 10:
                self.rotate_toward_ball(rate)
                consecutive_ignores = 0

            elif center_x == 0: #cant find ball
                print("Ignoring frame")
                consecutive_ignores += 1
                turtle.cmd_velocity(0.01, 0)  # small movement so it is possible to detect again
                rate.sleep()
                continue

            dist = get_depth(turtle, center_x, center_y, radius)

            diff = dist - final_dist

            if abs(diff) < 0.05:   # 5cm away
                CONSECUTIVE_READS_NEEDED = 1

            if diff <= DISTANCE_TOLERANCE:
                consecutive_readings += 1
                if consecutive_readings >= CONSECUTIVE_READS_NEEDED:
                    break
            else:
                consecutive_readings = 0  # Reset if we get a reading further away

            if self.stop:
                turtle.cmd_velocity(0, 0)
                rate.sleep()
                turtle.play_sound(4)
            else:
                self.go_forward(turtle.get_odometry()[2], 0, abs(diff)*0.55, prefered_lin_vel=None)
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
        distance_of_ball = turtle.get_odometry()[0]
        self.garage_ball_dist[1] += distance_of_ball


    def drive_around_ball(self, rate) -> None:
        """When close enough to the ball drive around it from point to point of calculated hexagon"""
        turtle = self.turtle
        wanted_distance = 0.2775  # 27,75 cm before ball stop
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

    def go_ptp(self, point, rate, point_of_return = False):
        """Function that navigates from one point of a hexagon to the next"""
        turtle = self.turtle
        cur_coords = turtle.get_odometry()
        start_coords = cur_coords

        # thresholds fo accurate enough stopping in given points
        dist_thresh = 0.0075

        # current location and distance from goal point
        x = point[0] - cur_coords[0]
        y = point[1] - cur_coords[1]
        d = sqrt(x ** 2 + y ** 2)

        # calculate angle to the next point
        angle = atan2(y, x)

        # while ferenc is not rotated at the calculated angle -> rotate
        self.rotate_to_angle(angle, rate)

        # reset params
        turtle.cmd_velocity(0, 0)

        # while ferenc is not located at x,y coords, drive forward:
        initial_angle = cur_coords[2]
        start_to_goal = [point[0] - start_coords[0], point[1] - start_coords[1]]
        while (not turtle.is_shutting_down()) and (d > dist_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                self.go_forward(cur_coords[2], initial_angle, abs(d)*2.7, prefered_lin_vel=None)

            cur_coords = turtle.get_odometry()
            x = point[0] - cur_coords[0]
            y = point[1] - cur_coords[1]
            d = sqrt(x ** 2 + y ** 2)  # distance from point
            
            # --- overshoot detection ---
            cur_to_goal = [x, y]
            dot = start_to_goal[0] * cur_to_goal[0] + start_to_goal[1] * cur_to_goal[1]

            if dot < 0:
                print("Overshot target → stopping")
                break

            rate.sleep()

        # reset params
        turtle.cmd_velocity(0, 0)

        if point_of_return:
            self.rotate_to_angle(point[2], rate)

        # reset params
        turtle.cmd_velocity(0, 0)


    def drive_closer(self, wanted_distance, starting_distance, rate) -> float:
        """Goes closer to the ball even if it is too close to see"""
        turtle = self.turtle
        turtle.reset_odometry()
        sleep(0.1)
        ball_radius = 0.041 # 4,1 cm

        cur_coords = turtle.get_odometry()
        final_distance = starting_distance - (cur_coords[0] + ball_radius)
        while not turtle.is_shutting_down() and final_distance > wanted_distance:
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                # dist_diff = wanted_distance - final_distance
                self.go_forward(cur_coords[2], 0, dist_diff = None , prefered_lin_vel = 0.08)

            cur_coords = turtle.get_odometry()
            final_distance = starting_distance - (cur_coords[0] + ball_radius)
            rate.sleep()

        turtle.cmd_velocity(0, 0)
        rate.sleep()
        self.garage_ball_dist[1] += starting_distance - final_distance - 0.06  # 6cm
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

    def go_forward(self, current_angle, needed_angle, dist_diff, prefered_lin_vel):
        """Simple P regulated driving in a straight line"""
        turtle = self.turtle
        angle_diff = self.normalize_angle(needed_angle - current_angle)

        # based on how off course is our robot rotated >>> steer it to go straight
        Kp_ang = 0.8
        angular_velocity = Kp_ang * angle_diff

        # speed dependent on how far from desired destination is ferenc located
        max_speed = 0.23
        Kp_lin = 0.4
        if dist_diff is None and prefered_lin_vel is not None:
            lin_velocity = prefered_lin_vel
        elif dist_diff is None and prefered_lin_vel is None:
            lin_velocity = max_speed
        else:
            lin_velocity = Kp_lin * abs(dist_diff)
        lin_velocity = min(lin_velocity, max_speed)   # limit max speed
        turtle.cmd_velocity(lin_velocity, angular_velocity)

    def rotate_to_angle(self, angle, rate):
        angle_thresh = 0.017
        turtle = self.turtle
        cur_coords = turtle.get_odometry()
        angle_diff = self.normalize_angle(angle - cur_coords[2])
        while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
            if self.stop:
                turtle.cmd_velocity(0, 0)
                turtle.play_sound(4)
            else:
                self.angular_P_reg(angle_diff)

            cur_coords = turtle.get_odometry()
            angle_diff = self.normalize_angle(angle - cur_coords[2])

            rate.sleep()


    def angular_P_reg(self, angle_diff):
        """Simple P regulated rotating to wanted angle"""
        turtle = self.turtle
        max_speed = 0.7
        min_speed = 0.1
        Kp = 4.6
        ang_vel = Kp * angle_diff
        if 0 < ang_vel < min_speed:
            ang_vel = min_speed
        elif 0 > ang_vel > -min_speed:
            ang_vel = -min_speed
        else:
            ang_vel = max(min(ang_vel, max_speed), -max_speed)   # limit max speed

        turtle.cmd_velocity(0, ang_vel)

    def return_to_garage_from_odometry(self, rate):
        turtle = self.turtle
        turtle.reset_odometry()
        rate.sleep()
        #drives back the distance
        dist = self.garage_ball_dist[1]
        self.go_ptp([dist,0,0], rate)

        turtle.reset_odometry()
        rate.sleep()
        #rotates back toward garage
        angle = self.garage_ball_dist[0]
        print("angle it wants to rotate ", angle, "current angle: ", self.turtle.get_odometry()[2])
        self.rotate_to_angle(angle, rate)


    def go_home(self, rate):
      turtle = self.turtle

      Kp = 0.005
      Ki = 0.0001
      Kd = 0.001

      integral = 0
      prev_error = 0
      prev_time = get_time()

      TARGET_X = 640 // 2
      TARGET_DEPTH = 0.17

      gate_detected = False

      while not turtle.is_shutting_down():
        rectangles = detect_rectangles(turtle)

        current_time = get_time()
        dt = current_time - prev_time

        if rectangles and len(rectangles) == 3 and dt > 0 and not self.stop:
          gate_detected = True

          left, right, center = rectangles

          center_x, center_y = center
          left_x, left_y = left
          right_x, right_y = right

          error = TARGET_X - center_x

          proportional = Kp * error
          integral += error * dt
          derivative = (error - prev_error) / dt

          pid_output = proportional + (Ki * integral) + (Kd * derivative)

          turtle.cmd_velocity(linear=0.24, angular=pid_output)

          prev_error = error
          prev_time = current_time
        else:
          if not gate_detected:
            turtle.cmd_velocity(linear=0.0, angular=0.5)
          elif not self.stop:
            center_depth = get_depth(turtle, TARGET_X, 240, 2)
            diferenc = center_depth - TARGET_DEPTH
            if (diferenc > 0.1):
              turtle.cmd_velocity(linear=diferenc*0.15, angular=0.0)
            else: 
              turtle.cmd_velocity(0,0)
              print("hotovo")
              break
          else:
            turtle.cmd_velocity(linear=0.0, angular=0.0)
          integral = 0
          prev_time = current_time

        if cv2.waitKey(1) & 0xFF == ord('q'):
          break

        rate.sleep()

      cv2.destroyAllWindows()


if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
