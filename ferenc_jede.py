# Ferenc je robot

from __future__ import print_function

from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from math import pi, cos, sqrt, sin, atan2
from typing import Optional, Tuple, List
import cv2

from callbacks import callback_bumper_stop, callback_button0_resume
from visuals import detect_ball, space_infront, get_depth, detect_rectangles
from utils import normalize_angle, calculate_points

BALL_DISTANCE_TO_SKIP_EXIT = 0.82
EXIT_GARAGE_DURATION = 3.25

BALL_RADIUS = 0.041 # 4,1 cm
EXIT_CENTER_TOLERANCE_PIXEL_BAND = 90

BALL_ROTATION_TOLERANCE_PIXEL_BAND = 2
BALL_ROTATION_CAMERA_CENTER_X = 355
BALL_ROTATION_ANGLE_THRESHOLD = 0.0075

PIXELS_TO_RAD = (29 / 320) * (pi/180)  # pixels to degrees  to radian conversion

BALL_APPROACH_DISTANCE_TOLERANCE = 0.04 # 4cm
BALL_APPROACH_CONSECUTIVE_READS_NEEDED = 2

P_ANGULAR_MAX_SPEED = 0.6
P_ANGULAR_MIN_SPEED = 0.04

P_ANGULAR_KP = 2.25
P_ANGULAR_KI = 0.08
P_ANGULAR_KD = 0.21
MAX_I_TERM = 0.2

RETURN_PID_KP = 0.005
RETURN_PID_KI = 0.0001
RETURN_PID_KD = 0.001
RETURN_TARGET_SCREEN_CENTER = 640 // 2
RETURN_TARGET_DEPTH = 0.185
HOME_SOUND = 0

class Ferenc:
    
    def __init__(self):
        """
        Initialize the Ferenc robot controller.
        
        Sets up the Turtlebot with RGB and point cloud sensors, initializes
        the stop flag, and sets return navigation and distance parameters to zero.
        """
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False
        self.start = False
        self.return_angle = 0.0
        self.return_distance = 0.0
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.distance_to_ball = 0.0

    def _handle_stop(self) -> bool:
        """
        Check the stop flag and halt the robot if it is set.
        
        Sends zero velocity and plays a sound when stopped.
        
        Returns:
            bool: True if the robot is stopped, False otherwise.
        """
        if self.stop:
            self.turtle.cmd_velocity(0, 0)
            self.turtle.play_sound(4)
            return True
        return False

    def _get_coords(self) -> Tuple[float,float,float]:
        """
        Retrieve the robot's current odometry coords.
        
        Returns:
            tuple[float, float, float]: The (x, y, theta) position and angle.
        """
        x, y, theta = self.turtle.get_odometry()
        return x, y, theta

    def _get_x(self) -> float:
        """
        Retrieve the robot's current odometry x coordinate.
        
        Returns:
            float: The x coordinate
        """
        return self._get_coords()[0]

    def _get_y(self) -> float:
        """
        Retrieve the robot's current odometry y coordinate.
        
        Returns:
            float: The y coordinate
        """
        return self._get_coords()[1]

    def _get_angle(self) -> float:
        """
        Retrieve the robot's current odometry angle.
        
        Returns:
            float: The angle
        """
        return self._get_coords()[2]

    def _stop_and_wait(self, rate):
        """
        Stop the robot and sleep for one cycle.
        
        Args:
            rate: A Rate object to control timing.
        """
        self.turtle.cmd_velocity(0, 0)
        rate.sleep()

    def initialize_turtle_and_wait(self, rate) -> Turtlebot:
        turtle = self.turtle
        turtle.wait_for_point_cloud()
        # initialize bumber and buttons
        turtle.register_bumper_event_cb(lambda msge : callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge : callback_button0_resume(self, msge))
         # start on button press
        print("Zmáčkněte tlačítko B0 pro start!")
        while not turtle.is_shutting_down() and not self.start:
           rate.sleep()
        print("Ferenc jede!")
        return turtle

    def main(self):
        """
        Execute the full sequence.
        
        Initializes sensors and callbacks, then runs the robot through its
        complete mission: exiting the garage, locating and driving around the ball
        and returning to the starting position.
        """

        rate = Rate(10)
        turtle = self.initialize_turtle_and_wait(rate)

        # spin until robot finds garage exit
        self.find_exit(rate)

        #measure distance to ball and its location with camera
        self.distance_to_ball = self.get_average_ball_depth(3)
        (cx, _), _ = detect_ball(turtle)
        rate.sleep()
        center_dist = BALL_ROTATION_CAMERA_CENTER_X - cx

        print("\n\n Vzdálenost míčku od středu:",center_dist,"\nVzdálenost míčku od garáže:", self.distance_to_ball,"\n")
        #if doesn't see ball or ball is far enough to drives forward for full duration
        if self.distance_to_ball is None or self.distance_to_ball >= BALL_DISTANCE_TO_SKIP_EXIT:
            self.exit_garage(rate, EXIT_GARAGE_DURATION)
        #ball is in FOV near center drives forward for half duration
        elif abs(center_dist) > EXIT_CENTER_TOLERANCE_PIXEL_BAND:
            self.exit_garage(rate, EXIT_GARAGE_DURATION/1.75)

        #rotate towards ball
        if not turtle.is_shutting_down():
            self.rotate_toward_ball(rate)

        #measure distance after it exits garage
        self.distance_to_ball = self.get_average_ball_depth(3)

        print("Vzdálenost míčku od garáže po výjezdu ",self.distance_to_ball)

        ball_is_far = self.distance_to_ball > 1
        ball_return_closer_dist = 0.025 if self.distance_to_ball > 1 else 0
        final_ball_distance = 0.312 if self.distance_to_ball >= BALL_DISTANCE_TO_SKIP_EXIT else 0.292  # 31 cm or 29 cm before ball stop with certainty

        # if Ferenc is far from ball drive forward from garage and only then drive toward ball 
        if not turtle.is_shutting_down() and ball_is_far:
            #rotates back
            self.rotate_to_angle(0, rate, point_of_return = False)
            obstacle_dist = get_depth(turtle, 320, 240, 100)
            #no distance in front of robot, drive a bit forward again
            if obstacle_dist is None or obstacle_dist > 0.4:
                self.exit_garage(rate, EXIT_GARAGE_DURATION)

            self.rotate_toward_ball(rate)
            self.drive_toward_ball(rate, 0.58)

        #ball is not that far and not that close goes toward ball
        elif not turtle.is_shutting_down() and self.distance_to_ball >= BALL_DISTANCE_TO_SKIP_EXIT:
            self.drive_toward_ball(rate, 0.58)
        
        if not turtle.is_shutting_down():
            self.drive_around_ball(rate, final_ball_distance, ball_return_closer_dist)

        if not turtle.is_shutting_down():
            self.return_to_garage_from_odometry(rate, ball_is_far)

    def find_exit(self, rate) -> None:
        """
        Spin in place until an open space is detected in front of the robot.
        
        Rotates continuously at a fixed angular velocity, getting point
        cloud data on each cycle, and stops as soon as a clear path is found.
        
        Args:
            rate: A Rate object used to control timing.
        """
        turtle = self.turtle
        space = space_infront(turtle=turtle)
        while (not turtle.is_shutting_down()) and not space:
            if self._handle_stop():
                continue
            else:
                turtle.cmd_velocity(0, 0.37)
            space = space_infront(turtle=turtle)
            rate.sleep()

        self._stop_and_wait(rate)

    def exit_garage(self, rate, exit_time) -> None:
        """
        Drive forward out of the garage for a fixed time after exit detection.
        
        Resets odometry, then moves forward for
        fixed time with given speed to exit the garage.
        
        Args:
            rate: A Rate object used to control timing.
            exit_time: How long should ferenc drive from the garage
        """
        space_detect_time = get_time()
        turtle = self.turtle
        turtle.reset_odometry()
        rate.sleep()
        while (not turtle.is_shutting_down()) and (get_time() - space_detect_time < exit_time):
            if self._handle_stop():
                continue
            else:
                self.go_forward(self._get_angle(), 0, dist_diff = None, prefered_lin_vel = 0.24)
                rate.sleep()

        self._stop_and_wait(rate)

    def go_forward(self, current_angle, needed_angle, dist_diff, prefered_lin_vel):
        """
        Apply P-regulated forward motion with heading correction.

        Computes angular velocity to correct heading error and linear velocity
        either from a P-controller on dist_diff or from a preferred constant
        speed, capping both at their respective maximum.

        Args:
            current_angle (float): Robot's current heading in radians.
            needed_angle (float): Desired heading in radians.
            dist_diff (float or None): Remaining distance to target; if None,
                prefered_lin_vel is used instead.
            prefered_lin_vel (float or None): Constant linear speed override;
                used when dist_diff is None.
        """
        turtle = self.turtle
        angle_diff = normalize_angle(needed_angle - current_angle)

        # based on how off course is our robot rotated >>> steer it to go straight
        Kp_ang = 0.8
        angular_velocity = Kp_ang * angle_diff

        # speed dependent on how far from desired destination is ferenc located
        max_speed = 0.19
        Kp_lin = 0.38
        if dist_diff is None and prefered_lin_vel is not None:
            lin_velocity = prefered_lin_vel
        elif dist_diff is None and prefered_lin_vel is None:
            lin_velocity = max_speed
        else:
            lin_velocity = Kp_lin * abs(dist_diff)
        lin_velocity = min(lin_velocity, max_speed)  # limit max speed
        turtle.cmd_velocity(lin_velocity, angular_velocity)

    def rotate_toward_ball(self, rate) -> None:
        """
        Rotates the robot until the center of ball is in BALL_ROTATION_CAMERA_CENTER_X within BALL_ROTATION_TOLERANCE_PIXEL_BAND.
        
        Rotates at constant speed while camera can't see ball. When ball is seen it constantly rotates at 2 times of our minimal angular speed
        this goes until the ball is in the right position, there is a slight chance of overshoot, due to increasing speed of rotation,
        this is quickly solved by repeated measurement.
        takes around 20-30 sec to center
        
        Args:
            rate: A Rate object used to control timing.
        """
        turtle = self.turtle
        turtle.reset_odometry()
        rate.sleep()

        while not turtle.is_shutting_down():
            (center_x, _), _ = detect_ball(turtle)

            if self._handle_stop():
                continue

            # sees nothing(or its far away), rotate
            if abs(center_x-BALL_ROTATION_CAMERA_CENTER_X) >= 90:
                turtle.cmd_velocity(0, P_ANGULAR_MAX_SPEED)
                rate.sleep()
                continue

            # sees something - measure
            self._stop_and_wait(rate)

            measurements = []
            for _ in range(5):
                (cx, _), _ = detect_ball(turtle)
                if cx != 0: #append only non zero values
                    measurements.append(cx)

            if not measurements:
                print("no measurement")
                continue

            
            avg_center = sum(measurements) / len(measurements)
            dist = BALL_ROTATION_CAMERA_CENTER_X - avg_center

            if abs(dist) <= BALL_ROTATION_TOLERANCE_PIXEL_BAND:
                break

            consecutive_ignores = 0
            while not turtle.is_shutting_down() and not self._handle_stop():
                (cx, _), _ = detect_ball(turtle)
                print("center found on", cx)
                dist = BALL_ROTATION_CAMERA_CENTER_X - cx

                if abs(dist) <= BALL_ROTATION_TOLERANCE_PIXEL_BAND:
                    break

                if cx == 0:
                    consecutive_ignores += 1
                    if consecutive_ignores == 5:
                        break
                    continue
                else:
                    consecutive_ignores = 0

                print("im turning now")
                if dist > 0:
                    turtle.cmd_velocity(0, 3.2*P_ANGULAR_MIN_SPEED)
                else:
                    turtle.cmd_velocity(0, -3.2* P_ANGULAR_MIN_SPEED)
                rate.sleep()
            
        # reset params
        self._stop_and_wait(rate)

        # save this drive to robot
        ball_angle = self._get_angle()
        self.return_angle = -1 * normalize_angle(ball_angle)

    def drive_toward_ball(self, rate, final_dist) -> None:
        """
        Drive forward until the ball is approximately final_dist away.
        
        Uses depth readings from the point cloud to measure distance to the
        ball and applies P-regulated linear velocity.
        Accumulates the distance driven into self.return_distance.
        If the ball is not seen for few consecutive times, then it aborts.

        Args:
            rate: A Rate object used to control timing.
            final_dist (float): Target stopping distance from the ball in metres.
        """
        turtle = self.turtle
        consecutive_readings = 0
        consecutive_ignores = 0

        reads_needed = BALL_APPROACH_CONSECUTIVE_READS_NEEDED

        turtle.reset_odometry()
        rate.sleep()

        while not turtle.is_shutting_down():
            (center_x, center_y), radius = detect_ball(turtle)

            if consecutive_ignores == 10:
                print("Object not seen")
                break

            elif center_x == 0: #cant find ball
                print("Ignoring frame")
                consecutive_ignores += 1
                turtle.cmd_velocity(0.01, 0)  # small movement so it is possible to detect again
                rate.sleep()
                continue

            dist = get_depth(turtle, center_x, center_y, radius)

            if dist is None:
                rate.sleep()
                continue

            diff = dist - final_dist

            if abs(diff) < 0.05:   # 5cm away
                reads_needed = 1

            consecutive_readings += 1
            if consecutive_readings >= reads_needed:
                break
            else:
                consecutive_readings = 0  # Reset if we get a reading further away

            if self._handle_stop():
                continue
            else:
                self.go_forward(self._get_angle(), 0, abs(diff)*0.6, prefered_lin_vel=None)
                rate.sleep()

        # reset params
        self._stop_and_wait(rate)

        (center_x, center_y), radius = detect_ball(turtle)
        dist = get_depth(turtle, center_x, center_y, radius)
        if dist is None:
            print("NO DISTANCE!!!")
        #save this drive to robot
        distance_of_ball = self._get_x()
        self.return_distance += distance_of_ball


    def drive_around_ball(self, rate, final_ball_dist, ball_return_closer_dist) -> None:
        """
        Drive around the ball along a hexagonal path.
        
        Measures the current distance to the ball, drives to the desired
        close distance (ferenc does not see the ball any longer),
        computes hexagon waypoints centred on the ball,
        and visits each waypoint in sequence using go_ptp.
        
        Args:
            rate: A Rate object used to control timing.
            final_ball_dist: Distance in front of the ball
            ball_return_closer_dist: how far from garage return point ferenc should stop
        """
        turtle = self.turtle
        rate.sleep()
        rate.sleep()

        dist = self.get_average_ball_depth(3)
        if dist is None:
            print("Object not seen")
            return

        final_dist = self.drive_closer(final_ball_dist, dist, rate, ball_return_closer_dist)

        turtle.reset_odometry()
        rate.sleep()
        current_coords = turtle.get_odometry()
        # calculate hexagon trajectory
        points = calculate_points(final_dist, current_coords, BALL_RADIUS)

        # go from point to point for each point of the hexagon
        p_num = 0
        point_of_return = False  # final point the hexagon (starting point but rotated by 180°)
        for point in points:
            p_num += 1
            if p_num == len(points):
                point_of_return = True
            self.go_ptp(point, rate, point_of_return)

    def get_average_ball_depth(self, samples: int) -> Optional[float]:
        """
        Compute an averaged depth reading to the ball over three samples.

        Ignores frames where the ball is not detected. Returns None if no
        valid depth reading is obtained.

        Returns:
            float or None: Average distance to the ball in metres, or None
                if the ball could not be seen in any sample.
        """
        turtle = self.turtle
        distance_sum = 0
        avg_den = 0
        for i in range(samples):
            (center_x, center_y), radius = detect_ball(turtle)
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

    def drive_closer(self, wanted_distance, starting_distance, rate, ball_return_closer_dist) -> float:
        """
        Drive forward until the robot is wanted_distance from the ball.

        Uses odometry to infer remaining distance when the ball may be too
        close for reliable depth sensing. Accumulates the distance driven into
        self.return_distance.

        Args:
            wanted_distance (float): Desired final stand-off distance in metres.
            starting_distance (float): Measured distance to the ball before closing.
            rate: A Rate object used to control loop timing.
            ball_return_closer_dist: ow far from garage return point should ferenc stop
        Returns:
            float: The final computed distance to the ball after closing.
        """
        turtle = self.turtle
        turtle.reset_odometry()
        sleep(0.1)

        cur_coords = turtle.get_odometry()
        final_distance = starting_distance - (cur_coords[0] + BALL_RADIUS)
        while not turtle.is_shutting_down() and final_distance > wanted_distance:
            if self._handle_stop():
                continue
            else:
                self.go_forward(cur_coords[2], 0, dist_diff=None, prefered_lin_vel=0.08)

            cur_coords = turtle.get_odometry()
            final_distance = starting_distance - (cur_coords[0] + BALL_RADIUS)
            rate.sleep()

        self._stop_and_wait(rate)
        self.return_distance += starting_distance - final_distance - ball_return_closer_dist
        return final_distance

    def go_ptp(self, point, rate, point_of_return = False):
        """
        Navigate to a single waypoint using rotate-then-drive control.
        
        Args:
            point (list[float]): Target waypoint as [x, y, angle].
            rate: A Rate object used to control loop timing.
            point_of_return (bool): If True, calculate angle to garage return point after arriving.
        """
        turtle = self.turtle
        cur_coords = turtle.get_odometry()
        start_coords = cur_coords

        # thresholds fo accurate enough stopping in given points
        dist_thresh = 0.012

        # current location and distance from goal point
        x = point[0] - cur_coords[0]
        y = point[1] - cur_coords[1]
        d = sqrt(x ** 2 + y ** 2)

        # calculate angle to the next point
        angle = atan2(y, x)

        # while ferenc is not rotated at the calculated angle -> rotate
        self.rotate_to_angle(angle, rate, point_of_return=False)

        # reset params
        self._stop_and_wait(rate)

        # while ferenc is not located at x,y coords, drive forward:
        cur_coords = turtle.get_odometry()
        initial_angle = cur_coords[2]
        start_to_goal = [point[0] - start_coords[0], point[1] - start_coords[1]]
        while (not turtle.is_shutting_down()) and (d > dist_thresh):
            if self._handle_stop():
                continue
            else:
                self.go_forward(cur_coords[2], initial_angle, abs(d)*1.75, prefered_lin_vel=None)

            cur_coords = turtle.get_odometry()
            x = point[0] - cur_coords[0]
            y = point[1] - cur_coords[1]
            d = sqrt(x ** 2 + y ** 2)  # distance from point
            
            # --- overshoot detection ---
            cur_to_goal = [x, y]
            dot = start_to_goal[0] * cur_to_goal[0] + start_to_goal[1] * cur_to_goal[1]

            if dot < 0:
                break

            rate.sleep()

        self._stop_and_wait(rate)

        if point_of_return:
            (x, y, _) = turtle.get_odometry()
            final_vector = (-x - self.return_distance, - y)

            angle = atan2(final_vector[1], final_vector[0])

            self.rotate_to_angle(angle, rate, point_of_return)

        self._stop_and_wait(rate)


    def rotate_to_angle(self, angle, rate, point_of_return):
        """
        Rotate in place until the robot's heading matches the target angle.

        Uses PID-regulated angular velocity and stops when the heading error
        falls within BALL_ROTATION_ANGLE_THRESHOLD.

        Args:
            angle (float): Target heading in radians.
            rate: A Rate object used to control loop timing.
            point_of_return: If it is final point of hexagon
        """
        turtle = self.turtle
        cur_coords = turtle.get_odometry()
        angle_diff = normalize_angle(angle - cur_coords[2])
        if point_of_return:
            threshold = BALL_ROTATION_ANGLE_THRESHOLD * 0.4
        else:
            threshold = BALL_ROTATION_ANGLE_THRESHOLD
        while (not turtle.is_shutting_down()) and (abs(angle_diff) > threshold):
            if self._handle_stop():
                continue
            else:
                self.angular_PID_reg(angle_diff, 0.1)

            cur_coords = turtle.get_odometry()
            angle_diff = normalize_angle(angle - cur_coords[2])

            rate.sleep()
        self.integral_error = 0.0
        self.previous_error = 0.0

    def angular_PID_reg(self, angle_diff, dt):
        """
        PID-regulated angular velocity rotation.

        Args:
        angle_diff (float): Signed angular error in radians.
        dt (float): Delta time (seconds) since the last control loop.
        """
        turtle = self.turtle

        # --- P ---
        p_action = P_ANGULAR_KP * angle_diff

        # --- I ---
        self.integral_error += (angle_diff * dt)
        max_i_accumulated = MAX_I_TERM / (P_ANGULAR_KI if P_ANGULAR_KI > 0 else 1)  # Prevent div by zero
        self.integral_error = max(min(self.integral_error, max_i_accumulated), -max_i_accumulated)
        i_action = P_ANGULAR_KI * self.integral_error

        # --- D ---
        if dt > 0:
            derivative = (angle_diff - self.previous_error) / dt
        else:
            derivative = 0.0

        d_action = P_ANGULAR_KD * derivative

        # Update previous error for the NEXT loop
        self.previous_error = angle_diff

        # --- PID ---
        ang_vel = p_action + i_action + d_action

        # --- Limits ---
        if 0 < ang_vel < P_ANGULAR_MIN_SPEED:
            ang_vel = P_ANGULAR_MIN_SPEED
        elif 0 > ang_vel > -P_ANGULAR_MIN_SPEED:
            ang_vel = -P_ANGULAR_MIN_SPEED
        else:
            ang_vel = max(min(ang_vel, P_ANGULAR_MAX_SPEED), -P_ANGULAR_MAX_SPEED)

        turtle.cmd_velocity(0, ang_vel)

    def return_to_garage_from_odometry(self, rate, gohome_mode = False):
        """
        Return the robot to its garage starting position.
        
        Drives back the accumulated return_distance along the x-axis, then
        rotates to the saved return_angle and calls go_in or gohome_mode to complete parking.
        
        Args:
            rate: A Rate object used to control loop timing.
            gohome_mode: True if robot should return to garage by camera
        """
        turtle = self.turtle
        turtle.reset_odometry()
        rate.sleep()
        # drives back the distance it drove
        dist = self.return_distance
        if not turtle.is_shutting_down():
            self.go_ptp([dist,0,0], rate)

        if not turtle.is_shutting_down():
            turtle.reset_odometry()
            rate.sleep()
            # rotates back toward garage
            angle = self.return_angle
            self.rotate_to_angle(angle, rate, point_of_return=True)
            if gohome_mode:
                self.go_home(rate)
            else:
                self.go_in(rate)

    def go_in(self, rate):
        """
        Drive forward into the garage until the robot reaches the target depth.

        Uses a centre-screen depth reading to compute remaining distance and
        applies proportional linear velocity until the robot is within
        TARGET_DEPTH of the back wall.

        Args:
            rate: A Rate object used to control loop timing.
        """
        turtle = self.turtle
        turtle.reset_odometry()
        rate.sleep()
        rate.sleep()
        while not turtle.is_shutting_down():
            if self._handle_stop():
                continue
            else:
                center_depth = get_depth(turtle, BALL_ROTATION_CAMERA_CENTER_X, 240, 2)
                diff = center_depth - RETURN_TARGET_DEPTH

                angle = self._get_angle()
                if (diff > 0.1):
                    self.go_forward(angle, 0, dist_diff=None, prefered_lin_vel=diff * 0.15)
                else:
                    turtle.cmd_velocity(0, 0)
                    print("Ferenc je doma :)")
                    home_time = get_time()
                    while not turtle.is_shutting_down() and (get_time() - home_time) < 1:
                        turtle.play_sound(HOME_SOUND)
                        rate.sleep()
                    break
            rate.sleep()

    def go_home(self, rate):
        """
        Navigate back to the garage using visual detection of the gate markers.
        
        Applies a PID controller on the horizontal error between the detected
        gate centre marker and the screen centre to steer toward the garage.
        Rotates in place if the gate is not yet visible. Once the gate fills
        the frame, switches to depth-based forward control to complete parking.
        
        Args:
            rate: A Rate object used to control loop timing.
        """
        turtle = self.turtle

        integral = 0
        prev_error = 0
        prev_time = get_time()


        gate_detected = False

        while not turtle.is_shutting_down():
          if self._handle_stop():
              continue
          rectangles = detect_rectangles(turtle)

          current_time = get_time()
          dt = current_time - prev_time

          if rectangles and len(rectangles) == 3 and dt > 0 and not self.stop:
            gate_detected = True

            left, right, center = rectangles

            center_x, center_y = center

            error = BALL_ROTATION_CAMERA_CENTER_X - center_x

            proportional = RETURN_PID_KP * error
            integral += error * dt
            derivative = (error - prev_error) / dt

            pid_output = proportional + (RETURN_PID_KI * integral) + (RETURN_PID_KD * derivative)

            turtle.cmd_velocity(linear=0.16, angular=pid_output)

            prev_error = error
            prev_time = current_time
          else:
            if not gate_detected:
              turtle.cmd_velocity(linear=0.0, angular=0.5)
            else:
              break
            integral = 0
            prev_time = current_time

          if cv2.waitKey(1) & 0xFF == ord('q'):
            break

          rate.sleep()

        self.go_in(rate)

        self._stop_and_wait(rate)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()
