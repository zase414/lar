from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat
import math

from image_proccesing import get_depth 
from typing import Tuple, List

import numpy as np
import cv2
import time

Vec3Int = Tuple[int, int, int]

def get_focal_length(width_pixels, fov_degrees):
    # Convert degrees to radians for math.tan
    fov_rad = math.radians(fov_degrees)
    return width_pixels / (2 * math.tan(fov_rad / 2))

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True, pc=True)
        self.stop = False

    def main(self):
        turtle = self.turtle
        sleep(2)

        turtle.wait_for_point_cloud()

        turtle.register_bumper_event_cb(lambda msge: callback_bumper_stop(self, msge))
        turtle.register_button_event_cb(lambda msge: callback_button0_resume(self, msge))
        rate = Rate(10)

        # -----------------------------------
        # NEW ODOMETRY TARGET & GAINS
        # -----------------------------------
        # -----------------------------------
        # NEW ODOMETRY TARGET & GAINS
        # -----------------------------------
        target_odo_x = None
        target_odo_y = None
        
        # Moving Average Filter Setup
        target_buffer_x = []
        target_buffer_y = []
        REQUIRED_READINGS = 5  # Number of valid frames to collect before locking

        Kp_linear = 0.3      # Forward speed cap
        Kp_angular = 1.5     # Steering aggressiveness
        GOAL_TOLERANCE = 0.05  # 5 cm stopping threshold

        WIDTH = 640
        H_FOV = 75.0
        fx = get_focal_length(WIDTH, H_FOV)

        while not self.turtle.is_shutting_down():
            if self.stop:
                turtle.cmd_velocity(linear=0.0, angular=0.0)
                rate.sleep()
                continue

            # 1. Get current Odometry pose (x, y, theta)
            pose = turtle.get_odometry()
            if pose is None:
                rate.sleep()
                continue
            curr_x, curr_y, curr_theta = pose

            # 2. Lock target in Odometry frame using a Simple Moving Average
            if target_odo_x is None:
                rectangles = self.detect_rectangles(turtle=turtle)

                if rectangles and len(rectangles) >= 2:
                    left, right = rectangles[0], rectangles[1]
                    left_x, left_dep = left[0], left[2]
                    right_x, right_dep = right[0], right[2]

                    # GUARD: Reject bad depth readings (0, negative, or NaN)
                    if (left_dep <= 0.01 or right_dep <= 0.01 or 
                        math.isnan(left_dep) or math.isnan(right_dep)):
                        print("Invalid depth detected. Skipping frame...")
                        rate.sleep()
                        continue

                    X_L = (left_x - 320) * left_dep / fx
                    Z_L = left_dep
                    X_R = (right_x - 320) * right_dep / fx
                    Z_R = right_dep

                    X_mid = (X_L + X_R) / 2.0
                    Z_mid = (Z_L + Z_R) / 2.0

                    # Create standard offset in front of garage
                    D_safe = 0.4
                    dX, dZ = X_R - X_L, Z_R - Z_L
                    length = math.hypot(dZ, -dX)

                    n_X, n_Z = (dZ / length, -dX / length) if length > 0 else (0, 1)

                    X_target_cam = X_mid + D_safe * n_X
                    Z_target_cam = Z_mid + D_safe * n_Z

                    # Convert Camera coordinates to Local Robot Frame
                    local_x = Z_target_cam
                    local_y = -X_target_cam

                    # Rotate and Translate into Global Odometry frame
                    calc_odo_x = curr_x + local_x * math.cos(curr_theta) - local_y * math.sin(curr_theta)
                    calc_odo_y = curr_y + local_x * math.sin(curr_theta) + local_y * math.cos(curr_theta)

                    # Add to our moving average buffer
                    target_buffer_x.append(calc_odo_x)
                    target_buffer_y.append(calc_odo_y)
                    print(f"Collected reading {len(target_buffer_x)}/{REQUIRED_READINGS}...")

                    # Lock the target once we have enough clean data
                    if len(target_buffer_x) >= REQUIRED_READINGS:
                        target_odo_x = sum(target_buffer_x) / REQUIRED_READINGS
                        target_odo_y = sum(target_buffer_y) / REQUIRED_READINGS
                        print(f"--> Target LOCKED at averaged pos: ({target_odo_x:.2f}, {target_odo_y:.2f})")
                    else:
                        # Stop moving while we collect the remaining readings
                        turtle.cmd_velocity(linear=0.0, angular=0.0)
                        rate.sleep()
                        continue
                        
                else:
                    # Still looking for target: turn in place to scan
                    turtle.cmd_velocity(linear=0.0, angular=0.4)
                    
                rate.sleep()
                continue # Skip the driving step until target_odo_x is firmly locked

            # 3. Odometry Control Loop: Navigate to the locked target
            # ... (Keep your existing Step 3 code here) ...

            # 3. Odometry Control Loop: Navigate to the locked target
            dx = target_odo_x - curr_x
            dy = target_odo_y - curr_y
            distance = math.hypot(dx, dy)

            # Check if we arrived
            if distance < GOAL_TOLERANCE:
                print("Distance to point is 0! Objective complete.")
                turtle.cmd_velocity(linear=0.0, angular=0.0)
                break

            # Calculate steering angle
            desired_theta = math.atan2(dy, dx)
            angle_error = desired_theta - curr_theta

            # Normalize angle to [-PI, PI] to avoid spinning the long way
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            # State Machine: If severely misaligned, turn in place. Otherwise, drive forward.
            if abs(angle_error) > 0.3:
                v = 0.0
                w = Kp_angular * angle_error
            else:
                v = min(Kp_linear, distance) # Smoothly slow down as you approach
                w = Kp_angular * angle_error

            turtle.cmd_velocity(linear=v, angular=w)
            rate.sleep()

        cv2.destroyAllWindows()


    def detect_rectangles(self, turtle) -> List[Vec3Int]:
        HUE_LOW   = 110
        HUE_HIGH  = 140
        SAT_MIN   = 50
        VALUE_MIN = 40

        im = turtle.get_rgb_image()
        
        if im is None:
            return None
            
        cv2.imshow("IMAGE", im)

        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        lower_bound = np.array([HUE_LOW, SAT_MIN, VALUE_MIN])
        upper_bound = np.array([HUE_HIGH, 255, 255])

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        filtered = cv2.bitwise_and(im, im, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        vertical_rects = []
        for c in contours:
            area = cv2.contourArea(c)
            if area > 300:
                x, y, w, h = cv2.boundingRect(c)
                if w > 0:
                    aspect_ratio = float(h) / w
                    if aspect_ratio > 1.5:
                        vertical_rects.append((x, y, w, h))

        # left/right
        vertical_rects = sorted(vertical_rects, key=lambda r: r[0])

        if len(vertical_rects) < 2:
            return None

        found_pair = vertical_rects[:2]

        ret: List[Vec3Int] = []
        
        for (x, y, w, h) in found_pair:
            cv2.rectangle(filtered, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(filtered, (center_x, center_y), 2, (0, 0, 255), 3)
            
            ret.append((center_x, center_y, get_depth(turtle, center_x, center_y, 2)))
        
        # calc center
        avg_x = (ret[0][0] + ret[1][0]) // 2
        avg_y = (ret[0][1] + ret[1][1]) // 2
        avg_dep = (ret[0][2] + ret[1][2]) // 2
        ret.append((avg_x, avg_y, avg_dep))

        cv2.circle(filtered, (avg_x, avg_y), 2, (0, 255, 0), 3)
        
        cv2.imshow("CONTOURS", filtered)
        cv2.waitKey(1)

        return ret

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()