from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat
from time import sleep
import math

from image_proccesing import get_depth 
from typing import Tuple, List

import numpy as np
import cv2
import time

Vec3Int = Tuple[int, int, int]

def get_focal_length(width_pixels, fov_degrees):
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

        state = "SCANNING" 
        
        target_odo_x = None
        target_odo_y = None
        target_odo_theta = None
        start_x = None
        start_y = None
        
        measurements_X = []
        measurements_Z = []
        measurements_Yaw = []
        REQUIRED_READINGS = 10  

        Kp_linear = 0.25     
        Kp_angular = 1.0     
        GOAL_TOLERANCE = 0.05  

        WIDTH = 640
        H_FOV = 60.0
        fx = get_focal_length(WIDTH, H_FOV)

        PYLON_WIDTH = 0.07
        D_SAFE = 0.4

        while not self.turtle.is_shutting_down():
            if self.stop:
                turtle.cmd_velocity(linear=0.0, angular=0.0)
                rate.sleep()
                continue

            pose = turtle.get_odometry()
            if pose is None:
                rate.sleep()
                continue
            curr_x, curr_y, curr_theta = pose

            # ---------------------------------------------------------
            # STATE: SCANNING
            # ---------------------------------------------------------
            if state == "SCANNING":
                rectangles = self.detect_rectangles(turtle=turtle)
                if rectangles and len(rectangles) >= 2:
                    sleep(1)
                    print("Pylons detected! Stopping to measure...")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    measurements_X.clear()
                    measurements_Z.clear()
                    measurements_Yaw.clear()
                    state = "MEASURING"
                else:
                    turtle.cmd_velocity(linear=0.0, angular=0.4)
                rate.sleep()
                continue

            # ---------------------------------------------------------
            # STATE: MEASURING 
            # ---------------------------------------------------------
            elif state == "MEASURING":
                turtle.cmd_velocity(linear=0.0, angular=0.0) 
                rectangles = self.detect_rectangles(turtle=turtle)
                
                if not rectangles or len(rectangles) < 2:
                    state = "SCANNING"
                    rate.sleep()
                    continue

                left, right = rectangles[0], rectangles[1]
                
                # Assuming your detect_rectangles returns bounding boxes like [x, y, w, h]
                left_x, left_w = left[0], left[2]  
                right_x, right_w = right[0], right[2]

                if left_w <= 0 or right_w <= 0:
                    rate.sleep()
                    continue

                # 1. Get depth to EACH pylon individually using their bounding box widths
                # (Note: If you have point cloud access, extracting the true depth here is much better!)
                Z_L = (PYLON_WIDTH * fx) / left_w
                Z_R = (PYLON_WIDTH * fx) / right_w

                # 2. Get X coordinates in Camera Frame
                X_L = (left_x - (WIDTH / 2.0)) * Z_L / fx
                X_R = (right_x - (WIDTH / 2.0)) * Z_R / fx

                # 3. Find the Gate Center
                X_C = (X_L + X_R) / 2.0
                Z_C = (Z_L + Z_R) / 2.0

                # 4. Find the Normal Vector (Perpendicular to the gate, pointing at camera)
                V_X = X_R - X_L
                V_Z = Z_R - Z_L
                
                N_X = V_Z
                N_Z = -V_X

                # Normalize the vector
                length = math.hypot(N_X, N_Z)
                n_x = N_X / length
                n_z = N_Z / length

                # 5. Calculate Parking Target (D_SAFE meters out along the normal)
                X_target_cam = X_C + (n_x * D_SAFE)
                Z_target_cam = Z_C + (n_z * D_SAFE)

                # 6. Calculate target Yaw to face the gate (opposite of normal vector)
                # In robot frame (X forward, Y left), the target direction is (-n_z, n_x)
                local_target_yaw = math.atan2(n_x, -n_z)
                
                measurements_X.append(X_target_cam)
                measurements_Z.append(Z_target_cam)
                measurements_Yaw.append(local_target_yaw)

                if len(measurements_X) >= REQUIRED_READINGS:
                    avg_X = sum(measurements_X) / len(measurements_X)
                    avg_Z = sum(measurements_Z) / len(measurements_Z)
                    avg_local_yaw = sum(measurements_Yaw) / len(measurements_Yaw)

                    # Transform to Robot Frame
                    local_x = avg_Z
                    local_y = -avg_X

                    # Transform to Global Odometry
                    target_odo_x = curr_x + local_x * math.cos(curr_theta) - local_y * math.sin(curr_theta)
                    target_odo_y = curr_y + local_x * math.sin(curr_theta) + local_y * math.cos(curr_theta)
                    target_odo_theta = (curr_theta + avg_local_yaw) % (2 * math.pi)

                    print("Target Locked! Moving to parking spot.")
                    state = "DRIVING_TO_PARK"
                
                rate.sleep()
                continue

            # ---------------------------------------------------------
            # STATE: DRIVING TO PARK
            # ---------------------------------------------------------
            elif state == "DRIVING_TO_PARK":
                dx = target_odo_x - curr_x
                dy = target_odo_y - curr_y
                distance = math.hypot(dx, dy)

                if distance < GOAL_TOLERANCE:
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    print("Parked. Aligning to face gate...")
                    state = "ALIGNING"
                    rate.sleep()
                    continue

                desired_theta = math.atan2(dy, dx)
                angle_error = (desired_theta - curr_theta + math.pi) % (2 * math.pi) - math.pi

                if abs(angle_error) > 0.5:
                    v = 0.0
                    w = math.copysign(0.5, angle_error) 
                else:
                    v = max(0.05, min(Kp_linear, distance * 0.8)) 
                    w = Kp_angular * angle_error

                turtle.cmd_velocity(linear=v, angular=w)
                self.detect_rectangles(turtle=turtle) 
                rate.sleep()

            # ---------------------------------------------------------
            # STATE: ALIGNING (Rotate to face the gate directly)
            # ---------------------------------------------------------
            elif state == "ALIGNING":
                angle_error = (target_odo_theta - curr_theta + math.pi) % (2 * math.pi) - math.pi
                
                if abs(angle_error) < 0.05:
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    print("Aligned! Driving forward 50 cm.")
                    start_x, start_y = curr_x, curr_y
                    state = "DRIVING_THROUGH"
                else:
                    turtle.cmd_velocity(linear=0.0, angular=math.copysign(0.5, angle_error))
                
                rate.sleep()

            # ---------------------------------------------------------
            # STATE: DRIVING THROUGH (Drive exactly 50cm)
            # ---------------------------------------------------------
            elif state == "DRIVING_THROUGH":
                dist_driven = math.hypot(curr_x - start_x, curr_y - start_y)
                
                if dist_driven >= 0.5:
                    print("Gate successfully navigated!")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    break
                else:
                    turtle.cmd_velocity(linear=0.15, angular=0.0) # Slow, straight line

                rate.sleep()

        cv2.destroyAllWindows()

    def detect_rectangles(self, turtle) -> List[Vec3Int]:
        # [I kept your exact CV2 implementation here, it looked solid]
        HUE_LOW   = 110
        HUE_HIGH  = 140
        SAT_MIN   = 50
        VALUE_MIN = 40

        im = turtle.get_rgb_image()
        if im is None: return None
            
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

        vertical_rects = sorted(vertical_rects, key=lambda r: r[0])

        if len(vertical_rects) < 2:
            cv2.waitKey(1)
            return None

        found_pair = vertical_rects[:2]
        ret: List[Vec3Int] = []
        
        for (x, y, w, h) in found_pair:
            cv2.rectangle(filtered, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(filtered, (center_x, center_y), 2, (0, 0, 255), 3)
            ret.append((center_x, center_y, 
                #get_depth(turtle, center_x, center_y, 2)
                ))
        
        avg_x = (ret[0][0] + ret[1][0]) // 2
        avg_y = (ret[0][1] + ret[1][1]) // 2
        # avg_dep = (ret[0][2] + ret[1][2]) // 2
        ret.append((avg_x, avg_y, 
        #avg_dep
        0
        ))

        cv2.circle(filtered, (avg_x, avg_y), 2, (0, 255, 0), 3)
        cv2.imshow("CONTOURS", filtered)
        cv2.waitKey(1)

        return ret

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()