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

        # Explicit states make the logic much more robust
        state = "SCANNING" 
        
        target_odo_x = None
        target_odo_y = None
        
        # Buffer for Camera Frame coordinates
        measurements_x = []
        measurements_z = []
        REQUIRED_READINGS = 10  # We can take more because we will be stationary

        Kp_linear = 0.25     
        Kp_angular = 1.0     
        GOAL_TOLERANCE = 0.05  

        WIDTH = 640
        H_FOV = 50.0
        fx = get_focal_length(WIDTH, H_FOV)

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
            # STATE: SCANNING (Looking for the pylons)
            # ---------------------------------------------------------
            if state == "SCANNING":
                rectangles = self.detect_rectangles(turtle=turtle)
                if rectangles and len(rectangles) >= 2:
                    print("Pylons detected! Stopping to measure...")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    measurements_x.clear()
                    measurements_z.clear()
                    state = "MEASURING"
                else:
                    turtle.cmd_velocity(linear=0.0, angular=0.4)
                
                rate.sleep()
                continue

            # ---------------------------------------------------------
            # STATE: MEASURING (Stationary data collection)
            # ---------------------------------------------------------
            elif state == "MEASURING":
                turtle.cmd_velocity(linear=0.0, angular=0.0) # Ensure we are stopped
                rectangles = self.detect_rectangles(turtle=turtle)
                
                if not rectangles or len(rectangles) < 2:
                    print("Lost pylons during measurement. Resuming scan...")
                    state = "SCANNING"
                    rate.sleep()
                    continue

                # 1. Setup our known geometric constants
                PYLON_SPACE_BETWEEN = 0.5
                D_SAFE = 0.4
                
                left, right = rectangles[0], rectangles[1]
                left_x = left[0]
                right_x = right[0]

                # 2. Find the apparent width in pixels
                px_dist = right_x - left_x
                
                if px_dist <= 0:
                    print("Invalid bounding boxes. Skipping frame...")
                    rate.sleep()
                    continue

                # 3. Calculate distance (Z) to the gate using similar triangles
                # Formula: Z = (Real_Width * Focal_Length) / Pixel_Width
                Z_gate = (PYLON_SPACE_BETWEEN * fx) / px_dist

                # 4. Calculate the lateral offset (X) of the gate's center
                mid_x = (left_x + right_x) / 2.0
                X_gate = (mid_x - (WIDTH / 2.0)) * Z_gate / fx

                # 5. Set our target point (Stop D_SAFE meters straight back)
                X_target_cam = X_gate
                Z_target_cam = Z_gate - D_SAFE
                
                measurements_x.append(X_target_cam)
                measurements_z.append(Z_target_cam)
                print(f"Collected clean geometric reading {len(measurements_x)}/{REQUIRED_READINGS}")

                if len(measurements_x) >= REQUIRED_READINGS:
                    # Average the raw camera coordinates
                    avg_X_cam = sum(measurements_x) / len(measurements_x)
                    avg_Z_cam = sum(measurements_z) / len(measurements_z)

                    # Transform to Robot Frame (ROS standard: X forward, Y left)
                    local_x = avg_Z_cam
                    local_y = -avg_X_cam

                    # Transform ONE time to global Odometry frame
                    target_odo_x = curr_x + local_x * math.cos(curr_theta) - local_y * math.sin(curr_theta)
                    target_odo_y = curr_y + local_x * math.sin(curr_theta) + local_y * math.cos(curr_theta)

                    print(f"--> Target LOCKED at Odometry: ({target_odo_x:.2f}, {target_odo_y:.2f})")
                    state = "DRIVING"
                
                rate.sleep()
                continue

            # ---------------------------------------------------------
            # STATE: DRIVING (Navigating to locked Odometry point)
            # ---------------------------------------------------------
            elif state == "DRIVING":
                dx = target_odo_x - curr_x
                dy = target_odo_y - curr_y
                distance = math.hypot(dx, dy)

                if distance < GOAL_TOLERANCE:
                    print("Arrived at target point!")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    break

                desired_theta = math.atan2(dy, dx)
                angle_error = (desired_theta - curr_theta + math.pi) % (2 * math.pi) - math.pi

                # Smoother control logic
                if abs(angle_error) > 0.5:
                    # If heavily misaligned, turn in place, but smoothly
                    v = 0.0
                    w = math.copysign(0.5, angle_error) # Cap turn speed
                else:
                    # Blend forward and turning smoothly
                    v = max(0.05, min(Kp_linear, distance * 0.8)) # Don't go slower than 0.05 m/s
                    w = Kp_angular * angle_error

                turtle.cmd_velocity(linear=v, angular=w)
                
                # Optional: keep updating camera window so it doesn't freeze
                self.detect_rectangles(turtle=turtle) 
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