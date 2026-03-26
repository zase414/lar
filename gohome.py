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

        # Added states to match the required sequence
        state = "SCANNING" 
        
        target_odo_x = None
        target_odo_y = None
        target_heading = None
        drive_start_x = None
        drive_start_y = None
        
        # Buffers for target normal and position
        measurements_Px = []
        measurements_Pz = []
        measurements_Nx = []
        measurements_Nz = []
        REQUIRED_READINGS = 10  

        Kp_linear = 0.25     
        Kp_angular = 1.0     
        GOAL_TOLERANCE = 0.05  

        WIDTH = 640
        H_FOV = 60.0
        fx = get_focal_length(WIDTH, H_FOV)
        
        # Standard Horizon Line (Center Y for a 640x480 resolution)
        # Adjust if your camera resolution differs.
        CY = 240.0 

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
                    sleep(1)
                    print("Pylons detected! Stopping to measure...")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    measurements_Px.clear()
                    measurements_Pz.clear()
                    measurements_Nx.clear()
                    measurements_Nz.clear()
                    state = "MEASURING"
                else:
                    turtle.cmd_velocity(linear=0.0, angular=0.4)
                
                rate.sleep()
                continue

            # ---------------------------------------------------------
            # STATE: MEASURING (Extracting 3D geometry from 2D frame)
            # ---------------------------------------------------------
            elif state == "MEASURING":
                turtle.cmd_velocity(linear=0.0, angular=0.0) 
                rectangles = self.detect_rectangles(turtle=turtle)
                
                if not rectangles or len(rectangles) < 2:
                    print("Lost pylons during measurement. Resuming scan...")
                    state = "SCANNING"
                    rate.sleep()
                    continue

                PYLON_SPACE_BETWEEN = 0.5
                D_SAFE = 0.5 # Distance to park in front of the gate
                
                left, right = rectangles[0], rectangles[1]
                left_x = left[0]
                right_x = right[0]

                # 1. Perspective Depth Ratio using Bounding Box Y Centers
                # The further away an object is on the ground, the closer it moves to the horizon line (CY)
                dy_L = max(float(left[1]) - CY, 1.0)
                dy_R = max(float(right[1]) - CY, 1.0)
                R = dy_R / dy_L  # Ratio of Z distances: Z_L = R * Z_R
                
                # 2. X coordinates in unit camera plane
                u_L = (left_x - (WIDTH / 2.0)) / fx
                u_R = (right_x - (WIDTH / 2.0)) / fx
                
                # 3. Solve for exact Z distances using the known width between pylons
                denom = math.sqrt((u_L * R - u_R)**2 + (R - 1.0)**2)
                if denom < 0.0001: # Avoid division by zero
                    rate.sleep()
                    continue
                    
                Z_R = PYLON_SPACE_BETWEEN / denom
                Z_L = R * Z_R
                
                # 4. Reconstruct True 3D Coordinates of both pylons in Camera Frame
                X_L = u_L * Z_L
                X_R = u_R * Z_R
                
                C_x = (X_L + X_R) / 2.0
                C_z = (Z_L + Z_R) / 2.0
                
                # 5. Extract the Normal Vector (Perpendicular line coming out of the gate)
                V_x = X_R - X_L
                V_z = Z_R - Z_L
                W_calc = math.hypot(V_x, V_z)
                if W_calc < 0.0001: W_calc = 0.0001
                
                # Normal facing towards the camera
                N_x = V_z / W_calc
                N_z = -V_x / W_calc 
                
                # 6. Set Target Parking Spot D_SAFE meters along the Normal
                P_cam_x = C_x + D_SAFE * N_x
                P_cam_z = C_z + D_SAFE * N_z
                
                measurements_Px.append(P_cam_x)
                measurements_Pz.append(P_cam_z)
                measurements_Nx.append(N_x)
                measurements_Nz.append(N_z)
                print(f"Collected geometric reading {len(measurements_Px)}/{REQUIRED_READINGS}")

                if len(measurements_Px) >= REQUIRED_READINGS:
                    # Average the noise out
                    avg_Px = sum(measurements_Px) / len(measurements_Px)
                    avg_Pz = sum(measurements_Pz) / len(measurements_Pz)
                    avg_Nx = sum(measurements_Nx) / len(measurements_Nx)
                    avg_Nz = sum(measurements_Nz) / len(measurements_Nz)

                    local_x = avg_Pz
                    local_y = -avg_Px

                    target_odo_x = curr_x + local_x * math.cos(curr_theta) - local_y * math.sin(curr_theta)
                    target_odo_y = curr_y + local_x * math.sin(curr_theta) + local_y * math.cos(curr_theta)
                    
                    # Calculate the absolute global heading to look directly at the center of the gate
                    target_heading = curr_theta + math.atan2(avg_Nx, -avg_Nz)
                    target_heading = (target_heading + math.pi) % (2 * math.pi) - math.pi

                    print(f"--> Parking Spot LOCKED: ({target_odo_x:.2f}, {target_odo_y:.2f})")
                    state = "DRIVING_TO_PARK"
                
                rate.sleep()
                continue

            # ---------------------------------------------------------
            # STATE: DRIVING_TO_PARK (Navigate to perpendicular spot)
            # ---------------------------------------------------------
            elif state == "DRIVING_TO_PARK":
                dx = target_odo_x - curr_x
                dy = target_odo_y - curr_y
                distance = math.hypot(dx, dy)

                if distance < GOAL_TOLERANCE:
                    print("Parked in front of gate! Now rotating...")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    state = "TURNING_TO_GATE"
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
                self.detect_rectangles(turtle=turtle) # Keep camera alive
                rate.sleep()

            # ---------------------------------------------------------
            # STATE: TURNING_TO_GATE (Align to the normal vector)
            # ---------------------------------------------------------
            elif state == "TURNING_TO_GATE":
                angle_error = (target_heading - curr_theta + math.pi) % (2 * math.pi) - math.pi
                
                if abs(angle_error) < 0.05: # Roughly 3 degrees tolerance
                    print("Aligned with gate! Driving 50cm forward...")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    drive_start_x = curr_x
                    drive_start_y = curr_y
                    state = "DRIVING_THROUGH"
                    rate.sleep()
                    continue
                    
                v = 0.0
                w = max(0.1, min(0.5, abs(angle_error) * Kp_angular)) * math.copysign(1.0, angle_error)
                turtle.cmd_velocity(linear=v, angular=w)
                self.detect_rectangles(turtle=turtle)
                rate.sleep()

            # ---------------------------------------------------------
            # STATE: DRIVING_THROUGH (Drive precisely 50cm forward)
            # ---------------------------------------------------------
            elif state == "DRIVING_THROUGH":
                distance_driven = math.hypot(curr_x - drive_start_x, curr_y - drive_start_y)
                
                if distance_driven >= 0.5:
                    print("Mission Accomplished! Driven 50cm.")
                    turtle.cmd_velocity(linear=0.0, angular=0.0)
                    break
                    
                # Small corrective angular velocity to drive perfectly straight
                angle_error = (target_heading - curr_theta + math.pi) % (2 * math.pi) - math.pi
                w = Kp_angular * angle_error * 0.5
                
                turtle.cmd_velocity(linear=0.15, angular=w)
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