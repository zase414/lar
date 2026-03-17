from __future__ import print_function
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat
from image_proccesing import get_depth
from typing import Tuple

import numpy as np
import cv2
import time

def main():
    turtle = Turtlebot(rgb=True, pc=True)
    sleep(2)

    # --- PID Tuning Constants ---
    # You will need to tune these values based on your robot's responsiveness
    Kp = 0.005  # Proportional: Reacts to current error
    Ki = 0.0001 # Integral: Reacts to accumulated past error
    Kd = 0.001  # Derivative: Reacts to the rate of change of the error

    # --- PID State Variables ---
    integral = 0
    prev_error = 0
    prev_time = time.time()
    
    # Target Y-coordinate (e.g., center of a 480p image is 240)
    # Adjust this to match half the height of your specific camera resolution
    TARGET_Y = 240 

    while True:
        # Assuming detect_rectangles returns an (x, y) tuple, or None if nothing is found
        center = detect_rectangles(turtle=turtle)
        
        current_time = time.time()
        dt = current_time - prev_time
        
        if center is not None and dt > 0:
            cx, cy = center
            
            # 1. Calculate Error
            error = TARGET_Y - cy
            
            # 2. Calculate P, I, and D terms
            proportional = Kp * error
            integral += error * dt
            derivative = (error - prev_error) / dt
            
            # 3. Calculate total PID output
            pid_output = proportional + (Ki * integral) + (Kd * derivative)
            
            # 4. Regulate Velocity
            # Applying PID to angular velocity. Cap the max speed if necessary.
            turtle.cmd_velocity(linear=0.4, angular=pid_output)
            
            # 5. Store current values for the next loop iteration
            prev_error = error
            
        else:
            # Safety fallback: Stop moving if no rectangle is detected
            turtle.cmd_velocity(linear=0.0, angular=0.0)
            
        # Update prev_time regardless of detection to prevent massive dt spikes
        prev_time = current_time 

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


def save_img(turtle):
    sleep(2)
    im = turtle.get_rgb_image()
    # get K, images, and point cloud
    data = dict()
    data['image_rgb'] = im
    data['image_hsv'] = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # save data to .mat file
    filename = datetime.today().strftime("%Y-%m-%d-%H-%M-%S") + ".mat"
    savemat(filename, data)

    print('Data saved in {}'.format(filename))

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        hsv = param
        h, s, v = hsv[y, x]
        print(f"x:{x} y:{y} -> H:{h} S:{s} V:{v}")

def detect_rectangles(turtle) -> Tuple[int, int]:

    HUE_LOW = 110
    HUE_HIGH = 140
    SAT_MIN = 40
    VALUE_MIN = 40

    im = turtle.get_rgb_image()
    if im is None:
        return []
        
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
        if area > 300: # no noise
            
            x, y, w, h = cv2.boundingRect(c)
            
            if w > 0:
                aspect_ratio = float(h) / w
               
                if aspect_ratio > 1.5:
                    vertical_rects.append((x, y, w, h))

    # sort
    vertical_rects = sorted(vertical_rects, key=lambda r: r[0])
    
    # find | |
    found_pair = []

    center = (0,0)
    if len(vertical_rects) >= 2:
        # first 2 from the left
        found_pair = vertical_rects[:2]
        
        total_x = 0
        total_y = 0
        # draw boundaries and dots
        for (x, y, w, h) in found_pair:
            # box
            cv2.rectangle(filtered, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # center of each rect
            center_x = x + w // 2
            center_y = y + h // 2
            total_x += center_x
            total_y += center_y
            cv2.circle(filtered, (center_x, center_y), 2, (0, 0, 255), 3)
        if len(found_pair) > 0:
            avg_x = total_x // len(found_pair)
            avg_y = total_y // len(found_pair)
            cv2.circle(filtered, (avg_x, avg_y), 2, (0, 0, 255), 3)
            center = (avg_x, avg_y)
            print(center)

    cv2.imshow("CONTOURS", filtered)
    cv2.imshow("IMAGE", im)
    # cv2.setMouseCallback("IMAGE", mouse_callback, hsv)
    cv2.waitKey(1)
    
    return center

if __name__ == "__main__":
    main()