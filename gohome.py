from __future__ import print_function
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat
from image_proccesing import get_depth # Assuming this is your custom module

import numpy as np
import cv2

def main():
    turtle = Turtlebot(rgb=True, pc=True)
    sleep(2)

    # takes picture and saves it on launch
    # save_img(turtle)
    while True:
        # Now expecting a list of detected rectangles
        rects = detect_rectangles(turtle=turtle)
        
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

def detect_rectangles(turtle) -> list:
    # --- 1. Adjust Color Bounds for Purple ---
    # OpenCV Hue goes from 0 to 179. Purple usually falls between 125 and 160.
    HUE_LOW = 125
    HUE_HIGH = 160
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
    
    # --- 2. Filter for Vertical Rectangles ---
    for c in contours:
        area = cv2.contourArea(c)
        if area > 300: # Filter out noise
            # Get the straight bounding rectangle
            x, y, w, h = cv2.boundingRect(c)
            
            # Prevent division by zero just in case
            if w > 0:
                aspect_ratio = float(h) / w
                
                # If height is 1.5x larger than width, it's a vertical rectangle
                if aspect_ratio > 1.5:
                    vertical_rects.append((x, y, w, h))

    # Sort the rectangles from left to right (based on x-coordinate)
    vertical_rects = sorted(vertical_rects, key=lambda r: r[0])
    
    # Look for at least two rectangles to represent your `| |`
    found_pair = []
    if len(vertical_rects) >= 2:
        # Grab the first two from left to right
        found_pair = vertical_rects[:2]
        
        # Draw them on the filtered image
        for (x, y, w, h) in found_pair:
            # Draw the bounding box
            cv2.rectangle(filtered, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate and draw the center point
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(filtered, (center_x, center_y), 2, (0, 0, 255), 3)

    cv2.imshow("CONTOURS", filtered)
    cv2.imshow("IMAGE", im)
    cv2.setMouseCallback("IMAGE", mouse_callback, hsv)
    cv2.waitKey(1)
    
    return found_pair

if __name__ == "__main__":
    main()