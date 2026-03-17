from __future__ import print_function
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat
from image_proccesing import get_depth
from typing import Tuple

import numpy as np

import cv2

def main():
    turtle = Turtlebot(rgb=True, pc=True)
    sleep(2)

    # takes picture and saves it on launch
    # save_img(turtle)
    while True:
        center, radius = detect_balls(turtle=turtle)
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

def detect_balls(turtle) -> Tuple[Tuple[int, int], int]:
    HUE_LOW = 25
    HUE_HIGH = 65
    SAT_MIN = 37
    VALUE_MIN = 40

    im = turtle.get_rgb_image()
    if im is None:
        return
        
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    
    lower_bound = np.array([HUE_LOW, SAT_MIN, VALUE_MIN])
    upper_bound = np.array([HUE_HIGH, 255, 255])

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    filtered = cv2.bitwise_and(im, im, mask=mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = (0, 0)
    radius = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area > 300:
            perimeter = cv2.arcLength(c, True)
            circularity = (4 * np.pi * area) / (perimeter**2) if perimeter > 0 else 0

            if circularity > 0.7:
                (y, x), radius = cv2.minEnclosingCircle(c)
                center = (int(y), int(x))

                cv2.circle(filtered, center, int(radius), (0, 255, 0), 2)
                cv2.circle(filtered, center, 2, (0, 0, 255), 3)

    #cv2.imshow("CONTOURS", filtered)
    #cv2.imshow("IMAGE", im)
    #cv2.setMouseCallback("IMAGE", mouse_callback, hsv)
    #cv2.waitKey(1)
    return center, radius

if __name__ == "__main__":
    main()
