from __future__ import print_function
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat

import numpy as np

import cv2

def main():
    turtle = Turtlebot(rgb=True, pc=True)

    # takes picture and saves it on launch
    # save_img(turtle)
    while True:
        detect_balls(turtle=turtle)
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

def detect_balls(turtle):
    HUE_REF = 45 
    HUE_MAX = 20
    SAT_MIN = 37
    VALUE_MIN = 40

    im = turtle.get_rgb_image()
    if im is None:
        return
        
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    lower_hue = HUE_REF - HUE_MAX
    upper_hue = HUE_REF + HUE_MAX

    if lower_hue < 0:
        mask1 = cv2.inRange(hsv, np.array([0, SAT_MIN, VALUE_MIN]), np.array([upper_hue, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([180 + lower_hue, SAT_MIN, VALUE_MIN]), np.array([179, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv, np.array([lower_hue, SAT_MIN, VALUE_MIN]), np.array([upper_hue, 255, 255]))

    #noise handler
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    #find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    output = im.copy()
    output[~mask] = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 10:
            
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            cv2.circle(output, center, int(radius), (0, 255, 0), 2)
            cv2.circle(output, center, 2, (0, 0, 255), 3)

    cv2.imshow("Detection", output)
    cv2.setMouseCallback("Detection", mouse_callback, hsv)
    cv2.waitKey(1)

if __name__ == "__main__":
    main()
