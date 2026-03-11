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
        HUE_SIZE = 179
        HUE_REF = 45 #random green from color picker
        HUE_MAX = 20
        SAT_MIN = 37
        VALUE_MIN = 40
  
        im = turtle.get_rgb_image()
        if im is None:
            return
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
                
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        mask = (
            (np.minimum(np.abs(h - HUE_REF), 180 - np.abs(h - HUE_REF)) < HUE_MAX)
            &
            (s > SAT_MIN)
            &
            (v > VALUE_MIN)
            )

        binary_mask = mask.astype(np.uint8) * 255
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        filtered = im.copy()
        filtered[~mask] = 0

        for cnt in contours:
            #pro vetsi kruhy nez 10
            if cv2.contourArea(cnt) > 10:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)

                
                cv2.circle(filtered, center, radius, (0, 255, 0), 2)
                
                cv2.circle(filtered, center, 2, (0, 0, 255), 3)

        cv2.imshow("CONTOURS", filtered)
        cv2.imshow("IMAGE", im)
        cv2.setMouseCallback("IMAGE", mouse_callback, hsv)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()
