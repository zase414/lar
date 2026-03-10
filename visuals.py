from __future__ import print_function
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time
from datetime import datetime

import numpy as np

import cv2

def main():
    turtle = Turtlebot(rgb=True)

    #takes picture and saves it on launch
    save_img(turtle)
    while True:
        detect_balls(turtle=turtle)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


def save_img(turtle):
    sleep(2)
    turtle.get_rgb_image()
    # get K, images, and point cloud
    data = dict()
    data['image_rgb'] = im
    data['image_hsv'] = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # save data to .mat file
    filename = datetime.today().strftime("%Y-%m-%d-%H-%M-%S") + ".mat"
    savemat(filename, data)

    print('Data saved in {}'.format(filename))


def detect_balls(turtle):
        HUE_SIZE = 179
        HUE_REF = 125 #random green from color picker
        HUE_MAX = 0.9
        SAT_MIN = 25
        VALUE_MIN = 25
  
        im = turtle.get_rgb_image()
        if im is None:
            return
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
                
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        mask = (
            (np.minimum(np.abs(h - HUE_REF), 180 - np.abs(h - HUE_REF)) < HUE_MAX)
#            &
#            (s > SAT_MIN)
#            &
#            (v > VALUE_MIN)
            )

        filtered = im.copy()
        filtered[~mask] = 0

        cv2.imshow("HSV_FILTER", filtered)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()
