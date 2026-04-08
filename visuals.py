from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat
from image_proccesing import get_depth
from typing import Tuple, List

Vec2Int = Tuple[int, int]

import numpy as np

import cv2

def main():
    turtle = Turtlebot(rgb=True, pc=True)
    sleep(2)
    rate = Rate(10)

    # takes picture and saves it on launch
    # save_img(turtle)
    DEAD_CENTER_X = 360
    TOLERANCE_PIXEL_BAND = 6

    while True:
        (center_x, _), _ = detect_balls(turtle=turtle)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        dist = DEAD_CENTER_X - center_x

        ang_speed = max(min(abs(dist * 0.01), 0.6), 0.09)
        ang_speed = -1 * ang_speed if dist < 0 else ang_speed

        print("balls position on camera x ", center_x, "calculated ang speed ", ang_speed)
        if (abs(dist) < TOLERANCE_PIXEL_BAND):
            print("centered")
        else:
            turtle.cmd_velocity(0, ang_speed)
            rate.sleep()


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
        if area > 200:
            perimeter = cv2.arcLength(c, True)
            circularity = (4 * np.pi * area) / (perimeter**2) if perimeter > 0 else 0

            if circularity > 0.7:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))

                cv2.circle(filtered, center, int(radius), (0, 255, 0), 2)
                cv2.circle(filtered, center, 2, (0, 0, 255), 3)

    #cv2.imshow("CONTOURS", filtered)
    #cv2.imshow("IMAGE", im)
    #cv2.setMouseCallback("IMAGE", mouse_callback, hsv)
    #cv2.waitKey(1)
    return center, radius

def detect_rectangles(turtle) -> List[Vec2Int]:
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

  ret: List[Vec2Int] = []
  
  for (x, y, w, h) in found_pair:
    cv2.rectangle(filtered, (x, y), (x + w, y + h), (0, 255, 0), 2)
    center_x = x + w // 2
    center_y = y + h // 2
    cv2.circle(filtered, (center_x, center_y), 2, (255, 0, 0), 3)
    
    ret.append((center_x, center_y))
  
  # calc center
  avg_x = (ret[0][0] + ret[1][0]) // 2
  avg_y = (ret[0][1] + ret[1][1]) // 2
  ret.append((avg_x, avg_y))

  cv2.circle(filtered, (avg_x, avg_y), 2, (0, 255, 0), 3)
  
  cv2.imshow("contours", filtered)

  return ret

if __name__ == "__main__":
    main()
