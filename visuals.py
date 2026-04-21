from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat
from image_proccesing import get_depth
from typing import Optional, Tuple, List

Vec2Int = Tuple[int, int]

import numpy as np
import cv2

# --- Ball detection constants ---
BALL_HUE_LOW   = 25
BALL_HUE_HIGH  = 65
BALL_SAT_MIN   = 40
BALL_VALUE_MIN = 40
BALL_MIN_AREA  = 200
BALL_CIRCULARITY_THRESH = 0.7

# --- Rectangle detection constants ---
RECT_HUE_LOW   = 110
RECT_HUE_HIGH  = 140
RECT_SAT_MIN   = 50
RECT_VALUE_MIN = 40
RECT_MIN_AREA  = 300
RECT_ASPECT_RATIO_MIN = 1.5

MORPH_KERNEL_SIZE = (5, 5)


def main() -> None:
    """Entry point. Initialises the robot and runs the ball-detection loop."""
    turtle = Turtlebot(rgb=True, pc=True)
    sleep(2)
    rate = Rate(10)

    while True:
        (center_x, _), _ = detect_balls(turtle=turtle)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


def mouse_callback(event, x, y, flags, param) -> None:
    """OpenCV mouse callback that prints HSV values at the cursor position.

    Args:
        param: HSV image array passed via cv2.setMouseCallback.
    """
    if event == cv2.EVENT_MOUSEMOVE:
        hsv = param
        h, s, v = hsv[y, x]
        print(f"x:{x} y:{y} -> H:{h} S:{s} V:{v}")


def detect_balls(turtle) -> Tuple[Tuple[int, int], int]:
    """Detects yellow/green balls in the robot's camera feed using HSV thresholding.

    Returns the centre pixel and radius of the most recently found circular contour.
    Returns ((0, 0), 0) when no qualifying contour is found.

    Args:
        turtle: Turtlebot instance with an active RGB camera.

    Returns:
        A tuple of (center, radius) where center is (x, y) in pixel coordinates
        and radius is in pixels. Returns ((0, 0), 0) if no frame is available
        or no ball is detected.

    Raises:
        ValueError: If the camera returns None unexpectedly after initialisation.
    """
    im = turtle.get_rgb_image()
    if im is None:
        raise ValueError("detect_balls: camera returned None — is the RGB stream active?")

    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    lower_bound = np.array([BALL_HUE_LOW,   BALL_SAT_MIN,   BALL_VALUE_MIN])
    upper_bound = np.array([BALL_HUE_HIGH,  255,             255])

    kernel = np.ones(MORPH_KERNEL_SIZE, np.uint8)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    filtered = cv2.bitwise_and(im, im, mask=mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    center, radius = (0, 0), 0
    for c in contours:
        area = cv2.contourArea(c)
        if area > BALL_MIN_AREA:
            perimeter = cv2.arcLength(c, True)
            circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0
            if circularity > BALL_CIRCULARITY_THRESH:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                cv2.circle(filtered, center, int(radius), (0, 255, 0), 2)
                cv2.circle(filtered, center, 2, (0, 0, 255), 3)

    return center, radius


def detect_rectangles(turtle) -> Optional[List[Vec2Int]]:
    """Detects a pair of blue vertical rectangles and returns their centres plus midpoint.

    Looks for the two leftmost qualifying contours, sorts them left-to-right,
    and appends their average centre as a third entry.

    Args:
        turtle: Turtlebot instance with an active RGB camera.

    Returns:
        A list of three (x, y) pixel coordinates: [left_centre, right_centre, midpoint],
        or None if the camera returns no frame or fewer than two rectangles are found.
    """
    im = turtle.get_rgb_image()
    if im is None:
        return None

    cv2.imshow("IMAGE", im)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    lower_bound = np.array([RECT_HUE_LOW,   RECT_SAT_MIN,   RECT_VALUE_MIN])
    upper_bound = np.array([RECT_HUE_HIGH,  255,             255])

    kernel = np.ones(MORPH_KERNEL_SIZE, np.uint8)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    filtered = cv2.bitwise_and(im, im, mask=mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    vertical_rects = []
    for c in contours:
        area = cv2.contourArea(c)
        if area > RECT_MIN_AREA:
            x, y, w, h = cv2.boundingRect(c)
            if w > 0 and float(h) / w > RECT_ASPECT_RATIO_MIN:
                vertical_rects.append((x, y, w, h))

    vertical_rects = sorted(vertical_rects, key=lambda r: r[0])

    if len(vertical_rects) < 2:
        return None

    ret: List[Vec2Int] = []
    for (x, y, w, h) in vertical_rects[:2]:
        cv2.rectangle(filtered, (x, y), (x + w, y + h), (0, 255, 0), 2)
        center_x = x + w // 2
        center_y = y + h // 2
        cv2.circle(filtered, (center_x, center_y), 2, (255, 0, 0), 3)
        ret.append((center_x, center_y))

    avg_x = (ret[0][0] + ret[1][0]) // 2
    avg_y = (ret[0][1] + ret[1][1]) // 2
    ret.append((avg_x, avg_y))
    cv2.circle(filtered, (avg_x, avg_y), 2, (0, 255, 0), 3)
    cv2.imshow("contours", filtered)

    return ret


if __name__ == "__main__":
    main()