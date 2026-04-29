from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from scipy.io import savemat
from typing import Optional, Tuple, List

Vec2Int = Tuple[int, int]

import numpy as np
import cv2

# --- Ball detection constants ---
BALL_HUE_LOW   = 34
BALL_HUE_HIGH  = 65
BALL_SAT_MIN   = 50
BALL_VALUE_MIN = 40
BALL_MIN_AREA  = 120
BALL_CIRCULARITY_THRESH = 0.7

# --- Rectangle detection constants ---
RECT_HUE_LOW   = 110
RECT_HUE_HIGH  = 140
RECT_SAT_MIN   = 50
RECT_VALUE_MIN = 40
RECT_MIN_AREA  = 300
RECT_ASPECT_RATIO_MIN = 1.5

MORPH_KERNEL_SIZE = (5, 5)

# ---- Image depth constants ----
SCREEN_MAX_X = 640
SCREEN_MAX_Y = 480

FLOOR_THRESHOLD = 0.25
SPACE_PERCENT_NEEDED = 30
SPACE_METRES_INFRONT = 0.7
SPACE_MASK_SIZE_NEEDED = 43


def main() -> None:
    """Entry point. Initialises the robot and runs the ball-detection loop."""
    turtle = Turtlebot(rgb=True, pc=True)
    sleep(2)
    rate = Rate(10)

    while True:
        (center_x, _), _ = detect_ball(turtle=turtle)
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


def detect_ball(turtle) -> Tuple[Tuple[int, int], int]:
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

    #cv2.imshow("CONTOURS", filtered)
    #cv2.imshow("IMAGE", im)
    #cv2.setMouseCallback("IMAGE", mouse_callback, hsv)
    #cv2.waitKey(1)

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


def space_infront(turtle) -> bool:
    """
    Return whether the robot has space in front of it.

    Returns:
        bool: True when there is space in front of the robot.
    """
    pc = turtle.get_point_cloud()
    if pc is None:
        print('No point cloud')

    # mask out floor points
    mask = pc[:, :, 1] < FLOOR_THRESHOLD

    # mask point too far
    mask = np.logical_and(mask, pc[:, :, 2] < 3.5)

    mask = np.logical_and(mask, pc[:, :, 1] > -0.25)
    data = np.sort(pc[:, :, 2][mask])

    # if closest SPACE_PERCENT_NEEDED percent of depth data is further than SPACE_METRES_INFRONT meters --> return True
    if data.size > SPACE_MASK_SIZE_NEEDED:
        dist = np.percentile(data, SPACE_PERCENT_NEEDED)
        if dist > SPACE_METRES_INFRONT:
            return True

    return False


def get_depth(turtle, center_x, center_y, radius) -> Optional[float]:
    """
    Estimate the depth to a detected object using a small grid sample from the point cloud.

    Samples a 3x3 or 5x5 grid of points centred on (center_x, center_y),
    depending on the apparent radius of the detected object. Points closer
    than 0.1 m are discarded as noise. Returns the mean depth of the
    remaining valid readings.

    Args:
        turtle: The Turtlebot instance used to retrieve the point cloud.
        center_x (int): Horizontal pixel coordinate of the object centre.
        center_y (int): Vertical pixel coordinate of the object centre.
        radius (float): Detected pixel radius of the object. Values below 2
            are treated as unreliable and cause an early None return.
            Other values below 16 will be averaged over a 3x3 pattern
            and values higher than that will be averaged over a 5x5 pattern.

    Returns:
        float or None: Average depth in metres, or None if the object radius
            is too small, no point cloud is available, or all sampled points
            are closer than 0.1 m (object too close to sensor).
    """

    if radius < 2:
        return None

    pc = turtle.get_point_cloud()
    if pc is None:
        print('No point cloud')
        return None

    if radius < 16:
        radius = 1  # 3x3 grid
    else:
        radius = 2  # 5x5 grid

    depth_sum = 0.0
    val_count = 0

    for i in range(-radius, radius + 1):
        for j in range(-radius, radius + 1):
            y = center_y + i
            x = center_x + j

            if 0 <= y < SCREEN_MAX_Y and 0 <= x < SCREEN_MAX_X:
                point_data = pc[y][x][2]

                if point_data is not None:
                    point_depth = float(point_data)
                    if point_depth > 0.1:
                        depth_sum += point_depth
                        val_count += 1

    if val_count == 0:
        return None

    average_depth = depth_sum / val_count
    return average_depth


if __name__ == "__main__":
    main()