from __future__ import print_function

from numpy.ma.core import negative

from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from scipy.io import savemat

import numpy as np

ERROR = -100

import cv2

def main():
    turtle = Turtlebot(rgb=True, pc=True)

def space_infront(turtle) -> bool:
    pc = turtle.get_point_cloud()
    if pc is None:
        print('No point cloud')

    # mask out floor points
    mask = pc[:, :, 1] < 0.25

    # mask point too far
    mask = np.logical_and(mask, pc[:, :, 2] < 3.5)

    mask = np.logical_and(mask, pc[:, :, 1] > -0.25)
    data = np.sort(pc[:, :, 2][mask])

    # if closest 30 percent of depth data is further than 0,6 meters --> return True
    if data.size > 50:
        dist = np.percentile(data, 30)
        if dist > 0.7:
            return True

    return False

def get_depth(turtle, center_x, center_y, radius) -> float:
    """Gets depth of center of detected object."""
    pc = turtle.get_point_cloud()
    max_x = 639
    max_y = 479
    depth = float
    if pc is None:
        print('No point cloud')

    # předělat aby vyprůměroval body středu okolo středu
    if radius < 10:
        close_radius = 3
        val_num = (close_radius+1)^2

    elif radius < 2:
        return ERROR

    else:
        close_radius = 3
        val_num = (close_radius + 1) ^ 2

    for i in range(-close_radius, close_radius, 1):
        for j in range(-close_radius, close_radius, 1):
            if ((center_x + i > 0) and (center_y + j > 0)
                    and (center_x + i < max_x) and (center_y + j < max_y)):
                depth += float(pc[center_x+i][center_y+j])
            else:
                depth += 0
                val_num -= 1
    depth = depth/val_num


    print(depth)
    return depth


if __name__ == "__main__":
    main()
