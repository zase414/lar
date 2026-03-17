from __future__ import print_function

from numpy.ma.core import negative

from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from scipy.io import savemat

import numpy as np

import cv2

ERROR = -100

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
    if radius < 2:
        return ERROR

    pc = turtle.get_point_cloud()
    max_x = 639
    max_y = 479
    depth = float(0)
    if pc is None:
        print('No point cloud')

    # předělat aby vyprůměroval body středu okolo středu
    if radius < 16:   # pruměr z 9 hodnot na středu
        close_radius = 1

    else:
        close_radius = 2  # prumer z 25 hodnot

    val_num = (2 * close_radius + 1) ^ 2
    for i in range(-close_radius, close_radius, 1):
        for j in range(-close_radius, close_radius, 1):
            # ferenc zběsile filtruje data (kontroluje zda je v obraze a filtruje chybové délky)
            if pc[center_x + i][center_y + j][2] is None:
                val_num -= 1
            if ((center_x + i > 0) and (center_y + j > 0)
                and (center_x + i < max_x) and (center_y + j < max_y)
                and float(pc[center_x+i][center_y+j][2]) < 0.1):
                depth += float(pc[center_x+i][center_y+j][2])
            else:
                val_num -= 1
    depth = depth/val_num
    print("Objekt je daleko: ", depth, " m")
    return depth


if __name__ == "__main__":
    main()
