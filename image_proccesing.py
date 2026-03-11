from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from scipy.io import savemat

import numpy as np

import cv2

def main():
    turtle = Turtlebot(rgb=True, pc=True)

def space_infront(turtle) -> bool:
    pc = turtle.get_point_cloud()
    if pc is None:
        print('No point cloud')

    # mask out floor points
    mask = pc[:, :, 1] < 0.2

    # mask point too far
    mask = np.logical_and(mask, pc[:, :, 2] < 3.0)

    mask = np.logical_and(mask, pc[:, :, 1] > -0.2)
    data = np.sort(pc[:, :, 2][mask])

    # if closest 9 percent of depth data is further than 0,6 meters --> return True
    if data.size > 50:
        dist = np.percentile(data, 12)
        if dist > 0.5:
            return True

    return False


if __name__ == "__main__":
    main()
