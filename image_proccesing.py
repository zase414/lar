from __future__ import print_function

from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from scipy.io import savemat
from typing import Optional

import numpy as np

SCREEN_MAX_X = 640
SCREEN_MAX_Y = 480

FLOOR_THRESHOLD = 0.25
SPACE_PERCENT_NEEDED = 30
SPACE_METRES_INFRONT = 0.6
SPACE_MASK_SIZE_NEEDED = 50

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
