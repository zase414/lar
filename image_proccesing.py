from __future__ import print_function

from numpy.ma.core import negative

from enums import depth_ERR

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
    
    if radius < 2:
        return depth_ERR   # -100

    pc = turtle.get_point_cloud()
    if pc is None:
        print('No point cloud')
        return depth_ERR

    max_x = 640
    max_y = 480
    
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
            
            if 0 <= y < max_y and 0 <= x < max_x:
                point_data = pc[x][y][2]
                
                if point_data is not None:
                    point_depth = float(point_data)
                    if point_depth > 0.1:  
                        depth_sum += point_depth
                        val_count += 1

    if val_count == 0:
        print("Objekt je příliš blízko. Žádná data.")
        return depth_ERR
        
    average_depth = depth_sum / val_count
    print(f"Objekt je daleko: {average_depth:.2f} m")
    print(pc[center_x][center_y])
    return average_depth


if __name__ == "__main__":
    main()
