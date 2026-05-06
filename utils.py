from typing import List
from math import pi, cos, sqrt, sin, atan2


def _normalize_angle(angle) -> float:
    """
    Wrap an angle into the range (-pi, pi].

    Args:
        angle (float): Angle in radians.

    Returns:
        float: Equivalent angle in (-pi, pi].
    """
    return (angle + pi) % (2 * pi) - pi

def calculate_points(dist, coords, ball_radius) -> List[List[float]]:
    """
    Compute the waypoints of a hexagon encircling the ball (counterclockwise).
    
    Args:
        dist (float): Stand-off distance from the ball centre to the robot.
        coords (tuple): Current odometry pose (x, y, theta).
    
    Returns:
        list[list[float]]: A list of [x, y, angle] waypoints.
    """
    points = []

    # make all the points of a hexagon
    x = 0
    y = 0

    for i in range(5):
        angle = i * (pi / 3)
        if i == 0:
            # pythagoras theorem
            y = -(cos(pi / 6) * (dist + ball_radius))
            x = sqrt(((dist + ball_radius) ** 2) - (y ** 2))

        if i == 1:
            x += dist + ball_radius

        if i == 2:
            y = coords[1]
            x += sin(pi / 6) * (dist + ball_radius)

        if i == 3:
            y = cos(pi / 6) * (dist + ball_radius)
            x -= sin(pi / 6) * (dist + ball_radius)

        if i == 4:
            x -= dist + ball_radius
            angle = -2 * (pi / 3)

        points.append([x, y, angle])

    # point of return
    # starting point, but ferenc is looking the other way
    points.append([coords[0], coords[1], coords[2] + pi])

    return points