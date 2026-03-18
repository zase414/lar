
from math import pi, cos, sqrt, sin, atan2

"""Calculates coordinates of hexagon to drive around the ball"""
dist = 0.4
coords = [0, 0 ,0]
points = []
ball_radius = 0.004 # 4cm radius of ball

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
        x += dist + 0.004

    if i == 2:
        y = coords[1]
        x += sin(pi / 6) * (dist + ball_radius)

    if i == 3:
        y = cos(pi / 6) * (dist + ball_radius)
        x -= sin(pi / 6) * (dist + ball_radius)

    if i == 4:
        x -= dist + 0.004
        angle = -2*(pi/3)

    points.append([x, y, angle])

# point of return
# starting point, but ferenc is looking the other way
points.append([coords[0], coords[1], coords[2] + pi])
print(points)
