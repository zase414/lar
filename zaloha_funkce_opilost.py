from math import pi

def normalize_angle(angle):
    """Normalizes an angle to be strictly within -pi and pi"""
    return (angle + pi) % (2 * pi) - pi

angle = normalize_angle(-1/3*pi)
print(angle)