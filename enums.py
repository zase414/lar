from enum import IntEnum

class Button(IntEnum):
    BUTTON0 = 0
    BUTTON1 = 1
    BUTTON2 = 2

class State(IntEnum):
    PRESSED = 1
    RELEASED = 0

class Bumper(IntEnum):
    LEFT = 0
    CENTER = 1
    RIGHT = 2

depth_ERR = -100