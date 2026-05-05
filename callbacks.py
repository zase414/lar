from enums import Bumper, Button, State

def callback_button0_resume(ferenc, msg):
    """Button event"""

    if (msg.state == State.PRESSED) and (msg.button == Button.BUTTON0):
        ferenc.stop = False
        ferenc.start = True

    # usable for prints
    button_state = msg.state
    button = msg.button

def callback_bumper_stop(ferenc, msg):
    """Bumber callback."""
    
    if msg.state == State.PRESSED:
        ferenc.turtle.cmd_velocity(0, 0)
        ferenc.stop = True

    # usable for prints
    bumper_state = msg.state
    bumper = msg.bumper
