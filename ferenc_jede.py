# Ferenc

from __future__ import print_function

from robolab_turtlebot import Turtlebot, Rate, get_time

bumper_names = ['LEFT', 'CENTER', 'RIGHT']
state_names = ['RELEASED', 'PRESSED']


def bumper_cb(msg):
    """Bumber callback."""
    # msg.bumper stores the id of bumper 0:LEFT, 1:CENTER, 2:RIGHT
    bumper = bumper_names[msg.bumper]

    # msg.state stores the event 0:RELEASED, 1:PRESSED
    state = state_names[msg.state]

    # Print the event
    print('{} bumper {}'.format(bumper, state))

def main():
    turtle = Turtlebot()
    turtle.register_bumper_event_cb(bumper_cb)

    t = get_time()

    rate = Rate(10)
    while (not turtle.is_shutting_down()) and (get_time() - t < 0):
        turtle.cmd_velocity(0.1)
        rate.sleep()

    while (not turtle.is_shutting_down()) and (get_time() - t < (2*3.14)/0.6):
        turtle.cmd_velocity(-0.005, 0.6)
        rate.sleep()

if __name__ == "__main__":
    main()