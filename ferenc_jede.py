# Ferenc

from __future__ import print_function

from robolab_turtlebot import Turtlebot, Rate, get_time

bumper_names = ['LEFT', 'CENTER', 'RIGHT']
state_names = ['RELEASED', 'PRESSED']

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot()
        self.stop = False


    def bumper_cb(self, msg):
        """Bumber callback."""
        self.turtle.cmd_velocity(0, 0)
        self.stop = True

    def main(self):
        turtle = self.turtle
        turtle.register_bumper_event_cb(Ferenc.bumper_cb)

        t = get_time()

        rate = Rate(10)
        while (not turtle.is_shutting_down() or self.stop) and (get_time() - t < 0):
            turtle.cmd_velocity(0.1)
            rate.sleep()

        while (not turtle.is_shutting_down() or self.stop) and (get_time() - t < (2*3.14)/0.6):
            turtle.cmd_velocity(0.2, 0.4)
            rate.sleep()

if __name__ == "__main__":
    Ferenc.main()