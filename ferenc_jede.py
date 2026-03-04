# Ferenc

from __future__ import print_function

from robolab_turtlebot import Turtlebot, Rate, get_time

class BumperState(Enum):
    PRESSED = 1
    RELEASAED = 0

class Bumper(Enum):
    LEFT = 0
    RIGHT = 1
    CENTER = 2

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot()
        self.stop = False

    def _bumper_cb(self, msg):
        """Bumber callback."""
        
        if (msg.state == BumperState.PRESSED):
            self.turtle.cmd_velocity(0, 0)
            self.stop = True

        bumperstate = msg.state
        bumper = msg.bumper

        print('{} bumper {}'.format(bumper, bumperstate))

    def main(self):
        turtle = self.turtle
        turtle.register_bumper_event_cb(self._bumper_cb)

        t = get_time()

        rate = Rate(10)
        while (not turtle.is_shutting_down() or self.stop) and (get_time() - t < 0):
            turtle.cmd_velocity(0.1)
            rate.sleep()

        while (not turtle.is_shutting_down() or self.stop) and (get_time() - t < (2*3.14)/0.6):
            turtle.cmd_velocity(0.2, 0.4)
            rate.sleep()

if __name__ == "__main__":
    ferenc = Ferenc()
    ferenc.main()