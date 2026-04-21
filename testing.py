import time
from ferenc_jede import Ferenc
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep


def main():
    ferenc = Ferenc()
    rate = Rate(10)
    while True:

        ferenc.rotate_toward_ball(rate)
        print("i see the ball")
        time.sleep(10)







if __name__ == "__main__":
    main()