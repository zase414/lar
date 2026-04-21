import time
from ferenc_jede import Ferenc
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep


def main():
    ferenc = Ferenc()
    while True:
        ferenc.rotate_toward_ball()
        print("i see the ball")
        time.sleep(10)







if __name__ == "__main__":
    main()