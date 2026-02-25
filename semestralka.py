#!/usr/bin/env python
from __future__ import print_function

import threading
import time
from enum import Enum

import numpy as np
from robolab_turtlebot import Turtlebot


# ============================================================
#  STATE DEFINITIONS
# ============================================================

class RobotState(Enum):
    FORWARD = 1
    ROTATE = 2
    STOP = 3


# ============================================================
#  ROBOT CONTROLLER
# ============================================================

class RobotController:

    def __init__(self):
        # --- Robot hardware ---
        self.turtle = Turtlebot(pc=True)
        print("Waiting for point cloud...")
        self.turtle.wait_for_point_cloud()
        print("Point cloud ready.")

        # --- Shared state ---
        self.state = RobotState.FORWARD
        self.state_lock = threading.Lock()

        self.active = True
        self.linear_vel = 0.2
        self.angular_vel = 0.3
        self.rotate_direction = 1

        # --- Thread control ---
        self.running = True
        self.sensor_thread = threading.Thread(target=self._sensor_loop)
        self.sensor_thread.daemon = True  # auto-kill when main exits


    # ========================================================
    #  SENSOR LOOP (runs in separate thread)
    # ========================================================

    def _sensor_loop(self):
        """
        Reads point cloud and updates robot state.
        Runs independently of motion control.
        """
        while self.running and not self.turtle.is_shutting_down():

            pc = self.turtle.get_point_cloud()
            if pc is None:
                continue

            # Filter relevant region in front of robot
            mask = (pc[:, :, 1] < 0.2)            # remove high points
            mask &= (pc[:, :, 1] > -0.2)          # remove floor
            mask &= (pc[:, :, 2] < 3.0)           # ignore far objects

            distances = pc[:, :, 2][mask]

            new_state = RobotState.FORWARD

            if distances.size > 50:
                closest = np.percentile(distances, 10)

                if closest < 0.6:
                    new_state = RobotState.ROTATE

            # Update shared state safely
            with self.state_lock:
                self.state = new_state


    # ========================================================
    #  MOTION LOOP (main control loop, 10 Hz)
    # ========================================================

    def _motion_loop(self):
        """
        Sends velocity commands at fixed 10 Hz.
        This guarantees stable motion.
        """
        rate = 0.1  # 10 Hz

        while self.running and not self.turtle.is_shutting_down():

            with self.state_lock:
                current_state = self.state

            if not self.active:
                self.turtle.cmd_velocity(0, 0)

            elif current_state == RobotState.FORWARD:
                self.rotate_direction = 1
                self.turtle.cmd_velocity(linear=self.linear_vel)

            elif current_state == RobotState.ROTATE:
                self.turtle.cmd_velocity(
                    angular=self.rotate_direction * self.angular_vel
                )

            time.sleep(rate)


    # ========================================================
    #  PUBLIC INTERFACE
    # ========================================================

    def start(self):
        """Start robot system."""
        self.sensor_thread.start()
        self._motion_loop()

    def stop(self):
        """Stop robot safely."""
        self.running = False
        self.turtle.cmd_velocity(0, 0)
        self.sensor_thread.join()


# ============================================================
#  MAIN
# ============================================================

def main():
    controller = RobotController()

    try:
        controller.start()
    except KeyboardInterrupt:
        print("Shutting down...")
        controller.stop()


if __name__ == "__main__":
    main()