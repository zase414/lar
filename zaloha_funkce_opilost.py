def go_ptp(self, point, rate, point_of_return):
    """Function that navigates from one point of a hexagon to the next"""
    turtle = self.turtle
    cur_coords = turtle.get_odometry()

    # thresholds fo accurate enough stopping in given points
    dist_thresh = 0.028
    angle_thresh = 0.014
    angle_is_close_thresh = 0.06

    # current location and distance from goal point
    x = point[0] - cur_coords[0]
    y = point[1] - cur_coords[1]
    d = sqrt(x ** 2 + y ** 2)

    # calculate angle to the next point
    angle = atan2(y, x)

    # while ferenc is not rotated at the calculated angle -> rotate
    angle_diff = self.normalize_angle(angle - cur_coords[2])
    # self.rotate_to_angle(-0.5, angle_diff, angle_thresh)
    while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
        if self.stop:
            turtle.cmd_velocity(0, 0)
            turtle.play_sound(4)

        elif abs(angle_diff) < angle_is_close_thresh:
            turtle.cmd_velocity(0, -0.1)
        else:
            turtle.cmd_velocity(0, -0.5)

        cur_coords = turtle.get_odometry()
        angle_diff = self.normalize_angle(angle - cur_coords[2])

        rate.sleep()

    # while ferenc is not located at x,y coords, drive forward:
    while (not turtle.is_shutting_down()) and (d > dist_thresh):
        if self.stop:
            turtle.cmd_velocity(0, 0)
            turtle.play_sound(4)
        else:
            self.go_forward(0.23, cur_coords[2], angle)

        cur_coords = turtle.get_odometry()
        x = point[0] - cur_coords[0]
        y = point[1] - cur_coords[1]
        d = sqrt(x ** 2 + y ** 2)  # distance from point

        rate.sleep()

    # while ferenc is not rotated at the calculated angle -> rotate
    angle_diff = self.normalize_angle(
        (point[2] + 0.02) - cur_coords[2])  # little over-rotation so it can spin only in one direction
    while (not turtle.is_shutting_down()) and (abs(angle_diff) > angle_thresh):
        if self.stop:
            turtle.cmd_velocity(0, 0)
            turtle.play_sound(4)

        elif point_of_return:
            if abs(angle_diff) < angle_is_close_thresh:
                turtle.cmd_velocity(0, -0.1)
            else:
                turtle.cmd_velocity(0, -0.6)
        else:
            if abs(angle_diff) < angle_is_close_thresh:
                turtle.cmd_velocity(0, 0.1)
            else:
                turtle.cmd_velocity(0, 0.6)

        cur_coords = turtle.get_odometry()
        angle_diff = self.normalize_angle(
            (point[2] + 0.02) - cur_coords[2])  # little over-rotation so it can spin only in one direction

        rate.sleep()

    # reset params
    turtle.cmd_velocity(0, 0)



    def rotate_to_angle(self, angle_diff):
        """Simple P regulated driving in a straight line"""
        turtle = self.turtle
        Kp = 0.5
        ang_vel = Kp * angle_diff
        turtle.cmd_velocity(0, ang_vel)