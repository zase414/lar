from __future__ import print_function
from callbacks import callback_bumper_stop, callback_button0_resume
from enum import IntEnum
from robolab_turtlebot import Turtlebot, Rate, get_time, sleep
from datetime import datetime
from scipy.io import savemat

from typing import Tuple, List

import numpy as np
import cv2
import time

Vec2Int = Tuple[int, int]

class Ferenc:
	def __init__(self):
		self.turtle = Turtlebot(rgb=True)
		self.stop = False

	def main(self):
		turtle = self.turtle
		sleep(2)

		turtle.register_bumper_event_cb(lambda msge: callback_bumper_stop(self, msge))
		turtle.register_button_event_cb(lambda msge: callback_button0_resume(self, msge))
		rate = Rate(10)

		Kp = 0.01
		Ki = 0.0001
		Kd = 0.001

		integral = 0
		prev_error = 0
		prev_time = get_time()

		TARGET_X = 640 // 2

		gate_detected = False

		while not self.turtle.is_shutting_down():
			rectangles = self.detect_rectangles(turtle=turtle)

			current_time = get_time()
			dt = current_time - prev_time

			if rectangles and len(rectangles) == 3 and dt > 0 and not self.stop:
				gate_detected = True

				left, right, center = rectangles

				center_x, center_y = center
				left_x, left_y = left
				right_x, right_y = right

				error = TARGET_X - center_x

				proportional = Kp * error
				integral += error * dt
				derivative = (error - prev_error) / dt

				pid_output = proportional + (Ki * integral) + (Kd * derivative)

				turtle.cmd_velocity(linear=0.4, angular=pid_output)

				prev_error = error
				prev_time = current_time
			else:
				if not gate_detected:
					turtle.cmd_velocity(linear=0.0, angular=0.5)
				else:
					turtle.cmd_velocity(linear=0.1, angular=0.0)
				integral = 0
				prev_time = current_time

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

			rate.sleep()

		cv2.destroyAllWindows()

	def detect_rectangles(self, turtle) -> List[Vec2Int]:
		HUE_LOW   = 110
		HUE_HIGH  = 140
		SAT_MIN   = 50
		VALUE_MIN = 40

		im = turtle.get_rgb_image()
		
		if im is None:
			return None
				
		cv2.imshow("IMAGE", im)

		hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

		lower_bound = np.array([HUE_LOW, SAT_MIN, VALUE_MIN])
		upper_bound = np.array([HUE_HIGH, 255, 255])

		kernel = np.ones((5, 5), np.uint8)
		mask = cv2.inRange(hsv, lower_bound, upper_bound)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

		filtered = cv2.bitwise_and(im, im, mask=mask)

		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		vertical_rects = []
		for c in contours:
			area = cv2.contourArea(c)
			if area > 300:
				x, y, w, h = cv2.boundingRect(c)
				if w > 0:
					aspect_ratio = float(h) / w
					if aspect_ratio > 1.5:
						vertical_rects.append((x, y, w, h))

		# left/right
		vertical_rects = sorted(vertical_rects, key=lambda r: r[0])

		if len(vertical_rects) < 2:
			return None

		found_pair = vertical_rects[:2]

		ret: List[Vec2Int] = []
		
		for (x, y, w, h) in found_pair:
			cv2.rectangle(filtered, (x, y), (x + w, y + h), (0, 255, 0), 2)
			center_x = x + w // 2
			center_y = y + h // 2
			cv2.circle(filtered, (center_x, center_y), 2, (255, 0, 0), 3)
			
			ret.append((center_x, center_y))
		
		# calc center
		avg_x = (ret[0][0] + ret[1][0]) // 2
		avg_y = (ret[0][1] + ret[1][1]) // 2
		ret.append((avg_x, avg_y))

		cv2.circle(filtered, (avg_x, avg_y), 2, (0, 255, 0), 3)
		
		cv2.imshow("contours", filtered)
		cv2.waitKey(1)

		return ret

if __name__ == "__main__":
	ferenc = Ferenc()
	ferenc.main()