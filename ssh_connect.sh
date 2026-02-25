ssh -X -t cvut-username@192.168.65.32 "mount /local 2>/dev/null; singularity shell /local/robolab_noetic_amd64.simg"

# next:
# source /opt/ros/lar/setup.bash

# 1st console: robot backend:
# roslaunch robolab_turtlebot bringup_realsense_D435.launch

# 2nd console: run python:
# python3 /opt/ros/lar/share/robolab_turtlebot/scripts/bumper_test.py

