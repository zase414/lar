#!/bin/bash
ssh -X -t kecekjak@192.168.65.29


mount /local
singularity shell /local/robolab_noetic_amd64.simg
source /opt/ros/lar/setup.bash

### 1st console: robot backend:
robolab_turtlebot bringup_realsense_D435.launch

### 2nd console: run python:
python3 /opt/ros/lar/share/robolab_turtlebot/scripts/bumper_test.py






#Buildeni
sphinx-build -b html docs/ docs/_build/html