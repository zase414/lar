#!/bin/bash
ssh -X -t vosahond@192.168.65.32

### next:
# mount /local
# singularity shell /local/robolab_noetic_amd64.simg
# source /opt/ros/lar/setup.bash

### 1st console: robot backend:
# roslaunch robolab_turtlebot bringup_realsense_D435.launch

### 2nd console: run python:
# python3 /opt/ros/lar/share/robolab_turtlebot/scripts/bumper_test.py

