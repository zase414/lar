ssh -X -t cvutusername@192.168.65.32 "mount /local 2>/dev/null; singularity shell /local/robolab_noetic_amd64.simg; source /opt/ros/lar/setup.bash"

# robot backend:
# roslaunch robolab_turtlebot bringup_realsense_D435.launch
