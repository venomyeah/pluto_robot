#!/bin/zsh
# Add this line to to your .zsh config:
#  alias pluto='. ~/ros/pluto_ws/src/pluto_robot/scripts/setup_pluto_workspace.zsh'

source ~/ros/pluto_ws/devel/setup.zsh
#export ROS_MASTER_URI=http://192.168.1.192:11311
cd ~/ros/pluto_ws/src/pluto_robot
wstool
