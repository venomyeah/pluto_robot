#!/bin/zsh
# Add this line to to your .zsh config:
#  alias pluto='. ~/ros/pluto_ws/src/pluto_robot/scripts/setup_pluto_workspace.zsh'

source ~/ros/pluto_ws/devel/setup.zsh
cd ~/ros/pluto_ws/src/pluto_robot
wstool
