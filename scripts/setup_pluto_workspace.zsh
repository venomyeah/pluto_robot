#!/bin/zsh
# Add this line to to your .zsh config:
#  alias pluto='. ~/ros/pluto_ws/src/pluto_robot/scripts/setup_pluto_workspace.zsh'
PLUTO_WS_FOLDER=~/ros/pluto_ws/
PLUTO_SRC_FOLDER=$PLUTO_WS_FOLDER/src
PLUTO_REPO_FOLDER=$PLUTO_SRC_FOLDER/pluto_robot
PLUTO_SCRIPTS_FOLDER=$PLUTO_REPO_FOLDER/scripts
source $PLUTO_WS_FOLDER/devel/setup.zsh
#export ROS_MASTER_URI=http://192.168.1.192:11311
export ROSCONSOLE_CONFIG_FILE=$PLUTO_SCRIPTS_FOLDER/rosconsole.yaml
cd $PLUTO_REPO_FOLDER
wstool
