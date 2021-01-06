#!/bin/zsh
# Instals the pluto workspace with all deps.

# Create workspace
cd ~/ros
mkdir pluto_ws
cd pluto_ws
wstool init src

# Install Google Cartographer and its deps
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build stow
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Only on Ubuntu 16 / ROS Kinetic:
src/cartographer/scripts/install_proto3.sh

src/cartographer/scripts/install_abseil.sh
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp

cd abseil-cpp/build
cmake ..
ninja
sudo ninja install

# Install pluto_robot
cd ~/ros/pluto_ws/src
git clone git@github.com:venomyeah/pluto_robot.git
wstool merge src/pluto_robot/rosinstall/rosinstall
wstool update

# Build
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin build
