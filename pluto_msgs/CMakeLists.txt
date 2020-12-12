cmake_minimum_required(VERSION 2.8.3)
project(pluto_msgs)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Messages and services
#add_service_files(
#  FILES
#)
#add_message_files(
#    FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
 #INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
 #DEPENDS
)
