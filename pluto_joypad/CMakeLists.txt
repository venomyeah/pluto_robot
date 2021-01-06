cmake_minimum_required(VERSION 2.8.3)
project(pluto_joypad)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

set (SRC
 src/pluto_joypad_node.cpp
 src/pluto_joypad.cpp
)

add_executable(${PROJECT_NAME}_node ${SRC})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${LIBRARIES}
)
