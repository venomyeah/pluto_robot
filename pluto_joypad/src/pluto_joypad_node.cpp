/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include "ros/ros.h"
#include <pluto_joypad.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pluto_joypad_node");

  ros::NodeHandle nh;

  PlutoJoypad pj;

  ros::spin();

  return 0;
}
