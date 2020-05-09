/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include "ros/ros.h"
#include <controller_manager/controller_manager.h>
#include <pluto_motors_driver.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pluto_motors_driver_node");

  PlutoMotorsDriver pmd;

  controller_manager::ControllerManager cm(&pmd);

  ros::spin();

  return 0;
}
