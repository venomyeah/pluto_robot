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

  ros::Rate r(100);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Time ts = ros::Time::now();
  while (ros::ok()) {

    ros::Duration d = ts - ros::Time::now();
    ts = ros::Time::now();
    pmd.read(ts, d);
    cm.update(ts, d);
    pmd.write(ts, d);

    r.sleep();
  }

  return 0;
}
