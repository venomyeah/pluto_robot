/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include "ros/ros.h"
#include <pluto_msgs/MotorsPower.h>
#include <pluto_msgs/SetMotorsPower.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#define PWM0 26
#define PWM1 23

class PlutoMotorsDriver {

public:
  PlutoMotorsDriver();

  bool setMotorsPower(pluto_msgs::SetMotorsPower::Request &req,
                      pluto_msgs::SetMotorsPower::Response &res);

  void setMotorsPowerCallback(const pluto_msgs::MotorsPower &mp);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_server_;
  ros::Subscriber topic_sub_;
};
