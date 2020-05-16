/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include "ros/ros.h"
#include <pluto_msgs/MotorsPower.h>
#include <pluto_msgs/SetMotorsPower.h>

// ros
#include <sensor_msgs/Joy.h>

// messages
#include <std_msgs/Float64.h>

// general
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

class PlutoJoypad {

public:
  PlutoJoypad();

private:
  ros::Subscriber joy_sub_;
  ros::Publisher motors_power_pub_;
  ros::Publisher left_wheel_pub_;
  ros::Publisher right_wheel_pub_;

  void joyCallback(const sensor_msgs::Joy joy_data);
};
