/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include <pluto_joypad.hpp>

// general
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

void PlutoJoypad::joyCallback(const sensor_msgs::Joy joy_data) {

  pluto_msgs::SetMotorsPower srv;

  auto left = joy_data.axes[1];
  auto right = joy_data.axes[4];

  left = fabs(left) < 0.1 ? 0.0 : left;
  right = fabs(right) < 0.1 ? 0.0 : right;

  // Use topic
  //pluto_msgs::MotorsPower msg;
  //msg.left_motor_power = static_cast<int>(left * 100);
  //msg.right_motor_power = static_cast<int>(right * 100);
  //motors_power_pub_.publish(msg);
  //
  //

  std_msgs::Float64 left_cmd(left * 30);
  std_msgs::Float64 right_cmd(right * 30);
  left_wheel_pub_.publish(left_cmd);
  right_wheel_cmd_.publish(right_cmd);
}

PlutoJoypad::PlutoJoypad() {

  ros::NodeHandle nh;

  joy_sub_ = nh.subscribe("/joy", 1000, &PlutoJoypad::joyCallback, this);

//  motors_power_pub_ =
//      nh.advertise<pluto_msgs::MotorsPower>("/motors_power", 1000);
//
  left_wheel_pub_= nh.advertise<std_msgs::Float64>("/pluto_motors/left_wheel_velocity_controller/command", 1000);
  right_wheel_pub_= nh.advertise<std_msgs::Float64>("/pluto_motors/right_wheel_velocity_controller/command", 1000);

}
