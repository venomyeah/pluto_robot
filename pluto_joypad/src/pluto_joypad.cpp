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

  if (0) {
    // Direct Diff Drive
    auto left = joy_data.axes[1];
    auto right = joy_data.axes[4];

    // noise removal
    left = fabs(left) < 0.1 ? 0.0 : left;
    right = fabs(right) < 0.1 ? 0.0 : right;

    // conversion
    std_msgs::Float64 left_cmd;
    left_cmd.data = -left * 3;
    std_msgs::Float64 right_cmd;
    right_cmd.data = right * 3;

    left_wheel_pub_.publish(left_cmd);
    right_wheel_pub_.publish(right_cmd);
  }

  if (1) {
    // twist
    auto x = joy_data.axes[1]; // linear
    auto z = joy_data.axes[3]; // angular

    // noise removal
    x = fabs(x) < 0.1 ? 0.0 : x;
    z = fabs(z) < 0.1 ? 0.0 : z;

    // conversion
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = z * 0.3;
    cmd_vel.angular.z = x * 1;

    // pub
    twist_pub_.publish(cmd_vel);
  }
}

PlutoJoypad::PlutoJoypad() {

  ros::NodeHandle nh;

  joy_sub_ = nh.subscribe("/joy", 10, &PlutoJoypad::joyCallback, this);

  left_wheel_pub_ = nh.advertise<std_msgs::Float64>(
      "/pluto_motors/left_wheel_velocity_controller/command", 1000);
  right_wheel_pub_ = nh.advertise<std_msgs::Float64>(
      "/pluto_motors/right_wheel_velocity_controller/command", 1000);
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(
      "/pluto_motors/diff_drive_controller/cmd_vel", 1000);
}
