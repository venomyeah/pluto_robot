/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include <pluto_motors_driver.hpp>

#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

// GLOBALS

// real hardware
#ifdef RPI
int serial_fd;
#endif

// Constants
const int PlutoMotorsDriver::LEFT_WHEEL_INDEX = 0;
const int PlutoMotorsDriver::RIGHT_WHEEL_INDEX = 1;

PlutoMotorsDriver::PlutoMotorsDriver() {

  // setup hardware interface..

  // connect and register the joint state interface
  hardware_interface::JointStateHandle l_wheel_state_handle(
      "left_wheel", &pos[LEFT_WHEEL_INDEX], &vel[LEFT_WHEEL_INDEX],
      &eff[LEFT_WHEEL_INDEX]);
  jnt_state_interface.registerHandle(l_wheel_state_handle);

  hardware_interface::JointStateHandle r_wheel_state_handle(
      "right_wheel", &pos[RIGHT_WHEEL_INDEX], &vel[RIGHT_WHEEL_INDEX],
      &eff[RIGHT_WHEEL_INDEX]);
  jnt_state_interface.registerHandle(r_wheel_state_handle);

  registerInterface(&jnt_state_interface);

  // connect and register the joint effort interfaces
  l_wheel_vel_handle_ = hardware_interface::JointHandle(
      jnt_state_interface.getHandle("left_wheel"), &vel_cmd[LEFT_WHEEL_INDEX]);
  jnt_vel_interface.registerHandle(l_wheel_vel_handle_);

  r_wheel_vel_handle_ = hardware_interface::JointHandle(
      jnt_state_interface.getHandle("right_wheel"),
      &vel_cmd[RIGHT_WHEEL_INDEX]);
  jnt_vel_interface.registerHandle(r_wheel_vel_handle_);

  registerInterface(&jnt_vel_interface);

  // subscribe to topics
  l_vel_setpoint_sub_ =
      nh_.subscribe("/pluto_motors/left_wheel_velocity_controller/command", 1,
                    &PlutoMotorsDriver::leftVelSetPointCb, this);
  r_vel_setpoint_sub_ =
      nh_.subscribe("/pluto_motors/right_wheel_velocity_controller/command", 1,
                    &PlutoMotorsDriver::rightVelSetPointCb, this);

// real hardware
#ifdef RPI
  if ((serial_fd = serialOpen("/dev/ttyACM0", 57600)) < 0) {
    fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    return 1;
  }
#endif

  memset(vel_cmd, 0, sizeof(vel_cmd));
  memset(prev_vel_cmd, 0, sizeof(prev_vel_cmd));
  memset(vel_cmd_count, 0, sizeof(vel_cmd_count));

  ROS_DEBUG_STREAM("PlutoMotorsiDriver started");
}

PlutoMotorsDriver::~PlutoMotorsDriver() {
#ifdef RPI
  // Default to zero speed
  // TODO
#endif
}

// setpoints
void PlutoMotorsDriver::leftVelSetPointCb(const std_msgs::Float64 &set_point) {
  l_vel_set_point_ = set_point.data;
}

void PlutoMotorsDriver::rightVelSetPointCb(const std_msgs::Float64 &set_point) {
  r_vel_set_point_ = set_point.data;
}

// helpers
int PlutoMotorsDriver::sign(double val) { return (0 <= val) - (val < 0); }

void PlutoMotorsDriver::read(const ros::Time &time,
                             const ros::Duration &period) {

// real hardware
#ifdef RPI
  // TODO read vel from serial
#endif

#ifndef RPI
  vel[0] = l_vel_set_point_;
  vel[1] = r_vel_set_point_;
#endif
}

void PlutoMotorsDriver::write(const ros::Time &time,
                              const ros::Duration &period) {

  std::stringstream cmd;
  cmd << "%VEL L" << vel_cmd[LEFT_WHEEL_INDEX] << " "
      << "R" << vel_cmd[RIGHT_WHEEL_INDEX] << "$#";

  std::cout << "SERIAL VEL " << vel_cmd[LEFT_WHEEL_INDEX] << " "
            << vel_cmd[RIGHT_WHEEL_INDEX] << " CMD: " << cmd.str() << std::endl;

// real hardware
#ifdef RPI
  serialPuts(serial_fd, cmd.str().c_str());
#endif

  prev_vel_cmd[LEFT_WHEEL_INDEX] = vel_cmd[LEFT_WHEEL_INDEX];
  prev_vel_cmd[RIGHT_WHEEL_INDEX] = vel_cmd[RIGHT_WHEEL_INDEX];
}
