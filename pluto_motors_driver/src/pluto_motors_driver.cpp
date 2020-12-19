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

//#include <asm/termbits.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// RPI
#define __RPI__

// GLOBALS

// real hardware
int serial_fd;

// Constants
const int PlutoMotorsDriver::LEFT_WHEEL_INDEX = 0;
const int PlutoMotorsDriver::RIGHT_WHEEL_INDEX = 1;

bool serialOpen() {

  system("stty -F /dev/ttyACM0 cs8 57600 ignbrk -brkint -icrnl -imaxbel -opost "
         "-onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke "
         "noflsh -ixon -crtscts");

  std::string deviceName = "/dev/ttyACM0";
  serial_fd = open(deviceName.c_str(), O_RDWR | O_NOCTTY /* | O_NONBLOCK */);

  if (serial_fd < 0)
    return false;

  return true;

  // NOT WORKING
  struct termios config;

  cfmakeraw(&config);
  config.c_cflag |= (CLOCAL | CREAD);
  config.c_iflag &= ~(IXOFF | IXANY);

  // set vtime, vmin, baud rate...
  config.c_cc[VMIN] = 0;  // you likely don't want to change this
  config.c_cc[VTIME] = 0; // or this

  cfsetispeed(&config, B57600);
  cfsetospeed(&config, B57600);

  // write port configuration to driver
  tcsetattr(serial_fd, TCSANOW, &config);

  return true;
}

void serialClose() { close(serial_fd); }

void serialTx(const std::string &str) {
  write(serial_fd, str.c_str(), str.size());
  // std::stringstream ss;
  // ss << "echo \"" << str << "\" > /dev/ttyACM0";
  // system(ss.str().c_str());
}

std::string serialRx() {}

PlutoMotorsDriver::PlutoMotorsDriver() {

#ifdef __RPI__
  ROS_INFO_STREAM("Raspberry Pi Detected!");
#endif

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
  if (!serialOpen()) {
    std::cout << "Unable to open serial device";
    return;
  }

  memset(vel_cmd, 0, sizeof(vel_cmd));
  memset(prev_vel_cmd, 0, sizeof(prev_vel_cmd));
  memset(vel_cmd_count, 0, sizeof(vel_cmd_count));

  ROS_DEBUG_STREAM("PlutoMotorsiDriver started");
}

PlutoMotorsDriver::~PlutoMotorsDriver() { serialClose(); }

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
#ifdef __RPI__
  // TODO read vel from serial
  vel[0] = l_vel_set_point_;
  vel[1] = r_vel_set_point_;
#endif

#ifndef __RPI__
  vel[0] = l_vel_set_point_;
  vel[1] = r_vel_set_point_;
#endif
}

void PlutoMotorsDriver::write(const ros::Time &time,
                              const ros::Duration &period) {

  std::stringstream cmd;
  cmd << "%" << vel_cmd[LEFT_WHEEL_INDEX] << " " << vel_cmd[RIGHT_WHEEL_INDEX]
      << "#";

  std::cout << "CMD: " << cmd.str() << std::endl;

// real hardware
#ifdef __RPI__
  serialTx(cmd.str());
#endif

  prev_vel_cmd[LEFT_WHEEL_INDEX] = vel_cmd[LEFT_WHEEL_INDEX];
  prev_vel_cmd[RIGHT_WHEEL_INDEX] = vel_cmd[RIGHT_WHEEL_INDEX];
}
