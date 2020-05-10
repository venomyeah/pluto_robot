/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include <pluto_msgs/MotorsPower.h>
#include <pluto_msgs/SetMotorsPower.h>

// ros
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// generic
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// RPI
//#define RPI
#ifdef RPI
#include <softPwm.h>
#include <wiringPi.h>
#define PWM0 26
#define PWM1 23
#define PIN_PWM0 26 // HW 32
#define PIN_DIR0 27 // HW 36
#define PIN_FG0 28  // HW 38
#define PIN_PWM1 22 // HW 31
#define PIN_DIR1 23 // HW 33
#define PIN_FG1 24  // HW 35
#define PULSES_PER_CYCLE 45 * 6
#define POWER_RANGE 100
#endif

class PlutoMotorsDriver : public hardware_interface::RobotHW {

public:
  PlutoMotorsDriver();

  void setMotorsPowerCallback(const pluto_msgs::MotorsPower &mp);

private:
  ros::NodeHandle nh_;
  ros::Subscriber topic_sub_;

  // hw interface
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

public:
  void read(const ros::Time &time, const ros::Duration &period);
  void write(const ros::Time &time, const ros::Duration &period);
};
