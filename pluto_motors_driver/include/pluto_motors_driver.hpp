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

// msgs
#include <std_msgs/Float64.h>

// generic
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// RPI
#define RPI //(*** ONLY ON RASPBERRY ***)
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
#define PULSES_PER_CYCLE 45 * 6 / 2.4
#define POWER_RANGE 100
#define FREQUENCY 10
#endif

class PlutoMotorsDriver : public hardware_interface::RobotHW {

public:
  PlutoMotorsDriver();
  ~PlutoMotorsDriver();


private:
  ros::NodeHandle nh_;

  // hw interface
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;

  hardware_interface::JointHandle l_wheel_eff_handle_;
  hardware_interface::JointHandle r_wheel_eff_handle_;

  // joint commands
  // double vel_cmd[2];
  double eff_cmd[2];
  double prev_eff_cmd[2];
  unsigned int eff_cmd_count[2];

  // joint state
  double pos[2];
  double vel[2];
  double eff[2];

  // helpers
  static int sign(double val);

  // setpoints
  ros::Subscriber l_vel_setpoint_sub_;
  ros::Subscriber r_vel_setpoint_sub_;
  double l_vel_set_point_;
  double r_vel_set_point_;
  void leftVelSetPointCb(const std_msgs::Float64 &set_point);
  void rightVelSetPointCb(const std_msgs::Float64 &set_point);

public:
  void read(const ros::Time &time, const ros::Duration &period);
  void write(const ros::Time &time, const ros::Duration &period);

  // RPi Hardware
private:
  //  double left_wheel_cycles_per_sec_;
  //  double right_wheel_cycles_per_sec_;
};
