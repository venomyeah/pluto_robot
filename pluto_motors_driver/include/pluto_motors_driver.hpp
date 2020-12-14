/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

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
#ifdef TARGET_LINUX_ARM
#include <wiringSerial.h>
#endif

class PlutoMotorsDriver : public hardware_interface::RobotHW {

public:
  static const int LEFT_WHEEL_INDEX;
  static const int RIGHT_WHEEL_INDEX;

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

  hardware_interface::JointHandle l_wheel_vel_handle_;
  hardware_interface::JointHandle r_wheel_vel_handle_;

  // joint commands
  double vel_cmd[2];
  double prev_vel_cmd[2];
  unsigned int vel_cmd_count[2];

  // joint state
  double pos[2];
  double vel[2];
  double eff[2];

  // helpers
  static inline int sign(double val);

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
};
