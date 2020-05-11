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
int fg0_prev_time = millis();
int fg0_counter = 0;
int fg0_prev_counter = 0;
int fg1_prev_time = millis();
int fg1_counter = 0;
int fg1_prev_counter = 0;
double left_wheel_cycles_per_sec_;
double right_wheel_cycles_per_sec_;
#endif

#ifdef RPI
void fg0Feedback() { fg0_counter++; }

void fg1Feedback() { fg1_counter++; }

void fg0FeedbackTimer() {
  const int feedback_rate = 10;
  ros::Rate r(feedback_rate);
  while (ros::ok()) {
    int fg0_counter_frozen = fg0_counter;
    left_wheel_cycles_per_sec_ =
        feedback_rate *
        static_cast<double>(fg0_counter_frozen - fg0_prev_counter) /
        static_cast<double>(PULSES_PER_CYCLE);

    fg0_prev_counter = fg0_counter_frozen;
    r.sleep();
  }
}

void fg1FeedbackTimer() {
  const int feedback_rate = 10;
  ros::Rate r(feedback_rate);
  while (ros::ok()) {
    int fg1_counter_frozen = fg1_counter;
    right_wheel_cycles_per_sec_ =
        feedback_rate *
        static_cast<double>(fg1_counter_frozen - fg1_prev_counter) /
        static_cast<double>(PULSES_PER_CYCLE);

    fg1_prev_counter = fg1_counter_frozen;
    r.sleep();
  }
}
#endif

PlutoMotorsDriver::PlutoMotorsDriver() {

  // setup hardware interface..

  // connect and register the joint state interface
  hardware_interface::JointStateHandle l_wheel_state_handle(
      "left_wheel", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(l_wheel_state_handle);

  hardware_interface::JointStateHandle r_wheel_state_handle(
      "right_wheel", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(r_wheel_state_handle);

  registerInterface(&jnt_state_interface);

  //  // connect and register the joint velocity interfaces
  //  hardware_interface::JointHandle l_wheel_vel_handle(
  //      jnt_state_interface.getHandle("left_wheel"), &vel_cmd[0]);
  //  jnt_vel_interface.registerHandle(l_wheel_vel_handle);

  //  hardware_interface::JointHandle r_wheel_vel_handle(
  //      jnt_state_interface.getHandle("right_wheel"), &vel_cmd[1]);
  //  jnt_vel_interface.registerHandle(r_wheel_vel_handle);

  //  registerInterface(&jnt_vel_interface);

  // connect and register the joint effort interfaces
  hardware_interface::JointHandle l_wheel_eff_handle(
      jnt_state_interface.getHandle("left_wheel"), &eff_cmd[0]);
  jnt_eff_interface.registerHandle(l_wheel_eff_handle);

  hardware_interface::JointHandle r_wheel_eff_handle(
      jnt_state_interface.getHandle("right_wheel"), &eff_cmd[1]);
  jnt_eff_interface.registerHandle(r_wheel_eff_handle);

  registerInterface(&jnt_eff_interface);

// real hardware
#ifdef RPI
  // Setup RPi4 hardware..

  if (wiringPiSetup() == -1) {
    ROS_ERROR_STREAM("Failed to setup RPI GPIO");
  }

  // Set input/output
  pinMode(PIN_DIR0, OUTPUT);
  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_FG0, INPUT);
  pinMode(PIN_FG1, INPUT);
  pinMode(PIN_PWM0, OUTPUT);
  pinMode(PIN_PWM1, OUTPUT);

  // Set IRQs
  wiringPiISR(PIN_FG0, INT_EDGE_FALLING, &fg0Feedback);
  wiringPiISR(PIN_FG1, INT_EDGE_FALLING, &fg1Feedback);

  // Default to zero speed
  softPwmCreate(PIN_PWM0, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM0, POWER_RANGE);
  softPwmCreate(PIN_PWM1, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM1, POWER_RANGE);

  std::thread fg0_timer(fg0FeedbackTimer);
  fg0_timer.detach();
  std::thread fg1_timer(fg1FeedbackTimer);
  fg1_timer.detach();

  ROS_INFO_STREAM("WiringPi setup done.");

#endif

  ROS_DEBUG_STREAM("PlutoMotorsiDriver started");
}

void PlutoMotorsDriver::read(const ros::Time &time,
                             const ros::Duration &period) {

// real hardware
#ifdef RPI
  // convert cycles per sec to angular velocity
  vel[0] = ((0 < eff_cmd[0]) - (eff_cmd[0] < 0)) * left_wheel_cycles_per_sec_ * M_PI * 2;
  vel[1] = ((0 < eff_cmd[0] - eff_cmd[0] < 0)) * right_wheel_cycles_per_sec_ * M_PI * 2;
#endif
}

void PlutoMotorsDriver::write(const ros::Time &time,
                              const ros::Duration &period) {

  pluto_msgs::MotorsPower mp;
  mp.left_motor_power = eff_cmd[0];
  mp.right_motor_power = eff_cmd[1];
  ROS_INFO_STREAM("write: eff_cmd[0]: " << eff_cmd[0]);

// real hardware
#ifdef RPI
  if (abs(mp.left_motor_power) > POWER_RANGE ||
      abs(mp.right_motor_power) > POWER_RANGE) {
    ROS_ERROR_STREAM("MotorsPower values outside range..limiting to max");
    mp.left_motor_power = ((0 < mp.left_motor_power) - (mp.left_motor_power < 0)) * POWER_RANGE;
    mp.right_motor_power = ((0 < mp.right_motor_power) - (mp.right_motor_power < 0)) * POWER_RANGE;
  }

  // Set direction
  if (mp.left_motor_power < 0) {
    digitalWrite(PIN_DIR0, HIGH);
  }
  if (mp.left_motor_power >= 0) {
    digitalWrite(PIN_DIR0, LOW);
  }
  if (mp.right_motor_power < 0) {
    digitalWrite(PIN_DIR1, LOW);
  }
  if (mp.right_motor_power >= 0) {
    digitalWrite(PIN_DIR1, HIGH);
  }

  // Set Duty Cycle
  // ROS_INFO_STREAM(abs(mp.left_motor_power));
  // ROS_INFO_STREAM(abs(mp.right_motor_power));
  ;
  softPwmWrite(PIN_PWM0, (POWER_RANGE - abs(mp.left_motor_power)));
  softPwmWrite(PIN_PWM1, (POWER_RANGE - abs(mp.right_motor_power)));
#endif
}
