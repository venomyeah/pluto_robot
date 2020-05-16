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
double g_wheel_cycles_per_sec[2];
#endif

#ifdef RPI
void fg0Feedback() { fg0_counter++; }

void fg1Feedback() { fg1_counter++; }

void fg0FeedbackTimer() {
  const int feedback_rate = 10;
  ros::Rate r(feedback_rate);
  while (ros::ok()) {
    int fg0_counter_frozen = fg0_counter;
    g_wheel_cycles_per_sec[LEFT_WHEEL_INDEX] =
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
    g_wheel_cycles_per_sec[RIGHT_WHEEL_INDEX] =
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
      "left_wheel", &pos[LEFT_WHEEL_INDEX], &vel[LEFT_WHEEL_INDEX],
      &eff[LEFT_WHEEL_INDEX]);
  jnt_state_interface.registerHandle(l_wheel_state_handle);

  hardware_interface::JointStateHandle r_wheel_state_handle(
      "right_wheel", &pos[RIGHT_WHEEL_INDEX], &vel[RIGHT_WHEEL_INDEX],
      &eff[RIGHT_WHEEL_INDEX]);
  jnt_state_interface.registerHandle(r_wheel_state_handle);

  registerInterface(&jnt_state_interface);

  // connect and register the joint effort interfaces
  l_wheel_eff_handle_ = hardware_interface::JointHandle(
      jnt_state_interface.getHandle("left_wheel"), &eff_cmd[LEFT_WHEEL_INDEX]);
  jnt_eff_interface.registerHandle(l_wheel_eff_handle_);

  r_wheel_eff_handle_ = hardware_interface::JointHandle(
      jnt_state_interface.getHandle("right_wheel"),
      &eff_cmd[RIGHT_WHEEL_INDEX]);
  jnt_eff_interface.registerHandle(r_wheel_eff_handle_);

  registerInterface(&jnt_eff_interface);

  // subscribe to topics
  l_vel_setpoint_sub_ =
      nh_.subscribe("/pluto_motors/left_wheel_velocity_controller/command", 1,
                    &PlutoMotorsDriver::leftVelSetPointCb, this);
  r_vel_setpoint_sub_ =
      nh_.subscribe("/pluto_motors/right_wheel_velocity_controller/command", 1,
                    &PlutoMotorsDriver::rightVelSetPointCb, this);

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

  memset(eff_cmd, 0, sizeof(eff_cmd));
  memset(prev_eff_cmd, 0, sizeof(prev_eff_cmd));
  memset(eff_cmd_count, 0, sizeof(eff_cmd_count));

  ROS_DEBUG_STREAM("PlutoMotorsiDriver started");
}

PlutoMotorsDriver::~PlutoMotorsDriver() {
  // Default to zero speed
  softPwmCreate(PIN_PWM0, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM0, POWER_RANGE);
  softPwmCreate(PIN_PWM1, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM1, POWER_RANGE);
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
  // convert cycles per sec to angular velocity
  for (size_t i = 0; i < 2; i++) {

    // update count
    if (sign(eff_cmd[i]) != sign(prev_eff_cmd[i])) {
      eff_cmd_count[i] = 0;
    } else {
      eff_cmd_count[i]++;
    }
    if (eff_cmd_count[i] > FEEDBACK_SIGN_MANAGEMENT_MAX_WINDOW) {
      eff_cmd_count[i] = FEEDBACK_SIGN_MANAGEMENT_MAX_WINDOW;
    }

    if (eff_cmd_count[i] == FEEDBACK_SIGN_MANAGEMENT_MAX_WINDOW) {
      vel[i] = sign(eff_cmd[i]) * g_wheel_cycles_per_sec[i] * M_PI * 2;
    }
  }

#endif

#ifndef RPI
  vel[0] = l_vel_set_point_;
  vel[1] = r_vel_set_point_;
#endif
}

void PlutoMotorsDriver::write(const ros::Time &time,
                              const ros::Duration &period) {

  pluto_msgs::MotorsPower mp;
  mp.left_motor_power = eff_cmd[LEFT_WHEEL_INDEX];
  mp.right_motor_power = eff_cmd[RIGHT_WHEEL_INDEX];

// real hardware
#ifdef RPI
  if (abs(mp.left_motor_power) > POWER_RANGE) {
    mp.left_motor_power = sign(mp.left_motor_power) * POWER_RANGE;
  }
  if (abs(mp.right_motor_power) > POWER_RANGE) {
    mp.right_motor_power = sign(mp.right_motor_power) * POWER_RANGE;
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
  // ROS_DEBUG_STREAM(abs(mp.left_motor_power));
  // ROS_DEBUG_STREAM(abs(mp.right_motor_power));
  ;
  softPwmWrite(PIN_PWM0, (POWER_RANGE - abs(mp.left_motor_power)));
  softPwmWrite(PIN_PWM1, (POWER_RANGE - abs(mp.right_motor_power)));
#endif

  prev_eff_cmd[LEFT_WHEEL_INDEX] = eff_cmd[LEFT_WHEEL_INDEX];
  prev_eff_cmd[RIGHT_WHEEL_INDEX] = eff_cmd[RIGHT_WHEEL_INDEX];
}
