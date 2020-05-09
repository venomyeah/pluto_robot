/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include <pluto_motors_driver.hpp>

#include <iostream>
#include <softPwm.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <wiringPi.h>

#define PIN_PWM0 26 // HW 32
#define PIN_DIR0 27 // HW 36
#define PIN_FG0 28  // HW 38
#define PIN_PWM1 22 // HW 31
#define PIN_DIR1 23 // HW 33
#define PIN_FG1 24  // HW 35
#define PULSES_PER_CYCLE 45 * 6
#define POWER_RANGE 100

int fg0_prev_time = millis();
int fg0_counter = 0;
int fg0_prev_counter = 0;

bool PlutoMotorsDriver::setMotorsPower(
    pluto_msgs::SetMotorsPower::Request &req,
    pluto_msgs::SetMotorsPower::Response &res) {
  auto time = ros::Time::now();

  if (abs(req.left_motor_power) > POWER_RANGE ||
      abs(req.right_motor_power) > POWER_RANGE) {
    res.retval = pluto_msgs::SetMotorsPower::Response::WRONG_MOTOR_POWER_VALUE;
    return true;
  }

  // Set direction
  if (req.left_motor_power < 0) {
    digitalWrite(PIN_DIR0, HIGH);
  }
  if (req.left_motor_power >= 0) {
    digitalWrite(PIN_DIR0, LOW);
  }
  if (req.right_motor_power < 0) {
    digitalWrite(PIN_DIR1, LOW);
  }
  if (req.right_motor_power >= 0) {
    digitalWrite(PIN_DIR1, HIGH);
  }

  // Set Duty Cycle
  ROS_INFO_STREAM(abs(req.left_motor_power));
  ROS_INFO_STREAM(abs(req.right_motor_power));
  ;
  softPwmWrite(PIN_PWM0, (POWER_RANGE - abs(req.left_motor_power)));
  softPwmWrite(PIN_PWM1, (POWER_RANGE - abs(req.right_motor_power)));

  res.retval = pluto_msgs::SetMotorsPower::Response::SUCCESS;

  ROS_INFO_STREAM(ros::Time::now() - time);

  return true;
}

void PlutoMotorsDriver::setMotorsPowerCallback(
    const pluto_msgs::MotorsPower &mp) {
  auto time = ros::Time::now();
  if (abs(mp.left_motor_power) > POWER_RANGE ||
      abs(mp.right_motor_power) > POWER_RANGE) {
    ROS_ERROR_STREAM("MotorsPower values outside range!");
    return;
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
  ROS_INFO_STREAM(abs(mp.left_motor_power));
  ROS_INFO_STREAM(abs(mp.right_motor_power));
  ;
  softPwmWrite(PIN_PWM0, (POWER_RANGE - abs(mp.left_motor_power)));
  softPwmWrite(PIN_PWM1, (POWER_RANGE - abs(mp.right_motor_power)));

  ROS_INFO_STREAM(ros::Time::now() - time);
}

void fg0Feedback() {
  fg0_counter++;
  // int cur_time = millis();
  // int time_delta = cur_time - fg0_prev_time;
  // fg0_prev_time = cur_time;

  // TODO Measure time between this and previous IRQ
  // printf("%d\n", fg0_counter);
  // fflush(stdout);
}

void fg0FeedbackTimer() {

  while (1) {

    printf("%d\n", fg0_counter - fg0_prev_counter);
    fflush(stdout);

    fg0_prev_counter = fg0_counter;
    delay(1000);
  }
}

PlutoMotorsDriver::PlutoMotorsDriver() {
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
  wiringPiISR(PIN_FG0, INT_EDGE_RISING, &fg0Feedback);

  // Default to zero speed
  softPwmCreate(PIN_PWM0, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM0, POWER_RANGE);
  softPwmCreate(PIN_PWM1, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM1, POWER_RANGE);
  std::thread fg0_timer(fg0FeedbackTimer);

  fg0_timer.detach();

  // Start Service
  service_server_ = nh_.advertiseService(
      "set_motors_power", &PlutoMotorsDriver::setMotorsPower, this);

  // Start Subscriber
  topic_sub_ = nh_.subscribe("/motors_power", 1000,
                             &PlutoMotorsDriver::setMotorsPowerCallback, this);
}
