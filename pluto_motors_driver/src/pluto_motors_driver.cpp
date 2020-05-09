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

#ifdef RPI
int fg0_prev_time = millis();
int fg0_counter = 0;
int fg0_prev_counter = 0;
#endif

void PlutoMotorsDriver::setMotorsPowerCallback(
    const pluto_msgs::MotorsPower &mp) {
  auto time = ros::Time::now();

#ifdef RPI
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
#endif

  ROS_INFO_STREAM(ros::Time::now() - time);
}

#ifdef RPI
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
#endif

PlutoMotorsDriver::PlutoMotorsDriver() {

  // setup hardware interface..

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0],
                                                      &eff[0]);
  jnt_state_interface.registerHandle(state_handle_a);

  hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1],
                                                      &eff[1]);
  jnt_state_interface.registerHandle(state_handle_b);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_a(
      jnt_state_interface.getHandle("A"), &cmd[0]);
  jnt_pos_interface.registerHandle(pos_handle_a);

  hardware_interface::JointHandle pos_handle_b(
      jnt_state_interface.getHandle("B"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_b);

  registerInterface(&jnt_pos_interface);

#ifdef NOTRPI
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
  wiringPiISR(PIN_FG0, INT_EDGE_RISING, &fg0Feedback);

  // Default to zero speed
  softPwmCreate(PIN_PWM0, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM0, POWER_RANGE);
  softPwmCreate(PIN_PWM1, POWER_RANGE, POWER_RANGE);
  softPwmWrite(PIN_PWM1, POWER_RANGE);

  std::thread fg0_timer(fg0FeedbackTimer);

  fg0_timer.detach();
#endif

  // Start Subscriber
  topic_sub_ = nh_.subscribe("/motors_power", 1000,
                             &PlutoMotorsDriver::setMotorsPowerCallback, this);
}
