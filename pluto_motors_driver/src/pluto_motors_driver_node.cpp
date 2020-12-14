/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include "ros/ros.h"
#include <controller_manager/controller_manager.h>
#include <pluto_motors_driver.hpp>
#include <wiringSerial.h>
#define FREQUENCY 10

int ser_fd;

int main(int argc, char **argv) {
  ros::init(argc, argv, "pluto_motors_driver_node");

  PlutoMotorsDriver pmd;

  controller_manager::ControllerManager cm(&pmd);

  ros::Rate r(FREQUENCY);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //if ((ser_fd = serialOpen("/dev/ttyACM0", 57600)) < 0)
  //{
  //  fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
  //  return -2;
  //}

  ros::Time ts = ros::Time::now();
  while (ros::ok()) {

    ros::Duration d = ts - ros::Time::now();
    ts = ros::Time::now();
    pmd.read(ts, d);
    cm.update(ts, d);
    pmd.write(ts, d);
    r.sleep();
  }
  return 0;

  sleep(5);
  for(int i = 0 ; i < 30;  i ++ ){
  
   if (i >= 0 && i < 10){
    std::cout << "R1" << std::endl;
    serialPuts(ser_fd, "%0 1#");
   }

   if (i >= 10 && i < 20){
    std::cout << "R2" << std::endl;
    serialPuts(ser_fd, "%0 2#");
   } 

   if (i >= 20 && i < 30){
    std::cout << "R1" << std::endl;
    serialPuts(ser_fd, "%0 1#");
   }
   
   usleep(10000);
  }

  //std::cout << "R0" << std::endl;
  //serialPuts(ser_fd, "%0 0#");  

  //sleep(1);
  //serialClose(ser_fd);
  return 0;
}

