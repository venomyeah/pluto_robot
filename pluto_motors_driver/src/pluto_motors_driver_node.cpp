/*
 * Motors Driver for Pluto robot
 * Vittorio Lumare venom@venom.it
 */

#include <pluto_motors_driver.hpp>
#include "ros/ros.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pluto_motors_driver_node");    

    PlutoMotorsDriver pmd;    

    ros::spin();

    return 0;
}
