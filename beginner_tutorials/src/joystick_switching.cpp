
/*
 * Dominic Guri
 * Joy Node in C++ 
 *      > Testing a simple C++ class for simulating a joystick pablish
 *        by routing specific axes and buttons from an input joystick
 *      > The input joystick is under the namespace '/monitor'
 *      > The publisher name is '/joy'
 */

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <vector>
#include <algorithm>
#include <beginner_tutorials/joystick_combination.h>

int main(int argc, char **argv)
{
    // ROS Initilization & Running
	ros::init(argc, argv, "joystickCombination");
    // JoystickCombination joyCollaboration;
    JoystickCombination joyCollaboration;
    
    ros::spin();
}

/////////////////////////////////////////////////////////////////////////
