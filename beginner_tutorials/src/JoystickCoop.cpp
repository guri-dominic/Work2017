#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <vector>

#include "beginner_tutorials/JoystickCoop.h"
//#include <beginner_tutorials/JoystickCoop.h>

// -------------------------------------------------------------------
JoystickCoop::JoystickCoop()
{
	// constructor
	int masterAxIdx[] = {1, 0, 2, 3};
	int masterBtIdx[] = {12, 14, 15};
	int subjectAxIdx[] = {0, 1, 2, 3};
	// int subjectBtIdx[] = {};
	
	// select joystick axes and buttons
	// double ax[4]; // axis values from joystick
	// double bt[3]; // collect 2 buttons (takeoff, land, emergency)
}

// -------------------------------------------------------------------
void JoystickCoop::joyCB(const sensor_msgs::Joy::ConstPtr &msg)
{
	// store joystick values of interest
	for(int i = 0; i < 4; i++)
	{
		ax[i] = msg->axes[masterAxIdx[i]];
		if(i < 3)
		{
			bt[i] = msg->buttons[masterBtIdx[i]];
		}
	}

	// DEBUG: log to console
	ROS_INFO("axes: %f, %f, %f, %f", ax[0],ax[1],ax[2],ax[3]);
}

// -------------------------------------------------------------------
void JoystickCoop::setMasterAxes()
{
    //
}
// -------------------------------------------------------------------
void JoystickCoop::setMasterButtons()
{
    //
}

// -------------------------------------------------------------------
void JoystickCoop::setSubjectAxes()
{
    //
}

// -------------------------------------------------------------------
void JoystickCoop::setInputChannels()
{
    //
}
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------
