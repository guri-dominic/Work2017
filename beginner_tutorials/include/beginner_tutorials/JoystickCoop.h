#ifdef JOYSTICKCOOP_H
#define JOYSTICKCOOP_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <vector>

// Class: JoystickCoop
// Inputs: 	2 joysticks (joy nodes), 
// 		disturbance trigger (boolean)
// 		Operating function (function pointer)	


class JoystickCoop
{
    private:
        // Parameters
		ros::NodeHandle nh_c;
		ros::Subscriber joySB;
	    ros::Publisher quadPB;

        // joystick data variable
        sensor_msgs::Joy dronejs;

        // JOYSTICK INDICES (Yaw, Lift, Roll, Pitch)
        // Axes Indices
        int masterAxIdx[4];
        int subjectAxIdx[4];
        // Button Indices
        int masterBtIdx[3];
        // int subjectBtIdx[] = {};
        
        // Axes Vector:
        //      - [0] : Yaw
        //      - [1] : Lift
        //      - [2] : Roll
        //      - [3] : Pitch
        std::vector<float> ax;	// axis values from joystick

        // Button Vector:
        //      - Takeoff
        //      - Land
        //      - Emergency
        std::vector<int> bt;	
        ros::Time msg_time;

	public:
        // Constructor
		JoystickCoop();

        // Callback functions
		void joyCB(const sensor_msgs::Joy::ConstPtr &msg);

        // Set Functions
        void setMasterAxes();
        void setMasterButtons();
        void setSubjectAxes();
        void setInputChannels();
};

#endif 

