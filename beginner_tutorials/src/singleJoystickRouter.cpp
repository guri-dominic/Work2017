
/*
 * Dominic Guri
 * Joy Node in C++ 
 *      > Testing a simple C++ class for simulating a joystick pablish
 *        by routing specific axes and buttons from an input joystick
 *      > The input joystick is under the namespace '/monitor'
 *      > The publisher name is '/joy_node'
 */

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
//#include <vector>

// ----------------------------------------------------------------------
class JoystickCoop
{
    private:
        // NodeHandle, Subscribers & Publishers
        ros::NodeHandle nh;
		ros::Subscriber mJoySub;
	    ros::Publisher joyPub;
               
	    // joystick data variable
        sensor_msgs::Joy dronejs;

	public:
        // CONSTRUCTOR:
		JoystickCoop();

        // CALLBACK FUNCTIONS:
        // Two joystick callback functions
		void monitorJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
};
// ----------------------------------------------------------------------
JoystickCoop::JoystickCoop()
{
    // Set dronjs (publish message)
    dronejs.axes.resize(4);
    dronejs.buttons.resize(3);

    //ros::Subscriber sJoySub;
    mJoySub = nh.subscribe(
            "/monitor/joy", 
            2, 
            &JoystickCoop::monitorJoysticksCallback,
            this);
    //ros::Publisher joyPub;
	joyPub = nh.advertise<sensor_msgs::Joy>(
            "joy_node",
            1);
}
// -------------------------------------------------------------------
void JoystickCoop::monitorJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg)
{
    // Data Indices
    // 0:yaw, 1:lift, 2:roll, 3:pitch
    int axIdx[] = {0, 1, 2, 3};
    // 12: takeoff, 14:land, 15:emergency
    int btIdx[] = {12, 14, 15};

    // Collect axes and button values of interest
    for(int i = 0; i < 4; i++)
    {
        this->dronejs.axes[i] = msg->axes[axIdx[i]];
        if(i < 3)
        {
            this->dronejs.buttons[i] = msg->buttons[btIdx[i]];
        }
    }
    // Update time
    this->dronejs.header.stamp = msg->header.stamp;

	// DEBUG: log to console
    ROS_INFO("axes: %f, %f, %f, %f",
            this->dronejs.axes[0],
            this->dronejs.axes[1],
            this->dronejs.axes[2],
            this->dronejs.axes[3]);
    ROS_INFO("buttons: %d, %d, %d",
            this->dronejs.buttons[0],
            this->dronejs.buttons[1],
            this->dronejs.buttons[2]);
    // PUBLISH:
    joyPub.publish(this->dronejs);
}

/////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // ROS Initilization & Running
	ros::init(argc, argv, "joystickCombination");
    // JoystickCoop joyCollaboration;
    JoystickCoop joyCollaboration;
    
    ros::spin();
}

/////////////////////////////////////////////////////////////////////////
