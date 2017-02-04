
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
        // Joysticks' axes and buttons indices
        // Axes Indices (0:Yaw, 1:Lift, 2:Roll, 3:Pitch)
        std::vector<int> mAxIdx;
        std::vector<int> sAxIdx;
        // Button Indices (0:Takeoff, 1:Land, 2:Emergency)
        std::vector<int> mBtIdx;
        // std::vector<int> sBtIdx;

        // NodeHandle, Subscribers & Publishers
        ros::NodeHandle nh;
		ros::Subscriber mJoySub;
		ros::Subscriber sJoySub;
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
		void subjectJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
};
// ----------------------------------------------------------------------
JoystickCoop::JoystickCoop()
{
    // Set dronjs (publish message)
    dronejs.axes.resize(4);
    dronejs.buttons.resize(3);

    // Set indices
    // Axes
    mAxIdx.push_back(0);
    mAxIdx.push_back(1);
    mAxIdx.push_back(2);
    sAxIdx.push_back(3);
    // Buttons
    mBtIdx.push_back(12);
    mBtIdx.push_back(14);
    mBtIdx.push_back(15);

    //ros::Subscriber mJoySub;
    mJoySub = nh.subscribe(
            "/monitor/joy", 
            2, 
            &JoystickCoop::monitorJoysticksCallback,
            this);

    //ros::Subscriber sJoySub;
    sJoySub = nh.subscribe(
            "/subject/joy", 
            2, 
            &JoystickCoop::subjectJoysticksCallback,
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
    // Collect axes and button values of interest
    // 0:yaw, 1:lift, 2:roll, 3:pitch
    // 12: takeoff, 14:land, 15:emergency
    // Populate monitor axes and buttons
    for(int i = 0; i < mAxIdx.size(); i++)
    {
        this->dronejs.axes[mAxIdx[i]] = msg->axes[mAxIdx[i]];
    }
    for(int i = 0; i < mBtIdx.size(); i++)
    {
        this->dronejs.buttons[i] = msg->buttons[mBtIdx[i]];
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
// ---------------------------
void JoystickCoop::subjectJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg)
{
    // Collect axes and button values of interest
    // 0:yaw, 1:lift, 2:roll, 3:pitch
    // 12: takeoff, 14:land, 15:emergency
    // Populate subject axes
    for(int i = 0; i < sAxIdx.size(); i++)
    {
        // reverse joystick axes (to match master sign)
        this->dronejs.axes[sAxIdx[i]] = -1.0 * msg->axes[sAxIdx[i]];
    }

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
