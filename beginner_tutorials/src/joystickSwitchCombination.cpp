
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
		ros::Subscriber sJoySub;
		ros::Subscriber controlSwitchSub;
	    ros::Publisher joyPub;
               
        // Axes Indices (0:Yaw, 1:Lift, 2:Roll, 3:Pitch)
        std::vector<int> mAxIdx;
        std::vector<int> sAxIdx;
        // Button Indices (0:Takeoff, 1:Land, 2:Emergency)
        std::vector<int> mBtIdx;
        // std::vector<int> sBtIdx;

	    // joystick data variable
        sensor_msgs::Joy dronejs;

        // functions
        // TODO: change from (void) input to (ros::Time &)
        // publishCombination(ros::Time &t)
        void publishCombination(void);

	public:
        // CONSTRUCTOR:
		JoystickCoop();

        // CALLBACK FUNCTIONS:
        // Two joystick callback functions
		void monitorJoystickCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
		void subjectJoystickCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
		void disturbanceCallback(
                const std_msgs::Bool::ConstPtr &msg);
};
// ----------------------------------------------------------------------
JoystickCoop::JoystickCoop()
{
    // Set dronjs (publish message)
    this->dronejs.axes.resize(4);
    this->dronejs.buttons.resize(3);

    // Set indices
    // Axes
    this->mAxIdx.push_back(0);
    this->mAxIdx.push_back(1);
    this->mAxIdx.push_back(2);
    this->sAxIdx.push_back(3);
    // Buttons
    this->mBtIdx.push_back(12);
    this->mBtIdx.push_back(14);
    this->mBtIdx.push_back(15);

    //ros::Subscriber mJoySub;
    mJoySub = nh.subscribe(
            "/monitor/joy", 
            2, 
            &JoystickCoop::monitorJoystickCallback,
            this);

    //ros::Subscriber sJoySub;
    sJoySub = nh.subscribe(
            "/subject/joy", 
            2, 
            &JoystickCoop::subjectJoystickCallback,
            this);

    //ros::Subscriber sJoySub;
    sJoySub = nh.subscribe(
            "/disturbance", 
            2, 
            &JoystickCoop::disturbanceCallback,
            this);

    //ros::Publisher joyPub;
	joyPub = nh.advertise<sensor_msgs::Joy>(
            "joy_node",
            1);
}
// -------------------------------------------------------------------
void JoystickCoop::disturbanceCallback(
                const std_msgs::Bool::ConstPtr &msg)
{
	// DEBUG: log to console
    ROS_INFO("Disturbance: %s",
            msg->data ? "true" : "false"); // print True/False

    // TODO: Apply control switching
    // - Use function pointer to an external algorithm that 
    //   applies the TEST-ALGORITHM
    // - FORM: func(int &monitorAxValue, int &alternativeAxValue)

    // this->mAxIdx, this->sAxIdx, this->mBtIdx
}

void JoystickCoop::monitorJoystickCallback(
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

    // TODO: Remove Time update, and pass as variable to 
    // publishCombination(ros::Time &t)
    // Update time
    this->dronejs.header.stamp = msg->header.stamp;
    this->publishCombination();
}
// ---------------------------
void JoystickCoop::subjectJoystickCallback(
                const sensor_msgs::Joy::ConstPtr &msg)
{
    // Collect axes and button values of interest
    // 0:yaw, 1:lift, 2:roll, 3:pitch
    // 12: takeoff, 14:land, 15:emergency
    // Populate subject axes
    for(int i = 0; i < this->sAxIdx.size(); i++)
    {
        // multiply by -1 to reverse joystick axes (to match master sign)
        this->dronejs.axes[this->sAxIdx[i]] = msg->axes[this->sAxIdx[i]];
    }

    // TODO: Remove Time update, and pass as variable to 
    // publishCombination(ros::Time &t)
    // Update time
    this->publishCombination();
}

// TODO: change from (void) input to (ros::Time &)
void JoystickCoop::publishCombination(void)
{
	// DEBUG: log to console
    ROS_INFO("axes(s): %f, %f, %f, %f",
            this->dronejs.axes[0],
            this->dronejs.axes[1],
            this->dronejs.axes[2],
            this->dronejs.axes[3]);
    ROS_INFO("buttons(s): %d, %d, %d",
            this->dronejs.buttons[0],
            this->dronejs.buttons[1],
            this->dronejs.buttons[2]);
    ROS_INFO("sAxIdx.size(): %ld",
            this->sAxIdx.size());
    // PUBLISH:
    // TODO: populate Time before publishing
    // this->dronejs.header.stamp = msg->header.stamp;
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
