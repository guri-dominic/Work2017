/*
 * Dominic Guri
 * Joy Node in C++ 
 * > Subscribe & Publish joy stick data from two joysticks
 * > Use a boolean operator to switch between joysticks
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
		ros::Subscriber sJoySub;
		ros::Subscriber mJoySub;
        ros::Subscriber joySwitchSub;
	    ros::Publisher joyPub;
               
	    // joystick data variable
        sensor_msgs::Joy dronejs;

        // JOYSTICK INDICES 
        // Axes Indices (0:Yaw, 1:Lift, 2:Roll, 3:Pitch)
        std::vector<int> mAxIdx;
        std::vector<int> sAxIdx;
        // Button Indices (0:Takeoff, 1:Land, 2:Emergency)
        std::vector<int> mBtIdx;
        // std::vector<int> sBtIdx;

        // BINARY SWITCH BETWEEN JOYSTICKS
        // Binary Switch between subject and monitor
        bool useSubjectJoy;
        
        // TIME: LAST INPUT TIME
        ros::Time inTime;

	public:
        // CONSTRUCTOR:
		JoystickCoop();
        JoystickCoop(
                std::vector<int> mAxIdx,
                std::vector<int> sAxIdx,
                std::vector<int> mBtIdx); 

        // CALLBACK FUNCTIONS:
        // Two joystick callback functions
		void monitorJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
		void subjectJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
        // Binary Switch callback functions
        void joystickBinarySwitchCallback(
                const std_msgs::Bool::ConstPtr &msg);

        // Set Functions
        void setMonitorAxes(std::vector<int> indices);
        void setMonitorButtons(std::vector<int> indices);
        void setSubjectAxes(std::vector<int> indices);
        void setInputChannels(
                    std::vector<int> mAxIndices, 
                    std::vector<int> sAxIndices,
                    std::vector<int> mBtIndices);
        void getInputChannels();
};

// ----------------------------------------------------------------------
JoystickCoop::JoystickCoop()
{
    //ros::Subscriber sJoySub;
    sJoySub = nh.subscribe(
            "/monitor/joy", 
            2, 
            &JoystickCoop::monitorJoysticksCallback,
            this);
    //ros::Subscriber mJoySub;
    mJoySub = nh.subscribe(
            "/subject/joy", 
            2, 
            &JoystickCoop::subjectJoysticksCallback,
            this);
    //ros::Subscriber joySwitchSub;
    joySwitchSub = nh.subscribe(
            "/disturbance/switch",
            1,
            &JoystickCoop::joystickBinarySwitchCallback,
            this);
    //ros::Publisher joyPub;
	joyPub = nh.advertise<sensor_msgs::Joy>(
            "joy_node",
            1);
}
// ------------------------
JoystickCoop::JoystickCoop(
                std::vector<int> mAxIdx,
                std::vector<int> sAxIdx,
                std::vector<int> mBtIdx): 
                    mAxIdx(mAxIdx),
                    sAxIdx(sAxIdx),
                    mBtIdx(mBtIdx)
{
    /* constructor */
}

// -------------------------------------------------------------------
void JoystickCoop::monitorJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg)
{
	// store joystick values of interest
    //this->dronejs = *msg;
    // Populate monitor axes
    for(int i = 0; i < mAxIdx.size(); i++)
    {
        this->dronejs.axes[i] = msg->axes[i];
    }
    // Populate monitor buttons
    for(int i = 0; i < mBtIdx.size(); i++)
    {
        this->dronejs.axes[i] = msg->axes[i];
    }
	// DEBUG: log to console
    // ROS_INFO("axes: %f, %f, %f, %f",
            // this->dronejs.axes[0],
            // this->dronejs.axes[1],
            // this->dronejs.axes[2],
            // this->dronejs.axes[3]);
    // PUBLISH:
    //joyPub
    joyPub.publish(this->dronejs);
}
// -----------------------------------------
void JoystickCoop::subjectJoysticksCallback(
                const sensor_msgs::Joy::ConstPtr &msg)
{
	// store joystick values of interest
    //this->dronejs = *msg;
    for(int i = 0; i < sAxIdx.size(); i++)
    {
        this->dronejs.axes[i] = msg->axes[i];
    }
	// DEBUG: log to console
	// ROS_INFO("axes: %f, %f, %f, %f",
            // this->dronejs.axes[0],
            // this->dronejs.axes[1],
            // this->dronejs.axes[2],
            // this->dronejs.axes[3]);
    // PUBLISH:
    joyPub.publish(this->dronejs);
}

// -------------------------------------------------------------------
void JoystickCoop::joystickBinarySwitchCallback(
                const std_msgs::Bool::ConstPtr &msg)
{
	// store joystick values of interest
    useSubjectJoy = msg->data;
}

// -------------------------------------------------------------------
// Assigning Axes indices (monitor & subject)
void JoystickCoop::setMonitorAxes(std::vector<int> indices)
{
    // Axes Indices
    mAxIdx = indices;
}
// -----
void JoystickCoop::setSubjectAxes(std::vector<int> indices)
{
    // subject axes indices
    sAxIdx = indices;
}

// -------------------------------------------------------------------
void JoystickCoop::setMonitorButtons(std::vector<int> indices)
{
    // Button Indices
    mBtIdx = indices;
}
void JoystickCoop::getInputChannels()
{
    // Print Master Joystick Axes
    // Print Subject Joystick Axes
    // Print Current Joystick Switch State
    // ALTERNATIVELY: Print Axes Vector sizes
	ROS_INFO("Index Array Sldzes (mAx, mBt, sAx): %ld, %ld, %ld", 
            this->mAxIdx.size(),
            this->mBtIdx.size(),
            this->sAxIdx.size()
            );
}

// -------------------------------------------------------------------
// Assigning button indices (only for monitor joystick)
void JoystickCoop::setInputChannels(std::vector<int> mAxIndices, 
                                    std::vector<int> sAxIndices,
                                    std::vector<int> mBtIndices)
{
    mAxIdx = mAxIndices;
    sAxIdx = sAxIndices;
    mBtIdx = mBtIndices;
}

/////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // ROS Initilization & Running
	ros::init(argc, argv, "joystickCombination");
    std::vector<int> mAxIndices;
    std::vector<int> sAxIndices;
    std::vector<int> mBtIndices;
    // for this specific case the axes match for the monitor and subject
    // 0:yaw, 1:lift, 2:roll, 3:pitch
    int yaw = 0;
    int lift = 1;
    int roll = 2;
    int pitch = 3;

    mAxIndices.push_back(yaw);
    mAxIndices.push_back(lift);
    mAxIndices.push_back(roll);
    sAxIndices.push_back(pitch);
    mBtIndices.push_back(12);   // Takeoff      [triangle]
    mBtIndices.push_back(14);   // Land         [X]
    mBtIndices.push_back(15);   // Emergency    [square]

    // JoystickCoop joyCollaboration;
    JoystickCoop joyCollaboration(
            mAxIndices,
            sAxIndices,
            mBtIndices);
    
    joyCollaboration.getInputChannels(); 
    ros::spin();
}

//struct joystick{
//    int yaw = 0; 
//    int lift = 1;
//    int roll = 2; 
//    int pitch = 3;
//} mjoystick, sjoystick;
/////////////////////////////////////////////////////////////////////////
JoystickCoop work;

// joystick instance for global stirage
sensor_msgs::Joy dronejs;
ros::Time mTime;

// -------------------------------------------------------------------
void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    dronejs = *msg;    

    // ROS_INFO("axes: %f, %f, %f, %f",
            // dronejs.axes[0],
            // dronejs.axes[1],
            // dronejs.axes[2],
            // dronejs.axes[3]);
}

// -------------------------------------------------------------------
int mainOLD(int argc, char **argv)
{
	ros::init(argc, argv, "joy_combination");
	ros::NodeHandle nh;
	ros::Subscriber m_sub = nh.subscribe("/monitor/joy", 1, joyCallback);
	ros::Subscriber s_sub = nh.subscribe("/subject/joy", 1, joyCallback);
	ros::Publisher joyPub = 
	nh.advertise<sensor_msgs::Joy>("joy_node", 2);

	dronejs.header.frame_id = "joy_node";

	while (!joyPub)
	{
		dronejs.header.stamp = mTime;
		joyPub.publish(dronejs);

		ros::spin();
	}

	return 0;
}

// -------------------------------------------------------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------

