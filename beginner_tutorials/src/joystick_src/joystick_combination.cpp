#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <vector>

#include "beginner_tutorials/joystick_combination.h"
//#include <beginner_tutorials/joystick_combination.h>

// ----------------------------------------------------------------------
// TODO: Create an independent class of axes swapping utilities

// Clears a given value (val) from the std::vector<int> &(vec)
// 
void JoystickCombination::clearIndex(std::vector<int> &vec, int val)
{
    //check val is in vec
    //clear if present
    vec.erase(
            std::remove(
                vec.begin(), 
                vec.end(), 
                val
                ), 
            vec.end());
}

// ----------------------------------------------------------------------
// Adds the value (val) to the std::vector<int> &(vec)
void JoystickCombination::setIndex(std::vector<int> &vec, int val)
{
    //check val is in vec
    // if not, set the value
    clearIndex(vec, val);
    vec.push_back(val);
}

// ----------------------------------------------------------------------
// 'Moves' a value (val) 
//      from std::vector<int> &vStart
//      to std::vector<int> &vEnd
void JoystickCombination::swapIndex(std::vector<int> &vStart, 
                    std::vector<int> &vEnd,
                    int val)
{
    //
    clearIndex(vStart, val);
    setIndex(vEnd, val);
}
// ----------------------------------------------------------------------
JoystickCombination::JoystickCombination()
{
    // Set dronjs (publish message)
    this->drone_js_.axes.resize(4);
    this->drone_js_.buttons.resize(3);

    // Set indices
    // Axes
    this->monitor_axes_indices_.push_back(0);
    this->monitor_axes_indices_.push_back(1);
    this->monitor_axes_indices_.push_back(2);
    this->subject_axes_indices_.push_back(3);
    // Buttons
    this->monitor_buttons_indices_.push_back(12);
    this->monitor_buttons_indices_.push_back(14);
    this->monitor_buttons_indices_.push_back(15);

    //ros::Subscriber sub_joystick_monitor_;
    sub_joystick_monitor_ = nh.subscribe(
            "/monitor/joy", 
            1, 
            &JoystickCombination::monitorJoystickCallback,
            this);

    //ros::Subscriber sub_joystick_subject_;
    sub_joystick_subject_ = nh.subscribe(
            "/subject/joy", 
            1, 
            &JoystickCombination::subjectJoystickCallback,
            this);

    //ros::Subscriber sub_control_switch_;
    sub_control_switch_ = nh.subscribe(
            "/disturbance", 
            1, 
            &JoystickCombination::disturbanceCallback,
            this);

    //ros::Publisher pub_joystick_;
	pub_joystick_ = nh.advertise<sensor_msgs::Joy>(
            "joy",
            1);
}

// -------------------------------------------------------------------
void JoystickCombination::disturbanceCallback(
                const std_msgs::Bool::ConstPtr &msg)
{
	// DEBUG: log to console
    ROS_INFO("Disturbance: %s",
            msg->data ? "true" : "false"); // print True/False

    // TODO: Apply control switching
    // - Use function pointer to an external algorithm that 
    //   applies the TEST-ALGORITHM
    // - FORM: func(int &monitorAxValue, int &alternativeAxValue)
    // remove element of value equal to pitchIdx

    // Axes Indices (0:Yaw, 1:Lift, 2:Roll, 3:Pitch)
    // Button Indices (0:Takeoff, 1:Land, 2:Emergency)
    int pitchIdx = 3;
    // https://cse.sc.edu/~jokane/agitr/agitr-small-param.pdf
    // Section 7.3
    // void ros::param::set("param_name", value)
    // with declared value_container
    // bool ros::param::get("param_name", value_container)
    if(msg->data)
    {
        // if there's a distubance, the monitor takes over
        // swap from monitor_axes_indices_ to subject_axes_indices_
        this->swapIndex(this->monitor_axes_indices_, this->subject_axes_indices_, pitchIdx);
        // ros::param::set("/joystick_controller/ScalePitch",-1);
    }
    else
    {
        // from subject_axes_indices_ to monitor_axes_indices_
        this->swapIndex(this->subject_axes_indices_, this->monitor_axes_indices_, pitchIdx);
        // ros::param::set("/joystick_controller/ScalePitch",1);
    }
}

// ----------------------------------------------------------------------
// Callback for /monitor/joy
void JoystickCombination::monitorJoystickCallback(
                const sensor_msgs::Joy::ConstPtr &msg)
{
    // Collect axes and button values of interest
    // 0:yaw, 1:lift, 2:roll, 3:pitch
    // 12: takeoff, 14:land, 15:emergency
    // Populate monitor axes and buttons
    for(int i = 0; i < monitor_axes_indices_.size(); i++)
    {
        this->drone_js_.axes[monitor_axes_indices_[i]] = msg->axes[monitor_axes_indices_[i]];
    }
    for(int i = 0; i < monitor_buttons_indices_.size(); i++)
    {
        this->drone_js_.buttons[i] = msg->buttons[monitor_buttons_indices_[i]];
    }

    // TODO: Remove Time update, and pass as variable to 
    // publishCombination(ros::Time &t)
    // Update time
    this->drone_js_.header.stamp = msg->header.stamp;
    this->publishCombination();
}
// ----------------------------------------------------------------------
void JoystickCombination::subjectJoystickCallback(
                const sensor_msgs::Joy::ConstPtr &msg)
{
    // Collect axes and button values of interest
    // 0:yaw, 1:lift, 2:roll, 3:pitch
    // 12: takeoff, 14:land, 15:emergency
    // Populate subject axes
    for(int i = 0; i < this->subject_axes_indices_.size(); i++)
    {
        // multiply by -1 to reverse joystick axes (to match master sign)
        this->drone_js_.axes[this->subject_axes_indices_[i]] = -1.0*msg->axes[this->subject_axes_indices_[i]];
    }

    // TODO: Remove Time update, and pass as variable to 
    // publishCombination(ros::Time &t)
    // Update time
    this->drone_js_.header.stamp = msg->header.stamp;
    this->publishCombination();
}

// ----------------------------------------------------------------------
// TODO: change from (void) input to (ros::Time &)
void JoystickCombination::publishCombination(void)
{
	// DEBUG: log to console
    // ROS_INFO("axes(s): %f, %f, %f, %f",
            // this->drone_js_.axes[0],
            // this->drone_js_.axes[1],
            // this->drone_js_.axes[2],
            // this->drone_js_.axes[3]);
    // ROS_INFO("buttons(s): %d, %d, %d",
            // this->drone_js_.buttons[0],
            // this->drone_js_.buttons[1],
            // this->drone_js_.buttons[2]);
    // ROS_INFO("subject_axes_indices_.size(): %ld",
            // this->subject_axes_indices_.size());
    // PUBLISH:
    // TODO: populate Time before publishing
    // this->drone_js_.header.stamp = msg->header.stamp;
    pub_joystick_.publish(this->drone_js_);
}

// ----------------------------------------------------------------------
