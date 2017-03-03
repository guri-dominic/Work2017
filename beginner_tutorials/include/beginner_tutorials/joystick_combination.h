/*
 * Dominic Guri
 * - Joystick Utilities class for combining multiple joysticks for controlling 
 *   a single system
 * - https://goo.gl/NOn290
 */
#ifndef JOYSTICKCOOP_H
#define JOYSTICKCOOP_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <vector>
#include <algorithm>

// ----------------------------------------------------------------------
// - This case subscribes to joysticks under the namespaces 
//      /monitor
//      /subject
//   And a boolean topic "/disturbance"
//   >  the topic is used to switch pitch-control between
//      the /monitor/joy and /subject/joy
// - The resulting joystick combination is published under
//      /joy
class JoystickCombination
{
    private:
        // NodeHandle, Subscribers & Publishers
        ros::NodeHandle nh;
		ros::Subscriber sub_joystick_monitor_;
		ros::Subscriber sub_joystick_subject_;
		ros::Subscriber sub_control_switch_;
	    ros::Publisher pub_joystick_;
               
        // Axes Indices (0:Yaw, 1:Lift, 2:Roll, 3:Pitch)
        std::vector<int> monitor_axes_indices_;
        std::vector<int> subject_axes_indices_;
        // Button Indices (0:Takeoff, 1:Land, 2:Emergency)
        std::vector<int> monitor_buttons_indices_;
        // std::vector<int> subject_buttons_indices_;

	    // joystick data variable
        sensor_msgs::Joy drone_js_;

        // functions
        // TODO: change from (void) input to (ros::Time &)
        // publishCombination(ros::Time &t)
        void publishCombination(void);
        // Joystick Axes swapping - based on boolean input
        void clearIndex(std::vector<int> &vec, int val);
        void setIndex(std::vector<int> &vec, int val);
        void swapIndex(std::vector<int> &vStart, 
                    std::vector<int> &vEnd,
                    int val);


	public:
        // CONSTRUCTOR:
		JoystickCombination();

        // CALLBACK FUNCTIONS:
        // Two joystick callback functions
		void monitorJoystickCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
		void subjectJoystickCallback(
                const sensor_msgs::Joy::ConstPtr &msg);
		void disturbanceCallback(
                const std_msgs::Bool::ConstPtr &msg);
};

#endif 

