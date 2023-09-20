#ifndef ROBOT_H
#define ROBOT_H

#include "arms.h"
#include <ros.h>
#include <std_msgs/Int16.h>

class Robot {
    private:

    public:

        Robot(ros::NodeHandle* m_p_ros_pointer_arg, Arm* arms_array_arg[4]);

        void setup(int* arm_angle_up, int* arm_angle_down, int* arm_angle_down_gallery);
        void loop();

        static Arm** arms_array;

        ros::NodeHandle* m_p_ros_pointer;

        ros::Subscriber<std_msgs::Int16> m_subscriber_read;
        static std_msgs::Int16 msg_to_pub;
        static ros::Publisher m_publisher_send;

        std_msgs::Int16 pub;
        static void read_message(const std_msgs::Int16 &msg);

};

#endif