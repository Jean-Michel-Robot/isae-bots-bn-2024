#include "robot.h"

Arm** Robot::arms_array = NULL;
std_msgs::Int16 Robot::msg_to_pub;
ros::Publisher Robot::m_publisher_send("arm_feedback", &msg_to_pub);

Robot::Robot(ros::NodeHandle* m_p_ros_pointer_arg, Arm* arms_array_arg[4])
    : m_subscriber_read("arm_order", read_message)
{
    arms_array = arms_array_arg;
    this->m_p_ros_pointer = m_p_ros_pointer_arg;
}

void Robot::setup(int* arm_angle_up, int* arm_angle_down, int* arm_angle_down_gallery) {
    m_p_ros_pointer->subscribe(m_subscriber_read);
    m_p_ros_pointer->advertise(m_publisher_send);
    for (int i=0; i<4; i++) {
        this->arms_array[i]->setup(arm_angle_up[i], arm_angle_down[i], arm_angle_down_gallery[i]);
    }


}

void Robot::read_message(const std_msgs::Int16 &msg) {
    int message = msg.data;
    switch (message)
    {
    case (11):
        arms_array[0]->move_down_take();
        break;
    case (21):
        arms_array[1]->move_down_take();
        break;
    case (31):
        arms_array[2]->move_down_take();
        break;
    case (41):
        arms_array[3]->move_down_take();
        break;

    case (13):
        arms_array[0]->move_down_rack();
        break;
    case (43):
        arms_array[3]->move_down_rack();
        break;

    case (12):
        arms_array[0]->move_down_gallery();
        break;
    case (22):
        arms_array[1]->move_down_gallery();
        break;
    case (32):
        arms_array[2]->move_down_gallery();
        break;
    case (42):
        arms_array[3]->move_down_gallery();
        break;

    case (14):
        arms_array[0]->move_down_camp();
        break;
    case (24):
        arms_array[1]->move_down_camp();
        break;
    case (34):
        arms_array[2]->move_down_camp();
        break;
    case (44):
        arms_array[3]->move_down_camp();
        break;

    default:
        break;
    }
}