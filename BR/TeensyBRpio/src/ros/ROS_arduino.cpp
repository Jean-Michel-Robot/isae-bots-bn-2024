#ifdef ARDUINO

#include "ros/ROS_arduino.hpp"

void ROS::spin()
{
    rclc_executor_spin_some(executor.get(), 0);
}

void ROS::logPrint(LogType logtype, string_t msg)
{
    // TODO
}

ROS &ROS::instance()
{
    static ROS instance;
    return instance;
}

#endif