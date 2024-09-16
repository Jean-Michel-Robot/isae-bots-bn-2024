#ifndef ARDUINO

#include "ros/ROS_linux.hpp"

class ROSImpl : public ROS
{
public:
    static std::shared_ptr<ROSImpl> instance_shared();

private:
    ROSImpl() : ROS() {}
};

void ROS::spin()
{
    rclcpp::spin_some(ROSImpl::instance_shared());
}

void ROS::logPrint(LogType logtype, string_t msg)
{
    const char *msgRaw = msg.c_str();
    if (logtype == INFO)
    {
        RCLCPP_INFO(get_logger(), msgRaw);
    }
    else if (logtype == WARN)
    {
        RCLCPP_WARN(get_logger(), msgRaw);
    }
    else if (logtype == ERROR)
    {
        RCLCPP_ERROR(get_logger(), msgRaw);
    }
    else if (logtype == FATAL)
    {
        RCLCPP_FATAL(get_logger(), msgRaw);
    }
    else if (logtype == DEBUG)
    {
        RCLCPP_DEBUG(get_logger(), msgRaw);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unknown log type: %d; defaulting to INFO", logtype);
        logPrint(INFO, msgRaw);
    }
}

std::shared_ptr<ROSImpl> ROSImpl::instance_shared()
{
    static std::shared_ptr<ROSImpl> instance(new ROSImpl());
    return instance;
}

ROS &ROS::instance()
{
    return *ROSImpl::instance_shared();
}

#endif