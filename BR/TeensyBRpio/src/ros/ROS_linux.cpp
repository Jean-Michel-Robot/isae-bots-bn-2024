#ifndef ARDUINO

#include "ros/ROS_linux.hpp"

void ROS::spin() override {
    rclcpp::spin_some(ROSImpl::instance_shared());
}

void ROSImpl::logPrint(LogType logtype, const char *msg) override
{
    if (logtype == INFO)
    {
        RCLCPP_INFO(m_nodeHandle->get_logger(), msg);
    }
    else if (logtype == WARN)
    {
        RCLCPP_WARN(m_nodeHandle->get_logger(), msg);
    }
    else if (logtype == ERROR)
    {
        RCLCPP_ERROR(m_nodeHandle->get_logger(), msg);
    }
    else if (logtype == FATAL)
    {
        RCLCPP_FATAL(m_nodeHandle->get_logger(), msg);
    }
    else if (logtype == DEBUG)
    {
        RCLCPP_DEBUG(m_nodeHandle->get_logger(), msg);
    }
    else
    {
        RCLCPP_ERROR(m_nodeHandle->get_logger(), "Unknown log type: %d; defaulting to INFO", logType);
        logPrint(INFO, msg);
    }
}

std::shared_ptr<ROSImpl> ROSImpl::instance_shared() {
    static std::shared_ptr<ROSImpl> instance = std::make_shared();
    return instance;
}

ROS &ROS::instance() {
    return *ROSImpl::instance_shared();
}

#endif