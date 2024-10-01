#ifndef _SPEC_ROS_HPP_
#define _SPEC_ROS_HPP_

#include "specializations/actuators.hpp"
#include "specializations/clock.hpp"
#include "specializations/feedback.hpp"

#include "ros/ROS.hpp"

#ifdef ARDUINO

#include "ros/rclc/ROSImpl.hpp"
#include <micro_ros_platformio.h>
using ros_impl_t = ros_rclc::ROSImpl;

#else

#include "ros/rclcpp/ROSImpl.hpp"
using ros_impl_t = ros_rclcpp::ROSImpl;

#endif // #ifdef ARDUINO

using ros_t = ROS<ros_impl_t, actuators_t, feedback_t, _clock_t>;

#endif