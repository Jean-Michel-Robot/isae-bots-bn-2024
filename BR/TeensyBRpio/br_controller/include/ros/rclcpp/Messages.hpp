#ifndef _ROS_IMPL_RCLCPP_MESSAGES_HPP_
#define _ROS_IMPL_RCLCPP_MESSAGES_HPP_

#include "geometry/Position2D.hpp"
#include "ros/DisplacementOrder.hpp"

#include <cstdint>
#include <utility>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "br_messages/msg/gains_pid.hpp"
#include "br_messages/msg/log_entry.hpp"

namespace ros_rclcpp {

/**
 * Utility functions to convert type `T` from/to the implementation-specific corresponding message type.
 * This allows to use implementation-agnostic types in class ROS.
 */
template <typename T>
class Messages {};

template <>
class Messages<int16_t> {
  public:
    using type = std_msgs::msg::Int16;

    static void copy(type &msg, int16_t data) { msg.data = data; }
    static int16_t extract(const type &msg) { return msg.data; }
};

template <>
class Messages<bool> {
  public:
    using type = std_msgs::msg::Int16;

    static void copy(type &msg, bool data) { msg.data = data ? 1 : 0; }
    static bool extract(const type &msg) { return msg.data != 0; }
};

template <>
class Messages<std::pair<int32_t, int32_t>> {
  public:
    using type = std_msgs::msg::Int32MultiArray;

    static void copy(type &msg, std::pair<int32_t, int32_t> data) {
        msg.data.clear();
        msg.data.push_back(data.first);
        msg.data.push_back(data.second);
    }
    static std::pair<int32_t, int32_t> extract(const type &msg) { return {msg.data.at(0), msg.data.at(1)}; }
};

template <>
class Messages<DisplacementOrder> {
  public:
    using type = geometry_msgs::msg::Quaternion;

    static void copy(type &msg, const DisplacementOrder &data) {
        msg.x = data.position.x;
        msg.y = data.position.y;
        msg.z = Angle(data.position.theta);
        msg.w = data.type;
    }
    static DisplacementOrder extract(const type &msg) { return DisplacementOrder((int)msg.w, Position2D<Millimeter>(msg.x, msg.y, msg.z)); }
};

template <>
class Messages<Position2D<Millimeter>> {
  public:
    using type = geometry_msgs::msg::Pose2D;

    static void copy(type &msg, const Position2D<Millimeter> &data) {
        msg.x = data.x;
        msg.y = data.y;
        msg.theta = Angle(data.theta);
    }
    static Position2D<Millimeter> extract(const type &msg) { return Position2D<Millimeter>(msg.x, msg.y, msg.theta); }
};

template <>
class Messages<br_messages::msg::GainsPid> {
  public:
    using type = br_messages::msg::GainsPid;

    static void copy(type &msg, const type &data) { msg = data; }
    static type &extract(type &msg) { return msg; }
};

template <>
class Messages<br_messages::msg::LogEntry> {
  public:
    using type = br_messages::msg::LogEntry;

    static void copy(type &msg, const type &data) { msg = data; }
    static type &extract(type &msg) { return msg; }
};

} // namespace ros_rclcpp

#endif