#ifndef _ROS_IMPL_RCLC_MESSAGES_HPP_
#define _ROS_IMPL_RCLC_MESSAGES_HPP_

#include "logging.hpp"

#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/quaternion.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <rcl_interfaces/msg/log.h>

#include "br_messages/msg/gains_pid.h"
#include "br_messages/msg/log_entry.h"

#include <cstdint>
#include <functional>

namespace ros_rclc {

using support_t = const rosidl_message_type_support_t *const;

/**
 * Utility functions to convert type `T` from/to the implementation-specific corresponding message type.
 * This allows to use implementation-agnostic types in class ROS.
 */
template <typename T>
class Messages {};

template <>
class Messages<int16_t> {
  public:
    using type = std_msgs__msg__Int16;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16); }

    static void copy(type &msg, int16_t data) { msg.data = data; }
    static int16_t extract(const type &msg) { return msg.data; }
};

template <>
class Messages<bool> {
  public:
    using type = std_msgs__msg__Int16;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16); }

    static void copy(type &msg, bool data) { msg.data = data ? 1 : 0; }
    static bool extract(const type &msg) { return msg.data != 0; }
};

template <>
class Messages<std::pair<int32_t, int32_t>> {
  public:
    using type = std_msgs__msg__Int32MultiArray;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray); }

    static void copy(type &msg, std::pair<int32_t, int32_t> data) {
        if (msg.data.capacity < 2) {
            if (msg.data.capacity != 0 && msg.data.data != NULL) {
                delete[] msg.data.data;
            }
            msg.data.data = new int32_t[2];
            msg.data.capacity = 2;
        }
        msg.data.data[0] = data.first;
        msg.data.data[1] = data.second;
        msg.data.size = 2;
    }
    static std::pair<int32_t, int32_t> extract(const type &msg) {
        if (msg.data.capacity < 2 || msg.data.size != 2) {
            log(ERROR, "Cannot convert std_msgs__msg__Int32MultiArray message to pair (invalid capacity or size)");
            return {0, 0};
        }
        return {msg.data.data[0], msg.data.data[0]};
    }
};

template <>
class Messages<DisplacementOrder> {
  public:
    using type = geometry_msgs__msg__Quaternion;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion); }

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
    using type = geometry_msgs__msg__Pose2D;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D); }

    static void copy(type &msg, const Position2D<Millimeter> &data) {
        msg.x = data.x;
        msg.y = data.y;
        msg.theta = Angle(data.theta);
    }
    static Position2D<Millimeter> extract(const type &msg) { return Position2D<Millimeter>(msg.x, msg.y, msg.theta); }
};

template <>
class Messages<br_messages__msg__GainsPid> {
  public:
    using type = br_messages__msg__GainsPid;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, GainsPid); }

    static void copy(type &msg, const type &data) { msg = data; }
    static type extract(const type &msg) { return msg; }
};

template <>
class Messages<br_messages__msg__LogEntry> {
  public:
    using type = br_messages__msg__LogEntry;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, LogEntry); }

    static void copy(type &msg, const type &data) { msg = data; }
    static type extract(const type &msg) { return msg; }
};

template <>
class Messages<rcl_interfaces__msg__Log> {
  public:
    using type = rcl_interfaces__msg__Log;
    static support_t getTypeSupport() { return ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log); }

    static void copy(type &msg, rcl_interfaces__msg__Log data) { msg = data; }
    static rcl_interfaces__msg__Log extract(const type &msg) { return msg; }
};

} // namespace ros_rclc

#endif