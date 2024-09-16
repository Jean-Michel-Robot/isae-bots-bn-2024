#ifndef _ROS_ARDUINO_IMPL_H
#define _ROS_ARDUINO_IMPL_H

#include <memory>
#include <vector>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/quaternion.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

#define RCCHECK_HARD(fn)                                                              \
    {                                                                                 \
        rcl_ret_t temp_rc = fn;                                                       \
        if ((temp_rc != RCL_RET_OK))                                                  \
        {                                                                             \
            printf(                                                                   \
                "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            exit(1);                                                                  \
        }                                                                             \
    }

// TODO make more object-oriented
#define multi_array_get_raw_data(msg) (msg)->data.data
#define multi_array_set_data(msg, raw, len) \
    (msg).data.data = raw;               \
    (msg).data.size = len;               \
    (msg).data.capacity = len;

namespace std_msgs
{
    using Int16 = std_msgs__msg__Int16;
    using Float32MultiArray = std_msgs__msg__Float32MultiArray;
    using Int32MultiArray = std_msgs__msg__Int32MultiArray;
};
namespace geometry_msgs
{
    using Quaternion = geometry_msgs__msg__Quaternion;
    using Pose2D = geometry_msgs__msg__Pose2D;
}

class Node;

template <typename T>
class Publisher
{
public:
    void publish(const T &msg)
    {
        RCCHECK_HARD(rcl_publish(publisher.get(), &msg, NULL));
    }

private:
    friend class Node;

    Publisher(std::unique_ptr<rcl_publisher_t> inner) : publisher(std::move(inner)) {}

    std::unique_ptr<rcl_publisher_t> publisher;
};

class Node
{

public:
    Node(String name) : allocator(new rcl_allocator_t(rcl_get_default_allocator()))
    {
        RCCHECK_HARD(rclc_support_init(support.get(), 0, NULL, allocator.get()));
        RCCHECK_HARD(rclc_node_init_default(node.get(), name.c_str(), "", support.get()));
        RCCHECK_HARD(rclc_executor_init(executor.get(), &support->context, 5, allocator.get()));
    }

    template <typename MsgT>
    std::unique_ptr<rcl_subscription_t> create_subscription(String topic, void (*callback)(const MsgT *));

    template <typename MsgT>
    Publisher<MsgT> create_publisher(String topic);

protected:
    template <typename MsgT>
    std::unique_ptr<rcl_subscription_t> create_subscription(String topic, void (*callback)(const MsgT *), const rosidl_message_type_support_t *support)
    {
        MsgT *msg = new MsgT();

        std::unique_ptr<rcl_subscription_t> subscriber = std::make_unique<rcl_subscription_t>();
        RCCHECK_HARD(rclc_subscription_init_default(subscriber.get(), node.get(), support, topic.c_str()));
        RCCHECK_HARD(rclc_executor_add_subscription(
            executor.get(), subscriber.get(), msg, (rclc_subscription_callback_t)callback, ON_NEW_DATA));

        return subscriber;
    }

    template <typename MsgT>
    Publisher<MsgT> create_publisher(String topic, const rosidl_message_type_support_t *support)
    {
        std::unique_ptr<rcl_publisher_t> publisher = std::make_unique<rcl_publisher_t>();
        RCCHECK_HARD(rclc_publisher_init_default(publisher.get(), node.get(), support, topic.c_str()));
        return Publisher<MsgT>(std::move(publisher));
    }

    // Not sure if boxing is necessary
    std::unique_ptr<rcl_allocator_t> allocator;
    std::unique_ptr<rclc_support_t> support = std::make_unique<rclc_support_t>();
    std::unique_ptr<rclc_executor_t> executor = std::make_unique<rclc_executor_t>();
    std::unique_ptr<rcl_node_t> node = std::make_unique<rcl_node_t>();
};

template <>
inline std::unique_ptr<rcl_subscription_t> Node::create_subscription<geometry_msgs::Quaternion>(String topic, void (*callback)(const geometry_msgs::Quaternion *))
{
    return create_subscription<geometry_msgs::Quaternion>(topic, callback, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion));
}
template <>
inline std::unique_ptr<rcl_subscription_t> Node::create_subscription<std_msgs::Float32MultiArray>(String topic, void (*callback)(const std_msgs::Float32MultiArray *))
{
    return create_subscription<std_msgs::Float32MultiArray>(topic, callback, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray));
}
template <>
inline std::unique_ptr<rcl_subscription_t> Node::create_subscription<std_msgs::Int16>(String topic, void (*callback)(const std_msgs::Int16 *))
{
    return create_subscription<std_msgs::Int16>(topic, callback, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16));
}

template <>
inline Publisher<geometry_msgs::Quaternion> Node::create_publisher<geometry_msgs::Quaternion>(String topic)
{
    return create_publisher<geometry_msgs::Quaternion>(topic, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion));
}
template <>
inline Publisher<geometry_msgs::Pose2D> Node::create_publisher<geometry_msgs::Pose2D>(String topic)
{
    return create_publisher<geometry_msgs::Pose2D>(topic, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D));
}
template <>
inline Publisher<std_msgs::Int16> Node::create_publisher<std_msgs::Int16>(String topic)
{
    return create_publisher<std_msgs::Int16>(topic, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16));
}
template <>
inline Publisher<std_msgs::Float32MultiArray> Node::create_publisher<std_msgs::Float32MultiArray>(String topic)
{
    return create_publisher<std_msgs::Float32MultiArray>(topic, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray));
}
template <>
inline Publisher<std_msgs::Int32MultiArray> Node::create_publisher<std_msgs::Int32MultiArray>(String topic)
{
    return create_publisher<std_msgs::Int32MultiArray>(topic, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray));
}

template <typename T>
using Subscriber_t = std::unique_ptr<rcl_subscription_t>;

template <typename T>
using Publisher_t = Publisher<T>;

using Node_t = Node;

#include "ros/ROS.hpp"

#endif