#ifndef _ROS_LINUX_IMPL_H
#define _ROS_LINUX_IMPL_H

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <memory>
#include "utils/string.h"

// TODO make more object-oriented
#define multi_array_get_raw_data(msg) (msg)->data.data()
#define multi_array_set_data(msg, raw, len) \
    (msg).data.clear(); \
    std::copy(raw, raw + len, std::back_inserter((msg).data))

namespace std_msgs
{
    using namespace msg;
};
namespace geometry_msgs
{
    using namespace msg;
}

class NodeW;

template<typename MsgT>
class Publisher {
    public:
    void publish(const MsgT &msg) {
        publisher->template publish<MsgT>(msg);
    }
    private:
    friend class NodeW;
    Publisher(typename rclcpp::Publisher<MsgT>::SharedPtr inner) : publisher(inner) {}
    typename rclcpp::Publisher<MsgT>::SharedPtr publisher;
};

class NodeW: public rclcpp::Node {

    public:
    NodeW(string_t name): Node(name) {}

    template<typename MsgT>
    typename rclcpp::Subscription<MsgT>::SharedPtr create_subscription(string_t topic, void (*callback)(const MsgT*)) {
        return Node::create_subscription<MsgT>(topic, 10, [callback](std::unique_ptr<MsgT> msg) -> void { callback(msg.get()); });
    }

    template<typename MsgT>
    Publisher<MsgT> create_publisher(string_t topic) {
        return Publisher<MsgT>(Node::create_publisher<MsgT>(topic, 10));
    }

};

template <class T>
using Subscriber_t = typename rclcpp::Subscription<T>::SharedPtr;

template <class T>
using Publisher_t = Publisher<T>;

using Node_t = NodeW;

#include "ros/ROS.hpp"

#endif