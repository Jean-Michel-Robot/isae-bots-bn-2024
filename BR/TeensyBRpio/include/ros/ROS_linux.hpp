#ifndef _ROS_LINUX_IMPL_H
#define _ROS_LINUX_IMPL_H

#include "rclcpp/rclcpp.hpp"
#include <memory>

class NodeW: public rclcpp::Node {

    public:
    NodeW(String name): Node(name) {}

    template<typename MsgT>
     rclcpp::Subscription<MsgT>::SharedPtr create_subscription(String topic, void (*callback)(const MsgT&)) {
        return create_subscription<MsgT>(topic, 10, [](std::unique_ptr<MsgT> msg) { callback(*msg) });
    }

    template<typename MsgT>
    rclcpp::Publisher<MsgT>::SharedPtr create_publisher(String topic) {
        return create_publisher<MsgT>(topic, 10);
    }

}

template <class T>
using Subscriber_t = rclcpp::Subscription<T>::SharedPtr;

template <class T>
using Publisher_t = rclcpp::Publisher<T>::SharedPtr;

using Node_t = NodeW;

#include "ros/ROS.hpp"

class ROSImpl: public ROS {
    public:
    static std::shared_ptr<ROSImpl> instance_shared();
    private:
    ROSImpl(): ROS() {}
};

#endif