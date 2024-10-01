#ifndef _ROS_IMPL_RCLCPP_HPP_
#define _ROS_IMPL_RCLCPP_HPP_

#include "defines/string.h"
#include "ros/rclcpp/Messages.hpp"
#include "ros/rclcpp/Publisher.hpp"

#include <functional>

#include "rclcpp/rclcpp.hpp"

namespace ros_rclcpp {
class Node {
  public:
    Node(string_t name) : m_node(std::make_shared<rclcpp::Node>(name)) {}

    void spin_once() { rclcpp::spin_some(m_node); }

    template <typename T>
    rclcpp::Subscription<typename Messages<T>::type>::SharedPtr createSubscription(string_t topic, std::function<void(const T &)> callback) {
        return m_node->template create_subscription<typename Messages<T>::type>( //
            topic, 10, [callback](std::unique_ptr<typename Messages<T>::type> msg) -> void {
                T data = Messages<T>::extract(*msg);
                callback(data);
            });
    }

    template <typename T>
    Publisher<T> createPublisher(string_t topic) {
        return Publisher<T>(m_node->template create_publisher<typename Messages<T>::type>(topic, 10));
    }

    void sendLog(LogSeverity severity, string_t message) {
        const char *msgRaw = message.c_str();
        auto logger = m_node->get_logger();

        if (severity == INFO) {
            RCLCPP_INFO(logger, msgRaw);
        } else if (severity == WARN) {
            RCLCPP_WARN(logger, msgRaw);
        } else if (severity == ERROR) {
            RCLCPP_ERROR(logger, msgRaw);
        } else if (severity == FATAL) {
            RCLCPP_FATAL(logger, msgRaw);
        } else if (severity == DEBUG) {
            RCLCPP_DEBUG(logger, msgRaw);
        } else {
            RCLCPP_ERROR(logger, "Unknown log type: %d; defaulting to INFO", severity);
            sendLog(INFO, message);
        }
    }

  private:
    std::shared_ptr<rclcpp::Node> m_node;
};

class ROSImpl {
  public:
    using node_t = Node;
    template <typename T>
    using publisher_t = Publisher<T>;
    template <typename T>
    using subscription_t = rclcpp::Subscription<typename Messages<T>::type>::SharedPtr;

    using gains_t = br_messages::msg::GainsPid;
    using log_entry_t = br_messages::msg::LogEntry;
};
} // namespace ros_rclcpp

#endif