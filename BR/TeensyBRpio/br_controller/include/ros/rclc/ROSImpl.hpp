#ifndef _ROS_IMPL_RCLC_HPP_
#define _ROS_IMPL_RCLC_HPP_

#define TEENSY_RESTART SCB_AIRCR = 0x05FA0004

#define RCCHECK_HARD(fn)                                                                                                                             \
    {                                                                                                                                                \
        rcl_ret_t temp_rc = fn;                                                                                                                      \
        if ((temp_rc != RCL_RET_OK)) {                                                                                                               \
            delay(2000);                                                                                                                             \
            TEENSY_RESTART;                                                                                                                          \
        }                                                                                                                                            \
    }

#define RCCHECK_SOFT(fn)                                                                                                                             \
    { std::ignore = fn; }

#include "defines/string.h"
#include "ros/rclc/Messages.hpp"
#include "ros/rclc/Publisher.hpp"
#include "ros/rclc/Subscriber.hpp"

#include <optional>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/logging_macros.h>
#include <utility>

namespace ros_rclc {

class Node {

  public:
    Node(string_t name) : m_name(name), m_msgLog() {
        RCCHECK_HARD(rclc_support_init(m_support.get(), 0, NULL, m_allocator.get()));
        RCCHECK_HARD(rclc_node_init_default(m_node.get(), name.c_str(), "", m_support.get()));
        RCCHECK_HARD(rclc_executor_init(m_executor.get(), &m_support->context, 5, m_allocator.get()));

        m_logger.emplace(createPublisher<rcl_interfaces__msg__Log>("rosout"));
    }

    void spin_once() { RCCHECK_HARD(rclc_executor_spin_some(m_executor.get(), 0)); }

    template <typename T>
    Subscription<T> createSubscription(string_t topic, std::function<void(const T &)> callback) {
        return Subscription<T>(m_node, m_executor.get(), topic, callback);
    }

    template <typename T>
    Publisher<T> createPublisher(string_t topic) {
        return Publisher<T>(m_node, topic);
    }

    void sendLog(LogSeverity severity, string_t message) {
        switch (severity) {
            case INFO:
                m_msgLog.level = rcl_interfaces__msg__Log__INFO;
                break;
            case WARN:
                m_msgLog.level = rcl_interfaces__msg__Log__WARN;
                break;
            case ERROR:
                m_msgLog.level = rcl_interfaces__msg__Log__ERROR;
                break;
            case FATAL:
                m_msgLog.level = rcl_interfaces__msg__Log__FATAL;
                break;
            case DEBUG:
                m_msgLog.level = rcl_interfaces__msg__Log__DEBUG;
                break;
            default:
                sendLog(ERROR, "Unknown log type: %d; defaulting to INFO");
                m_msgLog.level = rcl_interfaces__msg__Log__INFO;
                break;
        }

        m_msgLog.name.data = (char *)m_name.c_str();
        m_msgLog.name.size = m_name.size();
        m_msgLog.name.capacity = m_name.capacity();

        m_msgLog.msg.data = (char *)message.c_str();
        m_msgLog.msg.size = message.size();
        m_msgLog.msg.capacity = message.capacity();

        if (m_logger) {
            m_logger->publish(m_msgLog);
        }
    }

  private:
    // Not sure if boxing is necessary
    std::unique_ptr<rcl_allocator_t> m_allocator = std::make_unique<rcl_allocator_t>(rcl_get_default_allocator());
    std::unique_ptr<rclc_support_t> m_support = std::make_unique<rclc_support_t>();
    std::unique_ptr<rclc_executor_t> m_executor = std::make_unique<rclc_executor_t>();
    std::shared_ptr<rcl_node_t> m_node = std::make_shared<rcl_node_t>();

    string_t m_name;
    rcl_interfaces__msg__Log m_msgLog;
    std::optional<Publisher<rcl_interfaces__msg__Log>> m_logger;
};

class ROSImpl {
  public:
    using node_t = Node;
    template <typename T>
    using publisher_t = Publisher<T>;
    template <typename T>
    using subscription_t = Subscription<T>;

    using gains_t = br_messages__msg__GainsPid;
    using log_entry_t = br_messages__msg__LogEntry;
};
} // namespace ros_rclc

#endif