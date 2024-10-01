#ifndef _ROS_IMPL_RCLC_HPP_
#define _ROS_IMPL_RCLC_HPP_

#define __RCCHECK(fn, hard)                                                                                                                          \
    {                                                                                                                                                \
        rcl_ret_t temp_rc = fn;                                                                                                                      \
        if ((temp_rc != RCL_RET_OK)) {                                                                                                               \
            std::string msg = "[rclc::ROSImpl]  Failed status on line " + std::to_string(__LINE__) + ": " + std::to_string((int)temp_rc) + ".";      \
            if (hard) {                                                                                                                              \
                msg += " Aborting.";                                                                                                                 \
            }                                                                                                                                        \
            Serial.print((msg + "\n").c_str());                                                                                                      \
            if (hard) {                                                                                                                              \
                exit(1);                                                                                                                             \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

#define RCCHECK_HARD(fn) __RCCHECK(fn, true)
#define RCCHECK_SOFT(fn) __RCCHECK(fn, false)

#include "defines/string.h"
#include "ros/rclc/Messages.hpp"
#include "ros/rclc/Publisher.hpp"
#include "ros/rclc/Subscriber.hpp"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/logging_macros.h>

namespace ros_rclc {

class Node {

  public:
    Node(string_t name) : m_name(name) {
        RCCHECK_HARD(rclc_support_init(m_support.get(), 0, NULL, m_allocator.get()));
        RCCHECK_HARD(rclc_node_init_default(m_node.get(), name.c_str(), "", m_support.get()));
        RCCHECK_HARD(rclc_executor_init(m_executor.get(), &m_support->context, 5, m_allocator.get()));
    }

    void spin_once() { rclc_executor_spin_some(m_executor.get(), 0); }

    template <typename T>
    Subscription<T> createSubscription(string_t topic, std::function<void(const T &)> callback) {
        return Subscription<T>(m_node, m_executor.get(), topic, callback);
    }

    template <typename T>
    Publisher<T> createPublisher(string_t topic) {
        return Publisher<T>(m_node, topic);
    }

    void sendLog(LogSeverity severity, string_t message) {
        const char *msgRaw = message.c_str();
        const char *logger = m_name.c_str();

        if (severity == INFO) {
            RCUTILS_LOG_INFO_NAMED(logger, msgRaw);
        } else if (severity == WARN) {
            RCUTILS_LOG_WARN_NAMED(logger, msgRaw);
        } else if (severity == ERROR) {
            RCUTILS_LOG_ERROR_NAMED(logger, msgRaw);
        } else if (severity == FATAL) {
            RCUTILS_LOG_FATAL_NAMED(logger, msgRaw);
        } else if (severity == DEBUG) {
            RCUTILS_LOG_DEBUG_NAMED(logger, msgRaw);
        } else {
            RCUTILS_LOG_ERROR_NAMED(logger, "Unknown log type: %d; defaulting to INFO", severity);
            sendLog(INFO, message);
        }
    }

  private:
    // Not sure if boxing is necessary
    std::unique_ptr<rcl_allocator_t> m_allocator = std::make_unique<rcl_allocator_t>(rcl_get_default_allocator());
    std::unique_ptr<rclc_support_t> m_support = std::make_unique<rclc_support_t>();
    std::unique_ptr<rclc_executor_t> m_executor = std::make_unique<rclc_executor_t>();
    std::shared_ptr<rcl_node_t> m_node = std::make_shared<rcl_node_t>();
    string_t m_name;
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