#ifndef _ROS_IMPL_RCLC_PUBLISHER_HPP_
#define _ROS_IMPL_RCLC_PUBLISHER_HPP_

#include <rcl/rcl.h>
#include <rclc/rclc.h>

namespace ros_rclc {

class Node;

template <typename T>
class Publisher {
  public:
    Publisher(Publisher<T> &&publisher)
        : m_node(std::move(publisher.m_node)), m_publisher(std::move(publisher.m_publisher)), m_msg(publisher.m_msg) {}

    void publish(const T &msg) {
        Messages<T>::copy(m_msg, msg);
        RCCHECK_HARD(rcl_publish(m_publisher.get(), &m_msg, NULL));
    }

    ~Publisher() { RCCHECK_SOFT(rcl_publisher_fini(m_publisher.get(), m_node.get())); }

  private:
    using MsgT = Messages<T>::type;

    friend class Node;

    Publisher(std::shared_ptr<rcl_node_t> node, string_t topic) : m_node(std::move(node)), m_publisher(std::make_unique<rcl_publisher_t>()), m_msg() {
        RCCHECK_HARD(rclc_publisher_init_best_effort(m_publisher.get(), m_node.get(), Messages<T>::getTypeSupport(), topic.c_str()));
    }

    std::shared_ptr<rcl_node_t> m_node;
    std::unique_ptr<rcl_publisher_t> m_publisher;
    MsgT m_msg;
};
} // namespace ros_rclc

#endif