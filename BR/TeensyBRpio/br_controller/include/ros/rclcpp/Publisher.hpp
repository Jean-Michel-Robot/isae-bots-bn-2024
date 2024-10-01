#ifndef _ROS_IMPL_RCLCPP_PUBLISHER_HPP_
#define _ROS_IMPL_RCLCPP_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"

namespace ros_rclcpp {

class Node;

template <typename T>
class Publisher {
  public:
    void publish(const T &msg) {
        Messages<T>::copy(m_msg, msg);
        m_publisher->template publish<MsgT>(m_msg);
    }

  private:
    using MsgT = Messages<T>::type;

    friend class Node;
    Publisher(typename rclcpp::Publisher<MsgT>::SharedPtr inner) : m_publisher(inner) {}

    typename rclcpp::Publisher<MsgT>::SharedPtr m_publisher;
    MsgT m_msg;
};
} // namespace ros_rclcpp

#endif