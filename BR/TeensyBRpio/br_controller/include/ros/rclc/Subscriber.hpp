#ifndef _ROS_IMPL_RCLC_SUBSCRIBER_HPP_
#define _ROS_IMPL_RCLC_SUBCRIBER_HPP_

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

namespace ros_rclc {

class Node;

template <typename T>
class MessageWrapper : public Messages<T>::type {
  public:
    MessageWrapper(std::function<void(const T &)> callback) : Messages<T>::type(), m_callback(callback) {}

    /* unsafe */ static void dispatch(const void *msg) {
        const MessageWrapper<T> *msgCast = static_cast<const MessageWrapper<T> *>(msg);
        const typename Messages<T>::type *rawMsg = static_cast<const typename Messages<T>::type *>(msgCast);
        T data = Messages<T>::extract(*rawMsg);
        msgCast->m_callback(data);
    }

  private:
    std::function<void(const T &)> m_callback;
};

template <typename T>
class Subscription {
  public:
    Subscription(Subscription<T> &&subscription)
        : m_node(std::move(subscription.m_node)), m_subscription(std::move(subscription.m_subscription)), m_msg(std::move(subscription.m_msg)) {}
    ~Subscription() { RCCHECK_SOFT(rcl_subscription_fini(m_subscription.get(), m_node.get())); }

  private:
    using MsgT = MessageWrapper<T>;

    friend class Node;
    Subscription(std::shared_ptr<rcl_node_t> node, rclc_executor_t *executor, string_t topic, std::function<void(const T &)> callback)
        : m_node(std::move(node)), m_subscription(std::make_unique<rcl_subscription_t>()), m_msg(std::make_unique<MsgT>(callback)) {

        RCCHECK_HARD(rclc_subscription_init_default(m_subscription.get(), m_node.get(), Messages<T>::getTypeSupport(), topic.c_str()));
        RCCHECK_HARD(rclc_executor_add_subscription(executor, m_subscription.get(), m_msg.get(),
                                                    (rclc_subscription_callback_t)MessageWrapper<T>::dispatch, ON_NEW_DATA));
    }

    std::shared_ptr<rcl_node_t> m_node;
    std::unique_ptr<rcl_subscription_t> m_subscription;
    std::unique_ptr<MsgT> m_msg;
};

} // namespace ros_rclc

#endif