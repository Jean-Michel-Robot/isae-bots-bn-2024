#ifndef _ROS_HPP_
#define _ROS_HPP_

#include "configuration.hpp"

#include "Clock.hpp"
#include "logging.hpp"
#include "ros/DisplacementOrder.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

/**
 * ROS2 node to receive displacement orders and send callbacks.
 *
 * @tparam ROSImpl The implementation of ROS to use on this system (micro_ros only supports the C library rclc)
 * @tparam TActuators,TFeedback must be the same as the manager
 */
template <typename ROSImpl, Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
class ROS : public ROSImpl::node_t {
  public:
    using controller_t = controller::UnicycleController<ProportionalIntegralDerivative<Vector2D<Meter>>>;
    using manager_t = manager::ControllerManager<TActuators, controller_t, TFeedback, TClock>;

    template <typename T>
    using subscription_t = typename ROSImpl::template subscription_t<T>;
    template <typename T>
    using publisher_t = typename ROSImpl::template publisher_t<T>;

    using gains_t = typename ROSImpl::gains_t;
    using disp_order_t = DisplacementOrder;
    using log_entry_t = typename ROSImpl::log_entry_t;

    /**
     * @param sendPositionInterval,logInterval In microseconds
     */
    ROS(TClock clock, duration_t sendPositionInterval, duration_t logInterval);
    ROS(const ROS<ROSImpl, TActuators, TFeedback, TClock> &) = delete;

    ROS() : ROS(TClock(), SEND_POSITION_INTERVAL * 1000, ROS_LOG_INTERVAL * 1000) {}

    /// The manager is attached later to enable logging of early errors during creation of the manager or its dependencies.
    /// The subscribtions are not created until this method is called. This method must be called exactly once.
    void attachManager(std::shared_ptr<manager_t> manager);

    /// Spins the ROS node and call "loop" on the attached manager. It is an error to call this function before a manager is attached.
    void loop();
    void sendLog(LogSeverity severity, string_t message);

  private:
    subscription_t<disp_order_t> createSubOrder();
    subscription_t<bool> createSubIdle();
    subscription_t<gains_t> createSubGain();
    subscription_t<int16_t> createSubSpeed();

    TClock m_clock;
    duration_t m_sendInterval;
    duration_t m_logInterval;
    std::shared_ptr<manager_t> m_manager;

    instant_t m_lastSend;
    instant_t m_lastLog;
    bool m_wasActive;

    log_entry_t m_log;

    /* SUBSCRIBERS */

    /// /nextPositionTeensy
    std::optional<subscription_t<disp_order_t>> m_subOrder;
    /// /br/idle
    std::optional<subscription_t<bool>> m_subIdle;
    /// /gains
    std::optional<subscription_t<gains_t>> m_subGains;
    /// /teensy/obstacle_seen
    std::optional<subscription_t<int16_t>> m_subSpeed;

    /* PUBLISHERS */

    /// /current_position
    publisher_t<Position2D<Millimeter>> m_pubPositionFeedback;
    /// /okPosition
    publisher_t<int16_t> m_pubHN;
    /// /logTotaleArray
    publisher_t<log_entry_t> m_pubLog;
    /// /odos_count
    publisher_t<std::pair<int32_t, int32_t>> m_pubOdosTicks;
};

#endif