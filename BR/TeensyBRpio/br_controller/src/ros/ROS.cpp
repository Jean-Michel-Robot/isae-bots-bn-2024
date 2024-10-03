#include "ros/ROS.hpp"
#include "defines/math.hpp"
#include "logging.hpp"
#include "manager/ManagerState.hpp"
#include "ros/Callbacks.hpp"

using UpdateResultCode = controller::UpdateResultCode;

#define TEMPLATE template <typename ROSImpl, Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
#define _ROS ROS<ROSImpl, TActuators, TFeedback, TClock>

template <typename T>
constexpr bool hasOdos = requires(const T &t) {
    t.getLeftOdoCount();
    t.getRighttOdoCount();
};

template <typename T>
constexpr bool hasTwoWheels = requires(const T &t) {
    t.getLastLeftSpeed();
    t.getLastRightSpeed();
};

TEMPLATE
_ROS::ROS(TClock clock, duration_t sendPositionInterval, duration_t logInterval)
    : ROSImpl::node_t("base_roulante"), m_clock(std::move(clock)), m_sendInterval(sendPositionInterval), m_logInterval(logInterval), m_lastSend(0),
      m_lastLog(0), m_wasActive(false), m_subOrder(), m_subIdle(), m_subGains(), m_subSpeed(),
      m_pubPositionFeedback(this->template createPublisher<Position2D<Millimeter>>("current_position")),
      m_pubHN(this->template createPublisher<int16_t>("okPosition")), //
      m_pubLog(this->template createPublisher<log_entry_t>("logTotaleArray")),
      m_pubOdosTicks(this->template createPublisher<std::pair<int32_t, int32_t>>("odos_count")) {}

TEMPLATE
void _ROS::attachManager(std::shared_ptr<manager_t> manager) {
    if (m_manager) {
        log(ERROR, "A manager is already attached to this ROS instance");
        return;
    }
    m_manager = std::move(manager);

    m_subOrder.emplace(createSubOrder());
    m_subIdle.emplace(createSubIdle());
    m_subGains.emplace(createSubGain());
    m_subSpeed.emplace(createSubSpeed());
}

TEMPLATE
typename _ROS::subscription_t<typename _ROS::disp_order_t> _ROS::createSubOrder() {
    return this->template createSubscription<disp_order_t>( //
        "nextPositionTeensy", [manager_weak = std::weak_ptr(m_manager)](const disp_order_t &order) {
            if (auto lock = manager_weak.lock()) {
                order(*lock);
            }
        });
}
TEMPLATE
typename _ROS::subscription_t<bool> _ROS::createSubIdle() {
    return this->template createSubscription<bool>("br/idle", [manager_weak = std::weak_ptr(m_manager)](const bool &active) {
        if (auto lock = manager_weak.lock()) {
            lock->setActive(active);
        }
    });
}
TEMPLATE
typename _ROS::subscription_t<typename _ROS::gains_t> _ROS::createSubGain() {
    return this->template createSubscription<gains_t>("gains", [manager_weak = std::weak_ptr(m_manager)](const gains_t &gains) {
        if (auto lock = manager_weak.lock()) {
            ProportionalIntegralDerivative<Vector2D<Meter>> pid = lock->getController().getErrorConverter();
            lock->getController().setErrorConverter(
                {gains.kp, gains.ti, gains.td, pid.filter(), pid.saturation(), pid.integralSaturation(), pid.derivativeSaturation()});
        }
    });
}
TEMPLATE
typename _ROS::subscription_t<int16_t> _ROS::createSubSpeed() {
    return this->template createSubscription<int16_t>("teensy/obstacle_seen", [manager_weak = std::weak_ptr(m_manager)](const int16_t &percentage) {
        if (auto lock = manager_weak.lock()) {
            Speeds maxSpeeds = lock->getController().getMaxSpeeds();
            lock->getController().setMaxSpeeds(maxSpeeds * clamp((double_t)percentage, 1., 100.) / 100., /* persist = */ false);
        }
    });
}

TEMPLATE
void _ROS::loop() {
    this->spin_once();

    if (!m_manager) {
        log(ERROR, "Called ROS::loop() before attaching a manager");
        return;
    }

    m_manager->loop([&]() {
        if (!m_wasActive && m_manager->getStatus() == manager::Active) {
            m_wasActive = true;
            m_pubHN.publish(OK_READY);
        } else if (m_wasActive && m_manager->getStatus() == manager::Idle) {
            m_wasActive = false;
            m_pubHN.publish(OK_IDLE);
        }

        UpdateResultCode event = m_manager->getController().getLastEvent();
        if (event & UpdateResultCode::ROTATION_COMPLETE) {
            m_pubHN.publish(OK_TURN);
        }
        if (event & UpdateResultCode::TRAJECTORY_COMPLETE) {
            if ((event & UpdateResultCode::WAS_REVERSE) && (event & UpdateResultCode::TERMINAL)) {
                m_pubHN.publish(OK_REVERSE);
            }
            m_pubHN.publish(OK_POS);
        }
    });

    duration_t now = m_clock.micros();
    if (getDurationMicros(m_lastSend, now) > m_sendInterval) {
        m_lastSend = now;
        m_pubPositionFeedback.publish(m_manager->getPositionFeedback().getRobotPosition().toMillimeters());

        if constexpr (hasOdos<TFeedback>) {
            const TFeedback &feedback = m_manager->getPositionFeedback();
            m_pubOdosTicks.publish(std::pair<int32_t, int32_t>(feedback.getLeftOdoCount(), feedback.getRightOdoCount()));
        }
    }

#ifdef _DEBUG
    if (getDurationMicros(m_lastLog, now) > m_logInterval) {
        m_lastLog = now;

        m_log.time = now;

        Position2D<Meter> robotPosition = m_manager->getPositionFeedback().getRobotPosition();
        m_log.robot_pos_x = robotPosition.x;
        m_log.robot_pos_y = robotPosition.y;
        m_log.robot_pos_theta = robotPosition.theta;

        Position2D<Meter> setpoint = m_manager->getController().getSetpoint();
        m_log.setpoint_pos_x = setpoint.x;
        m_log.setpoint_pos_y = setpoint.y;
        m_log.setpoint_pos_theta = setpoint.theta;

        Vector2D<Meter> goalPoint = m_manager->getController().getGoalPoint();
        m_log.goal_point_pos_x = goalPoint.x;
        m_log.goal_point_pos_y = goalPoint.y;

        Vector2D<Meter> goalPointSpeed = m_manager->getController().getGoalPointSpeed();
        m_log.goal_point_speed_x = goalPointSpeed.x;
        m_log.goal_point_speed_y = goalPointSpeed.y;

        Vector2D<Meter> lastError = m_manager->getController().getErrorConverter().lastError();
        m_log.asserv_error_x = lastError.x;
        m_log.asserv_error_y = lastError.y;

        Speeds lastCmd = m_manager->getController().getLastCommand();
        m_log.command_v = lastCmd.linear;
        m_log.command_omega = lastCmd.angular;

        if constexpr (hasTwoWheels<TActuators>) {
            m_log.commande_motor_r = m_manager->getActuators().getLastRightSpeed();
            m_log.commande_motor_l = m_manager->getActuators().getLastLeftSpeed();
        }

        m_log.manager_state = m_manager->getStatus();
        m_log.controller_state = m_manager->getController().getStatus();

        m_pubLog.publish(m_log);
    }
#endif
}

TEMPLATE
void _ROS::sendLog(LogSeverity severity, string_t message) {
    ROSImpl::node_t::sendLog(severity, message);
}

// Explicit instantiation of the ROS node
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/ros.hpp"
template class ROS<ros_impl_t, actuators_t, feedback_t, _clock_t>;