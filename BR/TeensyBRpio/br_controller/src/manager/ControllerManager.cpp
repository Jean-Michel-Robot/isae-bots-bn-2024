#include "manager/ControllerManager.hpp"
#include "logging.hpp"

#include <memory>
#include <string>

#define TEMPLATE template <Actuators TActuators, CanControl<TActuators> TController, PositionFeedback TFeedback, Clock TClock>
#define MANAGER ControllerManager<TActuators, TController, TFeedback, TClock>

namespace manager {

TEMPLATE
MANAGER::ControllerManager(duration_t updateInterval, TClock clock, TController controller, TActuators actuators, TFeedback feedback)
    : ControllerManager(updateInterval, updateInterval, std::move(clock), std::move(controller), std::move(actuators), std::move(feedback)) {}

TEMPLATE
MANAGER::ControllerManager(duration_t minUpdateInterval, duration_t maxUpdateInterval, TClock clock, TController controller, TActuators actuators,
                           TFeedback feedback)
    : m_clock(std::move(clock)), m_minUpdateInterval(minUpdateInterval), m_maxUpdateInterval(maxUpdateInterval), m_lastUpdate(),
      m_controller(std::move(controller)), m_actuators(std::move(actuators)), m_feedback(std::move(feedback)) {
    this->template setCurrentState<StateIdle>();
}

TEMPLATE
ManagerStatus MANAGER::getStatus() const {
    return this->getCurrentState().getStatus();
}

TEMPLATE
void MANAGER::setActive(bool active) {
    if (active) {
        if (this->getStatus() == Active) {
            log(WARN, "The controller is already active.");
        } else {
            this->template setCurrentState<StateActivating>(m_actuators);
        }
    } else {
        if (this->getStatus() == Idle) {
            log(WARN, "The controller is already inactive.");
        } else {
            m_controller.reset();
            this->template setCurrentState<StateDeactivating>(m_actuators);
        }
    }
}

TEMPLATE
void MANAGER::loop(std::optional<std::function<void()>> tickCallback) {
    ManagerStatus status = this->getCurrentState().update(m_actuators);
    if (status != this->getStatus()) {
        switch (status) {
            case Idle:
                this->template setCurrentState<StateIdle>();
                break;
            case Active:
                this->template setCurrentState<StateActive>();
                m_controller.reset(m_feedback.getRobotPosition());
                break;

            case Activating:
                // Only returned by StateActivating
            case Deactivating:
                // Only returned by StateDeactivating
            default:
                log(ERROR, "Invalid response from manager state!" + std::to_string(status));
        }
        return;
    }

    instant_t nowMicros = m_clock.micros();
    if (!m_lastUpdate) {
        m_lastUpdate.emplace(nowMicros);
        return;
    }

    duration_t intervalMicros = getDurationMicros(*m_lastUpdate, nowMicros);

    while (intervalMicros >= m_minUpdateInterval) {
        duration_t tickInterval = std::min(m_maxUpdateInterval, intervalMicros);
        intervalMicros -= tickInterval;
        *m_lastUpdate += tickInterval;

        double_t interval = (double_t)tickInterval / (double_t)1000000.0;

        m_feedback.update(interval);
        sendOrderInternal([&](TController &controller, Position2D<Meter> robotPosition) {
            auto command = controller.updateCommand(interval, robotPosition);
            m_actuators.sendCommand(command);
        });
        m_actuators.update(interval);

        if (tickCallback) {
            tickCallback->operator()();
        }
    }
}

TEMPLATE
bool MANAGER::sendOrder(std::function<void(TController &, Position2D<Meter>)> order) {
    bool result = sendOrderInternal(order);
    if (!result) {
        log(WARN, "The order cannot be processed due to the current state of the manager.");
    }
    return result;
}

// Private
TEMPLATE
bool MANAGER::sendOrderInternal(std::function<void(TController &, Position2D<Meter>)> order) {
    return this->getCurrentState().sendOrder(m_controller, m_feedback.getRobotPosition(), order);
}

TEMPLATE
duration_t MANAGER::getMinUpdateInterval() const {
    return m_minUpdateInterval;
}
TEMPLATE
duration_t MANAGER::getMaxUpdateInterval() const {
    return m_maxUpdateInterval;
}

TEMPLATE
void MANAGER::setMinUpdateInterval(duration_t updateInterval) {
    m_minUpdateInterval = updateInterval;
}

TEMPLATE
void MANAGER::setMaxUpdateInterval(duration_t updateInterval) {
    m_maxUpdateInterval = updateInterval;
}

TEMPLATE
void MANAGER::setUpdateInterval(duration_t updateInterval) {
    setMinUpdateInterval(updateInterval);
    setMaxUpdateInterval(updateInterval);
}

TEMPLATE
void MANAGER::resetPosition(Position2D<Millimeter> newPosition) {
    m_feedback.resetPosition(newPosition);
    sendOrderInternal([=](TController &controller, Position2D<Meter> robotPosition) { controller.reset(newPosition.toMeters()); });
}

TEMPLATE
const TController &MANAGER::getController() const {
    return m_controller;
}
TEMPLATE
TController &MANAGER::getController() {
    return m_controller;
}
TEMPLATE
const TActuators &MANAGER::getActuators() const {
    return m_actuators;
}
TEMPLATE
const TFeedback &MANAGER::getPositionFeedback() const {
    return m_feedback;
}

} // namespace manager

// Explicit instantiation of the manager
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/manager.hpp"
template class manager::ControllerManager<actuators_t, controller_t, feedback_t, _clock_t>;
