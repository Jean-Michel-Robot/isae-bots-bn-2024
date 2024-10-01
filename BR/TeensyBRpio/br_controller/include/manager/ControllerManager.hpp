#ifndef _CONTROLLER_MANAGER_HPP_
#define _CONTROLLER_MANAGER_HPP_

#include "configuration.hpp"
#include "defines/constraint.hpp"

#include "Actuators.hpp"
#include "Clock.hpp"
#include "PositionFeedback.hpp"
#include "fsm/StateMachine.hpp"

#include "manager/ManagerState.hpp"
#include "manager/states/StateActivating.hpp"
#include "manager/states/StateActive.hpp"
#include "manager/states/StateDeactivating.hpp"
#include "manager/states/StateIdle.hpp"

#include <functional>
#include <memory>
#include <optional>

namespace manager {

/**
 * A manager for a generic closed-loop controller. This class manages the state of the actuators, connects the actuators to the output
 * of the controller and provides the controller with the position feedback from the robot.
 *
 * The manager must be activated with setActive(true) before the controller can send commands to the actuators.
 *
 * This class is designed to be as generic as possible, hence the number of type parameters.
 *
 * @tparam TActuators See concept Actuators. Must be a concrete type.
 * @tparam TController The controller. See UnicycleController. Must be a concrete type.
 * @tparam TFeedback See concept PositionFeedback. Must be a concrete type.
 * @tparam TClock The clock to use to get time measurements. Must be a concrete type.
 *
 * Using virtual types as any of the type parameters is undefined behaviour.
 */
template <Actuators TActuators, CanControl<TActuators> TController, PositionFeedback TFeedback, Clock TClock>
class ControllerManager : private fsm::StateMachine<ManagerState<TActuators, TController>> {
  public:
    using Self = ControllerManager<TActuators, TController, TFeedback, TClock>;
    using StateIdle = manager::StateIdle<TActuators, TController>;
    using StateDeactivating = manager::StateDeactivating<TActuators, TController>;
    using StateActivating = manager::StateActivating<TActuators, TController>;
    using StateActive = manager::StateActive<TActuators, TController>;

    /**
     * @param updateInterval The interval (in microseconds) between two updates of the command to send to the motors. Multiple calls of loop()) within
     * this interval will result in the command being updated only once. Conversely, one call to loop() may result in the command being updated more
     * than once if the previous call to loop() was too old. In other words, `interval` will always have the same value when calling
     * "feedback.update()", "controller.updateCommand()" and "actuators.update()".
     */
    ControllerManager(duration_t updateInterval, TClock clock, TController controller, TActuators actuators, TFeedback feedback);

    ControllerManager(duration_t minUpdateInterval, duration_t maxUpdateInterval, TClock clock, TController controller, TActuators actuators,
                      TFeedback feedback);

    /// Initializes the manager with the default values from the configuration file.
    ControllerManager()
        requires Default<TClock> && Default<TController> && Default<TActuators> && Default<TFeedback>
        : ControllerManager(UPDATE_INTERVAL, TClock(), TController(), TActuators(), TFeedback()) {}

    /// Initializes the manager with the default values from the configuration file.
    ControllerManager(TActuators actuators, TFeedback feedback)
        requires Default<TClock> && Default<TController>
        : ControllerManager(UPDATE_INTERVAL, TClock(), TController(), std::move(actuators), std::move(feedback)) {}

    ManagerStatus getStatus() const;
    inline bool isActive() const { return getStatus() == Active; }
    void setActive(bool active);

    /**
     * Sends an order to the controller if this manager is active. If the manager is not active, this does nothing.
     *
     * @return true if the order was sent, false otherwise.
     */
    bool sendOrder(std::function<void(TController &, Position2D<Meter>)> order);

    /**
     * Updates the state of the manager. Updates and sends the command to the actuators if the manager is active.
     * Does not loop despite the name; must be called repeatedly.
     *
     * @param tickCallback Optional callback to be called every time the state of the controller is updated. The callback may be called zero,
     * one or multiple times depending on the update interval (see constructor documentation) and the last time function loop() was called.
     * The callback will not be called if the manager is currently inactive.
     */
    void loop(std::optional<std::function<void()>> tickCallback = {});
    void resetPosition(Position2D<Millimeter> newPosition);

    // Getters and setters

    duration_t getMinUpdateInterval() const;
    duration_t getMaxUpdateInterval() const;

    void setMinUpdateInterval(duration_t updateInterval);
    void setMaxUpdateInterval(duration_t updateInterval);
    void setUpdateInterval(duration_t updateInterval);

    const TController &getController() const;
    /**
     * This function can be used to make persistent changes to the controller even when the manager is inactive.
     * It should not be used to to update the command or start a displacement. Use sendOrder instead.
     */
    TController &getController();
    const TActuators &getActuators() const;
    const TFeedback &getPositionFeedback() const;

  private:
    bool sendOrderInternal(std::function<void(TController &, Position2D<Meter>)> order);

    TClock m_clock;
    duration_t m_minUpdateInterval; // µS
    duration_t m_maxUpdateInterval; // µS
    std::optional<instant_t> m_lastUpdate;

    TController m_controller;
    TActuators m_actuators;
    TFeedback m_feedback;
};

} // namespace manager

#endif