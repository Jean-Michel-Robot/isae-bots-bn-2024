#ifndef _MANAGER_STATE_HPP_
#define _MANAGER_STATE_HPP_

#include "Actuators.hpp"
#include <functional>

namespace manager {

enum ManagerStatus {
    /// Controller inactive
    Idle,
    /// setActive(false) called, waiting for the motors to get idle
    Deactivating,
    /// setActive(true) called, waiting for the motors to get ready
    Activating,
    /// Controller active
    Active
};

/// State of a controller.
template <Actuators TActuators, typename TController>
class ManagerState {
  public:
    virtual ManagerStatus getStatus() const = 0;
    /// @param actuators The reference must not escape the function
    virtual ManagerStatus update(TActuators &actuators) = 0;
    /// Optionally calls "order" with arguments "controller" and "robotPosition" if the current state of the manager allows to do so.
    /// @param controller The reference must not escape the function
    /// @param order The closure has no lifetime specification and should not escape this function.
    /// @return true if order was called, false otherwise.
    virtual bool sendOrder(TController &controller, Position2D<Meter> robotPosition, std::function<void(TController &, Position2D<Meter>)> order);

  protected:
    ManagerState();
};

} // namespace manager

#endif