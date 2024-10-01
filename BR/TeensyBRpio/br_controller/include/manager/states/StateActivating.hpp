#ifndef _MANAGER_STATE_ACTIVATING_HPP_
#define _MANAGER_STATE_ACTIVATING_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

/// Idle -> Closed loop
template <Actuators TActuators, typename TController>
class StateActivating : public ManagerState<TActuators, TController> {
  public:
    /// @param actuators the reference must not escape the function
    StateActivating(TActuators &actuators);
    ManagerStatus getStatus() const override;
    ManagerStatus update(TActuators &actuators) override;
};

} // namespace manager

#endif