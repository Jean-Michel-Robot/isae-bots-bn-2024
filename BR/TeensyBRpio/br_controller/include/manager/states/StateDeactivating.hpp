#ifndef _MANAGER_STATE_DEACTIVATING_HPP_
#define _MANAGER_STATE_DEACTIVATING_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

/// Closed loop -> Idle
template <Actuators TActuators, typename TController>
class StateDeactivating : public ManagerState<TActuators, TController> {
  public:
    /// @param actuators the reference must not escape the function
    StateDeactivating(TActuators &actuators);
    ManagerStatus getStatus() const override;
    ManagerStatus update(TActuators &actuators) override;
};

} // namespace manager

#endif