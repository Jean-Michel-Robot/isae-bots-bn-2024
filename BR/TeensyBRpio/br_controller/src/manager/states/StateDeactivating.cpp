#include "manager/states/StateDeactivating.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
StateDeactivating<TActuators, TController>::StateDeactivating(TActuators &actuators) {
    log(INFO, "Entering manager state: Deactivating");
    actuators.switchOff();
}

template <Actuators TActuators, typename TController>
ManagerStatus StateDeactivating<TActuators, TController>::getStatus() const {
    return Deactivating;
}

template <Actuators TActuators, typename TController>
ManagerStatus StateDeactivating<TActuators, TController>::update(TActuators &actuators) {
    // TODO: check `actuators.isReady()`?
    if (actuators.isIdle()) {
        return Idle;
    } else {
        return Deactivating;
    }
}

} // namespace manager

#include "specializations/actuators.hpp"
#include "specializations/controller.hpp"
template class manager::StateDeactivating<actuators_t, controller_t>;
