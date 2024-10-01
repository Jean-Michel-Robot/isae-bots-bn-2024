#include "manager/states/StateActivating.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
StateActivating<TActuators, TController>::StateActivating(TActuators &actuators) {
    log(INFO, "Entering manager state: Activating");
    actuators.switchOn();
}

template <Actuators TActuators, typename TController>
ManagerStatus StateActivating<TActuators, TController>::getStatus() const {
    return Activating;
}

template <Actuators TActuators, typename TController>
ManagerStatus StateActivating<TActuators, TController>::update(TActuators &actuators) {
    // TODO: check `actuators.isIdle()`?
    if (actuators.isReady()) {
        return Active;
    } else {
        return Activating;
    }
}

} // namespace manager

#include "specializations/actuators.hpp"
#include "specializations/controller.hpp"
template class manager::StateActivating<actuators_t, controller_t>;
