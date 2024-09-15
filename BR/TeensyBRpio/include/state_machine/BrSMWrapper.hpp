#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "state_machine/Events.hpp"
#include "state_machine/BrSM.hpp"

// TODO this is called BrSMWrapper for history reasons, but this no longer wraps any BrSM
// wrapper to manage the state machines
class BrSMWrapper
{
public :
    BrSMWrapper();

    void loop();

    static BrSMWrapper &instance();

private :
    BrUpdateEvent brUpdateEvent;
    uint32_t timer;
};

#endif  // STATE_MACHINE_H