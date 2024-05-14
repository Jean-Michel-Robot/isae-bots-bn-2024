#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "Events.hpp"
#include "BrSM/BrSM.hpp"


// TODO this is called BrSMWrapper for history reasons, but this no longer wraps any BrSM
// wrapper to manage the state machines
class BrSMWrapper
{
public :
    BrSMWrapper();

    void loop();

private :
    BrUpdateEvent brUpdateEvent;
    uint32_t timer;
};

#endif  // STATE_MACHINE_H