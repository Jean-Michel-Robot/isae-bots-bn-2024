#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "BrSM.hpp"
#include "Events.hpp"
// wrapper to manage the state machines



class BrSMWrapper : public BrSM
{
public :
    BrSMWrapper();

    void updateSM();

    int test;


private :
    BrSM brSM;
    BrUpdateEvent brUpdateEvent;
};

#endif  // STATE_MACHINE_H