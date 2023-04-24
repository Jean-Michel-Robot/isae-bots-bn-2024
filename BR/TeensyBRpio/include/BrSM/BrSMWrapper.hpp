#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "BrSM/BrSM.hpp"
#include "Events.hpp"
// wrapper to manage the state machines



class BrSMWrapper : public BrSM
{
public :
    BrSMWrapper();

    void updateSM();

    int test;

    BrSM brSM; //TODO remettre en private


private :
    BrUpdateEvent brUpdateEvent;
};

#endif  // STATE_MACHINE_H