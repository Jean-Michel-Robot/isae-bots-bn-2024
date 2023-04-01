#ifndef FSMLIST_HPP_INCLUDED
#define FSMLIST_HPP_INCLUDED

#include "tinyfsm/tinyfsm.hpp"

#include "BrSM.hpp"
#include "motorSM.hpp"

using fsm_list = tinyfsm::FsmList<MotorSM, MotorSM, BrSM>;

/** dispatch event to all state machines */
template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}


#endif
