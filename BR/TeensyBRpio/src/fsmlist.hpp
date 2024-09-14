#ifndef FSMLIST_HPP_INCLUDED
#define FSMLIST_HPP_INCLUDED

// #include "tinyfsm/tinyfsm.hpp"

#include "BrSM.hpp"
#include "motorSM.hpp"

// using fsm_list = tinyfsm::FsmList<MotorSM, MotorSM, BrSM>;  // if we want several state machines

// using fsm_handle = BrSM;  // if we only want one state machine

// using fsm_handle = tinyfsm::Fsm<BrSM>;  // if we only want one state machine
// fsm_handle fsm_handle1;

template<typename E>
void send_event(E const & event)
{
  BrSM::dispatch<E>(event);
}

// using MotorFSM1 = tinyfsm::Fsm<MotorSM>;
// using MotorFSM2 = tinyfsm::Fsm<MotorSM>;
// using BrSM1 = tinyfsm::Fsm<BrSM>;
// using MotorFSMList = tinyfsm::FsmList<MotorFSM1, MotorFSM2, BrSM1>;

/** dispatch event to all state machines */
// template<typename E>
// void send_event(E const & event)
// {
//   fsm_list::template dispatch<E>(event);
// }

// MotorFSMList fsm_list1;


// _state_instance state1 = fsm_list1.getStates();

// MotorSM *state1 = fsm_list.set_initial_state();

// void getStates() {
//   fsm_list.c
// }


#endif
