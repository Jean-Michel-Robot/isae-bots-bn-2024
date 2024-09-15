#ifndef MOTORSM_HPP_H
#define MOTORSM_HPP_H

#include "tinyfsm.hpp"


// ----------------------------------------------------------------------------
// Event declarations
//

struct CalibrationEvent   : tinyfsm::Event { };
struct StopEvent : tinyfsm::Event { };


// ----------------------------------------------------------------------------
// Motor (FSM base class) declaration
//
class MotorSM
: public tinyfsm::Fsm<MotorSM>
{
  /* NOTE: react(), entry() and exit() functions need to be accessible
   * from tinyfsm::Fsm class. You might as well declare friendship to
   * tinyfsm::Fsm, and make these functions private:
   *
   * friend class Fsm;
   */
public:

  /* default reaction for unhandled events */
  void react(tinyfsm::Event const &) { };

  /* non-virtual declaration: reactions are the same for all states */
  void react(CalibrationEvent   const &);
  void react(StopEvent const &);

  virtual void entry(void) = 0;  /* pure virtual: enforce implementation in all states */
  void exit(void)  { };          /* no exit actions at all */

protected:

  static int axisState;

public:
  static int getAxisState() { return axisState; }
};


#endif  // MOTORSM_HPP_H
