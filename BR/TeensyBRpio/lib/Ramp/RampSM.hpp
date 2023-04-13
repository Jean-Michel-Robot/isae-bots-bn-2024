#ifndef __H_RAMP_SM
#define __H_RAMP_SM

#include <tinyfsm/tinyfsm.hpp>
#include <Events.hpp>


enum RampState
{
	UNDEF = 0,
	STILL = 1,
	ACCEL = 2,
	CONSTANT = 3,
	DECEL = 4,
	BRAKE = 5,
};



class RampSM
: public tinyfsm::Fsm<RampSM>
{
	/* NOTE: react(), entry() and exit() functions need to be accessible
	* from tinyfsm::Fsm class. You might as well declare friendship to
	* tinyfsm::Fsm, and make these functions private:
	*
	* friend class Fsm;
	*/
public:

    RampSM();

	/* default reaction for unhandled events */
	void react(tinyfsm::Event const &) { };

    virtual void react(UpdateEvent const &);
	virtual void react(GoalSpeedChangeEvent const &);

	//   void         react(Alarm       const &);

	virtual void entry(void) { };  /* entry actions in some states */
	void         exit(void)  { };  /* if no exit actions at all */

	RampState getCurrentState();
    void setAccelParam(float accelParam);
    float getCurrentSpeed();

protected:

	// static constexpr int initial_floor = 0;
	// static int current_floor;
	// static int dest_floor;

	// static AxisStates axisStates;
	// static OrderType currentOrder;

    static float accelParam;  // pas en constexpr car potentiellement modifiable

    static float currentSpeed;
	static RampState currentState;
};

#endif