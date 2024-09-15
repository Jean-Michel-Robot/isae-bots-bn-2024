#ifndef __H_RAMP_SM
#define __H_RAMP_SM

#include "tinyfsm.hpp"
#include "state_machine/Events.hpp"


// enum RampState
// {
// 	RAMP_UNDEF = 0,
// 	RAMP_IDLE = 1,  // constant a 0 (pas de mouvement en cours)
// 	RAMP_SLOPE = 2,  // positive ou négative
// 	RAMP_CONSTANT = 3,
// 	RAMP_END = 4,
// 	RAMP_BRAKE = 5,
// };

// States
#define FOREACH_RAMPSTATE(RAMPSTATE) \
        RAMPSTATE(RAMP_UNDEF)   \
        RAMPSTATE(RAMP_IDLE)  \
        RAMPSTATE(RAMP_SLOPE)   \
        RAMPSTATE(RAMP_CONSTANT)  \
        RAMPSTATE(RAMP_END)   \
        RAMPSTATE(RAMP_BRAKE)  \

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

enum RampState {
    FOREACH_RAMPSTATE(GENERATE_ENUM)
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

	/* default reaction for unhandled events */
	void react(tinyfsm::Event const &) { };

	virtual void react(BeginRampEvent const & e);
    virtual void react(UpdateEvent const &);
	virtual void react(GoalSpeedChangeEvent const &);
	virtual void react(EmergencyBrakeEvent const & e);
	virtual void react(EndRampEvent const & e);
	virtual void react(SetRampToIdleEvent const &);

	virtual void entry(void) { };  /* entry actions in some states */
	void         exit(void)  { };  /* if no exit actions at all */

	static RampState getCurrentState();
	static String getCurrentStateStr();

	static void setAccelParam(float accelParam);
    static void setGoalSpeed(float goalSpeed);
    // void setT0(float t0);
    static float getCurrentSpeed();

    static void reset_and_start();

protected:

	RampSM() = default; // prevent direct initialization

	static uint32_t t0, t_start_slope; // + t_current si on a besoin du dt
	static float V_start_slope;

	static float accelParam;  // pas en define car potentiellement modifiable

    static float goalSpeed;
    static float currentSpeed;
	static RampState currentState;
};

#endif