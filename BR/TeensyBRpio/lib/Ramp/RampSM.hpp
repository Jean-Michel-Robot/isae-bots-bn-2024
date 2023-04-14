#ifndef __H_RAMP_SM
#define __H_RAMP_SM

#include <tinyfsm/tinyfsm.hpp>
#include <Events.hpp>

#include "defines.hpp"

enum RampState
{
	UNDEF = 0,
	IDLE = 1,  // constant a 0 (pas de mouvement en cours)
	SLOPE = 2,  // positive ou négative
	CONSTANT = 3,
	RAMP_END = 4,
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

	virtual void react(BeginRampEvent const & e);
    virtual void react(UpdateEvent const &);
	virtual void react(GoalSpeedChangeEvent const &);
	virtual void react(EmergencyBrakeEvent const & e);
	virtual void react(EndRampEvent const & e);

	virtual void entry(void) { };  /* entry actions in some states */
	void         exit(void)  { };  /* if no exit actions at all */

	RampState getCurrentState();
    void setAccelParam(float accelParam);
    void setGoalSpeed(float goalSpeed);
    // void setT0(float t0);
    float getCurrentSpeed();

    template<typename E>
    void send_event(E const & event)
    {
        dispatch<E>(event);
    }

protected:

    static float t0, t_start_slope, V_start_slope;  //t_current si on a besoin du dt
    static float d;

    static float accelParam;  // pas en constexpr car potentiellement modifiable
    static constexpr float accelBrake = ACCEL_BRAKE;

    static float goalSpeed;
    static float currentSpeed;
	static RampState currentState;
};

#endif