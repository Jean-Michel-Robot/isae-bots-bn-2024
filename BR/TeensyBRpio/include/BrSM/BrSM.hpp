#ifndef BR_SM_H
#define BR_SM_H

// #include "tinyfsm/tinyfsm.hpp"

#include <Events.hpp>

#include <Arduino.h>
#include <Timer.hpp>

#include "Trajectories/Trajectory.hpp"
#include <SwitchFiltered.hpp>

// #include <ROS.hpp>


// States
#define FOREACH_BRSTATE(BRSTATE) \
        BRSTATE(BR_UNDEF)   \
        BRSTATE(BR_IDLE)  \
        BRSTATE(BR_INITROT)   \
        BRSTATE(BR_FORWARD)  \
        BRSTATE(BR_FINALROT)   \
        BRSTATE(BR_READY)  \
		BRSTATE(BR_RECAL) \

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

enum BRState {
    FOREACH_BRSTATE(GENERATE_ENUM)
};



typedef struct AxisStates {
	int state_ax0;
	int state_ax1;
} AxisStates;



// ----------------------------------------------------------------------------
// Elevator (FSM base class) declaration
//

class BrSM
: public tinyfsm::Fsm<BrSM>
{
	/* NOTE: react(), entry() and exit() functions need to be accessible
	* from tinyfsm::Fsm class. You might as well declare friendship to
	* tinyfsm::Fsm, and make these functions private:
	*
	* friend class Fsm;
	*/
public:
	BrSM();

	/* default reaction for unhandled events */
	void react(tinyfsm::Event const &) { };

	virtual void react(OrderEvent        const &);
	virtual void react(GoalReachedEvent const &);
	virtual void react(ErrorEvent const &);
	
	// Update function in states, can be overwritten
	virtual void react(BrUpdateEvent const & e);


	//   void         react(Alarm       const &);

	virtual void entry(void) { };  /* entry actions in some states */
	void         exit(void)  { };  /* if no exit actions at all */

	BRState getCurrentState();
	String getCurrentStateStr();

	float getCurrentTargetSpeed();

	template<typename E>
    void send_event(E const & event)
    {
        dispatch<E>(event);
    }

	static Trajectory* currentTrajectory; //TODO remettre en private


protected:

	// static constexpr int initial_floor = 0;
	// static int current_floor;
	// static int dest_floor;

	static AxisStates axisStates;
	static OrderType currentOrder;

	static Timer recalTimer;

	static BRState currentState;  //TODO : besoin ou pas ? A priori oui ce sera plus simple

	static SwitchFiltered m_switchLeft;
	static SwitchFiltered m_switchRight;

	//NOTE les switches ont été mis comme objets statiques de la BrSM
	//ça marche bien si on ne les utilise que pour la SM
	static SwitchFiltered* m_switches[2];  // 0 : right, 1 : left


	static void setupTrajectory();
};

#endif  // BR_SM_H
