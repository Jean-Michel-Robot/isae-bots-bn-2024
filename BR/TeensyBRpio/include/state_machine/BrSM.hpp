#ifndef BR_SM_H
#define BR_SM_H

#include "tinyfsm.hpp"
#include "state_machine/Events.hpp"
#include "trajectories/Trajectory.hpp"

#include "utils/Timer.hpp"
#include "utils/SwitchFiltered.hpp"

#include <memory>


// States
#define FOREACH_BRSTATE(BRSTATE) \
        BRSTATE(BR_UNDEF)   \
        BRSTATE(BR_IDLE)  \
        BRSTATE(BR_INITROT)   \
        BRSTATE(BR_FORWARD)  \
        BRSTATE(BR_FINALROT)   \
        BRSTATE(BR_READY)  \
		BRSTATE(BR_RECAL_ASSERV) \
		BRSTATE(BR_RECAL_DETECT) \
		BRSTATE(BR_EMERGENCYSTOP) \

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
	/* default reaction for unhandled events */
	void react(tinyfsm::Event const &) { };

	virtual void react(OrderEvent        const &);
	virtual void react(GoalReachedEvent const &);
	virtual void react(ErrorEvent const &);
	virtual void react(BrGetReadyEvent const &);
	virtual void react(BrSetToIdleEvent const &);
	virtual void react(BrEmergencyBrakeEvent const &);
	virtual void react(ResetPosEvent const &);

	// Update function in states, common but can be overwritten
	virtual void react(BrUpdateEvent const & e);


	//   void         react(Alarm       const &);

	virtual void entry(void) { };  /* entry actions in some states */
	void         exit(void)  { };  /* if no exit actions at all */

	static BRState getCurrentState();
	static String getCurrentStateStr();

	static Position2D<Meter> getCurrentGoalPos();

	static float getCurrentTargetSpeed();

	static void setup();
	static std::unique_ptr<Trajectory> currentTrajectory; //TODO remettre en private


protected:

	BrSM() = default; // prevent direct initialization

	// static constexpr int initial_floor = 0;
	// static int current_floor;
	// static int dest_floor;

	static AxisStates axisStates;
	static OrderType currentOrder;

	static Timer recalAsservTimer;
	static Timer waitTimer;

	static BRState currentState;
	static BRState requestedState;
	static Position2D<Meter> currentGoalPos;

	static SwitchFiltered m_switchLeft;
	static SwitchFiltered m_switchRight;

	//NOTE les switches ont été mis comme objets statiques de la BrSM
	//ça marche bien si on ne les utilise que pour la SM
	static SwitchFiltered* m_switches[2];  // 0 : right, 1 : left

	// whether or not the BR should be in Idle, used to wait for the Odrive response
	// to change between Idle and Ready states
	static bool isSupposedToBeIdle;


	static void setupTrajectory();
};


template<typename E>
void send_event(E const & event)
{
  BrSM::dispatch<E>(event);
}


#endif  // BR_SM_H
