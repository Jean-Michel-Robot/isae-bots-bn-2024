#ifndef BR_SM_H
#define BR_SM_H

// #include "tinyfsm/tinyfsm.hpp"

#include <Events.hpp>

// #include <ROS.hpp>


// States
enum BRState
{
	UNDEF = 0,

	BR_IDLE = 1,
	INITROT = 2,
	FORWARD = 3,
	FINALROT = 4,
	ARRIVED = 5,
};


//TODO : mettre autre part
enum GoalType { // type d'objectif recu par le haut niveau
	UNVALID_GOALTYPE = -1, // sert a rejeter les valeurs non conformes

	FINAL = 0,             // point final, avec orientation
	TRANS = 1,             // point transitoire, sans orientation finale
	ORIENT = 2,            // orientation seule sur place

	STOP  = 8,             // freinage d'urgence
	RESET = 9,             // reset de la position odometrique
	CONTROL = 10,          // controle en commande directe
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

	//   void         react(Alarm       const &);

	virtual void entry(void) { };  /* entry actions in some states */
	void         exit(void)  { };  /* if no exit actions at all */

	BRState getCurrentState();

protected:

	// static constexpr int initial_floor = 0;
	// static int current_floor;
	// static int dest_floor;

	static AxisStates axisStates;
	static OrderType currentOrder;

	static BRState currentState;  //TODO : besoin ou pas ? A priori oui ce sera plus simple
};

#endif  // BR_SM_H
