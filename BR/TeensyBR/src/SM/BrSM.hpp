#ifndef BR_SM_H
#define BR_SM_H

#include "tinyfsm/tinyfsm.hpp"

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
// Event declarations
//
typedef struct OrderType
{
	float x = 0.0;
	float y = 0.0;
	float theta = 0.0;
	int goalType = 0;
} OrderType;

struct OrderEvent : tinyfsm::Event
{
	OrderType order;
	//NOTE : timestamp ?
};

struct GoalReachedEvent : tinyfsm::Event
{
	int goalType = 0;
};

struct ErrorEvent : tinyfsm::Event
{
	int errorCode = 0;
};

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
	void         exit(void)  { };  /* no exit actions at all */

protected:

	// static constexpr int initial_floor = 0;
	// static int current_floor;
	// static int dest_floor;

	static AxisStates axisStates;
	static OrderType currentOrder;
};

#endif  // BR_SM_H
