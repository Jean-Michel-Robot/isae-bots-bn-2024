#ifndef EVENTS_H
#define EVENTS_H

#include "tinyfsm/tinyfsm.hpp"

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

struct EmergencyBrakeEvent : tinyfsm::Event {};


// for ramp SM

struct GoalSpeedChangeEvent : tinyfsm::Event
{
	float newSpeed = 0;
};

struct EndRampEvent : tinyfsm::Event {};  // Signale qu'il faut rediriger la rampe vers 0



// for any SM ?
struct UpdateEvent : tinyfsm::Event
{
	float currentTime;
};

#endif  // EVENTS_H