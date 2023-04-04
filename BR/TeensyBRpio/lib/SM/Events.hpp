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

#endif  // EVENTS_H