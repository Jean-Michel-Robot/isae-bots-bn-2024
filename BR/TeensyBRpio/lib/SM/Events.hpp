#ifndef EVENTS_H
#define EVENTS_H

#include "tinyfsm/tinyfsm.hpp"
#include <Arduino.h>

// ----------------------------------------------------------------------------
// Event declarations
//
typedef struct OrderType
{
	//TODO refactor en Position2D + int
	float x = 0.0;
	float y = 0.0;
	float theta = 0.0;
	int goalType = 0;
} OrderType;

struct OrderEvent : tinyfsm::Event
{
	OrderType order;
};

struct GoalReachedEvent : tinyfsm::Event
{
	int goalType = 0;
};

// Event pour reset la pos des odos
struct ResetPosEvent : tinyfsm::Event
{
	float x = 0.0;
	float y = 0.0;
	float theta = 0.0;
};

struct ErrorEvent : tinyfsm::Event
{
	int errorCode = 0;
};

struct EmergencyBrakeEvent : tinyfsm::Event {};
struct BrEmergencyBrakeEvent : tinyfsm::Event {}; //TODO make it the same event and dispatch it properly between the SMs


// Event pour armer les moteurs (les passer en closed_loop)
//TODO Éventuellement en faisant une calibration d'abord si besoin
struct BrGetReadyEvent : tinyfsm::Event {};

// Event pour désarmer les moteurs (les passer en Idle)
//TODO peut se faire à n'importe quel moment en guise d'arrêt d'urgence ?
struct BrSetToIdleEvent : tinyfsm::Event {};



// for ramp SM

struct GoalSpeedChangeEvent : tinyfsm::Event
{
	float newSpeed = 0;
};

struct BeginRampEvent : tinyfsm::Event
{
	float t0;
};

struct EndRampEvent : tinyfsm::Event {};  // Signale qu'il faut rediriger la rampe vers 0

// Utilisé si la trajectoire est terminée mais que la rampe continue un peu
// On fait alors un échelon de vitesse à 0 et on force la rampe en état Idle
struct SetRampToIdleEvent : tinyfsm::Event {};

// for any SM ?
struct UpdateEvent : tinyfsm::Event
{
	uint32_t currentTime;
};

struct BrUpdateEvent : tinyfsm::Event
{
	uint32_t currentTime;
};



#endif  // EVENTS_H