#ifndef EVENTS_H
#define EVENTS_H

#include "tinyfsm.hpp"
#include "geometry/Position2D.h"

// ----------------------------------------------------------------------------
// Event declarations
//
class OrderType : public Position2D<Meter>
{
public:
	OrderType(): Position2D<Meter>() {};
	OrderType(Position2D<Meter> const& position, int goalType) : Position2D<Meter>(position), goalType(goalType) {};
	OrderType(float x, float y, float theta, int goalType): Position2D<Meter>(x, y, theta), goalType(goalType) {};
	int goalType = 0;
};

struct OrderEvent : tinyfsm::Event
{
	OrderType order;
};

struct GoalReachedEvent : tinyfsm::Event
{
	int goalType = 0;
};

// Event pour reset la pos des odos
class ResetPosEvent : public tinyfsm::Event, public Position2D<Millimeter>
{
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